// Vatti scanline polygon clipping — ported from Clipper2 (Angus Johnson,
// Boost licence) into session_cpp types. Closed-path NonZero fill only.
#include "boolean_polyline.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>

using namespace session_cpp;
namespace {

struct BIVec2 { int64_t x, y; };
inline bool operator==(BIVec2 a, BIVec2 b) { return a.x==b.x && a.y==b.y; }
inline bool operator!=(BIVec2 a, BIVec2 b) { return !(a==b); }

// SIMD rounding — SSE2 on x64 (MSVC, GCC, Clang all have it)
#if (defined(_MSC_VER) && (defined(_M_AMD64) || defined(_M_X64))) || \
    (defined(__SSE2__))
#include <emmintrin.h>
#define VATTI_HAS_SSE2 1
#define VATTI_NEARBYINT(a) _mm_cvtsd_si64(_mm_set_sd(a))
// Convert stride-3 double[x,y,z] → BIVec2{x,y} using packed SSE2 mul+cvt
inline BIVec2 v_cvt_to_i64(const double* p, __m128d scale) {
    __m128d xy = _mm_mul_pd(_mm_loadu_pd(p), scale);
    return {_mm_cvtsd_si64(xy), _mm_cvtsd_si64(_mm_unpackhi_pd(xy, xy))};
}
// Convert BIVec2{x,y} → stride-3 double[x,y,0] using packed SSE2 mul
inline void v_cvt_to_dbl(double* dst, BIVec2 pt, __m128d inv_scale) {
    __m128d xy = _mm_mul_pd(_mm_set_pd(double(pt.y), double(pt.x)), inv_scale);
    _mm_storeu_pd(dst, xy);
    dst[2] = 0.0;
}
#else
#define VATTI_HAS_SSE2 0
#define VATTI_NEARBYINT(a) static_cast<int64_t>(std::nearbyint(a))
inline BIVec2 v_cvt_to_i64(const double* p, double scale) {
    return {(int64_t)std::nearbyint(p[0]*scale), (int64_t)std::nearbyint(p[1]*scale)};
}
inline void v_cvt_to_dbl(double* dst, BIVec2 pt, double inv_scale) {
    dst[0] = pt.x * inv_scale; dst[1] = pt.y * inv_scale; dst[2] = 0.0;
}
#endif

enum : uint32_t { VF_None=0, VF_LocalMax=4, VF_LocalMin=8 };

struct VVertex {
    BIVec2 pt;
    VVertex* next = nullptr;
    VVertex* prev = nullptr;
    uint32_t flags = VF_None;
};

struct VLocalMinima { VVertex* vertex; int8_t polytype; };

struct VHorzSeg;
struct VOutPt {
    BIVec2 pt;
    VOutPt* next = nullptr;
    VOutPt* prev = nullptr;
    struct VOutRec* outrec;
    VHorzSeg* horz = nullptr;
};

struct VOutRec {
    size_t idx = 0;
    struct VActive* front_edge = nullptr;
    struct VActive* back_edge = nullptr;
    VOutPt* pts = nullptr;
    VOutRec* owner = nullptr;
};

struct VActive {
    BIVec2 bot, top;
    int64_t curr_x = 0;
    double dx = 0.0;
    int wind_dx = 1;
    int wind_cnt = 0;
    int wind_cnt2 = 0;
    VOutRec* outrec = nullptr;
    VActive* prev_in_ael = nullptr;
    VActive* next_in_ael = nullptr;
    VActive* prev_in_sel = nullptr;
    VActive* next_in_sel = nullptr;
    VActive* jump = nullptr;
    VVertex* vertex_top = nullptr;
    VLocalMinima* local_min = nullptr;
    bool is_left_bound = false;
    int8_t join_with = 0; // 0=None, 1=Left, 2=Right
};

struct VIntersectNode { BIVec2 pt; VActive* edge1; VActive* edge2; };
struct VHorzSeg { VOutPt* left_op; VOutPt* right_op = nullptr; bool left_to_right = true; };
struct VHorzJoin { VOutPt* op1; VOutPt* op2; };

template<typename T> struct Pool {
    std::vector<T> buf;
    size_t count = 0;
    void ensure(size_t n) { if (buf.size() < n) buf.resize(n); }
    T* alloc() {
        if (count >= buf.size()) buf.resize(std::max<size_t>(buf.size()*2, 256));
        return &buf[count++];
    }
    void reset() { count = 0; }
};

// Max-heap using sorted vector — avoids std::priority_queue realloc on reset.
struct ScanlineHeap {
    std::vector<int64_t> buf;
    size_t sz = 0;
    void clear() { sz = 0; }
    bool empty() const { return sz == 0; }
    void push(int64_t y) {
        if (sz >= buf.size()) buf.resize(std::max<size_t>(buf.size()*2, 64));
        buf[sz] = y;
        // sift up
        size_t i = sz++;
        while (i > 0) { size_t p=(i-1)/2; if (buf[p]>=buf[i]) break; std::swap(buf[p],buf[i]); i=p; }
    }
    int64_t top() const { return buf[0]; }
    void pop() {
        buf[0] = buf[--sz];
        // sift down
        size_t i=0;
        for(;;) { size_t l=2*i+1, r=l+1, m=i;
            if(l<sz&&buf[l]>buf[m]) m=l; if(r<sz&&buf[r]>buf[m]) m=r;
            if(m==i) break; std::swap(buf[i],buf[m]); i=m; }
    }
};

struct VattiScratch {
    Pool<VVertex> vtx_pool;
    Pool<VActive> act_pool;
    Pool<VOutPt>  opt_pool;
    Pool<VOutRec> orc_pool;
    std::vector<VLocalMinima> locmin_list;
    std::vector<VIntersectNode> intersect_nodes;
    std::vector<VHorzSeg> horz_seg_list;
    std::vector<VHorzJoin> horz_join_list;
    std::vector<VOutRec*> outrec_list;
    ScanlineHeap scanline_list;
    std::vector<BIVec2> va, vb;
    // Engine state
    VActive* actives = nullptr;
    VActive* sel = nullptr;
    int64_t bot_y = 0;
    size_t locmin_idx = 0;
    bool succeeded = true;
    void reset() {
        vtx_pool.reset(); act_pool.reset(); opt_pool.reset(); orc_pool.reset();
        locmin_list.clear(); intersect_nodes.clear(); horz_seg_list.clear();
        horz_join_list.clear(); outrec_list.clear(); scanline_list.clear();
        actives = nullptr; sel = nullptr; bot_y = 0; locmin_idx = 0; succeeded = true;
    }
    VVertex* new_vertex() { auto* v = vtx_pool.alloc(); *v = VVertex{}; return v; }
    VActive* new_active() { auto* a = act_pool.alloc(); *a = VActive{}; return a; }
    VOutPt* new_outpt(BIVec2 pt, VOutRec* rec) { auto* o = opt_pool.alloc(); *o = VOutPt{}; o->pt=pt; o->outrec=rec; o->next=o; o->prev=o; return o; }
    VOutRec* new_outrec() { auto* r = orc_pool.alloc(); *r = VOutRec{}; r->idx = outrec_list.size(); outrec_list.push_back(r); return r; }
};
static thread_local VattiScratch vtls;

// ── Geometry helpers ─────────────────────────────────────────────────────

inline double v_get_dx(BIVec2 p1, BIVec2 p2) {
    double dy = double(p2.y - p1.y);
    if (dy != 0) return double(p2.x - p1.x) / dy;
    return (p2.x > p1.x) ? -std::numeric_limits<double>::max() : std::numeric_limits<double>::max();
}

inline int64_t v_top_x(const VActive& ae, int64_t y) {
    if (y == ae.top.y || ae.top.x == ae.bot.x) return ae.top.x;
    if (y == ae.bot.y) return ae.bot.x;
    return ae.bot.x + VATTI_NEARBYINT(ae.dx * double(y - ae.bot.y));
}

inline bool v_is_horizontal(const VActive& e) { return e.top.y == e.bot.y; }
inline bool v_is_hot(const VActive& e) { return e.outrec != nullptr; }
inline bool v_is_maxima(const VVertex& v) { return (v.flags & VF_LocalMax) != 0; }
inline bool v_is_maxima(const VActive& e) { return v_is_maxima(*e.vertex_top); }
inline bool v_is_front(const VActive& e) { return &e == e.outrec->front_edge; }
inline bool v_is_joined(const VActive& e) { return e.join_with != 0; }
inline bool v_same_polytype(const VActive& a, const VActive& b) { return a.local_min->polytype == b.local_min->polytype; }
inline int8_t v_polytype(const VActive& e) { return e.local_min->polytype; }
inline void v_set_dx(VActive& e) { e.dx = v_get_dx(e.bot, e.top); }

inline VVertex* v_next_vertex(const VActive& e) { return (e.wind_dx > 0) ? e.vertex_top->next : e.vertex_top->prev; }
inline VVertex* v_prev_prev_vertex(const VActive& ae) { return (ae.wind_dx > 0) ? ae.vertex_top->prev->prev : ae.vertex_top->next->next; }

inline double v_cross_product(BIVec2 p1, BIVec2 p2, BIVec2 p3) {
    return double(p2.x-p1.x)*double(p3.y-p2.y) - double(p2.y-p1.y)*double(p3.x-p2.x);
}
inline double v_dot_product(BIVec2 p1, BIVec2 p2, BIVec2 p3) {
    return double(p2.x-p1.x)*double(p3.x-p2.x) + double(p2.y-p1.y)*double(p3.y-p2.y);
}

inline bool v_products_equal(int64_t a, int64_t b, int64_t c, int64_t d) {
#if (defined(__clang__) || defined(__GNUC__)) && UINTPTR_MAX >= UINT64_MAX
    return static_cast<__int128_t>(a)*static_cast<__int128_t>(b) == static_cast<__int128_t>(c)*static_cast<__int128_t>(d);
#else
    auto lo = [](uint64_t x){ return x & 0xFFFFFFFF; };
    auto hi = [](uint64_t x){ return x >> 32; };
    auto mul = [&](uint64_t a, uint64_t b) -> std::pair<uint64_t,uint64_t> {
        uint64_t x1=lo(a)*lo(b); uint64_t x2=hi(a)*lo(b)+hi(x1); uint64_t x3=lo(a)*hi(b)+lo(x2);
        return {lo(x3)<<32|lo(x1), hi(a)*hi(b)+hi(x2)+hi(x3)};
    };
    auto sign = [](int64_t x){ return (x>0)-(x<0); };
    uint64_t ua=std::abs(a), ub=std::abs(b), uc=std::abs(c), ud=std::abs(d);
    auto [r1,c1]=mul(ua,ub); auto [r2,c2]=mul(uc,ud);
    return r1==r2 && c1==c2 && sign(a)*sign(b)==sign(c)*sign(d);
#endif
}

inline bool v_is_collinear(BIVec2 p1, BIVec2 shared, BIVec2 p2) {
    return v_products_equal(shared.x-p1.x, p2.y-shared.y, shared.y-p1.y, p2.x-shared.x);
}

inline double v_perpendic_dist_sq(BIVec2 pt, BIVec2 l1, BIVec2 l2) {
    double a=double(pt.x-l1.x), b=double(pt.y-l1.y), c=double(l2.x-l1.x), d=double(l2.y-l1.y);
    if (c==0 && d==0) return 0;
    double e = a*d - c*b;
    return (e*e) / (c*c + d*d);
}

inline bool v_get_seg_isect_pt(BIVec2 a, BIVec2 b, BIVec2 c, BIVec2 d, BIVec2& ip) {
    double dx1=double(b.x-a.x), dy1=double(b.y-a.y), dx2=double(d.x-c.x), dy2=double(d.y-c.y);
    double det = dy1*dx2 - dy2*dx1;
    if (det == 0.0) return false;
    double t = (double(a.x-c.x)*dy2 - double(a.y-c.y)*dx2) / det;
    if (t <= 0.0) ip = a;
    else if (t >= 1.0) ip = b;
    else { ip.x = a.x + VATTI_NEARBYINT(t*dx1); ip.y = a.y + VATTI_NEARBYINT(t*dy1); }
    return true;
}

inline BIVec2 v_closest_pt_on_seg(BIVec2 pt, BIVec2 s1, BIVec2 s2) {
    if (s1==s2) return s1;
    double dx=double(s2.x-s1.x), dy=double(s2.y-s1.y);
    double q = (double(pt.x-s1.x)*dx + double(pt.y-s1.y)*dy) / (dx*dx + dy*dy);
    if (q<0) q=0; else if (q>1) q=1;
    return {s1.x + VATTI_NEARBYINT(q*dx), s1.y + VATTI_NEARBYINT(q*dy)};
}

inline bool v_segs_intersect(BIVec2 a, BIVec2 b, BIVec2 c, BIVec2 d) {
    auto sign = [](double v) -> int { if (!v) return 0; return v>0?1:-1; };
    return (sign(v_cross_product(a,c,d)) * sign(v_cross_product(b,c,d)) < 0) &&
           (sign(v_cross_product(c,a,b)) * sign(v_cross_product(d,a,b)) < 0);
}

inline double v_area_outpt(VOutPt* op) {
    double r = 0.0; VOutPt* o = op;
    do { r += double(o->prev->pt.y + o->pt.y) * double(o->prev->pt.x - o->pt.x); o = o->next; } while (o != op);
    return r * 0.5;
}

inline double v_area_tri(BIVec2 p1, BIVec2 p2, BIVec2 p3) {
    return double(p3.y+p1.y)*double(p3.x-p1.x) + double(p1.y+p2.y)*double(p1.x-p2.x) + double(p2.y+p3.y)*double(p2.x-p3.x);
}

inline bool v_pts_close(BIVec2 a, BIVec2 b) { return std::llabs(a.x-b.x)<2 && std::llabs(a.y-b.y)<2; }

inline bool v_very_small_tri(VOutPt& op) {
    return op.next->next==op.prev && (v_pts_close(op.prev->pt,op.next->pt) || v_pts_close(op.pt,op.next->pt) || v_pts_close(op.pt,op.prev->pt));
}

inline bool v_valid_closed(VOutPt* op) { return op && op->next!=op && op->next!=op->prev && !v_very_small_tri(*op); }

static bool pip_i(BIVec2 pt, const std::vector<BIVec2>& poly) {
    int winding = 0, n = (int)poly.size();
    for (int i = 0; i < n; i++) {
        BIVec2 a = poly[i], b = poly[(i+1)%n];
        if (a.y <= pt.y) { if (b.y > pt.y && (int64_t(b.x-a.x)*int64_t(pt.y-a.y) - int64_t(b.y-a.y)*int64_t(pt.x-a.x)) > 0) ++winding; }
        else { if (b.y <= pt.y && (int64_t(b.x-a.x)*int64_t(pt.y-a.y) - int64_t(b.y-a.y)*int64_t(pt.x-a.x)) < 0) --winding; }
    }
    return winding != 0;
}

// pip on VVertex circular linked list (for disjoint AABB fast-path without arrays)
static bool pip_vertex(BIVec2 pt, VVertex* head) {
    int winding = 0;
    VVertex* v = head;
    do {
        BIVec2 a = v->pt, b = v->next->pt;
        if (a.y <= pt.y) { if (b.y > pt.y && (int64_t(b.x-a.x)*int64_t(pt.y-a.y) - int64_t(b.y-a.y)*int64_t(pt.x-a.x)) > 0) ++winding; }
        else { if (b.y <= pt.y && (int64_t(b.x-a.x)*int64_t(pt.y-a.y) - int64_t(b.y-a.y)*int64_t(pt.x-a.x)) < 0) --winding; }
        v = v->next;
    } while (v != head);
    return winding != 0;
}

// ── Vertex building + local minima detection ─────────────────────────────

// Build VVertex linked list directly from stride-3 doubles — single pass,
// no intermediate BIVec2 array. Also computes AABB as side-effect.
// Returns pointer to first vertex (nullptr if degenerate).
#if VATTI_HAS_SSE2
static VVertex* v_add_path_from_doubles(const double* coords, int n, int8_t polytype,
    __m128d sv, VattiScratch& sc,
    int64_t& minX, int64_t& maxX, int64_t& minY, int64_t& maxY)
#else
static VVertex* v_add_path_from_doubles(const double* coords, int n, int8_t polytype,
    double sv, VattiScratch& sc,
    int64_t& minX, int64_t& maxX, int64_t& minY, int64_t& maxY)
#endif
{
    if (n < 3) return nullptr;
    auto& pool = sc.vtx_pool;
    pool.ensure(pool.count + n);
    VVertex* base = &pool.buf[pool.count];

#if VATTI_HAS_SSE2
    auto cvt = [&](const double* p) { return v_cvt_to_i64(p, sv); };
#else
    auto cvt = [&](const double* p) { return v_cvt_to_i64(p, sv); };
#endif
    // Only zero flags (must start at 0). Skip full 40-byte VVertex{} init per vertex.
    for (int i=0;i<n;i++) base[i].flags = VF_None;
    BIVec2 pt0 = cvt(coords);
    base[0].pt = pt0;
    minX = maxX = pt0.x; minY = maxY = pt0.y;
    VVertex* prev_v = &base[0];
    int cnt = 1;
    for (int i = 1; i < n; i++) {
        BIVec2 pt = cvt(coords + i*3);
        if (pt == prev_v->pt) continue;
        VVertex* cv = &base[cnt]; cv->pt = pt;
        cv->prev = prev_v; prev_v->next = cv;
        prev_v = cv; cnt++;
        if (pt.x < minX) minX = pt.x; else if (pt.x > maxX) maxX = pt.x;
        if (pt.y < minY) minY = pt.y; else if (pt.y > maxY) maxY = pt.y;
    }
    if (cnt < 3) return nullptr;
    if (prev_v->pt == base[0].pt) { prev_v = prev_v->prev; cnt--; }
    if (cnt < 3) return nullptr;
    pool.count += cnt;
    prev_v->next = &base[0]; base[0].prev = prev_v;

    // Local minima detection
    VVertex* pv = base[0].prev;
    while (pv != &base[0] && pv->pt.y == base[0].pt.y) pv = pv->prev;
    if (pv == &base[0]) return &base[0];
    bool going_up = pv->pt.y > base[0].pt.y, going_up0 = going_up;
    pv = &base[0]; VVertex* cv = base[0].next;
    while (cv != &base[0]) {
        if (cv->pt.y > pv->pt.y && going_up) { pv->flags |= VF_LocalMax; going_up = false; }
        else if (cv->pt.y < pv->pt.y && !going_up) { going_up = true; pv->flags |= VF_LocalMin; sc.locmin_list.push_back({pv, polytype}); }
        pv = cv; cv = cv->next;
    }
    if (going_up != going_up0) {
        if (going_up0) { pv->flags |= VF_LocalMin; sc.locmin_list.push_back({pv, polytype}); }
        else pv->flags |= VF_LocalMax;
    }
    return &base[0];
}

static void v_add_path(const std::vector<BIVec2>& pts, int n, int8_t polytype, VattiScratch& sc) {
    if (n < 3) return;
    // Bulk-allocate all vertices at once — one bounds check, contiguous in memory.
    auto& pool = sc.vtx_pool;
    pool.ensure(pool.count + n);
    VVertex* base = &pool.buf[pool.count];
    for (int i=0;i<n;i++) base[i].flags = VF_None;
    base[0].pt = pts[0];
    VVertex* prev_v = &base[0];
    int cnt = 1;
    for (int i = 1; i < n; i++) {
        if (pts[i] == prev_v->pt) continue;
        VVertex* cv = &base[cnt]; cv->pt = pts[i];
        cv->prev = prev_v; prev_v->next = cv;
        prev_v = cv; cnt++;
    }
    if (cnt < 3) return;
    if (prev_v->pt == base[0].pt) { prev_v = prev_v->prev; cnt--; }
    if (cnt < 3) return;
    pool.count += cnt; // commit allocation
    prev_v->next = &base[0]; base[0].prev = prev_v;

    // Find local minima in the closed path (single pass)
    VVertex* pv = base[0].prev;
    while (pv != &base[0] && pv->pt.y == base[0].pt.y) pv = pv->prev;
    if (pv == &base[0]) return;
    bool going_up = pv->pt.y > base[0].pt.y, going_up0 = going_up;
    pv = &base[0];
    VVertex* cv = base[0].next;
    while (cv != &base[0]) {
        if (cv->pt.y > pv->pt.y && going_up) { pv->flags |= VF_LocalMax; going_up = false; }
        else if (cv->pt.y < pv->pt.y && !going_up) { going_up = true; pv->flags |= VF_LocalMin; sc.locmin_list.push_back({pv, polytype}); }
        pv = cv; cv = cv->next;
    }
    if (going_up != going_up0) {
        if (going_up0) { pv->flags |= VF_LocalMin; sc.locmin_list.push_back({pv, polytype}); }
        else pv->flags |= VF_LocalMax;
    }
}

// ── AEL operations (doubly-linked list — all O(1)) ──────────────────────

inline VActive* v_get_maxima_pair(const VActive& e) {
    VActive* e2 = e.next_in_ael;
    while (e2) { if (e2->vertex_top == e.vertex_top) return e2; e2 = e2->next_in_ael; }
    return nullptr;
}

inline VVertex* v_get_curr_y_maxima(const VActive& e) {
    VVertex* r = e.vertex_top;
    if (e.wind_dx > 0) while (r->next->pt.y == r->pt.y) r = r->next;
    else while (r->prev->pt.y == r->pt.y) r = r->prev;
    return v_is_maxima(*r) ? r : nullptr;
}

inline VActive* v_get_prev_hot(const VActive& e) {
    VActive* p = e.prev_in_ael;
    while (p && !v_is_hot(*p)) p = p->prev_in_ael;
    return p;
}

static bool v_is_valid_ael_order(const VActive& resident, const VActive& newcomer) {
    if (newcomer.curr_x != resident.curr_x) return newcomer.curr_x > resident.curr_x;
    double d = v_cross_product(resident.top, newcomer.bot, newcomer.top);
    if (d != 0) return d < 0;
    if (!v_is_maxima(resident) && resident.top.y > newcomer.top.y)
        return v_cross_product(newcomer.bot, resident.top, v_next_vertex(resident)->pt) <= 0;
    if (!v_is_maxima(newcomer) && newcomer.top.y > resident.top.y)
        return v_cross_product(newcomer.bot, newcomer.top, v_next_vertex(newcomer)->pt) >= 0;
    int64_t y = newcomer.bot.y;
    if (resident.bot.y != y || resident.local_min->vertex->pt.y != y) return newcomer.is_left_bound;
    if (resident.is_left_bound != newcomer.is_left_bound) return newcomer.is_left_bound;
    if (v_is_collinear(v_prev_prev_vertex(resident)->pt, resident.bot, resident.top)) return true;
    return (v_cross_product(v_prev_prev_vertex(resident)->pt, newcomer.bot, v_prev_prev_vertex(newcomer)->pt) > 0) == newcomer.is_left_bound;
}

static void v_insert_left_edge(VattiScratch& sc, VActive& e) {
    if (!sc.actives) { e.prev_in_ael=nullptr; e.next_in_ael=nullptr; sc.actives=&e; }
    else if (!v_is_valid_ael_order(*sc.actives, e)) {
        e.prev_in_ael=nullptr; e.next_in_ael=sc.actives; sc.actives->prev_in_ael=&e; sc.actives=&e;
    } else {
        VActive* e2 = sc.actives;
        while (e2->next_in_ael && v_is_valid_ael_order(*e2->next_in_ael, e)) e2 = e2->next_in_ael;
        if (e2->join_with == 2/*Right*/) e2 = e2->next_in_ael;
        if (!e2) return;
        e.next_in_ael = e2->next_in_ael; if (e2->next_in_ael) e2->next_in_ael->prev_in_ael = &e;
        e.prev_in_ael = e2; e2->next_in_ael = &e;
    }
}

inline void v_insert_right_edge(VActive& e, VActive& e2) {
    e2.next_in_ael = e.next_in_ael; if (e.next_in_ael) e.next_in_ael->prev_in_ael = &e2;
    e2.prev_in_ael = &e; e.next_in_ael = &e2;
}

inline void v_swap_positions_in_ael(VattiScratch& sc, VActive& e1, VActive& e2) {
    VActive* next = e2.next_in_ael; if (next) next->prev_in_ael = &e1;
    VActive* prev = e1.prev_in_ael; if (prev) prev->next_in_ael = &e2;
    e2.prev_in_ael = prev; e2.next_in_ael = &e1; e1.prev_in_ael = &e2; e1.next_in_ael = next;
    if (!e2.prev_in_ael) sc.actives = &e2;
}

inline void v_delete_from_ael(VattiScratch& sc, VActive& e) {
    VActive* prev = e.prev_in_ael; VActive* next = e.next_in_ael;
    if (!prev && !next && &e != sc.actives) return;
    if (prev) prev->next_in_ael = next; else sc.actives = next;
    if (next) next->prev_in_ael = prev;
}

// ── Scanline ─────────────────────────────────────────────────────────────

inline void v_insert_scanline(VattiScratch& sc, int64_t y) { sc.scanline_list.push(y); }
inline bool v_pop_scanline(VattiScratch& sc, int64_t& y) {
    auto& sl = sc.scanline_list;
    if (sl.empty()) return false;
    y = sl.top(); sl.pop();
    while (!sl.empty() && y == sl.top()) sl.pop();
    return true;
}
inline bool v_pop_locmin(VattiScratch& sc, int64_t y, VLocalMinima*& lm) {
    if (sc.locmin_idx >= sc.locmin_list.size() || sc.locmin_list[sc.locmin_idx].vertex->pt.y != y) return false;
    lm = &sc.locmin_list[sc.locmin_idx++];
    return true;
}
inline void v_push_horz(VattiScratch& sc, VActive& e) { e.next_in_sel = sc.sel; sc.sel = &e; }
inline bool v_pop_horz(VattiScratch& sc, VActive*& e) { e = sc.sel; if (!e) return false; sc.sel = sc.sel->next_in_sel; return true; }

// ── Winding + contribution ───────────────────────────────────────────────

static void v_set_wind_count(VattiScratch& sc, VActive& e) {
    VActive* e2 = e.prev_in_ael;
    int8_t pt = v_polytype(e);
    while (e2 && v_polytype(*e2) != pt) e2 = e2->prev_in_ael;
    if (!e2) { e.wind_cnt = e.wind_dx; e2 = sc.actives; }
    else {
        if (e2->wind_cnt * e2->wind_dx < 0) {
            if (std::abs(e2->wind_cnt) > 1)
                e.wind_cnt = (e2->wind_dx * e.wind_dx < 0) ? e2->wind_cnt : e2->wind_cnt + e.wind_dx;
            else e.wind_cnt = e.wind_dx;
        } else {
            e.wind_cnt = (e2->wind_dx * e.wind_dx < 0) ? e2->wind_cnt : e2->wind_cnt + e.wind_dx;
        }
        e.wind_cnt2 = e2->wind_cnt2; e2 = e2->next_in_ael;
    }
    while (e2 != &e) {
        if (v_polytype(*e2) != pt) e.wind_cnt2 += e2->wind_dx;
        e2 = e2->next_in_ael;
    }
}

static bool v_is_contributing(const VActive& e, int cliptype) {
    // NonZero fill rule only
    if (std::abs(e.wind_cnt) != 1) return false;
    int wc2 = std::abs(e.wind_cnt2);
    if (cliptype == 0) return wc2 != 0;              // intersection
    if (cliptype == 1) return wc2 == 0;              // union
    // difference
    bool r = (wc2 == 0);
    return (v_polytype(e) == 0) ? r : !r; // subject vs clip
}

// ── Output operations ────────────────────────────────────────────────────

inline void v_set_sides(VOutRec& or_, VActive& f, VActive& b) { or_.front_edge = &f; or_.back_edge = &b; }

static void v_swap_outrecs(VActive& e1, VActive& e2) {
    VOutRec* or1=e1.outrec, *or2=e2.outrec;
    if (or1==or2) { VActive* t=or1->front_edge; or1->front_edge=or1->back_edge; or1->back_edge=t; return; }
    if (or1) { if (&e1==or1->front_edge) or1->front_edge=&e2; else or1->back_edge=&e2; }
    if (or2) { if (&e2==or2->front_edge) or2->front_edge=&e1; else or2->back_edge=&e1; }
    e1.outrec=or2; e2.outrec=or1;
}

static VOutPt* v_add_outpt(const VActive& e, BIVec2 pt, VattiScratch& sc) {
    VOutRec* outrec = e.outrec;
    bool to_front = v_is_front(e);
    VOutPt* op_front = outrec->pts, *op_back = op_front->next;
    if (to_front && pt == op_front->pt) return op_front;
    if (!to_front && pt == op_back->pt) return op_back;
    VOutPt* nop = sc.new_outpt(pt, outrec);
    op_back->prev = nop; nop->prev = op_front; nop->next = op_back; op_front->next = nop;
    if (to_front) outrec->pts = nop;
    return nop;
}

static VOutPt* v_add_local_min_poly(VActive& e1, VActive& e2, BIVec2 pt, VattiScratch& sc, bool is_new) {
    VOutRec* outrec = sc.new_outrec();
    e1.outrec = outrec; e2.outrec = outrec;
    VActive* prev_hot = v_get_prev_hot(e1);
    if (prev_hot) {
        if ((prev_hot == prev_hot->outrec->front_edge) == is_new) v_set_sides(*outrec, e2, e1);
        else v_set_sides(*outrec, e1, e2);
    } else {
        outrec->owner = nullptr;
        if (is_new) v_set_sides(*outrec, e1, e2); else v_set_sides(*outrec, e2, e1);
    }
    VOutPt* op = sc.new_outpt(pt, outrec);
    outrec->pts = op;
    return op;
}

static void v_uncouple(VActive& ae) {
    VOutRec* or_ = ae.outrec; if (!or_) return;
    or_->front_edge->outrec = nullptr; or_->back_edge->outrec = nullptr;
    or_->front_edge = nullptr; or_->back_edge = nullptr;
}

static void v_join_outrec_paths(VActive& e1, VActive& e2) {
    VOutPt* p1_st=e1.outrec->pts, *p2_st=e2.outrec->pts;
    VOutPt* p1_end=p1_st->next, *p2_end=p2_st->next;
    if (v_is_front(e1)) {
        p2_end->prev=p1_st; p1_st->next=p2_end; p2_st->next=p1_end; p1_end->prev=p2_st;
        e1.outrec->pts=p2_st; e1.outrec->front_edge=e2.outrec->front_edge;
        if (e1.outrec->front_edge) e1.outrec->front_edge->outrec=e1.outrec;
    } else {
        p1_end->prev=p2_st; p2_st->next=p1_end; p1_st->next=p2_end; p2_end->prev=p1_st;
        e1.outrec->back_edge=e2.outrec->back_edge;
        if (e1.outrec->back_edge) e1.outrec->back_edge->outrec=e1.outrec;
    }
    e2.outrec->front_edge=nullptr; e2.outrec->back_edge=nullptr; e2.outrec->pts=nullptr;
    e2.outrec->owner = e1.outrec;
    e1.outrec=nullptr; e2.outrec=nullptr;
}

static void v_split(VActive& e, BIVec2 pt, VattiScratch& sc); // forward decl

static VOutPt* v_add_local_max_poly(VActive& e1, VActive& e2, BIVec2 pt, VattiScratch& sc) {
    if (v_is_joined(e1)) v_split(e1, pt, sc);
    if (v_is_joined(e2)) v_split(e2, pt, sc);

    // Reset joins more carefully
    // (simplified: no open path handling)
    if (v_is_front(e1) == v_is_front(e2)) { sc.succeeded = false; return nullptr; }

    VOutPt* result = v_add_outpt(e1, pt, sc);
    if (e1.outrec == e2.outrec) {
        VOutRec& outrec = *e1.outrec; outrec.pts = result;
        v_uncouple(e1); result = outrec.pts;
    } else if (e1.outrec->idx < e2.outrec->idx) v_join_outrec_paths(e1, e2);
    else v_join_outrec_paths(e2, e1);
    return result;
}

// ── Split + CheckJoin ────────────────────────────────────────────────────

static void v_split(VActive& e, BIVec2 pt, VattiScratch& sc) {
    if (e.join_with == 2/*Right*/) {
        e.join_with = 0; e.next_in_ael->join_with = 0;
        v_add_local_min_poly(e, *e.next_in_ael, pt, sc, true);
    } else {
        e.join_with = 0; e.prev_in_ael->join_with = 0;
        v_add_local_min_poly(*e.prev_in_ael, e, pt, sc, true);
    }
}

static void v_check_join_left(VActive& e, BIVec2 pt, VattiScratch& sc, bool check_curr_x = false) {
    VActive* prev = e.prev_in_ael;
    if (!prev || !v_is_hot(e) || !v_is_hot(*prev) || v_is_horizontal(e) || v_is_horizontal(*prev)) return;
    if ((pt.y < e.top.y+2 || pt.y < prev->top.y+2) && (e.bot.y > pt.y || prev->bot.y > pt.y)) return;
    if (check_curr_x) { if (v_perpendic_dist_sq(pt, prev->bot, prev->top) > 0.25) return; }
    else if (e.curr_x != prev->curr_x) return;
    if (!v_is_collinear(e.top, pt, prev->top)) return;
    if (e.outrec->idx == prev->outrec->idx) v_add_local_max_poly(*prev, e, pt, sc);
    else if (e.outrec->idx < prev->outrec->idx) v_join_outrec_paths(e, *prev);
    else v_join_outrec_paths(*prev, e);
    prev->join_with = 2; e.join_with = 1;
}

static void v_check_join_right(VActive& e, BIVec2 pt, VattiScratch& sc, bool check_curr_x = false) {
    VActive* next = e.next_in_ael;
    if (!next || !v_is_hot(e) || !v_is_hot(*next) || v_is_horizontal(e) || v_is_horizontal(*next)) return;
    if ((pt.y < e.top.y+2 || pt.y < next->top.y+2) && (e.bot.y > pt.y || next->bot.y > pt.y)) return;
    if (check_curr_x) { if (v_perpendic_dist_sq(pt, next->bot, next->top) > 0.35) return; }
    else if (e.curr_x != next->curr_x) return;
    if (!v_is_collinear(e.top, pt, next->top)) return;
    if (e.outrec->idx == next->outrec->idx) v_add_local_max_poly(e, *next, pt, sc);
    else if (e.outrec->idx < next->outrec->idx) v_join_outrec_paths(e, *next);
    else v_join_outrec_paths(*next, e);
    e.join_with = 2; next->join_with = 1;
}

// ── IntersectEdges ───────────────────────────────────────────────────────

static void v_intersect_edges(VActive& e1, VActive& e2, BIVec2 pt, VattiScratch& sc, int cliptype) {
    // Closed paths only
    if (v_is_joined(e1)) v_split(e1, pt, sc);
    if (v_is_joined(e2)) v_split(e2, pt, sc);

    int old_e1_wc, old_e2_wc;
    if (v_polytype(e1) == v_polytype(e2)) {
        // NonZero fill
        if (e1.wind_cnt + e2.wind_dx == 0) e1.wind_cnt = -e1.wind_cnt; else e1.wind_cnt += e2.wind_dx;
        if (e2.wind_cnt - e1.wind_dx == 0) e2.wind_cnt = -e2.wind_cnt; else e2.wind_cnt -= e1.wind_dx;
    } else {
        e1.wind_cnt2 += e2.wind_dx;
        e2.wind_cnt2 -= e1.wind_dx;
    }
    old_e1_wc = std::abs(e1.wind_cnt);
    old_e2_wc = std::abs(e2.wind_cnt);

    bool e1_in01 = old_e1_wc==0||old_e1_wc==1;
    bool e2_in01 = old_e2_wc==0||old_e2_wc==1;
    if ((!v_is_hot(e1) && !e1_in01) || (!v_is_hot(e2) && !e2_in01)) return;

    if (v_is_hot(e1) && v_is_hot(e2)) {
        if ((old_e1_wc!=0&&old_e1_wc!=1) || (old_e2_wc!=0&&old_e2_wc!=1) || (v_polytype(e1)!=v_polytype(e2))) {
            v_add_local_max_poly(e1, e2, pt, sc);
        } else if (v_is_front(e1) || e1.outrec==e2.outrec) {
            v_add_local_max_poly(e1, e2, pt, sc);
            v_add_local_min_poly(e1, e2, pt, sc, false);
        } else {
            v_add_outpt(e1, pt, sc); v_add_outpt(e2, pt, sc);
            v_swap_outrecs(e1, e2);
        }
    } else if (v_is_hot(e1)) { v_add_outpt(e1, pt, sc); v_swap_outrecs(e1, e2); }
    else if (v_is_hot(e2)) { v_add_outpt(e2, pt, sc); v_swap_outrecs(e1, e2); }
    else {
        int64_t e1Wc2 = std::abs(e1.wind_cnt2), e2Wc2 = std::abs(e2.wind_cnt2);
        if (!v_same_polytype(e1, e2)) { v_add_local_min_poly(e1, e2, pt, sc, false); }
        else if (old_e1_wc==1 && old_e2_wc==1) {
            if (cliptype==0) { if (e1Wc2>0 && e2Wc2>0) v_add_local_min_poly(e1, e2, pt, sc, false); }
            else if (cliptype==1) { if (e1Wc2<=0 && e2Wc2<=0) v_add_local_min_poly(e1, e2, pt, sc, false); }
            else { // difference
                if ((v_polytype(e1)==1/*clip*/ && e1Wc2>0 && e2Wc2>0) ||
                    (v_polytype(e1)==0/*subj*/ && e1Wc2<=0 && e2Wc2<=0))
                    v_add_local_min_poly(e1, e2, pt, sc, false);
            }
        }
    }
}

// ── Horizontal edge processing ───────────────────────────────────────────

static void v_add_trial_horz_join(VattiScratch& sc, VOutPt* op) { sc.horz_seg_list.push_back({op}); }

inline VOutPt* v_get_last_op(const VActive& e) {
    VOutPt* r = e.outrec->pts;
    if (&e != e.outrec->front_edge) r = r->next;
    return r;
}

inline void v_update_edge_into_ael(VattiScratch& sc, VActive* e, int cliptype) {
    e->bot = e->top; e->vertex_top = v_next_vertex(*e); e->top = e->vertex_top->pt;
    e->curr_x = e->bot.x; v_set_dx(*e);
    if (v_is_joined(*e)) v_split(*e, e->bot, sc);
    if (v_is_horizontal(*e)) {
        // Trim 180-deg spikes
        BIVec2 pt = v_next_vertex(*e)->pt;
        while (pt.y == e->top.y) {
            if ((pt.x < e->top.x) != (e->bot.x < e->top.x)) break;
            e->vertex_top = v_next_vertex(*e); e->top = pt;
            if (v_is_maxima(*e)) break;
            pt = v_next_vertex(*e)->pt;
        }
        v_set_dx(*e);
        return;
    }
    v_insert_scanline(sc, e->top.y);
    v_check_join_left(*e, e->bot, sc);
    v_check_join_right(*e, e->bot, sc, true);
}

static bool v_reset_horz_dir(const VActive& horz, const VVertex* max_v, int64_t& left, int64_t& right) {
    if (horz.bot.x == horz.top.x) { left=horz.curr_x; right=horz.curr_x;
        VActive* e=horz.next_in_ael; while(e && e->vertex_top!=max_v) e=e->next_in_ael; return e!=nullptr; }
    if (horz.curr_x < horz.top.x) { left=horz.curr_x; right=horz.top.x; return true; }
    left=horz.top.x; right=horz.curr_x; return false;
}

static void v_do_horizontal(VActive& horz, VattiScratch& sc, int cliptype) {
    int64_t y = horz.bot.y;
    VVertex* vertex_max = v_get_curr_y_maxima(horz);
    int64_t horz_left, horz_right;
    bool is_ltr = v_reset_horz_dir(horz, vertex_max, horz_left, horz_right);
    if (v_is_hot(horz)) { VOutPt* op = v_add_outpt(horz, {horz.curr_x, y}, sc); v_add_trial_horz_join(sc, op); }

    while (true) {
        VActive* e = is_ltr ? horz.next_in_ael : horz.prev_in_ael;
        while (e) {
            if (e->vertex_top == vertex_max) {
                if (v_is_hot(horz) && v_is_joined(*e)) v_split(*e, e->top, sc);
                if (v_is_hot(horz)) {
                    while (horz.vertex_top != vertex_max) { v_add_outpt(horz, horz.top, sc); v_update_edge_into_ael(sc, &horz, cliptype); }
                    if (is_ltr) v_add_local_max_poly(horz, *e, horz.top, sc);
                    else v_add_local_max_poly(*e, horz, horz.top, sc);
                }
                v_delete_from_ael(sc, *e); v_delete_from_ael(sc, horz); return;
            }
            if (vertex_max != horz.vertex_top) {
                if ((is_ltr && e->curr_x > horz_right) || (!is_ltr && e->curr_x < horz_left)) break;
                if (e->curr_x == horz.top.x && !v_is_horizontal(*e)) {
                    BIVec2 pt2 = v_next_vertex(horz)->pt;
                    if (is_ltr) { if (v_top_x(*e, pt2.y) >= pt2.x) break; }
                    else { if (v_top_x(*e, pt2.y) <= pt2.x) break; }
                }
            }
            BIVec2 pt = {e->curr_x, horz.bot.y};
            if (is_ltr) {
                v_intersect_edges(horz, *e, pt, sc, cliptype);
                v_swap_positions_in_ael(sc, horz, *e);
                v_check_join_left(*e, pt, sc);
                horz.curr_x = e->curr_x; e = horz.next_in_ael;
            } else {
                v_intersect_edges(*e, horz, pt, sc, cliptype);
                v_swap_positions_in_ael(sc, *e, horz);
                v_check_join_right(*e, pt, sc);
                horz.curr_x = e->curr_x; e = horz.prev_in_ael;
            }
            if (horz.outrec) v_add_trial_horz_join(sc, v_get_last_op(horz));
        }
        if (v_next_vertex(horz)->pt.y != horz.top.y) break;
        if (v_is_hot(horz)) v_add_outpt(horz, horz.top, sc);
        v_update_edge_into_ael(sc, &horz, cliptype);
        is_ltr = v_reset_horz_dir(horz, vertex_max, horz_left, horz_right);
    }
    if (v_is_hot(horz)) { VOutPt* op = v_add_outpt(horz, horz.top, sc); v_add_trial_horz_join(sc, op); }
    v_update_edge_into_ael(sc, &horz, cliptype);
}

// ── HorzSeg/HorzJoin handling ────────────────────────────────────────────

static VOutPt* v_dup_outpt(VOutPt* op, bool after, VattiScratch& sc) {
    VOutPt* r = sc.new_outpt(op->pt, op->outrec);
    if (after) { r->next=op->next; r->next->prev=r; r->prev=op; op->next=r; }
    else { r->prev=op->prev; r->prev->next=r; r->next=op; op->prev=r; }
    return r;
}

static void v_convert_horz_segs_to_joins(VattiScratch& sc) {
    // Update horz segments to find proper left/right ops
    int valid = 0;
    for (auto& hs : sc.horz_seg_list) {
        VOutPt* op = hs.left_op;
        VOutRec* outrec = op->outrec; while (outrec && !outrec->pts) outrec = outrec->owner;
        if (!outrec) { hs.right_op = nullptr; continue; }
        bool has_edges = outrec->front_edge != nullptr;
        int64_t cy = op->pt.y;
        VOutPt* opP = op, *opN = op;
        if (has_edges) {
            VOutPt* opA=outrec->pts, *opZ=opA->next;
            while (opP!=opZ && opP->prev->pt.y==cy) opP=opP->prev;
            while (opN!=opA && opN->next->pt.y==cy) opN=opN->next;
        } else {
            while (opP->prev!=opN && opP->prev->pt.y==cy) opP=opP->prev;
            while (opN->next!=opP && opN->next->pt.y==cy) opN=opN->next;
        }
        if (opP->pt.x == opN->pt.x) { hs.right_op=nullptr; continue; }
        if (opP->pt.x < opN->pt.x) { hs.left_op=opP; hs.right_op=opN; hs.left_to_right=true; }
        else { hs.left_op=opN; hs.right_op=opP; hs.left_to_right=false; }
        if (hs.left_op->horz) { hs.right_op=nullptr; continue; }
        hs.left_op->horz = reinterpret_cast<VHorzSeg*>(1); // mark used
        valid++;
    }
    if (valid < 2) return;
    std::stable_sort(sc.horz_seg_list.begin(), sc.horz_seg_list.end(),
        [](const VHorzSeg& a, const VHorzSeg& b) { if (!a.right_op||!b.right_op) return (a.right_op!=nullptr); return b.left_op->pt.x > a.left_op->pt.x; });
    int j = valid;
    for (int i=0; i<j-1; i++) {
        auto& hs1 = sc.horz_seg_list[i];
        for (int k=i+1; k<j; k++) {
            auto& hs2 = sc.horz_seg_list[k];
            if (hs2.left_op->pt.x >= hs1.right_op->pt.x || hs2.left_to_right==hs1.left_to_right || hs2.right_op->pt.x <= hs1.left_op->pt.x) continue;
            int64_t cy = hs1.left_op->pt.y;
            if (hs1.left_to_right) {
                while (hs1.left_op->next->pt.y==cy && hs1.left_op->next->pt.x<=hs2.left_op->pt.x) hs1.left_op=hs1.left_op->next;
                while (hs2.left_op->prev->pt.y==cy && hs2.left_op->prev->pt.x<=hs1.left_op->pt.x) hs2.left_op=hs2.left_op->prev;
                sc.horz_join_list.push_back({v_dup_outpt(hs1.left_op, true, sc), v_dup_outpt(hs2.left_op, false, sc)});
            } else {
                while (hs1.left_op->prev->pt.y==cy && hs1.left_op->prev->pt.x<=hs2.left_op->pt.x) hs1.left_op=hs1.left_op->prev;
                while (hs2.left_op->next->pt.y==cy && hs2.left_op->next->pt.x<=hs1.left_op->pt.x) hs2.left_op=hs2.left_op->next;
                sc.horz_join_list.push_back({v_dup_outpt(hs2.left_op, true, sc), v_dup_outpt(hs1.left_op, false, sc)});
            }
        }
    }
}

static void v_fix_outrec_pts(VOutRec* outrec) { VOutPt* op=outrec->pts; do { op->outrec=outrec; op=op->next; } while(op!=outrec->pts); }

static void v_process_horz_joins(VattiScratch& sc) {
    for (auto& j : sc.horz_join_list) {
        VOutRec* or1 = j.op1->outrec; while(or1 && !or1->pts) or1=or1->owner;
        VOutRec* or2 = j.op2->outrec; while(or2 && !or2->pts) or2=or2->owner;
        VOutPt* op1b=j.op1->next, *op2b=j.op2->prev;
        j.op1->next=j.op2; j.op2->prev=j.op1; op1b->prev=op2b; op2b->next=op1b;
        if (or1==or2) {
            or2 = sc.new_outrec(); or2->pts = op1b; v_fix_outrec_pts(or2);
            if (or1->pts->outrec==or2) { or1->pts=j.op1; or1->pts->outrec=or1; }
            or2->owner = or1;
        } else { or2->pts=nullptr; or2->owner=or1; }
    }
}

// ── Intersection detection (merge sort) ──────────────────────────────────

inline void v_adjust_curr_x_copy_to_sel(VattiScratch& sc, int64_t top_y) {
    VActive* e = sc.actives; sc.sel = e;
    while (e) {
        e->prev_in_sel=e->prev_in_ael; e->next_in_sel=e->next_in_ael; e->jump=e->next_in_sel;
        if (e->join_with==1) e->curr_x = e->prev_in_ael->curr_x;
        else e->curr_x = v_top_x(*e, top_y);
        e = e->next_in_ael;
    }
}

inline VActive* v_extract_from_sel(VActive* ae) {
    VActive* res = ae->next_in_sel; if (res) res->prev_in_sel = ae->prev_in_sel;
    ae->prev_in_sel->next_in_sel = res; return res;
}
inline void v_insert1_before2_in_sel(VActive* a1, VActive* a2) {
    a1->prev_in_sel=a2->prev_in_sel; if(a1->prev_in_sel) a1->prev_in_sel->next_in_sel=a1;
    a1->next_in_sel=a2; a2->prev_in_sel=a1;
}

static void v_add_new_isect_node(VattiScratch& sc, VActive& e1, VActive& e2, int64_t top_y) {
    BIVec2 ip;
    if (!v_get_seg_isect_pt(e1.bot, e1.top, e2.bot, e2.top, ip)) ip = {e1.curr_x, top_y};
    if (ip.y > sc.bot_y || ip.y < top_y) {
        double ad1=std::fabs(e1.dx), ad2=std::fabs(e2.dx);
        if (ad1>100 && ad2>100) ip = (ad1>ad2) ? v_closest_pt_on_seg(ip,e1.bot,e1.top) : v_closest_pt_on_seg(ip,e2.bot,e2.top);
        else if (ad1>100) ip = v_closest_pt_on_seg(ip,e1.bot,e1.top);
        else if (ad2>100) ip = v_closest_pt_on_seg(ip,e2.bot,e2.top);
        else { if (ip.y<top_y) ip.y=top_y; else ip.y=sc.bot_y; ip.x = (ad1<ad2) ? v_top_x(e1,ip.y) : v_top_x(e2,ip.y); }
    }
    sc.intersect_nodes.push_back({ip, &e1, &e2});
}

static bool v_build_intersect_list(VattiScratch& sc, int64_t top_y) {
    if (!sc.actives || !sc.actives->next_in_ael) return false;
    v_adjust_curr_x_copy_to_sel(sc, top_y);
    VActive* left = sc.sel;
    while (left && left->jump) {
        VActive* prev_base = nullptr;
        while (left && left->jump) {
            VActive* curr_base = left;
            VActive* right = left->jump;
            VActive* l_end = right;
            VActive* r_end = right->jump;
            left->jump = r_end;
            while (left != l_end && right != r_end) {
                if (right->curr_x < left->curr_x) {
                    VActive* tmp = right->prev_in_sel;
                    for (;;) { v_add_new_isect_node(sc, *tmp, *right, top_y); if (tmp==left) break; tmp=tmp->prev_in_sel; }
                    tmp = right; right = v_extract_from_sel(tmp); l_end = right;
                    v_insert1_before2_in_sel(tmp, left);
                    if (left==curr_base) { curr_base=tmp; curr_base->jump=r_end; if(!prev_base) sc.sel=curr_base; else prev_base->jump=curr_base; }
                } else left = left->next_in_sel;
            }
            prev_base = curr_base; left = r_end;
        }
        left = sc.sel;
    }
    return !sc.intersect_nodes.empty();
}

static void v_process_intersect_list(VattiScratch& sc, int cliptype) {
    std::sort(sc.intersect_nodes.begin(), sc.intersect_nodes.end(),
        [](const VIntersectNode& a, const VIntersectNode& b) { return (a.pt.y==b.pt.y) ? a.pt.x<b.pt.x : a.pt.y>b.pt.y; });
    for (size_t i = 0; i < sc.intersect_nodes.size(); i++) {
        auto& node = sc.intersect_nodes[i];
        if (!(node.edge1->next_in_ael==node.edge2 || node.edge1->prev_in_ael==node.edge2)) {
            for (size_t j=i+1; j<sc.intersect_nodes.size(); j++) {
                if (sc.intersect_nodes[j].edge1->next_in_ael==sc.intersect_nodes[j].edge2 || sc.intersect_nodes[j].edge1->prev_in_ael==sc.intersect_nodes[j].edge2)
                    { std::swap(sc.intersect_nodes[i], sc.intersect_nodes[j]); node = sc.intersect_nodes[i]; break; }
            }
        }
        v_intersect_edges(*node.edge1, *node.edge2, node.pt, sc, cliptype);
        v_swap_positions_in_ael(sc, *node.edge1, *node.edge2);
        node.edge1->curr_x = node.pt.x; node.edge2->curr_x = node.pt.x;
        v_check_join_left(*node.edge2, node.pt, sc, true);
        v_check_join_right(*node.edge1, node.pt, sc, true);
    }
}

// ── InsertLocalMinimaIntoAEL ─────────────────────────────────────────────

static void v_insert_local_minima_into_ael(VattiScratch& sc, int64_t bot_y, int cliptype) {
    VLocalMinima* lm;
    while (v_pop_locmin(sc, bot_y, lm)) {
        VActive* lb = sc.new_active();
        lb->bot = lm->vertex->pt; lb->curr_x = lb->bot.x; lb->wind_dx = -1;
        lb->vertex_top = lm->vertex->prev; lb->top = lb->vertex_top->pt;
        lb->local_min = lm; v_set_dx(*lb);

        VActive* rb = sc.new_active();
        rb->bot = lm->vertex->pt; rb->curr_x = rb->bot.x; rb->wind_dx = 1;
        rb->vertex_top = lm->vertex->next; rb->top = rb->vertex_top->pt;
        rb->local_min = lm; v_set_dx(*rb);

        if (v_is_horizontal(*lb)) {
            if (lb->dx == -std::numeric_limits<double>::max()) std::swap(lb, rb);
        } else if (v_is_horizontal(*rb)) {
            if (rb->dx == std::numeric_limits<double>::max()) std::swap(lb, rb);
        } else if (lb->dx < rb->dx) std::swap(lb, rb);

        lb->is_left_bound = true;
        v_insert_left_edge(sc, *lb);
        v_set_wind_count(sc, *lb);
        bool contributing = v_is_contributing(*lb, cliptype);

        rb->is_left_bound = false;
        rb->wind_cnt = lb->wind_cnt; rb->wind_cnt2 = lb->wind_cnt2;
        v_insert_right_edge(*lb, *rb);
        if (contributing) {
            v_add_local_min_poly(*lb, *rb, lb->bot, sc, true);
            if (!v_is_horizontal(*lb)) v_check_join_left(*lb, lb->bot, sc);
        }
        while (rb->next_in_ael && v_is_valid_ael_order(*rb->next_in_ael, *rb)) {
            v_intersect_edges(*rb, *rb->next_in_ael, rb->bot, sc, cliptype);
            v_swap_positions_in_ael(sc, *rb, *rb->next_in_ael);
        }
        if (v_is_horizontal(*rb)) v_push_horz(sc, *rb);
        else { v_check_join_right(*rb, rb->bot, sc); v_insert_scanline(sc, rb->top.y); }
        if (v_is_horizontal(*lb)) v_push_horz(sc, *lb);
        else v_insert_scanline(sc, lb->top.y);
    }
}

// ── DoMaxima ─────────────────────────────────────────────────────────────

static VActive* v_do_maxima(VActive& e, VattiScratch& sc, int cliptype) {
    VActive* prev_e = e.prev_in_ael;
    VActive* next_e = e.next_in_ael;
    VActive* max_pair = v_get_maxima_pair(e);
    if (!max_pair) return next_e;
    if (v_is_joined(e)) v_split(e, e.top, sc);
    if (v_is_joined(*max_pair)) v_split(*max_pair, max_pair->top, sc);
    while (next_e != max_pair) {
        v_intersect_edges(e, *next_e, e.top, sc, cliptype);
        v_swap_positions_in_ael(sc, e, *next_e);
        next_e = e.next_in_ael;
    }
    if (v_is_hot(e)) v_add_local_max_poly(e, *max_pair, e.top, sc);
    v_delete_from_ael(sc, *max_pair); v_delete_from_ael(sc, e);
    return prev_e ? prev_e->next_in_ael : sc.actives;
}

// ── DoTopOfScanbeam ──────────────────────────────────────────────────────

static void v_do_top_of_scanbeam(VattiScratch& sc, int64_t y, int cliptype) {
    sc.sel = nullptr;
    VActive* e = sc.actives;
    while (e) {
        if (e->top.y == y) {
            e->curr_x = e->top.x;
            if (v_is_maxima(*e)) { e = v_do_maxima(*e, sc, cliptype); continue; }
            if (v_is_hot(*e)) v_add_outpt(*e, e->top, sc);
            v_update_edge_into_ael(sc, e, cliptype);
            if (v_is_horizontal(*e)) v_push_horz(sc, *e);
        } else e->curr_x = v_top_x(*e, y);
        e = e->next_in_ael;
    }
}

// ── CleanCollinear + FixSelfIntersects ───────────────────────────────────

static VOutPt* v_dispose_outpt(VOutPt* op) {
    VOutPt* r = op->next; op->prev->next=op->next; op->next->prev=op->prev; return r;
}

static void v_do_split_op(VattiScratch& sc, VOutRec* outrec, VOutPt* splitOp) {
    VOutPt* prevOp = splitOp->prev; VOutPt* nnOp = splitOp->next->next;
    outrec->pts = prevOp;
    BIVec2 ip;
    v_get_seg_isect_pt(prevOp->pt, splitOp->pt, splitOp->next->pt, nnOp->pt, ip);
    double area1 = v_area_outpt(outrec->pts);
    if (std::fabs(area1) < 2) { outrec->pts=nullptr; return; }
    double area2 = v_area_tri(ip, splitOp->pt, splitOp->next->pt);
    double absA2 = std::fabs(area2);
    if (ip==prevOp->pt || ip==nnOp->pt) { nnOp->prev=prevOp; prevOp->next=nnOp; }
    else {
        VOutPt* nop = sc.new_outpt(ip, prevOp->outrec);
        nop->prev=prevOp; nop->next=nnOp; nnOp->prev=nop; prevOp->next=nop;
    }
    if (absA2>=1 && (absA2>std::fabs(area1) || (area2>0)==(area1>0))) {
        VOutRec* nr = sc.new_outrec(); nr->owner=outrec->owner;
        splitOp->outrec=nr; splitOp->next->outrec=nr;
        VOutPt* nop = sc.new_outpt(ip, nr);
        nop->prev=splitOp->next; nop->next=splitOp; nr->pts=nop;
        splitOp->prev=nop; splitOp->next->next=nop;
    }
}

static void v_fix_self_intersects(VattiScratch& sc, VOutRec* outrec) {
    VOutPt* op2 = outrec->pts;
    for (;;) {
        if (op2->prev == op2->next->next) break;
        if (v_segs_intersect(op2->prev->pt, op2->pt, op2->next->pt, op2->next->next->pt)) {
            if (op2==outrec->pts || op2->next==outrec->pts) outrec->pts=outrec->pts->prev;
            v_do_split_op(sc, outrec, op2);
            if (!outrec->pts) break;
            op2 = outrec->pts; continue;
        }
        op2 = op2->next;
        if (op2 == outrec->pts) break;
    }
}

static void v_clean_collinear(VattiScratch& sc, VOutRec* outrec) {
    while (outrec && !outrec->pts) outrec=outrec->owner;
    if (!outrec) return;
    if (!v_valid_closed(outrec->pts)) { outrec->pts=nullptr; return; }
    VOutPt* startOp = outrec->pts, *op2 = startOp;
    for (;;) {
        if (v_is_collinear(op2->prev->pt, op2->pt, op2->next->pt) &&
            (op2->pt==op2->prev->pt || op2->pt==op2->next->pt || v_dot_product(op2->prev->pt, op2->pt, op2->next->pt)<0)) {
            if (op2==outrec->pts) outrec->pts=op2->prev;
            op2 = v_dispose_outpt(op2);
            if (!v_valid_closed(op2)) { outrec->pts=nullptr; return; }
            startOp = op2; continue;
        }
        op2 = op2->next;
        if (op2==startOp) break;
    }
    v_fix_self_intersects(sc, outrec);
}

// ── ExecuteInternal ──────────────────────────────────────────────────────

static bool v_execute_internal(VattiScratch& sc, int cliptype) {
    std::stable_sort(sc.locmin_list.begin(), sc.locmin_list.end(),
        [](const VLocalMinima& a, const VLocalMinima& b) {
            if (b.vertex->pt.y != a.vertex->pt.y) return b.vertex->pt.y < a.vertex->pt.y;
            return b.vertex->pt.x > a.vertex->pt.x;
        });
    for (auto& lm : sc.locmin_list) v_insert_scanline(sc, lm.vertex->pt.y);
    sc.locmin_idx = 0;

    int64_t y;
    if (!v_pop_scanline(sc, y)) return true;
    while (sc.succeeded) {
        v_insert_local_minima_into_ael(sc, y, cliptype);
        VActive* e;
        while (v_pop_horz(sc, e)) v_do_horizontal(*e, sc, cliptype);
        if (!sc.horz_seg_list.empty()) { v_convert_horz_segs_to_joins(sc); sc.horz_seg_list.clear(); }
        sc.bot_y = y;
        if (!v_pop_scanline(sc, y)) break;
        if (sc.succeeded && v_build_intersect_list(sc, y)) { v_process_intersect_list(sc, cliptype); sc.intersect_nodes.clear(); }
        v_do_top_of_scanbeam(sc, y, cliptype);
        while (v_pop_horz(sc, e)) v_do_horizontal(*e, sc, cliptype);
    }
    if (sc.succeeded) v_process_horz_joins(sc);
    return sc.succeeded;
}

// ── Build output paths from OutRec list ──────────────────────────────────

static bool v_build_path(VOutPt* op, std::vector<BIVec2>& path) {
    if (!op || op->next==op || op->next==op->prev) return false;
    path.clear();
    BIVec2 last = op->next->pt;
    VOutPt* op2 = op->next->next;
    path.push_back(last);
    while (op2 != op->next) { if (op2->pt != last) { last=op2->pt; path.push_back(last); } op2=op2->next; }
    return path.size() >= 3 && !v_very_small_tri(*op);
}

} // anonymous namespace

std::vector<Polyline> session_cpp::BooleanPolyline::compute(const Polyline& a, const Polyline& b, int clip_type) {
    const double* ca = a._coords.data();
    const double* cb = b._coords.data();
    int na = (int)(a._coords.size() / 3);
    int nb = (int)(b._coords.size() / 3);

    // Strip closing duplicate
    if (na>=2) { double dx=ca[(na-1)*3]-ca[0],dy=ca[(na-1)*3+1]-ca[1]; if(dx*dx+dy*dy<1e-20) --na; }
    if (nb>=2) { double dx=cb[(nb-1)*3]-cb[0],dy=cb[(nb-1)*3+1]-cb[1]; if(dx*dx+dy*dy<1e-20) --nb; }
    if (na < 3 || nb < 3) return {};

    constexpr double BOOL_SCALE = 1e9;
    constexpr double BOOL_INV_SCALE = 1e-9;
    VattiScratch& sc = vtls; sc.reset();
    int total = na + nb;
    sc.vtx_pool.ensure(total + 4);
    sc.act_pool.ensure(total * 2 + 4);
    sc.opt_pool.ensure(total * 4);
    sc.orc_pool.ensure(total);
    sc.locmin_list.reserve(total);
    sc.outrec_list.reserve(total);
    sc.scanline_list.buf.reserve(total * 2);

    // Small polygons: use intermediate arrays for containment check
    if ((int64_t)na * nb <= 400) {
        auto& va = sc.va; va.resize(na);
        auto& vb = sc.vb; vb.resize(nb);
        int64_t aMinX,aMaxX,aMinY,aMaxY,bMinX,bMaxX,bMinY,bMaxY;
#if VATTI_HAS_SSE2
        const __m128d sv = _mm_set1_pd(BOOL_SCALE);
        auto cvt = [&](const double* p) { return v_cvt_to_i64(p, sv); };
#else
        auto cvt = [](const double* p) { return BIVec2{VATTI_NEARBYINT(p[0]*BOOL_SCALE), VATTI_NEARBYINT(p[1]*BOOL_SCALE)}; };
#endif
        va[0]=cvt(ca); aMinX=aMaxX=va[0].x; aMinY=aMaxY=va[0].y;
        for(int i=1;i<na;i++) { va[i]=cvt(ca+i*3); int64_t x=va[i].x,y=va[i].y;
            if(x<aMinX) aMinX=x; else if(x>aMaxX) aMaxX=x; if(y<aMinY) aMinY=y; else if(y>aMaxY) aMaxY=y; }
        vb[0]=cvt(cb); bMinX=bMaxX=vb[0].x; bMinY=bMaxY=vb[0].y;
        for(int i=1;i<nb;i++) { vb[i]=cvt(cb+i*3); int64_t x=vb[i].x,y=vb[i].y;
            if(x<bMinX) bMinX=x; else if(x>bMaxX) bMaxX=x; if(y<bMinY) bMinY=y; else if(y>bMaxY) bMaxY=y; }
        // AABB disjoint
        if (aMaxX<bMinX||bMaxX<aMinX||aMaxY<bMinY||bMaxY<aMinY) {
            bool a_in_b=pip_i(va[0],vb), b_in_a=pip_i(vb[0],va);
            if(clip_type==0){if(a_in_b)return{a};if(b_in_a)return{b};return{};}
            if(clip_type==1){if(a_in_b)return{b};if(b_in_a)return{a};return{a,b};}
            if(a_in_b)return{};if(b_in_a)return{a};return{a};
        }
        // Containment: no crossings → pure containment
        bool any_cross=false;
        for(int i=0;i<na&&!any_cross;i++) {
            BIVec2 a1=va[i],a2=va[(i+1)%na];
            int64_t axmin=std::min(a1.x,a2.x),axmax=std::max(a1.x,a2.x),aymin=std::min(a1.y,a2.y),aymax=std::max(a1.y,a2.y);
            for(int j=0;j<nb&&!any_cross;j++) { BIVec2 b1=vb[j],b2=vb[(j+1)%nb];
                if(std::max(b1.x,b2.x)<axmin||std::min(b1.x,b2.x)>axmax||std::max(b1.y,b2.y)<aymin||std::min(b1.y,b2.y)>aymax) continue;
                any_cross=v_segs_intersect(a1,a2,b1,b2); } }
        if(!any_cross) {
            bool a_in_b=pip_i(va[0],vb),b_in_a=pip_i(vb[0],va);
            if(clip_type==0){if(a_in_b)return{a};if(b_in_a)return{b};return{};}
            if(clip_type==1){if(a_in_b)return{b};if(b_in_a)return{a};return{a,b};}
            if(a_in_b)return{};if(b_in_a)return{a};return{a};
        }
        v_add_path(va, na, 0, sc);
        v_add_path(vb, nb, 1, sc);
    }
    else {
        // Large polygons: single pass double→VVertex, no intermediate arrays.
        int64_t aMinX,aMaxX,aMinY,aMaxY,bMinX,bMaxX,bMinY,bMaxY;
#if VATTI_HAS_SSE2
        const __m128d sv = _mm_set1_pd(BOOL_SCALE);
#else
        double sv = BOOL_SCALE;
#endif
        VVertex* va_head = v_add_path_from_doubles(ca, na, 0, sv, sc, aMinX, aMaxX, aMinY, aMaxY);
        VVertex* vb_head = v_add_path_from_doubles(cb, nb, 1, sv, sc, bMinX, bMaxX, bMinY, bMaxY);
        if (!va_head || !vb_head) return {};
        // AABB disjoint
        if (aMaxX<bMinX||bMaxX<aMinX||aMaxY<bMinY||bMaxY<aMinY) {
            bool a_in_b=pip_vertex(va_head->pt, vb_head), b_in_a=pip_vertex(vb_head->pt, va_head);
            if(clip_type==0){if(a_in_b)return{a};if(b_in_a)return{b};return{};}
            if(clip_type==1){if(a_in_b)return{b};if(b_in_a)return{a};return{a,b};}
            if(a_in_b)return{};if(b_in_a)return{a};return{a};
        }
    }
    double scale = BOOL_SCALE;
    if (!v_execute_internal(sc, clip_type)) return {};

    // Extract: OutPt → Polyline._coords
    // Thread-local buffer avoids malloc/free per call (capacity persists across iterations).
#if VATTI_HAS_SSE2
    const __m128d isv = _mm_set1_pd(BOOL_INV_SCALE);
#endif
    static thread_local std::vector<double> tl_coords;
    std::vector<Polyline> out;
    for (size_t i = 0; i < sc.outrec_list.size(); i++) {
        VOutRec* outrec = sc.outrec_list[i];
        if (!outrec->pts) continue;
        v_clean_collinear(sc, outrec);
        if (!outrec->pts) continue;
        VOutPt* op = outrec->pts;
        if (!op || op->next==op || op->next==op->prev || v_very_small_tri(*op)) continue;
        // Count OutPt nodes for exact reserve (pool nodes are contiguous → fast traversal)
        int nop = 0; { VOutPt* o = op; do { nop++; o = o->next; } while (o != op); }
        // Write to thread-local buffer (no malloc after first call)
        if (tl_coords.capacity() < (size_t)nop * 3) tl_coords.reserve(nop * 4);
        tl_coords.clear();
        double* dst;
        // Reserve exact in tl_coords then direct-write
        tl_coords.resize(nop * 3);
        dst = tl_coords.data();
        double* dst0 = dst;
        VOutPt* o = op->next; BIVec2 last = o->pt;
#if VATTI_HAS_SSE2
        _mm_storeu_pd(dst, _mm_mul_pd(_mm_set_pd(double(last.y), double(last.x)), isv));
#else
        dst[0] = last.x * BOOL_INV_SCALE; dst[1] = last.y * BOOL_INV_SCALE;
#endif
        dst[2] = 0.0; dst += 3;
        for (o=o->next; o!=op->next; o=o->next) {
            if (o->pt==last) continue; last=o->pt;
#if VATTI_HAS_SSE2
            _mm_storeu_pd(dst, _mm_mul_pd(_mm_set_pd(double(last.y), double(last.x)), isv));
#else
            dst[0] = last.x * BOOL_INV_SCALE; dst[1] = last.y * BOOL_INV_SCALE;
#endif
            dst[2] = 0.0; dst += 3;
        }
        int cnt = (int)((dst - dst0) / 3);
        if (cnt < 3) continue;
        Polyline result;
        result._coords.assign(dst0, dst0 + cnt * 3); // exact-size malloc + memcpy
        out.push_back(std::move(result));
    }
    return out;
}
