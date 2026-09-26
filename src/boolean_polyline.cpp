#include "boolean_polyline.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <limits>
#include <optional>
#if (defined(_MSC_VER) && (defined(_M_AMD64) || defined(_M_X64))) || (defined(__SSE2__))
#define VATTI_HAS_SSE2 1
#include <emmintrin.h>
#else
#define VATTI_HAS_SSE2 0
#endif

using namespace session_cpp;

namespace {

// ═══════════════════════════════════════════════════════════════════════════
// Sweep structures
// ═══════════════════════════════════════════════════════════════════════════
struct BIVec2 {
    int64_t x; // Scaled integer x.
    int64_t y; // Scaled integer y.
};

inline bool operator==(BIVec2 a, BIVec2 b) {
    return a.x == b.x && a.y == b.y;
}

inline bool operator!=(BIVec2 a, BIVec2 b) {
    return !(a == b);
}

#if VATTI_HAS_SSE2
using VScale = __m128d;

inline int64_t v_nearbyint(double a) {
    return _mm_cvtsd_si64(_mm_set_sd(a));
}

inline VScale v_scale(double s) {
    return _mm_set1_pd(s);
}

inline BIVec2 v_cvt_to_i64(const double* p, VScale scale) {

    __m128d xy = _mm_mul_pd(_mm_loadu_pd(p), scale);

    return {_mm_cvtsd_si64(xy), _mm_cvtsd_si64(_mm_unpackhi_pd(xy, xy))};
}

inline void v_cvt_to_dbl(double* dst, BIVec2 pt, VScale inv_scale) {

    _mm_storeu_pd(dst, _mm_mul_pd(_mm_set_pd(double(pt.y), double(pt.x)), inv_scale));
    dst[2] = 0.0;
}
#else
using VScale = double;

inline int64_t v_nearbyint(double a) {
    return static_cast<int64_t>(std::nearbyint(a));
}

inline VScale v_scale(double s) {
    return s;
}

inline BIVec2 v_cvt_to_i64(const double* p, VScale scale) {
    return {v_nearbyint(p[0] * scale), v_nearbyint(p[1] * scale)};
}

inline void v_cvt_to_dbl(double* dst, BIVec2 pt, VScale inv_scale) {

    dst[0] = pt.x * inv_scale;
    dst[1] = pt.y * inv_scale;
    dst[2] = 0.0;
}
#endif

enum : uint32_t {
    VF_None = 0, // Plain vertex.
    VF_LocalMax = 4, // Local maximum in y.
    VF_LocalMin = 8, // Local minimum in y.
};

struct VVertex {
    BIVec2 pt; // Scaled position.
    VVertex* next = nullptr; // Next vertex of the ring.
    VVertex* prev = nullptr; // Previous vertex of the ring.
    uint32_t flags = VF_None; // Local extremum flags.
};

struct VLocalMinima {
    VVertex* vertex; // Vertex at the local minimum.
    int8_t polytype; // 0 subject, 1 clip.
};

struct VHorzSeg;

struct VOutPt {
    BIVec2 pt; // Scaled output position.
    VOutPt* next = nullptr; // Next point of the output ring.
    VOutPt* prev = nullptr; // Previous point of the output ring.
    struct VOutRec* outrec; // Owning output ring.
    VHorzSeg* horz = nullptr; // Horizontal segment starting here.
};

struct VOutRec {
    size_t idx = 0; // Index in the output list.
    struct VActive* front_edge = nullptr; // Edge adding points to the front.
    struct VActive* back_edge = nullptr; // Edge adding points to the back.
    VOutPt* pts = nullptr; // Entry point of the ring.
    VOutRec* owner = nullptr; // Ring this one was merged into.
};

struct VActive {
    BIVec2 bot; // Bottom of the edge.
    BIVec2 top; // Top of the edge.
    int64_t curr_x = 0; // x at the current scanline.
    double dx = 0.0; // Inverse slope.
    int wind_dx = 1; // Winding direction, 1 or -1.
    int wind_cnt = 0; // Winding count of its own polytype.
    int wind_cnt2 = 0; // Winding count of the other polytype.
    VOutRec* outrec = nullptr; // Output ring the edge contributes to.
    VActive* prev_in_ael = nullptr; // Previous edge in the active edge list.
    VActive* next_in_ael = nullptr; // Next edge in the active edge list.
    VActive* prev_in_sel = nullptr; // Previous edge in the sorted edge list.
    VActive* next_in_sel = nullptr; // Next edge in the sorted edge list.
    VActive* jump = nullptr; // Merge sort run boundary.
    VVertex* vertex_top = nullptr; // Vertex at the top of the edge.
    VLocalMinima* local_min = nullptr; // Local minimum the bound starts from.
    bool is_left_bound = false; // Left or right bound of its minimum.
    int8_t join_with = 0; // 0 none, 1 left, 2 right.
};

struct VIntersectNode {
    BIVec2 pt; // Intersection point.
    VActive* edge1; // Left edge.
    VActive* edge2; // Right edge.
};

struct VHorzSeg {
    VOutPt* left_op; // Left end of the segment.
    VOutPt* right_op = nullptr; // Right end of the segment.
    bool left_to_right = true; // Direction of the output ring.
};

struct VHorzJoin {
    VOutPt* op1; // First point to join.
    VOutPt* op2; // Second point to join.
};

struct VertexPool {
    std::vector<VVertex> buf; // Contiguous vertices reused between calls.
    size_t count = 0; // Vertices in use.

    void ensure(size_t n) {

        if (buf.size() < n)
            buf.resize(n);
    }

    void reset() {
        count = 0;
    }
};

template <typename T> struct Pool {
    std::deque<T> buf; // Storage reused between calls, addresses stable on growth.
    size_t count = 0; // Items in use.

    void ensure(size_t n) {

        if (buf.size() < n)
            buf.resize(n);
    }

    T* alloc() {

        if (count >= buf.size())
            buf.resize(std::max<size_t>(buf.size() * 2, 256));

        return &buf[count++];
    }

    void reset() {
        count = 0;
    }
};

struct ScanlineHeap {
    std::vector<int64_t> buf; // Max heap storage.
    size_t sz = 0; // Items in the heap.

    void clear() {
        sz = 0;
    }

    bool empty() const {
        return sz == 0;
    }

    void push(int64_t y) {

        if (sz >= buf.size())
            buf.resize(std::max<size_t>(buf.size() * 2, 64));

        buf[sz] = y;
        size_t i = sz++;

        while (i > 0) {
            size_t p = (i - 1) / 2;

            if (buf[p] >= buf[i])
                break;

            std::swap(buf[p], buf[i]);
            i = p;
        }
    }

    int64_t top() const {
        return buf[0];
    }

    void pop() {

        buf[0] = buf[--sz];
        size_t i = 0;

        while (true) {
            size_t l = 2 * i + 1;
            size_t r = l + 1;
            size_t m = i;

            if (l < sz && buf[l] > buf[m])
                m = l;

            if (r < sz && buf[r] > buf[m])
                m = r;

            if (m == i)
                break;

            std::swap(buf[i], buf[m]);
            i = m;
        }
    }
};

struct VattiScratch {
    VertexPool vtx_pool; // Vertices of both inputs.
    Pool<VActive> act_pool; // Active edges.
    Pool<VOutPt> opt_pool; // Output points.
    Pool<VOutRec> orc_pool; // Output rings.
    std::vector<VLocalMinima> locmin_list; // Local minima of both inputs.
    std::vector<VIntersectNode> intersect_nodes; // Intersections of the current scanbeam.
    std::vector<VHorzSeg> horz_seg_list; // Horizontal output segments of the current scanline.
    std::vector<VHorzJoin> horz_join_list; // Pending horizontal joins.
    std::vector<VOutRec*> outrec_list; // Output rings in creation order.
    ScanlineHeap scanline_list; // Pending scanlines.
    std::vector<BIVec2> va; // Scaled points of a.
    std::vector<BIVec2> vb; // Scaled points of b.
    VActive* actives = nullptr; // Head of the active edge list.
    VActive* sel = nullptr; // Head of the sorted edge list.
    int64_t bot_y = 0; // Bottom of the current scanbeam.
    size_t locmin_idx = 0; // Next local minimum to insert.
    bool succeeded = true; // False once the sweep failed.

    void reset(size_t total) {

        vtx_pool.reset();
        act_pool.reset();
        opt_pool.reset();
        orc_pool.reset();
        locmin_list.clear();
        intersect_nodes.clear();
        horz_seg_list.clear();
        horz_join_list.clear();
        outrec_list.clear();
        scanline_list.clear();
        actives = nullptr;
        sel = nullptr;
        bot_y = 0;
        locmin_idx = 0;
        succeeded = true;

        vtx_pool.ensure(total + 4);
        act_pool.ensure(total * 2 + 4);
        opt_pool.ensure(total * 4);
        orc_pool.ensure(total);
        locmin_list.reserve(total);
        outrec_list.reserve(total);
        scanline_list.buf.reserve(total * 2);
    }

    VActive* new_active() {

        VActive* a = act_pool.alloc();
        *a = VActive{};

        return a;
    }

    VOutPt* new_outpt(BIVec2 pt, VOutRec* rec) {

        VOutPt* o = opt_pool.alloc();
        *o = VOutPt{};
        o->pt = pt;
        o->outrec = rec;
        o->next = o;
        o->prev = o;

        return o;
    }

    VOutRec* new_outrec() {

        VOutRec* r = orc_pool.alloc();
        *r = VOutRec{};
        r->idx = outrec_list.size();
        outrec_list.push_back(r);

        return r;
    }
};

static thread_local VattiScratch vtls;

// ═══════════════════════════════════════════════════════════════════════════
// Geometry helpers
// ═══════════════════════════════════════════════════════════════════════════
inline double v_get_dx(BIVec2 p1, BIVec2 p2) {

    double dy = double(p2.y - p1.y);

    if (dy != 0)
        return double(p2.x - p1.x) / dy;

    return (p2.x > p1.x) ? -std::numeric_limits<double>::max() : std::numeric_limits<double>::max();
}

inline int64_t v_top_x(const VActive& ae, int64_t y) {

    if (y == ae.top.y || ae.top.x == ae.bot.x)
        return ae.top.x;

    if (y == ae.bot.y)
        return ae.bot.x;

    return ae.bot.x + v_nearbyint(ae.dx * double(y - ae.bot.y));
}

inline bool v_is_horizontal(const VActive& e) {
    return e.top.y == e.bot.y;
}

inline bool v_is_hot(const VActive& e) {
    return e.outrec != nullptr;
}

inline bool v_is_maxima(const VVertex& v) {
    return (v.flags & VF_LocalMax) != 0;
}

inline bool v_is_maxima(const VActive& e) {
    return v_is_maxima(*e.vertex_top);
}

inline bool v_is_front(const VActive& e) {
    return &e == e.outrec->front_edge;
}

inline bool v_is_joined(const VActive& e) {
    return e.join_with != 0;
}

inline bool v_same_polytype(const VActive& a, const VActive& b) {
    return a.local_min->polytype == b.local_min->polytype;
}

inline int8_t v_polytype(const VActive& e) {
    return e.local_min->polytype;
}

inline void v_set_dx(VActive& e) {
    e.dx = v_get_dx(e.bot, e.top);
}

inline VVertex* v_next_vertex(const VActive& e) {
    return (e.wind_dx > 0) ? e.vertex_top->next : e.vertex_top->prev;
}

inline VVertex* v_prev_prev_vertex(const VActive& ae) {
    return (ae.wind_dx > 0) ? ae.vertex_top->prev->prev : ae.vertex_top->next->next;
}

inline double v_cross_product(BIVec2 p1, BIVec2 p2, BIVec2 p3) {
    return double(p2.x - p1.x) * double(p3.y - p2.y) - double(p2.y - p1.y) * double(p3.x - p2.x);
}

inline double v_dot_product(BIVec2 p1, BIVec2 p2, BIVec2 p3) {
    return double(p2.x - p1.x) * double(p3.x - p2.x) + double(p2.y - p1.y) * double(p3.y - p2.y);
}

#if (defined(__clang__) || defined(__GNUC__)) && UINTPTR_MAX >= UINT64_MAX
inline bool v_products_equal(int64_t a, int64_t b, int64_t c, int64_t d) {
    return static_cast<__int128_t>(a) * static_cast<__int128_t>(b) ==
        static_cast<__int128_t>(c) * static_cast<__int128_t>(d);
}
#else
inline void v_mul_u128(uint64_t a, uint64_t b, uint64_t& lo, uint64_t& hi) {

    uint64_t x1 = (a & 0xFFFFFFFF) * (b & 0xFFFFFFFF);
    uint64_t x2 = (a >> 32) * (b & 0xFFFFFFFF) + (x1 >> 32);
    uint64_t x3 = (a & 0xFFFFFFFF) * (b >> 32) + (x2 & 0xFFFFFFFF);
    lo = ((x3 & 0xFFFFFFFF) << 32) | (x1 & 0xFFFFFFFF);
    hi = (a >> 32) * (b >> 32) + (x2 >> 32) + (x3 >> 32);
}

inline int v_sign(int64_t x) {
    return (x > 0) - (x < 0);
}

inline bool v_products_equal(int64_t a, int64_t b, int64_t c, int64_t d) {

    uint64_t lo1;
    uint64_t hi1;
    uint64_t lo2;
    uint64_t hi2;
    v_mul_u128(std::abs(a), std::abs(b), lo1, hi1);
    v_mul_u128(std::abs(c), std::abs(d), lo2, hi2);

    return lo1 == lo2 && hi1 == hi2 && v_sign(a) * v_sign(b) == v_sign(c) * v_sign(d);
}
#endif

inline bool v_is_collinear(BIVec2 p1, BIVec2 shared, BIVec2 p2) {
    return v_products_equal(shared.x - p1.x, p2.y - shared.y, shared.y - p1.y, p2.x - shared.x);
}

inline double v_perpendic_dist_sq(BIVec2 pt, BIVec2 l1, BIVec2 l2) {

    double a = double(pt.x - l1.x);
    double b = double(pt.y - l1.y);
    double c = double(l2.x - l1.x);
    double d = double(l2.y - l1.y);

    if (c == 0 && d == 0)
        return 0;

    double e = a * d - c * b;

    return (e * e) / (c * c + d * d);
}

inline bool v_get_seg_isect_pt(BIVec2 a, BIVec2 b, BIVec2 c, BIVec2 d, BIVec2& ip) {

    double dx1 = double(b.x - a.x);
    double dy1 = double(b.y - a.y);
    double dx2 = double(d.x - c.x);
    double dy2 = double(d.y - c.y);
    double det = dy1 * dx2 - dy2 * dx1;

    if (det == 0.0)
        return false;

    double t = (double(a.x - c.x) * dy2 - double(a.y - c.y) * dx2) / det;

    if (t <= 0.0)
        ip = a;
    else if (t >= 1.0)
        ip = b;
    else {
        ip.x = a.x + v_nearbyint(t * dx1);
        ip.y = a.y + v_nearbyint(t * dy1);
    }

    return true;
}

inline BIVec2 v_closest_pt_on_seg(BIVec2 pt, BIVec2 s1, BIVec2 s2) {

    if (s1 == s2)
        return s1;

    double dx = double(s2.x - s1.x);
    double dy = double(s2.y - s1.y);
    double q = (double(pt.x - s1.x) * dx + double(pt.y - s1.y) * dy) / (dx * dx + dy * dy);

    if (q < 0)
        q = 0;
    else if (q > 1)
        q = 1;

    return {s1.x + v_nearbyint(q * dx), s1.y + v_nearbyint(q * dy)};
}

inline int v_sign_d(double v) {
    return (v > 0) - (v < 0);
}

inline bool v_segs_intersect(BIVec2 a, BIVec2 b, BIVec2 c, BIVec2 d) {
    return (v_sign_d(v_cross_product(a, c, d)) * v_sign_d(v_cross_product(b, c, d)) < 0) &&
        (v_sign_d(v_cross_product(c, a, b)) * v_sign_d(v_cross_product(d, a, b)) < 0);
}

inline double v_area_outpt(VOutPt* op) {

    double r = 0.0;
    VOutPt* o = op;

    do {
        r += double(o->prev->pt.y + o->pt.y) * double(o->prev->pt.x - o->pt.x);
        o = o->next;
    } while (o != op);

    return r * 0.5;
}

/// Number of points in the output ring through op.
inline size_t v_ring_size(VOutPt* op) {

    size_t count = 0;
    VOutPt* o = op;

    do {
        count++;
        o = o->next;
    } while (o != op);

    return count;
}

inline double v_area_tri(BIVec2 p1, BIVec2 p2, BIVec2 p3) {
    return double(p3.y + p1.y) * double(p3.x - p1.x) + double(p1.y + p2.y) * double(p1.x - p2.x) +
        double(p2.y + p3.y) * double(p2.x - p3.x);
}

inline bool v_pts_close(BIVec2 a, BIVec2 b) {
    return std::llabs(a.x - b.x) < 2 && std::llabs(a.y - b.y) < 2;
}

inline bool v_very_small_tri(VOutPt& op) {
    return op.next->next == op.prev &&
        (v_pts_close(op.prev->pt, op.next->pt) || v_pts_close(op.pt, op.next->pt) || v_pts_close(op.pt, op.prev->pt));
}

inline bool v_valid_closed(VOutPt* op) {
    return op && op->next != op && op->next != op->prev && !v_very_small_tri(*op);
}

inline int v_winding_step(BIVec2 pt, BIVec2 a, BIVec2 b) {

    int64_t cross = int64_t(b.x - a.x) * int64_t(pt.y - a.y) - int64_t(b.y - a.y) * int64_t(pt.x - a.x);

    if (a.y <= pt.y)
        return (b.y > pt.y && cross > 0) ? 1 : 0;

    return (b.y <= pt.y && cross < 0) ? -1 : 0;
}

static bool pip_i(BIVec2 pt, const std::vector<BIVec2>& poly) {

    int winding = 0;
    int n = (int)poly.size();

    for (int i = 0; i < n; i++)
        winding += v_winding_step(pt, poly[i], poly[(i + 1) % n]);

    return winding != 0;
}

static bool pip_vertex(BIVec2 pt, VVertex* head) {

    int winding = 0;
    VVertex* v = head;

    do {
        winding += v_winding_step(pt, v->pt, v->next->pt);
        v = v->next;
    } while (v != head);

    return winding != 0;
}

// ═══════════════════════════════════════════════════════════════════════════
// Vertex building and local minima detection
// ═══════════════════════════════════════════════════════════════════════════
static void v_find_local_minima(VVertex* head, int8_t polytype, VattiScratch& sc) {

    VVertex* pv = head->prev;

    while (pv != head && pv->pt.y == head->pt.y)
        pv = pv->prev;

    if (pv == head)
        return;

    bool going_up = pv->pt.y > head->pt.y;
    bool going_up0 = going_up;
    pv = head;
    VVertex* cv = head->next;

    while (cv != head) {
        if (cv->pt.y > pv->pt.y && going_up) {
            pv->flags |= VF_LocalMax;
            going_up = false;
        } else if (cv->pt.y < pv->pt.y && !going_up) {
            going_up = true;
            pv->flags |= VF_LocalMin;
            sc.locmin_list.push_back({pv, polytype});
        }

        pv = cv;
        cv = cv->next;
    }

    if (going_up != going_up0) {
        if (going_up0) {
            pv->flags |= VF_LocalMin;
            sc.locmin_list.push_back({pv, polytype});
        } else
            pv->flags |= VF_LocalMax;
    }
}

/// Link n scaled points into a circular vertex list and return its head, or nullptr if degenerate.
static VVertex* v_link_path(VVertex* base, int n, int8_t polytype, VattiScratch& sc) {

    VVertex* prev_v = &base[0];
    int cnt = 1;

    for (int i = 1; i < n; i++) {
        if (base[i].pt == prev_v->pt)
            continue;

        VVertex* cv = &base[cnt];
        cv->pt = base[i].pt;
        cv->prev = prev_v;
        prev_v->next = cv;
        prev_v = cv;
        cnt++;
    }

    if (cnt >= 3 && prev_v->pt == base[0].pt) {
        prev_v = prev_v->prev;
        cnt--;
    }

    if (cnt < 3)
        return nullptr;

    sc.vtx_pool.count += cnt;
    prev_v->next = &base[0];
    base[0].prev = prev_v;
    v_find_local_minima(&base[0], polytype, sc);

    return &base[0];
}

static VVertex* v_add_path_from_doubles(
    const double* coords,
    int n,
    int8_t polytype,
    VScale sv,
    VattiScratch& sc,
    int64_t& min_x,
    int64_t& max_x,
    int64_t& min_y,
    int64_t& max_y
) {

    if (n < 3)
        return nullptr;

    VertexPool& pool = sc.vtx_pool;
    pool.ensure(pool.count + n);
    VVertex* base = &pool.buf[pool.count];

    for (int i = 0; i < n; i++) {
        base[i].flags = VF_None;
        base[i].pt = v_cvt_to_i64(coords + i * 3, sv);
    }

    min_x = max_x = base[0].pt.x;
    min_y = max_y = base[0].pt.y;

    for (int i = 1; i < n; i++) {
        BIVec2 pt = base[i].pt;

        if (pt.x < min_x)
            min_x = pt.x;
        else if (pt.x > max_x)
            max_x = pt.x;

        if (pt.y < min_y)
            min_y = pt.y;
        else if (pt.y > max_y)
            max_y = pt.y;
    }

    return v_link_path(base, n, polytype, sc);
}

static void v_add_path(const std::vector<BIVec2>& pts, int n, int8_t polytype, VattiScratch& sc) {

    if (n < 3)
        return;

    VertexPool& pool = sc.vtx_pool;
    pool.ensure(pool.count + n);
    VVertex* base = &pool.buf[pool.count];

    for (int i = 0; i < n; i++) {
        base[i].flags = VF_None;
        base[i].pt = pts[i];
    }

    v_link_path(base, n, polytype, sc);
}

// ═══════════════════════════════════════════════════════════════════════════
// AEL operations
// ═══════════════════════════════════════════════════════════════════════════
inline VActive* v_get_maxima_pair(const VActive& e) {

    VActive* e2 = e.next_in_ael;

    while (e2) {
        if (e2->vertex_top == e.vertex_top)
            return e2;

        e2 = e2->next_in_ael;
    }

    return nullptr;
}

inline VVertex* v_get_curr_y_maxima(const VActive& e) {

    VVertex* r = e.vertex_top;

    if (e.wind_dx > 0) {
        while (r->next->pt.y == r->pt.y)
            r = r->next;
    } else {
        while (r->prev->pt.y == r->pt.y)
            r = r->prev;
    }

    return v_is_maxima(*r) ? r : nullptr;
}

inline VActive* v_get_prev_hot(const VActive& e) {

    VActive* p = e.prev_in_ael;

    while (p && !v_is_hot(*p))
        p = p->prev_in_ael;

    return p;
}

static bool v_is_valid_ael_order(const VActive& resident, const VActive& newcomer) {

    if (newcomer.curr_x != resident.curr_x)
        return newcomer.curr_x > resident.curr_x;

    double d = v_cross_product(resident.top, newcomer.bot, newcomer.top);

    if (d != 0)
        return d < 0;

    if (!v_is_maxima(resident) && resident.top.y > newcomer.top.y)
        return v_cross_product(newcomer.bot, resident.top, v_next_vertex(resident)->pt) <= 0;

    if (!v_is_maxima(newcomer) && newcomer.top.y > resident.top.y)
        return v_cross_product(newcomer.bot, newcomer.top, v_next_vertex(newcomer)->pt) >= 0;

    int64_t y = newcomer.bot.y;

    if (resident.bot.y != y || resident.local_min->vertex->pt.y != y)
        return newcomer.is_left_bound;

    if (resident.is_left_bound != newcomer.is_left_bound)
        return newcomer.is_left_bound;

    if (v_is_collinear(v_prev_prev_vertex(resident)->pt, resident.bot, resident.top))
        return true;

    return (v_cross_product(v_prev_prev_vertex(resident)->pt, newcomer.bot, v_prev_prev_vertex(newcomer)->pt) > 0) ==
        newcomer.is_left_bound;
}

static void v_insert_left_edge(VattiScratch& sc, VActive& e) {

    if (!sc.actives) {
        e.prev_in_ael = nullptr;
        e.next_in_ael = nullptr;
        sc.actives = &e;
    } else if (!v_is_valid_ael_order(*sc.actives, e)) {
        e.prev_in_ael = nullptr;
        e.next_in_ael = sc.actives;
        sc.actives->prev_in_ael = &e;
        sc.actives = &e;
    } else {
        VActive* e2 = sc.actives;

        while (e2->next_in_ael && v_is_valid_ael_order(*e2->next_in_ael, e))
            e2 = e2->next_in_ael;

        if (e2->join_with == 2)
            e2 = e2->next_in_ael;

        if (!e2)
            return;

        e.next_in_ael = e2->next_in_ael;

        if (e2->next_in_ael)
            e2->next_in_ael->prev_in_ael = &e;

        e.prev_in_ael = e2;
        e2->next_in_ael = &e;
    }
}

inline void v_insert_right_edge(VActive& e, VActive& e2) {

    e2.next_in_ael = e.next_in_ael;

    if (e.next_in_ael)
        e.next_in_ael->prev_in_ael = &e2;

    e2.prev_in_ael = &e;
    e.next_in_ael = &e2;
}

inline void v_swap_positions_in_ael(VattiScratch& sc, VActive& e1, VActive& e2) {

    VActive* next = e2.next_in_ael;

    if (next)
        next->prev_in_ael = &e1;

    VActive* prev = e1.prev_in_ael;

    if (prev)
        prev->next_in_ael = &e2;

    e2.prev_in_ael = prev;
    e2.next_in_ael = &e1;
    e1.prev_in_ael = &e2;
    e1.next_in_ael = next;

    if (!e2.prev_in_ael)
        sc.actives = &e2;
}

inline void v_delete_from_ael(VattiScratch& sc, VActive& e) {

    VActive* prev = e.prev_in_ael;
    VActive* next = e.next_in_ael;

    if (!prev && !next && &e != sc.actives)
        return;

    if (prev)
        prev->next_in_ael = next;
    else
        sc.actives = next;

    if (next)
        next->prev_in_ael = prev;
}

// ═══════════════════════════════════════════════════════════════════════════
// Scanline
// ═══════════════════════════════════════════════════════════════════════════
inline void v_insert_scanline(VattiScratch& sc, int64_t y) {
    sc.scanline_list.push(y);
}

inline bool v_pop_scanline(VattiScratch& sc, int64_t& y) {

    ScanlineHeap& sl = sc.scanline_list;

    if (sl.empty())
        return false;

    y = sl.top();
    sl.pop();

    while (!sl.empty() && y == sl.top())
        sl.pop();

    return true;
}

inline bool v_pop_locmin(VattiScratch& sc, int64_t y, VLocalMinima*& lm) {

    if (sc.locmin_idx >= sc.locmin_list.size() || sc.locmin_list[sc.locmin_idx].vertex->pt.y != y)
        return false;

    lm = &sc.locmin_list[sc.locmin_idx++];

    return true;
}

inline void v_push_horz(VattiScratch& sc, VActive& e) {

    e.next_in_sel = sc.sel;
    sc.sel = &e;
}

inline bool v_pop_horz(VattiScratch& sc, VActive*& e) {

    e = sc.sel;

    if (!e)
        return false;

    sc.sel = sc.sel->next_in_sel;

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Winding and contribution
// ═══════════════════════════════════════════════════════════════════════════
static void v_set_wind_count(VattiScratch& sc, VActive& e) {

    int8_t pt = v_polytype(e);
    VActive* e2 = e.prev_in_ael;

    while (e2 && v_polytype(*e2) != pt)
        e2 = e2->prev_in_ael;

    if (!e2) {
        e.wind_cnt = e.wind_dx;
        e2 = sc.actives;
    } else {
        if (e2->wind_cnt * e2->wind_dx < 0) {
            if (std::abs(e2->wind_cnt) > 1)
                e.wind_cnt = (e2->wind_dx * e.wind_dx < 0) ? e2->wind_cnt : e2->wind_cnt + e.wind_dx;
            else
                e.wind_cnt = e.wind_dx;
        } else {
            e.wind_cnt = (e2->wind_dx * e.wind_dx < 0) ? e2->wind_cnt : e2->wind_cnt + e.wind_dx;
        }

        e.wind_cnt2 = e2->wind_cnt2;
        e2 = e2->next_in_ael;
    }

    while (e2 != &e) {
        if (v_polytype(*e2) != pt)
            e.wind_cnt2 += e2->wind_dx;

        e2 = e2->next_in_ael;
    }
}

static bool v_is_contributing(const VActive& e, int cliptype) {

    if (std::abs(e.wind_cnt) != 1)
        return false;

    int wc2 = std::abs(e.wind_cnt2);

    if (cliptype == 0)
        return wc2 != 0;

    if (cliptype == 1)
        return wc2 == 0;

    bool r = (wc2 == 0);

    return (v_polytype(e) == 0) ? r : !r;
}

// ═══════════════════════════════════════════════════════════════════════════
// Output operations
// ═══════════════════════════════════════════════════════════════════════════
inline void v_set_sides(VOutRec& or_, VActive& f, VActive& b) {

    or_.front_edge = &f;
    or_.back_edge = &b;
}

static void v_swap_outrecs(VActive& e1, VActive& e2) {

    VOutRec* or1 = e1.outrec;
    VOutRec* or2 = e2.outrec;

    if (or1 == or2) {
        VActive* t = or1->front_edge;
        or1->front_edge = or1->back_edge;
        or1->back_edge = t;

        return;
    }

    if (or1) {
        if (&e1 == or1->front_edge)
            or1->front_edge = &e2;
        else
            or1->back_edge = &e2;
    }

    if (or2) {
        if (&e2 == or2->front_edge)
            or2->front_edge = &e1;
        else
            or2->back_edge = &e1;
    }

    e1.outrec = or2;
    e2.outrec = or1;
}

static VOutPt* v_add_outpt(const VActive& e, BIVec2 pt, VattiScratch& sc) {

    VOutRec* outrec = e.outrec;
    bool to_front = v_is_front(e);
    VOutPt* op_front = outrec->pts;
    VOutPt* op_back = op_front->next;

    if (to_front && pt == op_front->pt)
        return op_front;

    if (!to_front && pt == op_back->pt)
        return op_back;

    VOutPt* nop = sc.new_outpt(pt, outrec);
    op_back->prev = nop;
    nop->prev = op_front;
    nop->next = op_back;
    op_front->next = nop;

    if (to_front)
        outrec->pts = nop;

    return nop;
}

static VOutPt* v_add_local_min_poly(VActive& e1, VActive& e2, BIVec2 pt, VattiScratch& sc, bool is_new) {

    VOutRec* outrec = sc.new_outrec();
    e1.outrec = outrec;
    e2.outrec = outrec;
    VActive* prev_hot = v_get_prev_hot(e1);

    if (prev_hot) {
        if ((prev_hot == prev_hot->outrec->front_edge) == is_new)
            v_set_sides(*outrec, e2, e1);
        else
            v_set_sides(*outrec, e1, e2);
    } else {
        outrec->owner = nullptr;

        if (is_new)
            v_set_sides(*outrec, e1, e2);
        else
            v_set_sides(*outrec, e2, e1);
    }

    VOutPt* op = sc.new_outpt(pt, outrec);
    outrec->pts = op;

    return op;
}

static void v_uncouple(VActive& ae) {

    VOutRec* or_ = ae.outrec;

    if (!or_)
        return;

    or_->front_edge->outrec = nullptr;
    or_->back_edge->outrec = nullptr;
    or_->front_edge = nullptr;
    or_->back_edge = nullptr;
}

static void v_join_outrec_paths(VActive& e1, VActive& e2) {

    VOutPt* p1_st = e1.outrec->pts;
    VOutPt* p2_st = e2.outrec->pts;
    VOutPt* p1_end = p1_st->next;
    VOutPt* p2_end = p2_st->next;

    if (v_is_front(e1)) {
        p2_end->prev = p1_st;
        p1_st->next = p2_end;
        p2_st->next = p1_end;
        p1_end->prev = p2_st;
        e1.outrec->pts = p2_st;
        e1.outrec->front_edge = e2.outrec->front_edge;

        if (e1.outrec->front_edge)
            e1.outrec->front_edge->outrec = e1.outrec;
    } else {
        p1_end->prev = p2_st;
        p2_st->next = p1_end;
        p1_st->next = p2_end;
        p2_end->prev = p1_st;
        e1.outrec->back_edge = e2.outrec->back_edge;

        if (e1.outrec->back_edge)
            e1.outrec->back_edge->outrec = e1.outrec;
    }

    e2.outrec->front_edge = nullptr;
    e2.outrec->back_edge = nullptr;
    e2.outrec->pts = nullptr;
    e2.outrec->owner = e1.outrec;
    e1.outrec = nullptr;
    e2.outrec = nullptr;
}

static void v_split(VActive& e, BIVec2 pt, VattiScratch& sc);

static VOutPt* v_add_local_max_poly(VActive& e1, VActive& e2, BIVec2 pt, VattiScratch& sc) {

    if (v_is_joined(e1))
        v_split(e1, pt, sc);

    if (v_is_joined(e2))
        v_split(e2, pt, sc);

    if (v_is_front(e1) == v_is_front(e2)) {
        sc.succeeded = false;

        return nullptr;
    }

    VOutPt* result = v_add_outpt(e1, pt, sc);

    if (e1.outrec == e2.outrec) {
        VOutRec& outrec = *e1.outrec;
        outrec.pts = result;
        v_uncouple(e1);
        result = outrec.pts;
    } else if (e1.outrec->idx < e2.outrec->idx)
        v_join_outrec_paths(e1, e2);
    else
        v_join_outrec_paths(e2, e1);

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Split and check join
// ═══════════════════════════════════════════════════════════════════════════
static void v_split(VActive& e, BIVec2 pt, VattiScratch& sc) {

    if (e.join_with == 2) {
        e.join_with = 0;
        e.next_in_ael->join_with = 0;
        v_add_local_min_poly(e, *e.next_in_ael, pt, sc, true);
    } else {
        e.join_with = 0;
        e.prev_in_ael->join_with = 0;
        v_add_local_min_poly(*e.prev_in_ael, e, pt, sc, true);
    }
}

static void v_check_join_left(VActive& e, BIVec2 pt, VattiScratch& sc, bool check_curr_x = false) {

    VActive* prev = e.prev_in_ael;

    if (!prev || !v_is_hot(e) || !v_is_hot(*prev) || v_is_horizontal(e) || v_is_horizontal(*prev))
        return;

    if ((pt.y < e.top.y + 2 || pt.y < prev->top.y + 2) && (e.bot.y > pt.y || prev->bot.y > pt.y))
        return;

    if (check_curr_x) {
        if (v_perpendic_dist_sq(pt, prev->bot, prev->top) > 0.25)
            return;
    } else if (e.curr_x != prev->curr_x)
        return;

    if (!v_is_collinear(e.top, pt, prev->top))
        return;

    if (e.outrec->idx == prev->outrec->idx)
        v_add_local_max_poly(*prev, e, pt, sc);
    else if (e.outrec->idx < prev->outrec->idx)
        v_join_outrec_paths(e, *prev);
    else
        v_join_outrec_paths(*prev, e);

    prev->join_with = 2;
    e.join_with = 1;
}

static void v_check_join_right(VActive& e, BIVec2 pt, VattiScratch& sc, bool check_curr_x = false) {

    VActive* next = e.next_in_ael;

    if (!next || !v_is_hot(e) || !v_is_hot(*next) || v_is_horizontal(e) || v_is_horizontal(*next))
        return;

    if ((pt.y < e.top.y + 2 || pt.y < next->top.y + 2) && (e.bot.y > pt.y || next->bot.y > pt.y))
        return;

    if (check_curr_x) {
        if (v_perpendic_dist_sq(pt, next->bot, next->top) > 0.35)
            return;
    } else if (e.curr_x != next->curr_x)
        return;

    if (!v_is_collinear(e.top, pt, next->top))
        return;

    if (e.outrec->idx == next->outrec->idx)
        v_add_local_max_poly(e, *next, pt, sc);
    else if (e.outrec->idx < next->outrec->idx)
        v_join_outrec_paths(e, *next);
    else
        v_join_outrec_paths(*next, e);

    e.join_with = 2;
    next->join_with = 1;
}

// ═══════════════════════════════════════════════════════════════════════════
// Intersect edges
// ═══════════════════════════════════════════════════════════════════════════
/// Update the winding counts of two edges that cross.
static void v_update_wind_counts(VActive& e1, VActive& e2) {

    if (v_polytype(e1) == v_polytype(e2)) {
        if (e1.wind_cnt + e2.wind_dx == 0)
            e1.wind_cnt = -e1.wind_cnt;
        else
            e1.wind_cnt += e2.wind_dx;

        if (e2.wind_cnt - e1.wind_dx == 0)
            e2.wind_cnt = -e2.wind_cnt;
        else
            e2.wind_cnt -= e1.wind_dx;
    } else {
        e1.wind_cnt2 += e2.wind_dx;
        e2.wind_cnt2 -= e1.wind_dx;
    }
}

static void v_intersect_edges(VActive& e1, VActive& e2, BIVec2 pt, VattiScratch& sc, int cliptype) {

    if (v_is_joined(e1))
        v_split(e1, pt, sc);

    if (v_is_joined(e2))
        v_split(e2, pt, sc);

    v_update_wind_counts(e1, e2);

    int old_e1_wc = std::abs(e1.wind_cnt);
    int old_e2_wc = std::abs(e2.wind_cnt);
    bool e1_in01 = old_e1_wc == 0 || old_e1_wc == 1;
    bool e2_in01 = old_e2_wc == 0 || old_e2_wc == 1;

    if ((!v_is_hot(e1) && !e1_in01) || (!v_is_hot(e2) && !e2_in01))
        return;

    if (v_is_hot(e1) && v_is_hot(e2)) {
        if ((old_e1_wc != 0 && old_e1_wc != 1) || (old_e2_wc != 0 && old_e2_wc != 1) ||
            (v_polytype(e1) != v_polytype(e2))) {
            v_add_local_max_poly(e1, e2, pt, sc);
        } else if (v_is_front(e1) || e1.outrec == e2.outrec) {
            v_add_local_max_poly(e1, e2, pt, sc);
            v_add_local_min_poly(e1, e2, pt, sc, false);
        } else {
            v_add_outpt(e1, pt, sc);
            v_add_outpt(e2, pt, sc);
            v_swap_outrecs(e1, e2);
        }
    } else if (v_is_hot(e1)) {
        v_add_outpt(e1, pt, sc);
        v_swap_outrecs(e1, e2);
    } else if (v_is_hot(e2)) {
        v_add_outpt(e2, pt, sc);
        v_swap_outrecs(e1, e2);
    } else {
        int64_t e1_wc2 = std::abs(e1.wind_cnt2);
        int64_t e2_wc2 = std::abs(e2.wind_cnt2);

        if (!v_same_polytype(e1, e2)) {
            v_add_local_min_poly(e1, e2, pt, sc, false);
        } else if (old_e1_wc == 1 && old_e2_wc == 1) {
            if (cliptype == 0) {
                if (e1_wc2 > 0 && e2_wc2 > 0)
                    v_add_local_min_poly(e1, e2, pt, sc, false);
            } else if (cliptype == 1) {
                if (e1_wc2 <= 0 && e2_wc2 <= 0)
                    v_add_local_min_poly(e1, e2, pt, sc, false);
            } else {
                if ((v_polytype(e1) == 1 && e1_wc2 > 0 && e2_wc2 > 0) ||
                    (v_polytype(e1) == 0 && e1_wc2 <= 0 && e2_wc2 <= 0))
                    v_add_local_min_poly(e1, e2, pt, sc, false);
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Horizontal edges
// ═══════════════════════════════════════════════════════════════════════════
static void v_add_trial_horz_join(VattiScratch& sc, VOutPt* op) {
    sc.horz_seg_list.push_back({op});
}

inline VOutPt* v_get_last_op(const VActive& e) {

    VOutPt* r = e.outrec->pts;

    if (&e != e.outrec->front_edge)
        r = r->next;

    return r;
}

inline void v_update_edge_into_ael(VattiScratch& sc, VActive* e) {

    e->bot = e->top;
    e->vertex_top = v_next_vertex(*e);
    e->top = e->vertex_top->pt;
    e->curr_x = e->bot.x;
    v_set_dx(*e);

    if (v_is_joined(*e))
        v_split(*e, e->bot, sc);

    if (v_is_horizontal(*e)) {
        BIVec2 pt = v_next_vertex(*e)->pt;

        while (pt.y == e->top.y) {
            if ((pt.x < e->top.x) != (e->bot.x < e->top.x))
                break;

            e->vertex_top = v_next_vertex(*e);
            e->top = pt;

            if (v_is_maxima(*e))
                break;

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

    if (horz.bot.x == horz.top.x) {
        left = horz.curr_x;
        right = horz.curr_x;
        VActive* e = horz.next_in_ael;

        while (e && e->vertex_top != max_v)
            e = e->next_in_ael;

        return e != nullptr;
    }

    if (horz.curr_x < horz.top.x) {
        left = horz.curr_x;
        right = horz.top.x;

        return true;
    }

    left = horz.top.x;
    right = horz.curr_x;

    return false;
}

/// Close a horizontal that reached the edge sharing its maximum vertex.
static void v_horz_meet_maxima(VActive& horz, VActive& e, VVertex* vertex_max, bool is_ltr, VattiScratch& sc) {

    if (v_is_hot(horz) && v_is_joined(e))
        v_split(e, e.top, sc);

    if (v_is_hot(horz)) {
        while (horz.vertex_top != vertex_max) {
            v_add_outpt(horz, horz.top, sc);
            v_update_edge_into_ael(sc, &horz);
        }

        if (is_ltr)
            v_add_local_max_poly(horz, e, horz.top, sc);
        else
            v_add_local_max_poly(e, horz, horz.top, sc);
    }

    v_delete_from_ael(sc, e);
    v_delete_from_ael(sc, horz);
}

/// Return whether a horizontal stops before edge e.
static bool v_horz_stops_at(
    const VActive& horz,
    const VActive& e,
    VVertex* vertex_max,
    bool is_ltr,
    int64_t horz_left,
    int64_t horz_right
) {

    if (vertex_max == horz.vertex_top)
        return false;

    if ((is_ltr && e.curr_x > horz_right) || (!is_ltr && e.curr_x < horz_left))
        return true;

    if (e.curr_x != horz.top.x || v_is_horizontal(e))
        return false;

    BIVec2 pt2 = v_next_vertex(horz)->pt;

    if (is_ltr)
        return v_top_x(e, pt2.y) >= pt2.x;

    return v_top_x(e, pt2.y) <= pt2.x;
}

/// Pass a horizontal over edge e and return the next edge in its direction.
static VActive* v_horz_cross_edge(VActive& horz, VActive& e, bool is_ltr, VattiScratch& sc, int cliptype) {

    BIVec2 pt = {e.curr_x, horz.bot.y};

    if (is_ltr) {
        v_intersect_edges(horz, e, pt, sc, cliptype);
        v_swap_positions_in_ael(sc, horz, e);
        v_check_join_left(e, pt, sc);
    } else {
        v_intersect_edges(e, horz, pt, sc, cliptype);
        v_swap_positions_in_ael(sc, e, horz);
        v_check_join_right(e, pt, sc);
    }

    horz.curr_x = e.curr_x;

    if (horz.outrec)
        v_add_trial_horz_join(sc, v_get_last_op(horz));

    return is_ltr ? horz.next_in_ael : horz.prev_in_ael;
}

static void v_do_horizontal(VActive& horz, VattiScratch& sc, int cliptype) {

    int64_t y = horz.bot.y;
    VVertex* vertex_max = v_get_curr_y_maxima(horz);
    int64_t horz_left;
    int64_t horz_right;
    bool is_ltr = v_reset_horz_dir(horz, vertex_max, horz_left, horz_right);

    if (v_is_hot(horz)) {
        VOutPt* op = v_add_outpt(horz, {horz.curr_x, y}, sc);
        v_add_trial_horz_join(sc, op);
    }

    const size_t max_iter = sc.vtx_pool.count;

    for (size_t iter = 0; iter < max_iter; iter++) {
        VActive* e = is_ltr ? horz.next_in_ael : horz.prev_in_ael;

        while (e) {
            if (e->vertex_top == vertex_max) {
                v_horz_meet_maxima(horz, *e, vertex_max, is_ltr, sc);

                return;
            }

            if (v_horz_stops_at(horz, *e, vertex_max, is_ltr, horz_left, horz_right))
                break;

            e = v_horz_cross_edge(horz, *e, is_ltr, sc, cliptype);
        }

        if (v_next_vertex(horz)->pt.y != horz.top.y)
            break;

        if (v_is_hot(horz))
            v_add_outpt(horz, horz.top, sc);

        v_update_edge_into_ael(sc, &horz);
        is_ltr = v_reset_horz_dir(horz, vertex_max, horz_left, horz_right);
    }

    if (v_is_hot(horz)) {
        VOutPt* op = v_add_outpt(horz, horz.top, sc);
        v_add_trial_horz_join(sc, op);
    }

    v_update_edge_into_ael(sc, &horz);
}

// ═══════════════════════════════════════════════════════════════════════════
// Horizontal joins
// ═══════════════════════════════════════════════════════════════════════════
static VOutPt* v_dup_outpt(VOutPt* op, bool after, VattiScratch& sc) {

    VOutPt* r = sc.new_outpt(op->pt, op->outrec);

    if (after) {
        r->next = op->next;
        r->next->prev = r;
        r->prev = op;
        op->next = r;
    } else {
        r->prev = op->prev;
        r->prev->next = r;
        r->next = op;
        op->prev = r;
    }

    return r;
}

static bool v_horz_seg_less(const VHorzSeg& a, const VHorzSeg& b) {

    if (!a.right_op || !b.right_op)
        return (a.right_op != nullptr);

    return b.left_op->pt.x > a.left_op->pt.x;
}

/// Order a horizontal run into its segment and return whether it can join.
static bool v_set_horz_segment(VHorzSeg& hs, VOutPt* op_p, VOutPt* op_n) {

    if (op_p->pt.x == op_n->pt.x) {
        hs.right_op = nullptr;

        return false;
    }

    if (op_p->pt.x < op_n->pt.x) {
        hs.left_op = op_p;
        hs.right_op = op_n;
        hs.left_to_right = true;
    } else {
        hs.left_op = op_n;
        hs.right_op = op_p;
        hs.left_to_right = false;
    }

    if (hs.left_op->horz) {
        hs.right_op = nullptr;

        return false;
    }

    hs.left_op->horz = &hs;

    return true;
}

/// Extend a trial segment to its full horizontal run and return whether it can join.
static bool v_update_horz_segment(VHorzSeg& hs) {

    VOutPt* op = hs.left_op;
    VOutRec* outrec = op->outrec;

    while (outrec && !outrec->pts)
        outrec = outrec->owner;

    if (!outrec) {
        hs.right_op = nullptr;

        return false;
    }

    bool has_edges = outrec->front_edge != nullptr;
    int64_t cy = op->pt.y;
    VOutPt* op_p = op;
    VOutPt* op_n = op;

    if (has_edges) {
        VOutPt* op_a = outrec->pts;
        VOutPt* op_z = op_a->next;

        while (op_p != op_z && op_p->prev->pt.y == cy)
            op_p = op_p->prev;

        while (op_n != op_a && op_n->next->pt.y == cy)
            op_n = op_n->next;
    } else {
        while (op_p->prev != op_n && op_p->prev->pt.y == cy)
            op_p = op_p->prev;

        while (op_n->next != op_p && op_n->next->pt.y == cy)
            op_n = op_n->next;
    }

    return v_set_horz_segment(hs, op_p, op_n);
}

/// Join two overlapping horizontal segments of opposite direction.
static void v_add_horz_join(VattiScratch& sc, VHorzSeg& hs1, VHorzSeg& hs2) {

    int64_t cy = hs1.left_op->pt.y;

    if (hs1.left_to_right) {
        while (hs1.left_op->next->pt.y == cy && hs1.left_op->next->pt.x <= hs2.left_op->pt.x)
            hs1.left_op = hs1.left_op->next;

        while (hs2.left_op->prev->pt.y == cy && hs2.left_op->prev->pt.x <= hs1.left_op->pt.x)
            hs2.left_op = hs2.left_op->prev;

        sc.horz_join_list.push_back({v_dup_outpt(hs1.left_op, true, sc), v_dup_outpt(hs2.left_op, false, sc)});
    } else {
        while (hs1.left_op->prev->pt.y == cy && hs1.left_op->prev->pt.x <= hs2.left_op->pt.x)
            hs1.left_op = hs1.left_op->prev;

        while (hs2.left_op->next->pt.y == cy && hs2.left_op->next->pt.x <= hs1.left_op->pt.x)
            hs2.left_op = hs2.left_op->next;

        sc.horz_join_list.push_back({v_dup_outpt(hs2.left_op, true, sc), v_dup_outpt(hs1.left_op, false, sc)});
    }
}

static void v_convert_horz_segs_to_joins(VattiScratch& sc) {

    int valid = 0;

    for (VHorzSeg& hs : sc.horz_seg_list)
        if (v_update_horz_segment(hs))
            valid++;

    if (valid < 2)
        return;

    std::stable_sort(sc.horz_seg_list.begin(), sc.horz_seg_list.end(), v_horz_seg_less);

    for (int i = 0; i < valid - 1; i++) {
        VHorzSeg& hs1 = sc.horz_seg_list[i];

        for (int k = i + 1; k < valid; k++) {
            VHorzSeg& hs2 = sc.horz_seg_list[k];

            if (hs2.left_op->pt.x >= hs1.right_op->pt.x || hs2.left_to_right == hs1.left_to_right ||
                hs2.right_op->pt.x <= hs1.left_op->pt.x)
                continue;

            v_add_horz_join(sc, hs1, hs2);
        }
    }
}

static void v_fix_outrec_pts(VOutRec* outrec) {

    VOutPt* op = outrec->pts;

    do {
        op->outrec = outrec;
        op = op->next;
    } while (op != outrec->pts);
}

static void v_process_horz_joins(VattiScratch& sc) {

    for (VHorzJoin& j : sc.horz_join_list) {
        VOutRec* or1 = j.op1->outrec;

        while (or1 && !or1->pts)
            or1 = or1->owner;

        VOutRec* or2 = j.op2->outrec;

        while (or2 && !or2->pts)
            or2 = or2->owner;

        VOutPt* op1b = j.op1->next;
        VOutPt* op2b = j.op2->prev;
        j.op1->next = j.op2;
        j.op2->prev = j.op1;
        op1b->prev = op2b;
        op2b->next = op1b;

        if (or1 == or2) {
            or2 = sc.new_outrec();
            or2->pts = op1b;
            v_fix_outrec_pts(or2);

            if (or1->pts->outrec == or2) {
                or1->pts = j.op1;
                or1->pts->outrec = or1;
            }

            or2->owner = or1;
        } else {
            or2->pts = nullptr;
            or2->owner = or1;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Intersection detection
// ═══════════════════════════════════════════════════════════════════════════
inline void v_adjust_curr_x_copy_to_sel(VattiScratch& sc, int64_t top_y) {

    VActive* e = sc.actives;
    sc.sel = e;

    while (e) {
        e->prev_in_sel = e->prev_in_ael;
        e->next_in_sel = e->next_in_ael;
        e->jump = e->next_in_sel;

        if (e->join_with == 1)
            e->curr_x = e->prev_in_ael->curr_x;
        else
            e->curr_x = v_top_x(*e, top_y);

        e = e->next_in_ael;
    }
}

inline VActive* v_extract_from_sel(VActive* ae) {

    VActive* res = ae->next_in_sel;

    if (res)
        res->prev_in_sel = ae->prev_in_sel;

    ae->prev_in_sel->next_in_sel = res;

    return res;
}

inline void v_insert1_before2_in_sel(VActive* a1, VActive* a2) {

    a1->prev_in_sel = a2->prev_in_sel;

    if (a1->prev_in_sel)
        a1->prev_in_sel->next_in_sel = a1;

    a1->next_in_sel = a2;
    a2->prev_in_sel = a1;
}

static void v_add_new_isect_node(VattiScratch& sc, VActive& e1, VActive& e2, int64_t top_y) {

    BIVec2 ip;

    if (!v_get_seg_isect_pt(e1.bot, e1.top, e2.bot, e2.top, ip))
        ip = {e1.curr_x, top_y};

    if (ip.y > sc.bot_y || ip.y < top_y) {
        double ad1 = std::fabs(e1.dx);
        double ad2 = std::fabs(e2.dx);

        if (ad1 > 100 && ad2 > 100)
            ip = (ad1 > ad2) ? v_closest_pt_on_seg(ip, e1.bot, e1.top) : v_closest_pt_on_seg(ip, e2.bot, e2.top);
        else if (ad1 > 100)
            ip = v_closest_pt_on_seg(ip, e1.bot, e1.top);
        else if (ad2 > 100)
            ip = v_closest_pt_on_seg(ip, e2.bot, e2.top);
        else {
            if (ip.y < top_y)
                ip.y = top_y;
            else
                ip.y = sc.bot_y;

            ip.x = (ad1 < ad2) ? v_top_x(e1, ip.y) : v_top_x(e2, ip.y);
        }
    }

    sc.intersect_nodes.push_back({ip, &e1, &e2});
}

static bool v_build_intersect_list(VattiScratch& sc, int64_t top_y) {

    if (!sc.actives || !sc.actives->next_in_ael)
        return false;

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
                    const size_t max_iter = sc.vtx_pool.count;

                    for (size_t iter = 0; iter < max_iter; iter++) {
                        v_add_new_isect_node(sc, *tmp, *right, top_y);

                        if (tmp == left)
                            break;

                        tmp = tmp->prev_in_sel;
                    }

                    tmp = right;
                    right = v_extract_from_sel(tmp);
                    l_end = right;
                    v_insert1_before2_in_sel(tmp, left);

                    if (left == curr_base) {
                        curr_base = tmp;
                        curr_base->jump = r_end;

                        if (!prev_base)
                            sc.sel = curr_base;
                        else
                            prev_base->jump = curr_base;
                    }
                } else
                    left = left->next_in_sel;
            }

            prev_base = curr_base;
            left = r_end;
        }

        left = sc.sel;
    }

    return !sc.intersect_nodes.empty();
}

static bool v_intersect_node_less(const VIntersectNode& a, const VIntersectNode& b) {
    return (a.pt.y == b.pt.y) ? a.pt.x < b.pt.x : a.pt.y > b.pt.y;
}

static void v_process_intersect_list(VattiScratch& sc, int cliptype) {

    std::stable_sort(sc.intersect_nodes.begin(), sc.intersect_nodes.end(), v_intersect_node_less);

    for (size_t i = 0; i < sc.intersect_nodes.size(); i++) {
        VIntersectNode& node = sc.intersect_nodes[i];

        if (!(node.edge1->next_in_ael == node.edge2 || node.edge1->prev_in_ael == node.edge2)) {
            for (size_t j = i + 1; j < sc.intersect_nodes.size(); j++) {
                if (sc.intersect_nodes[j].edge1->next_in_ael == sc.intersect_nodes[j].edge2 ||
                    sc.intersect_nodes[j].edge1->prev_in_ael == sc.intersect_nodes[j].edge2) {
                    std::swap(sc.intersect_nodes[i], sc.intersect_nodes[j]);
                    node = sc.intersect_nodes[i];
                    break;
                }
            }
        }

        v_intersect_edges(*node.edge1, *node.edge2, node.pt, sc, cliptype);
        v_swap_positions_in_ael(sc, *node.edge1, *node.edge2);
        node.edge1->curr_x = node.pt.x;
        node.edge2->curr_x = node.pt.x;
        v_check_join_left(*node.edge2, node.pt, sc, true);
        v_check_join_right(*node.edge1, node.pt, sc, true);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Local minima insertion
// ═══════════════════════════════════════════════════════════════════════════
/// New active edge leaving a local minimum, wind_dx -1 along prev and 1 along next.
static VActive* v_new_bound(VattiScratch& sc, VLocalMinima* lm, int wind_dx) {

    VActive* b = sc.new_active();
    b->bot = lm->vertex->pt;
    b->curr_x = b->bot.x;
    b->wind_dx = wind_dx;
    b->vertex_top = (wind_dx < 0) ? lm->vertex->prev : lm->vertex->next;
    b->top = b->vertex_top->pt;
    b->local_min = lm;
    v_set_dx(*b);

    return b;
}

static void v_insert_local_minima_into_ael(VattiScratch& sc, int64_t bot_y, int cliptype) {

    VLocalMinima* lm = nullptr;

    while (v_pop_locmin(sc, bot_y, lm)) {
        VActive* lb = v_new_bound(sc, lm, -1);
        VActive* rb = v_new_bound(sc, lm, 1);

        if (v_is_horizontal(*lb)) {
            if (lb->dx == -std::numeric_limits<double>::max())
                std::swap(lb, rb);
        } else if (v_is_horizontal(*rb)) {
            if (rb->dx == std::numeric_limits<double>::max())
                std::swap(lb, rb);
        } else if (lb->dx < rb->dx)
            std::swap(lb, rb);

        lb->is_left_bound = true;
        v_insert_left_edge(sc, *lb);
        v_set_wind_count(sc, *lb);
        bool contributing = v_is_contributing(*lb, cliptype);

        rb->is_left_bound = false;
        rb->wind_cnt = lb->wind_cnt;
        rb->wind_cnt2 = lb->wind_cnt2;
        v_insert_right_edge(*lb, *rb);

        if (contributing) {
            v_add_local_min_poly(*lb, *rb, lb->bot, sc, true);

            if (!v_is_horizontal(*lb))
                v_check_join_left(*lb, lb->bot, sc);
        }

        while (rb->next_in_ael && v_is_valid_ael_order(*rb->next_in_ael, *rb)) {
            v_intersect_edges(*rb, *rb->next_in_ael, rb->bot, sc, cliptype);
            v_swap_positions_in_ael(sc, *rb, *rb->next_in_ael);
        }

        if (v_is_horizontal(*rb))
            v_push_horz(sc, *rb);
        else {
            v_check_join_right(*rb, rb->bot, sc);
            v_insert_scanline(sc, rb->top.y);
        }

        if (v_is_horizontal(*lb))
            v_push_horz(sc, *lb);
        else
            v_insert_scanline(sc, lb->top.y);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Maxima
// ═══════════════════════════════════════════════════════════════════════════
static VActive* v_do_maxima(VActive& e, VattiScratch& sc, int cliptype) {

    VActive* prev_e = e.prev_in_ael;
    VActive* next_e = e.next_in_ael;
    VActive* max_pair = v_get_maxima_pair(e);

    if (!max_pair)
        return next_e;

    if (v_is_joined(e))
        v_split(e, e.top, sc);

    if (v_is_joined(*max_pair))
        v_split(*max_pair, max_pair->top, sc);

    while (next_e != max_pair) {
        v_intersect_edges(e, *next_e, e.top, sc, cliptype);
        v_swap_positions_in_ael(sc, e, *next_e);
        next_e = e.next_in_ael;
    }

    if (v_is_hot(e))
        v_add_local_max_poly(e, *max_pair, e.top, sc);

    v_delete_from_ael(sc, *max_pair);
    v_delete_from_ael(sc, e);

    return prev_e ? prev_e->next_in_ael : sc.actives;
}

// ═══════════════════════════════════════════════════════════════════════════
// Top of scanbeam
// ═══════════════════════════════════════════════════════════════════════════
static void v_do_top_of_scanbeam(VattiScratch& sc, int64_t y, int cliptype) {

    sc.sel = nullptr;
    VActive* e = sc.actives;

    while (e) {
        if (e->top.y == y) {
            e->curr_x = e->top.x;

            if (v_is_maxima(*e)) {
                e = v_do_maxima(*e, sc, cliptype);
                continue;
            }

            if (v_is_hot(*e))
                v_add_outpt(*e, e->top, sc);

            v_update_edge_into_ael(sc, e);

            if (v_is_horizontal(*e))
                v_push_horz(sc, *e);
        } else
            e->curr_x = v_top_x(*e, y);

        e = e->next_in_ael;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Collinear cleanup and self intersections
// ═══════════════════════════════════════════════════════════════════════════
static VOutPt* v_dispose_outpt(VOutPt* op) {

    VOutPt* r = op->next;
    op->prev->next = op->next;
    op->next->prev = op->prev;

    return r;
}

static void v_do_split_op(VattiScratch& sc, VOutRec* outrec, VOutPt* split_op) {

    VOutPt* prev_op = split_op->prev;
    VOutPt* nn_op = split_op->next->next;
    outrec->pts = prev_op;
    BIVec2 ip;

    if (!v_get_seg_isect_pt(prev_op->pt, split_op->pt, split_op->next->pt, nn_op->pt, ip))
        return;

    double area1 = v_area_outpt(outrec->pts);

    if (std::fabs(area1) < 2) {
        outrec->pts = nullptr;

        return;
    }

    double area2 = v_area_tri(ip, split_op->pt, split_op->next->pt);
    double abs_a2 = std::fabs(area2);

    if (ip == prev_op->pt || ip == nn_op->pt) {
        nn_op->prev = prev_op;
        prev_op->next = nn_op;
    } else {
        VOutPt* nop = sc.new_outpt(ip, prev_op->outrec);
        nop->prev = prev_op;
        nop->next = nn_op;
        nn_op->prev = nop;
        prev_op->next = nop;
    }

    if (abs_a2 >= 1 && (abs_a2 > std::fabs(area1) || (area2 > 0) == (area1 > 0))) {
        VOutRec* nr = sc.new_outrec();
        nr->owner = outrec->owner;
        split_op->outrec = nr;
        split_op->next->outrec = nr;
        VOutPt* nop = sc.new_outpt(ip, nr);
        nop->prev = split_op->next;
        nop->next = split_op;
        nr->pts = nop;
        split_op->prev = nop;
        split_op->next->next = nop;
    }
}

static void v_fix_self_intersects(VattiScratch& sc, VOutRec* outrec) {

    VOutPt* op2 = outrec->pts;
    const size_t count = v_ring_size(op2);
    const size_t max_iter = (count + 1) * (count + 1);

    for (size_t iter = 0; iter < max_iter; iter++) {
        if (op2->prev == op2->next->next)
            break;

        if (v_segs_intersect(op2->prev->pt, op2->pt, op2->next->pt, op2->next->next->pt)) {
            if (op2 == outrec->pts || op2->next == outrec->pts)
                outrec->pts = outrec->pts->prev;

            v_do_split_op(sc, outrec, op2);

            if (!outrec->pts)
                break;

            op2 = outrec->pts;
            continue;
        }

        op2 = op2->next;

        if (op2 == outrec->pts)
            break;
    }
}

static void v_clean_collinear(VattiScratch& sc, VOutRec* outrec) {

    while (outrec && !outrec->pts)
        outrec = outrec->owner;

    if (!outrec)
        return;

    if (!v_valid_closed(outrec->pts)) {
        outrec->pts = nullptr;

        return;
    }

    VOutPt* start_op = outrec->pts;
    VOutPt* op2 = start_op;
    const size_t count = v_ring_size(start_op);
    const size_t max_iter = (count + 1) * (count + 1);

    for (size_t iter = 0; iter < max_iter; iter++) {
        if (v_is_collinear(op2->prev->pt, op2->pt, op2->next->pt) &&
            (op2->pt == op2->prev->pt || op2->pt == op2->next->pt ||
             v_dot_product(op2->prev->pt, op2->pt, op2->next->pt) < 0)) {
            if (op2 == outrec->pts)
                outrec->pts = op2->prev;

            op2 = v_dispose_outpt(op2);

            if (!v_valid_closed(op2)) {
                outrec->pts = nullptr;

                return;
            }

            start_op = op2;
            continue;
        }

        op2 = op2->next;

        if (op2 == start_op)
            break;
    }

    v_fix_self_intersects(sc, outrec);
}

// ═══════════════════════════════════════════════════════════════════════════
// Sweep
// ═══════════════════════════════════════════════════════════════════════════
static bool v_locmin_less(const VLocalMinima& a, const VLocalMinima& b) {

    if (b.vertex->pt.y != a.vertex->pt.y)
        return b.vertex->pt.y < a.vertex->pt.y;

    return b.vertex->pt.x > a.vertex->pt.x;
}

static bool v_execute_internal(VattiScratch& sc, int cliptype) {

    std::stable_sort(sc.locmin_list.begin(), sc.locmin_list.end(), v_locmin_less);

    for (VLocalMinima& lm : sc.locmin_list)
        v_insert_scanline(sc, lm.vertex->pt.y);

    sc.locmin_idx = 0;

    int64_t y = 0;

    if (!v_pop_scanline(sc, y))
        return true;

    while (sc.succeeded) {
        v_insert_local_minima_into_ael(sc, y, cliptype);
        VActive* e = nullptr;

        while (v_pop_horz(sc, e))
            v_do_horizontal(*e, sc, cliptype);

        if (!sc.horz_seg_list.empty()) {
            v_convert_horz_segs_to_joins(sc);
            sc.horz_seg_list.clear();
        }

        sc.bot_y = y;

        if (!v_pop_scanline(sc, y))
            break;

        if (sc.succeeded && v_build_intersect_list(sc, y)) {
            v_process_intersect_list(sc, cliptype);
            sc.intersect_nodes.clear();
        }

        v_do_top_of_scanbeam(sc, y, cliptype);

        while (v_pop_horz(sc, e))
            v_do_horizontal(*e, sc, cliptype);
    }

    if (sc.succeeded)
        v_process_horz_joins(sc);

    return sc.succeeded;
}

// ═══════════════════════════════════════════════════════════════════════════
// Fast paths and extraction
// ═══════════════════════════════════════════════════════════════════════════
/// Drop a closing point that repeats the first one.
static void v_strip_closing(const double* c, int& n) {

    if (n < 2)
        return;

    double dx = c[(n - 1) * 3] - c[0];
    double dy = c[(n - 1) * 3 + 1] - c[1];

    if (dx * dx + dy * dy < 1e-20)
        --n;
}

/// Integer scale so that (max_coord * scale)^2 fits in int64.
static double v_bool_scale(const double* ca, int na, const double* cb, int nb) {

    double max_coord = 0;

    for (int i = 0; i < na; i++)
        max_coord = std::max(max_coord, std::max(std::abs(ca[i * 3]), std::abs(ca[i * 3 + 1])));

    for (int i = 0; i < nb; i++)
        max_coord = std::max(max_coord, std::max(std::abs(cb[i * 3]), std::abs(cb[i * 3 + 1])));

    if (max_coord < 1e-12)
        max_coord = 1.0;

    return std::floor(std::sqrt(static_cast<double>(std::numeric_limits<int64_t>::max())) / (2.0 * max_coord));
}

/// Result of a boolean when one polygon contains the other or they are disjoint.
static std::vector<Polyline> v_select(const Polyline& a, const Polyline& b, bool a_in_b, bool b_in_a, int clip_type) {

    if (clip_type == 0) {
        if (a_in_b)
            return {a};

        if (b_in_a)
            return {b};

        return {};
    }

    if (clip_type == 1) {
        if (a_in_b)
            return {b};

        if (b_in_a)
            return {a};

        return {a, b};
    }

    if (a_in_b)
        return {};

    return {a};
}

static int v_select_count(int a_count, int b_count, bool a_in_b, bool b_in_a, int clip_type) {

    if (clip_type == 0) {
        if (a_in_b)
            return a_count;

        if (b_in_a)
            return b_count;

        return 0;
    }

    if (clip_type == 1) {
        if (a_in_b)
            return b_count;

        if (b_in_a)
            return a_count;

        return a_count + b_count;
    }

    if (a_in_b)
        return 0;

    return a_count;
}

static void v_bounds(const std::vector<BIVec2>& v, int64_t& min_x, int64_t& max_x, int64_t& min_y, int64_t& max_y) {

    min_x = max_x = v[0].x;
    min_y = max_y = v[0].y;

    for (size_t i = 1; i < v.size(); i++) {
        if (v[i].x < min_x)
            min_x = v[i].x;
        else if (v[i].x > max_x)
            max_x = v[i].x;

        if (v[i].y < min_y)
            min_y = v[i].y;
        else if (v[i].y > max_y)
            max_y = v[i].y;
    }
}

static bool v_any_cross(const std::vector<BIVec2>& va, const std::vector<BIVec2>& vb) {

    int na = (int)va.size();
    int nb = (int)vb.size();

    for (int i = 0; i < na; i++) {
        BIVec2 a1 = va[i];
        BIVec2 a2 = va[(i + 1) % na];
        int64_t axmin = std::min(a1.x, a2.x);
        int64_t axmax = std::max(a1.x, a2.x);
        int64_t aymin = std::min(a1.y, a2.y);
        int64_t aymax = std::max(a1.y, a2.y);

        for (int j = 0; j < nb; j++) {
            BIVec2 b1 = vb[j];
            BIVec2 b2 = vb[(j + 1) % nb];

            if (std::max(b1.x, b2.x) < axmin || std::min(b1.x, b2.x) > axmax || std::max(b1.y, b2.y) < aymin ||
                std::min(b1.y, b2.y) > aymax)
                continue;

            if (v_segs_intersect(a1, a2, b1, b2))
                return true;
        }
    }

    return false;
}

static BIVec2 v_centroid(const std::vector<BIVec2>& v) {

    BIVec2 c{0, 0};

    for (size_t i = 0; i < v.size(); i++) {
        c.x += v[i].x;
        c.y += v[i].y;
    }

    c.x /= (int64_t)v.size();
    c.y /= (int64_t)v.size();

    return c;
}

/// Containment of non-crossing polygons by vertex, centroid and nudged centroid tests.
static void v_contains(const std::vector<BIVec2>& va, const std::vector<BIVec2>& vb, bool& a_in_b, bool& b_in_a) {

    a_in_b = pip_i(va[0], vb);
    b_in_a = pip_i(vb[0], va);
    BIVec2 ca_cen = v_centroid(va);
    BIVec2 cb_cen = v_centroid(vb);

    if (a_in_b && !pip_i(ca_cen, vb))
        a_in_b = false;

    if (b_in_a && !pip_i(cb_cen, va))
        b_in_a = false;

    if (a_in_b || b_in_a)
        return;

    a_in_b = pip_i(ca_cen, vb);
    b_in_a = pip_i(cb_cen, va);

    if (a_in_b || b_in_a)
        return;

    a_in_b = pip_i({ca_cen.x + 1, ca_cen.y + 1}, vb);
    b_in_a = pip_i({cb_cen.x + 1, cb_cen.y + 1}, va);
}

/// Add both inputs through an integer copy, or return the result when they do not cross.
static std::optional<std::vector<Polyline>> v_add_small_paths(
    const Polyline& a,
    const Polyline& b,
    int na,
    int nb,
    VScale sv,
    int clip_type,
    VattiScratch& sc
) {

    std::vector<BIVec2>& va = sc.va;
    va.resize(na);
    std::vector<BIVec2>& vb = sc.vb;
    vb.resize(nb);

    for (int i = 0; i < na; i++)
        va[i] = v_cvt_to_i64(a._coords.data() + i * 3, sv);

    for (int i = 0; i < nb; i++)
        vb[i] = v_cvt_to_i64(b._coords.data() + i * 3, sv);

    int64_t a_min_x = 0;
    int64_t a_max_x = 0;
    int64_t a_min_y = 0;
    int64_t a_max_y = 0;
    int64_t b_min_x = 0;
    int64_t b_max_x = 0;
    int64_t b_min_y = 0;
    int64_t b_max_y = 0;
    v_bounds(va, a_min_x, a_max_x, a_min_y, a_max_y);
    v_bounds(vb, b_min_x, b_max_x, b_min_y, b_max_y);

    if (a_max_x < b_min_x || b_max_x < a_min_x || a_max_y < b_min_y || b_max_y < a_min_y)
        return v_select(a, b, pip_i(va[0], vb), pip_i(vb[0], va), clip_type);

    if (!v_any_cross(va, vb)) {
        bool a_in_b;
        bool b_in_a;
        v_contains(va, vb, a_in_b, b_in_a);

        return v_select(a, b, a_in_b, b_in_a, clip_type);
    }

    v_add_path(va, na, 0, sc);
    v_add_path(vb, nb, 1, sc);

    return std::nullopt;
}

/// Add both inputs straight from doubles, or return the result when their bounds do not overlap.
static std::optional<std::vector<Polyline>> v_add_large_paths(
    const Polyline& a,
    const Polyline& b,
    int na,
    int nb,
    VScale sv,
    int clip_type,
    VattiScratch& sc
) {

    int64_t a_min_x = 0;
    int64_t a_max_x = 0;
    int64_t a_min_y = 0;
    int64_t a_max_y = 0;
    int64_t b_min_x = 0;
    int64_t b_max_x = 0;
    int64_t b_min_y = 0;
    int64_t b_max_y = 0;
    VVertex* va_head = v_add_path_from_doubles(a._coords.data(), na, 0, sv, sc, a_min_x, a_max_x, a_min_y, a_max_y);
    VVertex* vb_head = v_add_path_from_doubles(b._coords.data(), nb, 1, sv, sc, b_min_x, b_max_x, b_min_y, b_max_y);

    if (!va_head || !vb_head)
        return std::vector<Polyline>{};

    if (a_max_x < b_min_x || b_max_x < a_min_x || a_max_y < b_min_y || b_max_y < a_min_y)
        return v_select(a, b, pip_vertex(va_head->pt, vb_head), pip_vertex(vb_head->pt, va_head), clip_type);

    return std::nullopt;
}

/// First output point of a finished ring, or nullptr when the ring is degenerate.
static VOutPt* v_ring_start(VattiScratch& sc, VOutRec* outrec) {

    if (!outrec->pts)
        return nullptr;

    v_clean_collinear(sc, outrec);

    if (!outrec->pts)
        return nullptr;

    VOutPt* op = outrec->pts;

    if (op->next == op || op->next == op->prev || v_very_small_tri(*op))
        return nullptr;

    return op;
}

static std::vector<Polyline> v_extract(VattiScratch& sc, double inv_scale) {

    const VScale isv = v_scale(inv_scale);
    static thread_local std::vector<double> tl_coords;
    std::vector<Polyline> out;

    for (size_t i = 0; i < sc.outrec_list.size(); i++) {
        VOutPt* op = v_ring_start(sc, sc.outrec_list[i]);

        if (!op)
            continue;

        tl_coords.resize(v_ring_size(op) * 3);
        double* dst = tl_coords.data();
        VOutPt* o = op->next;
        BIVec2 last = o->pt;
        v_cvt_to_dbl(dst, last, isv);
        dst += 3;

        for (o = o->next; o != op->next; o = o->next) {
            if (o->pt == last)
                continue;

            last = o->pt;
            v_cvt_to_dbl(dst, last, isv);
            dst += 3;
        }

        int cnt = (int)((dst - tl_coords.data()) / 3);

        if (cnt < 3)
            continue;

        Polyline result;
        result._coords.assign(tl_coords.data(), tl_coords.data() + cnt * 3);
        out.push_back(std::move(result));
    }

    return out;
}

} // anonymous namespace

// ═══════════════════════════════════════════════════════════════════════════
// Boolean operations
// ═══════════════════════════════════════════════════════════════════════════
std::vector<Polyline> session_cpp::BooleanPolyline::compute(const Polyline& a, const Polyline& b, int clip_type) {

    const double* ca = a._coords.data();
    const double* cb = b._coords.data();
    int na = (int)(a._coords.size() / 3);
    int nb = (int)(b._coords.size() / 3);
    v_strip_closing(ca, na);
    v_strip_closing(cb, nb);

    if (na < 3 || nb < 3)
        return {};

    const double bool_scale = v_bool_scale(ca, na, cb, nb);
    const VScale sv = v_scale(bool_scale);
    VattiScratch& sc = vtls;
    sc.reset(na + nb);
    std::optional<std::vector<Polyline>> early;

    if ((int64_t)na * nb <= 400)
        early = v_add_small_paths(a, b, na, nb, sv, clip_type, sc);
    else
        early = v_add_large_paths(a, b, na, nb, sv, clip_type, sc);

    if (early)
        return std::move(*early);

    if (!v_execute_internal(sc, clip_type))
        return {};

    return v_extract(sc, 1.0 / bool_scale);
}

int session_cpp::BooleanPolyline::compute_count(const Polyline& a, const Polyline& b, int clip_type) {

    const double* ca = a._coords.data();
    const double* cb = b._coords.data();
    int na = (int)(a._coords.size() / 3);
    int nb = (int)(b._coords.size() / 3);
    v_strip_closing(ca, na);
    v_strip_closing(cb, nb);

    if (na < 3 || nb < 3)
        return 0;

    const VScale sv = v_scale(v_bool_scale(ca, na, cb, nb));
    VattiScratch& sc = vtls;
    sc.reset(na + nb);
    int64_t a_min_x = 0;
    int64_t a_max_x = 0;
    int64_t a_min_y = 0;
    int64_t a_max_y = 0;
    int64_t b_min_x = 0;
    int64_t b_max_x = 0;
    int64_t b_min_y = 0;
    int64_t b_max_y = 0;
    VVertex* va_head = v_add_path_from_doubles(ca, na, 0, sv, sc, a_min_x, a_max_x, a_min_y, a_max_y);
    VVertex* vb_head = v_add_path_from_doubles(cb, nb, 1, sv, sc, b_min_x, b_max_x, b_min_y, b_max_y);

    if (!va_head || !vb_head)
        return 0;

    if (a_max_x < b_min_x || b_max_x < a_min_x || a_max_y < b_min_y || b_max_y < a_min_y) {
        return v_select_count(
            (int)(a._coords.size() / 3),
            (int)(b._coords.size() / 3),
            pip_vertex(va_head->pt, vb_head),
            pip_vertex(vb_head->pt, va_head),
            clip_type
        );
    }

    if (!v_execute_internal(sc, clip_type))
        return 0;

    int total = 0;

    for (size_t i = 0; i < sc.outrec_list.size(); i++) {
        VOutPt* op = v_ring_start(sc, sc.outrec_list[i]);

        if (!op)
            continue;

        total += (int)v_ring_size(op);
    }

    return total;
}

int session_cpp::BooleanPolyline::compute_raw(
    const double* a_xy,
    int na,
    const double* b_xy,
    int nb,
    int clip_type,
    double* out_xy,
    int max_out
) {

    Polyline a;
    Polyline b;
    a._coords.resize(na * 3);
    b._coords.resize(nb * 3);

    for (int i = 0; i < na; i++) {
        a._coords[i * 3] = a_xy[i * 2];
        a._coords[i * 3 + 1] = a_xy[i * 2 + 1];
        a._coords[i * 3 + 2] = 0.0;
    }

    for (int i = 0; i < nb; i++) {
        b._coords[i * 3] = b_xy[i * 2];
        b._coords[i * 3 + 1] = b_xy[i * 2 + 1];
        b._coords[i * 3 + 2] = 0.0;
    }

    std::vector<Polyline> result = compute(a, b, clip_type);
    int total = 0;

    for (size_t r = 0; r < result.size(); r++) {
        const std::vector<double>& c = result[r]._coords;

        for (size_t i = 0; i < c.size() / 3; i++) {
            if (total < max_out) {
                out_xy[total * 2] = c[i * 3];
                out_xy[total * 2 + 1] = c[i * 3 + 1];
            }

            total++;
        }
    }

    return total;
}

// ═══════════════════════════════════════════════════════════════════════════
// Open subject against closed clip
// ═══════════════════════════════════════════════════════════════════════════
namespace {

/// Even-odd ray cast of (px, py) against the first nc points of cc.
static bool v_point_in_poly(const double* cc, int nc, double px, double py) {

    bool inside = false;

    for (int i = 0, j = nc - 1; i < nc; j = i++) {
        double xi = cc[i * 3];
        double yi = cc[i * 3 + 1];
        double xj = cc[j * 3];
        double yj = cc[j * 3 + 1];

        if ((yi > py) == (yj > py))
            continue;

        double xint = xj + (py - yj) * (xi - xj) / (yi - yj);

        if (px < xint)
            inside = !inside;
    }

    return inside;
}

/// Sorted parameters in (0, 1] where segment (a, b) crosses an edge of the clip.
static std::vector<double> v_crossings(const double* cc, int nc, double ax, double ay, double dx, double dy) {

    std::vector<double> ts;

    for (int i = 0, j = nc - 1; i < nc; j = i++) {
        double ex = cc[i * 3] - cc[j * 3];
        double ey = cc[i * 3 + 1] - cc[j * 3 + 1];
        double denom = dy * ex - dx * ey;

        if (std::fabs(denom) < 1e-18)
            continue;

        double rx = cc[j * 3] - ax;
        double ry = cc[j * 3 + 1] - ay;
        double t = (ry * ex - rx * ey) / denom;
        double u = (ry * dx - rx * dy) / denom;

        if (t > 1e-12 && t <= 1.0 + 1e-12 && u >= -1e-9 && u <= 1.0 + 1e-9)
            ts.push_back(std::min(std::max(t, 0.0), 1.0));
    }

    std::sort(ts.begin(), ts.end());

    return ts;
}

static void v_push_xy(std::vector<double>& cur, double x, double y) {

    size_t n = cur.size();

    if (n >= 3 && std::fabs(cur[n - 3] - x) < 1e-9 && std::fabs(cur[n - 2] - y) < 1e-9)
        return;

    cur.push_back(x);
    cur.push_back(y);
    cur.push_back(0.0);
}

static void v_flush(std::vector<double>& cur, std::vector<Polyline>& result) {

    if (cur.size() >= 6) {
        Polyline p;
        p._coords = cur;
        result.push_back(std::move(p));
    }

    cur.clear();
}

} // anonymous namespace

std::vector<Polyline>
session_cpp::BooleanPolyline::clip_open_against_closed(const Polyline& open_subject, const Polyline& closed_clip) {

    std::vector<Polyline> result;
    const double* cs = open_subject._coords.data();
    const double* cc = closed_clip._coords.data();
    int ns = (int)(open_subject._coords.size() / 3);
    int nc = (int)(closed_clip._coords.size() / 3);
    v_strip_closing(cc, nc);

    if (ns < 2 || nc < 3)
        return result;

    std::vector<double> cur;

    if (v_point_in_poly(cc, nc, cs[0], cs[1]))
        v_push_xy(cur, cs[0], cs[1]);

    for (int si = 0; si + 1 < ns; ++si) {
        double ax = cs[si * 3];
        double ay = cs[si * 3 + 1];
        double bx = cs[(si + 1) * 3];
        double by = cs[(si + 1) * 3 + 1];
        double dx = bx - ax;
        double dy = by - ay;
        std::vector<double> ts = v_crossings(cc, nc, ax, ay, dx, dy);
        double prev_t = 0.0;

        for (size_t k = 0; k < ts.size(); k++) {
            double t = ts[k];

            if (t - prev_t < 1e-12) {
                prev_t = t;
                continue;
            }

            double mid_t = 0.5 * (prev_t + t);
            v_push_xy(cur, ax + dx * t, ay + dy * t);

            if (v_point_in_poly(cc, nc, ax + dx * mid_t, ay + dy * mid_t))
                v_flush(cur, result);

            prev_t = t;
        }

        if (prev_t >= 1.0 - 1e-12)
            continue;

        double mid_t = 0.5 * (prev_t + 1.0);

        if (v_point_in_poly(cc, nc, ax + dx * mid_t, ay + dy * mid_t))
            v_push_xy(cur, bx, by);
    }

    v_flush(cur, result);

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Ring sets
// ═══════════════════════════════════════════════════════════════════════════
namespace {

/// Signed xy area of the first count points of flat coordinates, positive counter-clockwise.
static double v_ring_area(const double* coords, int count) {

    double area = 0.0;

    for (int i = 0; i < count; i++)
        area += coords[i * 3] * coords[((i + 1) % count) * 3 + 1] - coords[((i + 1) % count) * 3] * coords[i * 3 + 1];

    return area / 2.0;
}

/// The rings of one operand as flat coordinates without closing points, outer counter-clockwise and holes clockwise by how many other rings hold a point just inside each.
static std::vector<std::vector<double>> v_oriented(const std::vector<Polyline>& rings) {

    std::vector<std::vector<double>> flat;

    for (const Polyline& ring : rings) {
        int count = (int)(ring._coords.size() / 3);
        v_strip_closing(ring._coords.data(), count);

        if (count >= 3)
            flat.emplace_back(ring._coords.begin(), ring._coords.begin() + count * 3);
    }

    std::vector<std::vector<double>> oriented = flat;

    for (size_t i = 0; i < flat.size(); i++) {
        const int count = (int)(flat[i].size() / 3);
        const double area = v_ring_area(flat[i].data(), count);
        const double dx = flat[i][3] - flat[i][0];
        const double dy = flat[i][4] - flat[i][1];
        const double side = area > 0.0 ? Tolerance::RELATIVE : -Tolerance::RELATIVE;
        const double px = (flat[i][0] + flat[i][3]) * 0.5 - dy * side;
        const double py = (flat[i][1] + flat[i][4]) * 0.5 + dx * side;
        int depth = 0;

        for (size_t j = 0; j < flat.size(); j++)
            depth += j != i && v_point_in_poly(flat[j].data(), (int)(flat[j].size() / 3), px, py) ? 1 : 0;

        if ((area > 0.0) == (depth % 2 == 0))
            continue;

        for (int k = 0; k < count; k++)
            for (int axis = 0; axis < 3; axis++)
                oriented[i][k * 3 + axis] = flat[i][(count - 1 - k) * 3 + axis];
    }

    return oriented;
}

} // anonymous namespace

std::vector<Polyline> session_cpp::BooleanPolyline::compute_regions(const std::vector<Polyline>& a, const std::vector<Polyline>& b, int clip_type) {

    const std::vector<std::vector<double>> rings_a = v_oriented(a);
    const std::vector<std::vector<double>> rings_b = v_oriented(b);
    std::vector<double> ca;
    std::vector<double> cb;

    for (const std::vector<double>& ring : rings_a)
        ca.insert(ca.end(), ring.begin(), ring.end());

    for (const std::vector<double>& ring : rings_b)
        cb.insert(cb.end(), ring.begin(), ring.end());

    const double bool_scale = v_bool_scale(ca.data(), (int)(ca.size() / 3), cb.data(), (int)(cb.size() / 3));
    VattiScratch& sc = vtls;
    sc.reset(ca.size() / 3 + cb.size() / 3);
    int64_t min_x = 0;
    int64_t max_x = 0;
    int64_t min_y = 0;
    int64_t max_y = 0;

    for (const std::vector<double>& ring : rings_a)
        v_add_path_from_doubles(ring.data(), (int)(ring.size() / 3), 0, v_scale(bool_scale), sc, min_x, max_x, min_y, max_y);

    for (const std::vector<double>& ring : rings_b)
        v_add_path_from_doubles(ring.data(), (int)(ring.size() / 3), 1, v_scale(bool_scale), sc, min_x, max_x, min_y, max_y);

    if (!v_execute_internal(sc, clip_type))
        return {};

    std::vector<Polyline> rings = v_extract(sc, 1.0 / bool_scale);

    for (Polyline& ring : rings)
        ring.add_point(ring.get_point(0));

    return rings;
}
