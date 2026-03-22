#include "trimmedsurface.h"
#include "closest.h"
#include "primitives.h"
#include "remesh_nurbssurface_grid.h"
#include "triangulation_2d.h"
#include "fmt/core.h"
#include <fstream>
#include <set>
#include "trimmedsurface.pb.h"

namespace session_cpp {
namespace {

// ---- FlatMap64: open-addressing hash map with uint64_t keys ----

template<typename V>
class FlatMap64 {
    static constexpr uint64_t EMPTY_KEY = ~uint64_t(0);
    struct Slot { uint64_t key = EMPTY_KEY; V value; };
    std::vector<Slot> slots_;
    size_t size_ = 0;
    size_t shift_ = 64;

    static size_t log2_pot(size_t n) {
        size_t r = 0; while ((size_t(1) << r) < n) ++r; return r;
    }
    size_t probe(uint64_t key) const {
        return (size_t)((key * uint64_t(0x9E3779B97F4A7C15ULL)) >> shift_);
    }
    void grow() {
        size_t new_cap = slots_.empty() ? 16 : slots_.size() * 2;
        std::vector<Slot> old = std::move(slots_);
        slots_.assign(new_cap, Slot{EMPTY_KEY, V{}});
        shift_ = 64 - log2_pot(new_cap);
        size_ = 0;
        for (auto& s : old) if (s.key != EMPTY_KEY) insert_impl(s.key, std::move(s.value));
    }
    void insert_impl(uint64_t key, V val) {
        size_t mask = slots_.size() - 1, i = probe(key);
        while (slots_[i].key != EMPTY_KEY) {
            if (slots_[i].key == key) { slots_[i].value = std::move(val); return; }
            i = (i + 1) & mask;
        }
        slots_[i].key = key; slots_[i].value = std::move(val); ++size_;
    }
public:
    FlatMap64() = default;
    void reserve(size_t n) {
        size_t need = n * 2;
        if (need <= slots_.size()) return;
        size_t cap = 16; while (cap < need) cap *= 2;
        std::vector<Slot> old = std::move(slots_);
        slots_.assign(cap, Slot{EMPTY_KEY, V{}});
        shift_ = 64 - log2_pot(cap); size_ = 0;
        for (auto& s : old) if (s.key != EMPTY_KEY) insert_impl(s.key, std::move(s.value));
    }
    V* find(uint64_t key) {
        if (slots_.empty()) return nullptr;
        size_t mask = slots_.size() - 1, i = probe(key);
        while (slots_[i].key != EMPTY_KEY) {
            if (slots_[i].key == key) return &slots_[i].value;
            i = (i + 1) & mask;
        }
        return nullptr;
    }
    const V* find(uint64_t key) const {
        if (slots_.empty()) return nullptr;
        size_t mask = slots_.size() - 1, i = probe(key);
        while (slots_[i].key != EMPTY_KEY) {
            if (slots_[i].key == key) return &slots_[i].value;
            i = (i + 1) & mask;
        }
        return nullptr;
    }
    V& operator[](uint64_t key) {
        if (size_ * 2 >= slots_.size()) grow();
        size_t mask = slots_.size() - 1, i = probe(key);
        while (slots_[i].key != EMPTY_KEY) {
            if (slots_[i].key == key) return slots_[i].value;
            i = (i + 1) & mask;
        }
        slots_[i].key = key; slots_[i].value = V{}; ++size_;
        return slots_[i].value;
    }
    void erase(uint64_t key) {
        if (slots_.empty()) return;
        size_t mask = slots_.size() - 1, i = probe(key);
        while (slots_[i].key != EMPTY_KEY) {
            if (slots_[i].key == key) {
                --size_; size_t j = i;
                while (true) {
                    j = (j + 1) & mask;
                    if (slots_[j].key == EMPTY_KEY) break;
                    size_t k = probe(slots_[j].key);
                    bool move = (i <= j) ? (k <= i || k > j) : (k <= i && k > j);
                    if (move) { slots_[i] = std::move(slots_[j]); i = j; }
                }
                slots_[i].key = EMPTY_KEY; return;
            }
            i = (i + 1) & mask;
        }
    }
    void clear() { for (auto& s : slots_) s.key = EMPTY_KEY; size_ = 0; }
    size_t size() const { return size_; }
};

// ---- Delaunay2D: Bowyer-Watson CDT in 2D UV space ----

struct Vertex2D { double x = 0.0, y = 0.0; };

struct Triangle {
    int  v[3]           = {-1, -1, -1};
    int  adj[3]         = {-1, -1, -1};
    bool constrained[3] = {false, false, false};
    bool alive = true;
};

class Delaunay2D {
public:
    std::vector<Vertex2D> vertices;
    std::vector<Triangle> triangles;
    int super_v[3] = {-1, -1, -1};
    FlatMap64<std::pair<int,int>> edge_map;
    int last_found_ = 0;

    static uint64_t edge_key(int a, int b) {
        return ((uint64_t)(uint32_t)std::min(a,b) << 32) | (uint64_t)(uint32_t)std::max(a,b);
    }

    Delaunay2D(double xmin, double ymin, double xmax, double ymax);
    int  insert(double x, double y);
    void insert_constraint(int v0, int v1);
    void cleanup();
    std::vector<std::array<int,3>> get_triangles() const;

    static double in_circumcircle(double ax, double ay, double bx, double by,
                                  double cx, double cy, double dx, double dy);
    static double orient2d(double ax, double ay, double bx, double by, double cx, double cy);
    static void circumcenter(double ax, double ay, double bx, double by,
                             double cx, double cy, double& ux, double& uy);
    int locate(double x, double y, int start_tri = 0) const;

private:
    int visit_epoch_ = 0;
    std::vector<int> visit_stamp_;
    mutable std::vector<int> bad_;
    struct BEdge { int e0, e1; bool constrained = false; };
    mutable std::vector<BEdge> polygon_;
    void register_triangle_edges(int ti);
    void unregister_triangle_edges(int ti);
};

double Delaunay2D::in_circumcircle(double ax, double ay, double bx, double by,
                                    double cx, double cy, double dx, double dy) {
    double adx = ax-dx, ady = ay-dy, bdx = bx-dx, bdy = by-dy, cdx = cx-dx, cdy = cy-dy;
    return (adx*adx+ady*ady)*(bdx*cdy-cdx*bdy)
         + (bdx*bdx+bdy*bdy)*(cdx*ady-adx*cdy)
         + (cdx*cdx+cdy*cdy)*(adx*bdy-bdx*ady);
}

double Delaunay2D::orient2d(double ax, double ay, double bx, double by, double cx, double cy) {
    return (bx-ax)*(cy-ay)-(by-ay)*(cx-ax);
}

void Delaunay2D::circumcenter(double ax, double ay, double bx, double by,
                               double cx, double cy, double& ux, double& uy) {
    double D = 2.0*(ax*(by-cy)+bx*(cy-ay)+cx*(ay-by));
    if (std::abs(D) < 1e-30) { ux=(ax+bx+cx)/3.0; uy=(ay+by+cy)/3.0; return; }
    double a2=ax*ax+ay*ay, b2=bx*bx+by*by, c2=cx*cx+cy*cy;
    ux=(a2*(by-cy)+b2*(cy-ay)+c2*(ay-by))/D;
    uy=(a2*(cx-bx)+b2*(ax-cx)+c2*(bx-ax))/D;
}

void Delaunay2D::register_triangle_edges(int ti) {
    const Triangle& t = triangles[ti];
    for (int k = 0; k < 3; ++k) {
        int a = t.v[(k+1)%3], b = t.v[(k+2)%3];
        auto key = edge_key(a, b);
        auto* val = edge_map.find(key);
        if (val) {
            auto [oti, ok] = *val;
            triangles[ti].adj[k] = oti; triangles[oti].adj[ok] = ti;
            edge_map.erase(key);
        } else { edge_map[key] = {ti, k}; }
    }
}

void Delaunay2D::unregister_triangle_edges(int ti) {
    const Triangle& t = triangles[ti];
    for (int k = 0; k < 3; ++k) {
        int a = t.v[(k+1)%3], b = t.v[(k+2)%3];
        auto key = edge_key(a, b);
        int adj_ti = t.adj[k];
        if (adj_ti >= 0 && triangles[adj_ti].alive) {
            Triangle& adj = triangles[adj_ti];
            for (int kk = 0; kk < 3; ++kk)
                if (adj.adj[kk] == ti) {
                    adj.adj[kk] = -1;
                    edge_map[edge_key(adj.v[(kk+1)%3], adj.v[(kk+2)%3])] = {adj_ti, kk};
                    break;
                }
        } else {
            auto* val = edge_map.find(key);
            if (val && val->first == ti) edge_map.erase(key);
        }
    }
}

Delaunay2D::Delaunay2D(double xmin, double ymin, double xmax, double ymax) {
    double dx = xmax-xmin, dy = ymax-ymin, d = std::max(dx,dy);
    double cx = (xmin+xmax)*0.5, cy = (ymin+ymax)*0.5, scale = 20.0;
    vertices.push_back({cx-scale*d, cy-scale*d});
    vertices.push_back({cx+scale*d, cy-scale*d});
    vertices.push_back({cx, cy+scale*d});
    super_v[0]=0; super_v[1]=1; super_v[2]=2;
    triangles.reserve(4096); vertices.reserve(2048); edge_map.reserve(2048);
    visit_stamp_.reserve(4096); bad_.reserve(32); polygon_.reserve(32);
    Triangle t; t.v[0]=0; t.v[1]=1; t.v[2]=2; t.adj[0]=t.adj[1]=t.adj[2]=-1; t.alive=true;
    triangles.push_back(t); register_triangle_edges(0);
}

int Delaunay2D::locate(double x, double y, int start_tri) const {
    if (start_tri < 0 || start_tri >= (int)triangles.size() || !triangles[start_tri].alive) {
        start_tri = (int)triangles.size()-1;
        while (start_tri >= 0 && !triangles[start_tri].alive) --start_tri;
        if (start_tri < 0) return -1;
    }
    int cur = start_tri, max_iter = (int)triangles.size();
    for (int iter = 0; iter < max_iter; ++iter) {
        const Triangle& tri = triangles[cur]; bool moved = false;
        for (int k = 0; k < 3; ++k) {
            int a = tri.v[k], b = tri.v[(k+1)%3];
            if (orient2d(vertices[a].x,vertices[a].y,vertices[b].x,vertices[b].y,x,y) < 0) {
                int opp = (k+2)%3;
                if (tri.adj[opp]>=0 && triangles[tri.adj[opp]].alive)
                    { cur = tri.adj[opp]; moved = true; break; }
            }
        }
        if (!moved) return cur;
    }
    return cur;
}

int Delaunay2D::insert(double x, double y) {
    int start = locate(x, y, last_found_);
    if (start >= 0 && triangles[start].alive)
        for (int k = 0; k < 3; ++k) {
            int vi2 = triangles[start].v[k];
            double ddx = vertices[vi2].x-x, ddy = vertices[vi2].y-y;
            if (ddx*ddx+ddy*ddy < 1e-12) return vi2;
        }
    int vi = (int)vertices.size();
    vertices.push_back({x, y});
    ++visit_epoch_;
    if ((int)visit_stamp_.size() < (int)triangles.size()+64)
        visit_stamp_.resize(triangles.size()+64, 0);
    bad_.clear();
    bad_.push_back(start); visit_stamp_[start] = visit_epoch_;
    int bfs_front = 0;
    while (bfs_front < (int)bad_.size()) {
        int ti = bad_[bfs_front++];
        const Triangle& tri = triangles[ti];
        if (!tri.alive) { bad_[bfs_front-1] = -1; continue; }
        const Vertex2D& a=vertices[tri.v[0]], &b=vertices[tri.v[1]], &c=vertices[tri.v[2]];
        double o = orient2d(a.x,a.y,b.x,b.y,c.x,c.y);
        double ic = (o>0) ? in_circumcircle(a.x,a.y,b.x,b.y,c.x,c.y,x,y)
                          : in_circumcircle(a.x,a.y,c.x,c.y,b.x,b.y,x,y);
        if (ic > 0) {
            for (int k = 0; k < 3; ++k) {
                if (tri.constrained[k]) continue;
                int nb = tri.adj[k];
                if (nb>=0 && nb<(int)visit_stamp_.size() && visit_stamp_[nb]!=visit_epoch_)
                    { visit_stamp_[nb]=visit_epoch_; bad_.push_back(nb); }
            }
        } else { bad_[bfs_front-1] = -1; }
    }
    int write = 0;
    for (int i = 0; i < (int)bad_.size(); ++i) if (bad_[i]>=0) bad_[write++]=bad_[i];
    bad_.resize(write);
    if (bad_.empty()) { vertices.pop_back(); return -1; }
    polygon_.clear();
    for (int ti : bad_) {
        const Triangle& tri = triangles[ti];
        for (int k = 0; k < 3; ++k) {
            int nb = tri.adj[k]; bool nb_bad = false;
            if (nb >= 0) for (int bi : bad_) if (bi==nb) { nb_bad=true; break; }
            if (!nb_bad) polygon_.push_back({tri.v[(k+1)%3], tri.v[(k+2)%3], tri.constrained[k]});
        }
    }
    for (int ti : bad_) { unregister_triangle_edges(ti); triangles[ti].alive=false; }
    for (const auto& edge : polygon_) {
        double o = orient2d(vertices[vi].x,vertices[vi].y,
                            vertices[edge.e0].x,vertices[edge.e0].y,
                            vertices[edge.e1].x,vertices[edge.e1].y);
        if (std::abs(o) < 1e-20) continue;
        int new_ti = (int)triangles.size();
        Triangle nt; nt.v[0]=vi;
        if (o>0) { nt.v[1]=edge.e0; nt.v[2]=edge.e1; }
        else     { nt.v[1]=edge.e1; nt.v[2]=edge.e0; }
        nt.constrained[0]=edge.constrained; nt.adj[0]=nt.adj[1]=nt.adj[2]=-1; nt.alive=true;
        triangles.push_back(nt); register_triangle_edges(new_ti);
    }
    last_found_ = (int)triangles.size()-1;
    return vi;
}

void Delaunay2D::insert_constraint(int v0, int v1) {
    if (v0 == v1) return;
    for (int ti = 0; ti < (int)triangles.size(); ++ti) {
        if (!triangles[ti].alive) continue;
        for (int k = 0; k < 3; ++k) {
            int e0=triangles[ti].v[(k+1)%3], e1=triangles[ti].v[(k+2)%3];
            if ((e0==v0&&e1==v1)||(e0==v1&&e1==v0)) {
                triangles[ti].constrained[k] = true;
                int nb = triangles[ti].adj[k];
                if (nb>=0&&triangles[nb].alive)
                    for (int kk=0;kk<3;++kk)
                        if (triangles[nb].adj[kk]==ti) { triangles[nb].constrained[kk]=true; break; }
                return;
            }
        }
    }
    int start_ti = -1;
    for (int i=(int)triangles.size()-1;i>=0;--i)
        if (triangles[i].alive)
            for (int k=0;k<3;++k)
                if (triangles[i].v[k]==v0) { start_ti=i; break; }
    if (start_ti < 0) return;
    const Vertex2D& a=vertices[v0], &b=vertices[v1];
    int iVL=-1,iVR=-1,iT=-1;
    {
        int ti=start_ti, guard=(int)triangles.size()+4;
        do {
            if (!triangles[ti].alive) break;
            const Triangle& t=triangles[ti];
            int k=-1; for (int i=0;i<3;++i) if (t.v[i]==v0) { k=i; break; }
            if (k<0) break;
            int iP2=t.v[(k+1)%3], iP1=t.v[(k+2)%3];
            double oP2=orient2d(a.x,a.y,b.x,b.y,vertices[iP2].x,vertices[iP2].y);
            double oP1=orient2d(a.x,a.y,b.x,b.y,vertices[iP1].x,vertices[iP1].y);
            if (oP2<0&&oP1>=0) { iVL=iP1; iVR=iP2; iT=ti; break; }
            int next=t.adj[(k+1)%3];
            if (next<0||!triangles[next].alive) break;
            ti=next;
        } while (ti!=start_ti&&--guard>0);
    }
    if (iT < 0) return;
    std::vector<int> polyL={v0,iVL}, polyR={v0,iVR}, intersected={iT};
    int iV=v0, cur_iT=iT, guard=(int)triangles.size()*2+8;
    auto tri_has = [&](int ti, int v) {
        return triangles[ti].v[0]==v||triangles[ti].v[1]==v||triangles[ti].v[2]==v;
    };
    while (!tri_has(cur_iT, v1) && --guard > 0) {
        const Triangle& t=triangles[cur_iT];
        int k_iV=-1; for (int i=0;i<3;++i) if (t.v[i]==iV) { k_iV=i; break; }
        if (k_iV<0) break;
        int iTopo=t.adj[k_iV];
        if (iTopo<0||!triangles[iTopo].alive) break;
        int iVopo=-1;
        for (int k=0;k<3;++k) if (triangles[iTopo].adj[k]==cur_iT) { iVopo=triangles[iTopo].v[k]; break; }
        if (iVopo<0) break;
        double o=orient2d(a.x,a.y,b.x,b.y,vertices[iVopo].x,vertices[iVopo].y);
        if (o<0) { if (iVopo!=v1) polyR.push_back(iVopo); iV=iVR; iVR=iVopo; }
        else     { if (iVopo!=v1) polyL.push_back(iVopo); iV=iVL; iVL=iVopo; }
        intersected.push_back(iTopo); cur_iT=iTopo;
    }
    polyL.push_back(v1); polyR.push_back(v1);
    for (int ti : intersected) { unregister_triangle_edges(ti); triangles[ti].alive=false; }
    int first_new=(int)triangles.size();
    auto emit=[&](int pa,int pb,int pc) {
        double o=orient2d(vertices[pa].x,vertices[pa].y,vertices[pb].x,vertices[pb].y,
                          vertices[pc].x,vertices[pc].y);
        if (std::abs(o)<1e-20) return;
        Triangle nt; nt.v[0]=pa;
        if (o>0){nt.v[1]=pb;nt.v[2]=pc;}else{nt.v[1]=pc;nt.v[2]=pb;}
        nt.adj[0]=nt.adj[1]=nt.adj[2]=-1; nt.constrained[0]=nt.constrained[1]=nt.constrained[2]=false;
        nt.alive=true; int new_ti=(int)triangles.size(); triangles.push_back(nt);
        register_triangle_edges(new_ti);
    };
    { int apex=v1; for (int i=0;i+2<(int)polyL.size();++i) emit(apex,polyL[i+1],polyL[i]); }
    { int apex=v0; for (int i=1;i+1<(int)polyR.size();++i) emit(apex,polyR[i],polyR[i+1]); }
    for (int new_ti=first_new;new_ti<(int)triangles.size();++new_ti) {
        if (!triangles[new_ti].alive) continue;
        Triangle& nt=triangles[new_ti];
        for (int k=0;k<3;++k) {
            int nb=nt.adj[k];
            if (nb<0||nb>=first_new||!triangles[nb].alive) continue;
            Triangle& nb_t=triangles[nb];
            for (int kk=0;kk<3;++kk)
                if (nb_t.adj[kk]==new_ti&&nb_t.constrained[kk]) { nt.constrained[k]=true; break; }
        }
    }
    // mark constrained flag on the new shared edge
    for (auto& tri : triangles) {
        if (!tri.alive) continue;
        for (int k=0;k<3;++k) {
            int e0=tri.v[(k+1)%3],e1=tri.v[(k+2)%3];
            if ((e0==v0&&e1==v1)||(e0==v1&&e1==v0)) tri.constrained[k]=true;
        }
    }
}

void Delaunay2D::cleanup() {
    for (auto& tri : triangles) {
        if (!tri.alive) continue;
        for (int k=0;k<3;++k)
            if (tri.v[k]==super_v[0]||tri.v[k]==super_v[1]||tri.v[k]==super_v[2])
                { unregister_triangle_edges((int)(&tri-&triangles[0])); tri.alive=false; break; }
    }
    last_found_=0;
    for (int i=0;i<(int)triangles.size();++i) if (triangles[i].alive) { last_found_=i; break; }
}

std::vector<std::array<int,3>> Delaunay2D::get_triangles() const {
    std::vector<std::array<int,3>> result;
    for (const auto& tri : triangles) {
        if (!tri.alive) continue;
        double o=orient2d(vertices[tri.v[0]].x,vertices[tri.v[0]].y,
                          vertices[tri.v[1]].x,vertices[tri.v[1]].y,
                          vertices[tri.v[2]].x,vertices[tri.v[2]].y);
        result.push_back(o>0 ? std::array<int,3>{tri.v[0],tri.v[1],tri.v[2]}
                              : std::array<int,3>{tri.v[0],tri.v[2],tri.v[1]});
    }
    return result;
}

} // anonymous namespace

///////////////////////////////////////////////////////////////////////////////////////////
// Constructors
///////////////////////////////////////////////////////////////////////////////////////////

TrimmedSurface::TrimmedSurface() {}

TrimmedSurface::TrimmedSurface(const TrimmedSurface& other) {
    deep_copy_from(other);
}

TrimmedSurface& TrimmedSurface::operator=(const TrimmedSurface& other) {
    if (this != &other) deep_copy_from(other);
    return *this;
}

bool TrimmedSurface::operator==(const TrimmedSurface& other) const {
    if (name != other.name) return false;
    if (width != other.width) return false;
    if (surfacecolor != other.surfacecolor) return false;
    if (xform != other.xform) return false;
    if (m_surface != other.m_surface) return false;
    return true;
}

bool TrimmedSurface::operator!=(const TrimmedSurface& other) const {
    return !(*this == other);
}

TrimmedSurface::~TrimmedSurface() {}

void TrimmedSurface::deep_copy_from(const TrimmedSurface& src) {
    guid = ::guid();
    name = src.name;
    width = src.width;
    surfacecolor = src.surfacecolor;
    xform = src.xform;
    m_surface = src.m_surface;
    m_outer_loop = src.m_outer_loop;
    m_inner_loops = src.m_inner_loops;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Static Factory Methods
///////////////////////////////////////////////////////////////////////////////////////////

TrimmedSurface TrimmedSurface::create(const NurbsSurface& surface, const NurbsCurve& outer_loop) {
    TrimmedSurface ts;
    ts.m_surface = surface;
    ts.m_outer_loop = outer_loop;
    return ts;
}

TrimmedSurface TrimmedSurface::create_planar(const NurbsCurve& boundary) {
    NurbsSurface srf = Primitives::create_planar(boundary);
    if (!srf.is_valid()) return TrimmedSurface();

    Point p00 = srf.get_cv(0, 0);
    Point p10 = srf.get_cv(1, 0);
    Point p01 = srf.get_cv(0, 1);
    Vector u_axis(p10[0]-p00[0], p10[1]-p00[1], p10[2]-p00[2]);
    Vector v_axis(p01[0]-p00[0], p01[1]-p00[1], p01[2]-p00[2]);
    double u_len2 = u_axis[0]*u_axis[0] + u_axis[1]*u_axis[1] + u_axis[2]*u_axis[2];
    double v_len2 = v_axis[0]*v_axis[0] + v_axis[1]*v_axis[1] + v_axis[2]*v_axis[2];
    if (u_len2 < 1e-28 || v_len2 < 1e-28) return TrimmedSurface();

    auto project_to_uv = [&](const Point& pt) -> Point {
        double dx = pt[0]-p00[0], dy = pt[1]-p00[1], dz = pt[2]-p00[2];
        double nu = (dx*u_axis[0] + dy*u_axis[1] + dz*u_axis[2]) / u_len2;
        double nv = (dx*v_axis[0] + dy*v_axis[1] + dz*v_axis[2]) / v_len2;
        return Point(nu, nv, 0.0);
    };

    std::vector<Point> uv_pts;
    if (boundary.degree() <= 1) {
        for (int i = 0; i < boundary.cv_count(); ++i)
            uv_pts.push_back(project_to_uv(boundary.get_cv(i)));
    } else {
        auto spans = boundary.get_span_vector();
        for (size_t si = 0; si + 1 < spans.size(); ++si) {
            int n_sub = 10;
            for (int k = 0; k <= n_sub; ++k) {
                double t = spans[si] + (spans[si+1] - spans[si]) * k / n_sub;
                Point uv = project_to_uv(boundary.point_at(t));
                if (uv_pts.empty() || (uv[0]-uv_pts.back()[0])*(uv[0]-uv_pts.back()[0]) +
                    (uv[1]-uv_pts.back()[1])*(uv[1]-uv_pts.back()[1]) > 1e-24)
                    uv_pts.push_back(uv);
            }
        }
    }

    TrimmedSurface ts;
    ts.m_surface = srf;
    if (uv_pts.size() >= 3)
        ts.m_outer_loop = NurbsCurve::create(false, 1, uv_pts);
    return ts;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////////////////

NurbsSurface TrimmedSurface::surface() const { return m_surface; }
NurbsCurve TrimmedSurface::get_outer_loop() const { return m_outer_loop; }
void TrimmedSurface::set_outer_loop(const NurbsCurve& loop) { m_outer_loop = loop; }
bool TrimmedSurface::is_trimmed() const { return m_outer_loop.is_valid(); }
bool TrimmedSurface::is_valid() const { return m_surface.is_valid(); }

///////////////////////////////////////////////////////////////////////////////////////////
// Inner Loops
///////////////////////////////////////////////////////////////////////////////////////////

void TrimmedSurface::add_inner_loop(const NurbsCurve& loop_2d) {
    m_inner_loops.push_back(loop_2d);
}

void TrimmedSurface::add_hole(const NurbsCurve& curve_3d) {
    auto dom = curve_3d.domain();
    auto sdom_u = m_surface.domain(0);
    auto sdom_v = m_surface.domain(1);
    double range_u = sdom_u.second - sdom_u.first;
    double range_v = sdom_v.second - sdom_v.first;
    int n_samples = std::max(curve_3d.cv_count() * 4, 32);
    std::vector<Point> uv_pts;
    for (int i = 0; i < n_samples; ++i) {
        double t = dom.first + (dom.second - dom.first) * i / n_samples;
        Point pt3d = curve_3d.point_at(t);
        auto [u, v, dist] = Closest::surface_point(m_surface, pt3d);
        double nu = (u - sdom_u.first) / range_u;
        double nv = (v - sdom_v.first) / range_v;
        uv_pts.push_back(Point(nu, nv, 0.0));
    }
    if (uv_pts.size() >= 3)
        m_inner_loops.push_back(NurbsCurve::create(true, 1, uv_pts));
}

void TrimmedSurface::add_holes(const std::vector<NurbsCurve>& curves_3d) {
    for (const auto& crv : curves_3d) add_hole(crv);
}

NurbsCurve TrimmedSurface::get_inner_loop(int index) const { return m_inner_loops[index]; }
int TrimmedSurface::inner_loop_count() const { return static_cast<int>(m_inner_loops.size()); }
void TrimmedSurface::clear_inner_loops() { m_inner_loops.clear(); }

///////////////////////////////////////////////////////////////////////////////////////////
// Evaluation
///////////////////////////////////////////////////////////////////////////////////////////

Point TrimmedSurface::point_at(double u, double v) const { return m_surface.point_at(u, v); }
Vector TrimmedSurface::normal_at(double u, double v) const { return m_surface.normal_at(u, v); }

///////////////////////////////////////////////////////////////////////////////////////////
// Meshing
///////////////////////////////////////////////////////////////////////////////////////////

Mesh TrimmedSurface::mesh() const {
    if (!is_trimmed()) return m_surface.mesh();

    bool planar = m_surface.is_planar();

    // Planar: control points only (no subdivision) — minimal CDT triangulation
    // Non-planar: curvature-adaptive sampling to capture boundary shape
    auto disc_loop = [&](const NurbsCurve& crv) -> std::vector<Point> {
        std::vector<Point> pts;
        if (crv.degree() <= 1 && !crv.is_rational()) {
            for (int i = 0; i < crv.cv_count(); ++i) pts.push_back(crv.get_cv(i));
        } else {
            int n = std::max(crv.cv_count() * 4, 16);
            auto [sampled, params] = crv.divide_by_count(n);
            pts = sampled;
        }
        while (pts.size() > 1) {
            double dx = pts.front()[0] - pts.back()[0];
            double dy = pts.front()[1] - pts.back()[1];
            if (dx*dx + dy*dy < 1e-20) pts.pop_back();
            else break;
        }
        return pts;
    };

    auto outer_uv = disc_loop(m_outer_loop);
    std::vector<std::vector<Point>> hole_uvs;
    for (const auto& inner : m_inner_loops)
        hole_uvs.push_back(disc_loop(inner));
    if (outer_uv.size() < 3) return m_surface.mesh();

    // UV bounding box from outer loop
    double bb_umin = 1e30, bb_vmin = 1e30, bb_umax = -1e30, bb_vmax = -1e30;
    for (const auto& p : outer_uv) {
        if (p[0] < bb_umin) bb_umin = p[0];
        if (p[1] < bb_vmin) bb_vmin = p[1];
        if (p[0] > bb_umax) bb_umax = p[0];
        if (p[1] > bb_vmax) bb_vmax = p[1];
    }

    // Flat coords for point_in_polygon_2d
    auto to_flat = [](const std::vector<Point>& pts) -> std::vector<double> {
        std::vector<double> c;
        c.reserve(pts.size() * 2);
        for (const auto& p : pts) { c.push_back(p[0]); c.push_back(p[1]); }
        return c;
    };
    auto outer_coords = to_flat(outer_uv);
    std::vector<std::vector<double>> hole_coords;
    for (const auto& hp : hole_uvs) hole_coords.push_back(to_flat(hp));

    auto inside_trim = [&](double u, double v) -> bool {
        if (!Triangulation2D::point_in_polygon_2d(u, v, outer_coords)) return false;
        for (const auto& hc : hole_coords)
            if (Triangulation2D::point_in_polygon_2d(u, v, hc)) return false;
        return true;
    };

    // Create CDT
    Delaunay2D dt(bb_umin, bb_vmin, bb_umax, bb_vmax);

    // Insert boundary vertices + constrain edges
    auto insert_loop = [&](const std::vector<Point>& pts) {
        std::vector<int> vis;
        vis.reserve(pts.size());
        for (const auto& p : pts) vis.push_back(dt.insert(p[0], p[1]));
        for (size_t i = 0; i < vis.size(); ++i) {
            size_t j = (i + 1) % vis.size();
            if (vis[i] >= 0 && vis[j] >= 0 && vis[i] != vis[j])
                dt.insert_constraint(vis[i], vis[j]);
        }
    };
    insert_loop(outer_uv);
    for (const auto& hp : hole_uvs) insert_loop(hp);

    // Interior Steiner points — non-planar only (span-adaptive grid filtered by trim)
    // Planar uses control-point boundary only: CDT produces minimal triangulation directly
    if (!planar) {
        // Curvature-adaptive span subdivision
        auto usp = m_surface.get_span_vector(0);
        auto vsp = m_surface.get_span_vector(1);
        int deg_u = m_surface.degree(0), deg_v = m_surface.degree(1);
        int ns_u = (int)usp.size() - 1, ns_v = (int)vsp.size() - 1;

        // 3D bbox diagonal for chord tolerance
        double bmin[3] = {1e30, 1e30, 1e30}, bmax[3] = {-1e30, -1e30, -1e30};
        for (int i = 0; i < m_surface.cv_count(0); ++i)
            for (int j = 0; j < m_surface.cv_count(1); ++j) {
                Point p = m_surface.get_cv(i, j);
                for (int k = 0; k < 3; ++k) {
                    if (p[k] < bmin[k]) bmin[k] = p[k];
                    if (p[k] > bmax[k]) bmax[k] = p[k];
                }
            }
        double bbox_diag = std::sqrt((bmax[0]-bmin[0])*(bmax[0]-bmin[0])+
            (bmax[1]-bmin[1])*(bmax[1]-bmin[1])+(bmax[2]-bmin[2])*(bmax[2]-bmin[2]));
        double max_angle_deg = 20.0;

        // Edge length limit
        Point pe00 = m_surface.point_at(usp.front(), vsp.front());
        Point pe10 = m_surface.point_at(usp.back(), vsp.front());
        Point pe01 = m_surface.point_at(usp.front(), vsp.back());
        double edx1 = pe10[0]-pe00[0], edy1 = pe10[1]-pe00[1], edz1 = pe10[2]-pe00[2];
        double edx2 = pe01[0]-pe00[0], edy2 = pe01[1]-pe00[1], edz2 = pe01[2]-pe00[2];
        double max_dim = std::max(std::sqrt(edx1*edx1+edy1*edy1+edz1*edz1),
                                  std::sqrt(edx2*edx2+edy2*edy2+edz2*edz2));
        double max_edge_len = (max_dim > 1e-10) ? max_dim / 10.0 : 0.0;

        auto span_subs_fn = [&](int dir, const std::vector<double>& sp,
                                const std::vector<double>& osp) -> std::vector<int> {
            int n = (int)sp.size() - 1;
            std::vector<int> subs(n, 1);
            int n_other = (int)osp.size() - 1;
            std::vector<double> s_pos(n_other);
            for (int k = 0; k < n_other; ++k)
                s_pos[k] = (osp[k] + osp[k + 1]) * 0.5;
            int degree_dir = (dir == 0) ? deg_u : deg_v;
            for (int i = 0; i < n; ++i) {
                double t0 = sp[i], t1 = sp[i + 1];
                if (degree_dir > 1) {
                    double ma = 0.0;
                    for (int si = 0; si < n_other; ++si) {
                        double s = s_pos[si];
                        double pnx = 0, pny = 0, pnz = 0, ta = 0.0;
                        for (int k = 0; k <= 4; ++k) {
                            double t = t0 + k * (t1 - t0) / 4.0;
                            Vector nrm = (dir == 0) ? m_surface.normal_at(t, s)
                                                    : m_surface.normal_at(s, t);
                            if (k > 0) {
                                double d = pnx*nrm[0]+pny*nrm[1]+pnz*nrm[2];
                                d = std::max(-1.0, std::min(1.0, d));
                                ta += std::acos(d) * 180.0 / Tolerance::PI;
                            }
                            pnx = nrm[0]; pny = nrm[1]; pnz = nrm[2];
                        }
                        if (ta > ma) ma = ta;
                    }
                    subs[i] = std::max(1, std::min((int)std::ceil(ma / max_angle_deg), 24));
                }
                {
                    double chord_tol = bbox_diag * 0.005;
                    double max_dev = 0.0;
                    int nc = std::min(n_other, 3);
                    for (int ci = 0; ci <= nc; ++ci) {
                        double s = osp.front() + ci * (osp.back() - osp.front()) / std::max(nc, 1);
                        double px0, py0, pz0, px1, py1, pz1;
                        if (dir == 0) {
                            m_surface.point_at(t0, s, px0, py0, pz0);
                            m_surface.point_at(t1, s, px1, py1, pz1);
                        } else {
                            m_surface.point_at(s, t0, px0, py0, pz0);
                            m_surface.point_at(s, t1, px1, py1, pz1);
                        }
                        for (int k = 1; k <= 3; ++k) {
                            double frac = k / 4.0;
                            double tm = t0 + frac * (t1 - t0);
                            double pmx, pmy, pmz;
                            if (dir == 0) m_surface.point_at(tm, s, pmx, pmy, pmz);
                            else          m_surface.point_at(s, tm, pmx, pmy, pmz);
                            double lx = px0+frac*(px1-px0), ly = py0+frac*(py1-py0), lz = pz0+frac*(pz1-pz0);
                            double ddx = pmx-lx, ddy = pmy-ly, ddz = pmz-lz;
                            double dev = std::sqrt(ddx*ddx+ddy*ddy+ddz*ddz);
                            if (dev > max_dev) max_dev = dev;
                        }
                    }
                    if (max_dev > chord_tol) {
                        int cs = std::max(2, (int)std::ceil(std::sqrt(max_dev / chord_tol)));
                        subs[i] = std::max(subs[i], std::min(cs, 24));
                    }
                }
                if (max_edge_len > 0) {
                    double s_mid = (osp.front() + osp.back()) * 0.5;
                    double px0, py0, pz0, px1, py1, pz1;
                    if (dir == 0) {
                        m_surface.point_at(t0, s_mid, px0, py0, pz0);
                        m_surface.point_at(t1, s_mid, px1, py1, pz1);
                    } else {
                        m_surface.point_at(s_mid, t0, px0, py0, pz0);
                        m_surface.point_at(s_mid, t1, px1, py1, pz1);
                    }
                    double sl = std::sqrt((px1-px0)*(px1-px0)+(py1-py0)*(py1-py0)+(pz1-pz0)*(pz1-pz0));
                    int es = std::max(1, (int)std::ceil(sl / max_edge_len));
                    subs[i] = std::max(subs[i], std::min(es, 64));
                }
                if (degree_dir > 1) subs[i] = std::max(subs[i], 2);
            }
            return subs;
        };

        auto u_subs = span_subs_fn(0, usp, vsp);
        auto v_subs = span_subs_fn(1, vsp, usp);

        // Build parameter arrays
        std::vector<double> us, vs;
        for (int i = 0; i < ns_u; ++i)
            for (int s = 0; s < u_subs[i]; ++s)
                us.push_back(usp[i] + s * (usp[i+1] - usp[i]) / u_subs[i]);
        us.push_back(usp.back());
        for (int i = 0; i < ns_v; ++i)
            for (int s = 0; s < v_subs[i]; ++s)
                vs.push_back(vsp[i] + s * (vsp[i+1] - vsp[i]) / v_subs[i]);
        vs.push_back(vsp.back());

        // Insert grid points inside trim region
        for (size_t i = 0; i < us.size(); ++i)
            for (size_t j = 0; j < vs.size(); ++j)
                if (inside_trim(us[i], vs[j])) dt.insert(us[i], vs[j]);
    }

    // Cleanup super-triangle
    dt.cleanup();

    // Remove exterior triangles
    for (auto& tri : dt.triangles) {
        if (!tri.alive) continue;
        double cu = (dt.vertices[tri.v[0]].x + dt.vertices[tri.v[1]].x + dt.vertices[tri.v[2]].x) / 3.0;
        double cv = (dt.vertices[tri.v[0]].y + dt.vertices[tri.v[1]].y + dt.vertices[tri.v[2]].y) / 3.0;
        if (!inside_trim(cu, cv)) tri.alive = false;
    }

    // Build output mesh
    auto tris = dt.get_triangles();
    if (tris.empty()) return m_surface.mesh();

    Mesh result;
    std::vector<size_t> vert_map(dt.vertices.size(), SIZE_MAX);
    for (const auto& tri : tris)
        for (int k = 0; k < 3; ++k) {
            int vi = tri[k];
            if (vert_map[vi] != SIZE_MAX) continue;
            vert_map[vi] = result.add_vertex(m_surface.point_at(dt.vertices[vi].x, dt.vertices[vi].y));
        }
    for (const auto& tri : tris) {
        size_t v0 = vert_map[tri[0]], v1 = vert_map[tri[1]], v2 = vert_map[tri[2]];
        if (v0 == v1 || v1 == v2 || v2 == v0) continue;
        result.add_face({v0, v1, v2});
    }

    // Normals
    if (planar) {
        auto du = m_surface.domain(0), dv = m_surface.domain(1);
        Vector nrm = m_surface.normal_at((du.first+du.second)/2, (dv.first+dv.second)/2);
        for (auto& [vk, vd] : result.vertex) vd.set_normal(nrm[0], nrm[1], nrm[2]);
    } else {
        for (size_t vi = 0; vi < vert_map.size(); ++vi) {
            if (vert_map[vi] == SIZE_MAX) continue;
            Vector nrm = m_surface.normal_at(dt.vertices[vi].x, dt.vertices[vi].y);
            result.vertex[vert_map[vi]].set_normal(nrm[0], nrm[1], nrm[2]);
        }
    }
    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Transformation
///////////////////////////////////////////////////////////////////////////////////////////

void TrimmedSurface::transform() {
    m_surface.xform = xform;
    m_surface.transform();
    xform = Xform::identity();
}

TrimmedSurface TrimmedSurface::transformed() const {
    TrimmedSurface ts = *this;
    ts.transform();
    return ts;
}

///////////////////////////////////////////////////////////////////////////////////////////
// JSON Serialization
///////////////////////////////////////////////////////////////////////////////////////////

nlohmann::ordered_json TrimmedSurface::jsondump() const {
    nlohmann::ordered_json j;
    j["guid"] = guid;
    j["inner_loops"] = nlohmann::ordered_json::array();
    for (const auto& loop : m_inner_loops)
        j["inner_loops"].push_back(loop.jsondump());
    j["name"] = name;
    if (m_outer_loop.is_valid())
        j["outer_loop"] = m_outer_loop.jsondump();
    j["surface"] = m_surface.jsondump();
    j["surfacecolor"] = surfacecolor.jsondump();
    j["type"] = "TrimmedSurface";
    j["width"] = width;
    j["xform"] = xform.jsondump();
    return j;
}

TrimmedSurface TrimmedSurface::jsonload(const nlohmann::json& data) {
    TrimmedSurface ts;
    if (data.contains("guid")) ts.guid = data["guid"];
    if (data.contains("name")) ts.name = data["name"];
    if (data.contains("width")) ts.width = data["width"];
    if (data.contains("surfacecolor")) ts.surfacecolor = Color::jsonload(data["surfacecolor"]);
    if (data.contains("xform")) ts.xform = Xform::jsonload(data["xform"]);
    if (data.contains("surface")) ts.m_surface = NurbsSurface::jsonload(data["surface"]);
    if (data.contains("outer_loop")) ts.m_outer_loop = NurbsCurve::jsonload(data["outer_loop"]);
    if (data.contains("inner_loops")) {
        for (const auto& loop_data : data["inner_loops"])
            ts.m_inner_loops.push_back(NurbsCurve::jsonload(loop_data));
    }
    return ts;
}

std::string TrimmedSurface::json_dumps() const { return jsondump().dump(); }
TrimmedSurface TrimmedSurface::json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void TrimmedSurface::json_dump(const std::string& filename) const {
    std::ofstream file(filename);
    file << jsondump().dump(4);
}

TrimmedSurface TrimmedSurface::json_load(const std::string& filename) {
    std::ifstream file(filename);
    nlohmann::json data;
    file >> data;
    return jsonload(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Protobuf Serialization
///////////////////////////////////////////////////////////////////////////////////////////

std::string TrimmedSurface::pb_dumps() const {
    session_proto::TrimmedSurface proto;
    proto.set_guid(guid);
    proto.set_name(name);
    proto.set_width(width);

    // Surface
    std::string srf_data = m_surface.pb_dumps();
    auto* srf_proto = proto.mutable_surface();
    srf_proto->ParseFromString(srf_data);

    // Outer loop
    if (is_trimmed()) {
        std::string loop_data = m_outer_loop.pb_dumps();
        auto* ol = proto.mutable_outer_loop();
        ol->ParseFromString(loop_data);
    }

    // Inner loops
    for (const auto& inner : m_inner_loops) {
        std::string loop_data = inner.pb_dumps();
        auto* il = proto.add_inner_loops();
        il->ParseFromString(loop_data);
    }

    // Color
    auto* color_proto = proto.mutable_surfacecolor();
    color_proto->set_name(surfacecolor.name);
    color_proto->set_r(surfacecolor.r);
    color_proto->set_g(surfacecolor.g);
    color_proto->set_b(surfacecolor.b);
    color_proto->set_a(surfacecolor.a);

    // Transform
    auto* xform_proto = proto.mutable_xform();
    xform_proto->set_guid(xform.guid);
    xform_proto->set_name(xform.name);
    for (int i = 0; i < 16; ++i)
        xform_proto->add_matrix(xform.m[i]);

    return proto.SerializeAsString();
}

TrimmedSurface TrimmedSurface::pb_loads(const std::string& data) {
    session_proto::TrimmedSurface proto;
    proto.ParseFromString(data);

    TrimmedSurface ts;
    ts.guid = proto.guid();
    ts.name = proto.name();
    ts.width = proto.width();

    // Surface
    if (proto.has_surface()) {
        std::string srf_data = proto.surface().SerializeAsString();
        ts.m_surface = NurbsSurface::pb_loads(srf_data);
    }

    // Outer loop
    if (proto.has_outer_loop()) {
        std::string loop_data = proto.outer_loop().SerializeAsString();
        ts.m_outer_loop = NurbsCurve::pb_loads(loop_data);
    }

    // Inner loops
    for (int i = 0; i < proto.inner_loops_size(); ++i) {
        std::string loop_data = proto.inner_loops(i).SerializeAsString();
        ts.m_inner_loops.push_back(NurbsCurve::pb_loads(loop_data));
    }

    // Color
    const auto& color_proto = proto.surfacecolor();
    ts.surfacecolor.name = color_proto.name();
    ts.surfacecolor.r = color_proto.r();
    ts.surfacecolor.g = color_proto.g();
    ts.surfacecolor.b = color_proto.b();
    ts.surfacecolor.a = color_proto.a();

    // Transform
    const auto& xform_proto = proto.xform();
    ts.xform.guid = xform_proto.guid();
    ts.xform.name = xform_proto.name();
    for (int i = 0; i < 16 && i < xform_proto.matrix_size(); ++i)
        ts.xform.m[i] = xform_proto.matrix(i);

    return ts;
}

void TrimmedSurface::pb_dump(const std::string& filename) const {
    std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

TrimmedSurface TrimmedSurface::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)),
                     std::istreambuf_iterator<char>());
    return pb_loads(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// String Representation
///////////////////////////////////////////////////////////////////////////////////////////

std::string TrimmedSurface::str() const {
    return fmt::format("TrimmedSurface(name={}, trimmed={}, holes={})",
                       name, is_trimmed() ? "true" : "false", inner_loop_count());
}

std::string TrimmedSurface::repr() const {
    return fmt::format("TrimmedSurface(\n  name={},\n  trimmed={},\n  holes={},\n  surface={}\n)",
                       name, is_trimmed() ? "true" : "false", inner_loop_count(), m_surface.str());
}

std::ostream& operator<<(std::ostream& os, const TrimmedSurface& ts) {
    os << ts.str();
    return os;
}

} // namespace session_cpp
