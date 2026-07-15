// Mesh CSG (difference / union / intersection) via a signed-distance field + marching cubes.
// Robust fallback for imported / freeform solids whose exact surface-surface intersection the
// BRep boolean cannot trace: sample a signed-distance field of the combined solids on a grid,
// extract a watertight isosurface (edge-welded vertices, marching-cubes ambiguity holes filled),
// return a closed mesh. Higher resolution trades speed for accuracy.
#include "mesh.h"
#include "intersection.h"
#include "spatial_bvh.h"
#include "aabb.h"
#include <array>
#include <cmath>

namespace session_cpp {
namespace {

static const int MC_EDGE[12][2] = {
    {0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}
};
#include "mc_tritable.inc"   // static const int MC_TRI[256][16];

struct Tri { Point a, b, c; };

// Squared distance from point p to triangle (a,b,c) — Ericson closest-point-on-triangle.
double pt_tri_dist2(const Point& p, const Point& a, const Point& b, const Point& c) {
    double ab[3]={b[0]-a[0],b[1]-a[1],b[2]-a[2]}, ac[3]={c[0]-a[0],c[1]-a[1],c[2]-a[2]}, ap[3]={p[0]-a[0],p[1]-a[1],p[2]-a[2]};
    double d1=ab[0]*ap[0]+ab[1]*ap[1]+ab[2]*ap[2], d2=ac[0]*ap[0]+ac[1]*ap[1]+ac[2]*ap[2];
    double cx,cy,cz;
    if (d1<=0 && d2<=0) { cx=a[0];cy=a[1];cz=a[2]; }
    else {
        double bp[3]={p[0]-b[0],p[1]-b[1],p[2]-b[2]};
        double d3=ab[0]*bp[0]+ab[1]*bp[1]+ab[2]*bp[2], d4=ac[0]*bp[0]+ac[1]*bp[1]+ac[2]*bp[2];
        if (d3>=0 && d4<=d3) { cx=b[0];cy=b[1];cz=b[2]; }
        else {
            double cp[3]={p[0]-c[0],p[1]-c[1],p[2]-c[2]};
            double d5=ab[0]*cp[0]+ab[1]*cp[1]+ab[2]*cp[2], d6=ac[0]*cp[0]+ac[1]*cp[1]+ac[2]*cp[2];
            if (d6>=0 && d5<=d6) { cx=c[0];cy=c[1];cz=c[2]; }
            else {
                double vc=d1*d4-d3*d2;
                if (vc<=0 && d1>=0 && d3<=0) { double v=d1/(d1-d3); cx=a[0]+v*ab[0];cy=a[1]+v*ab[1];cz=a[2]+v*ab[2]; }
                else {
                    double vb=d5*d2-d1*d6;
                    if (vb<=0 && d2>=0 && d6<=0) { double w=d2/(d2-d6); cx=a[0]+w*ac[0];cy=a[1]+w*ac[1];cz=a[2]+w*ac[2]; }
                    else {
                        double va=d3*d6-d5*d4;
                        if (va<=0 && (d4-d3)>=0 && (d5-d6)>=0) { double w=(d4-d3)/((d4-d3)+(d5-d6)); cx=b[0]+w*(c[0]-b[0]);cy=b[1]+w*(c[1]-b[1]);cz=b[2]+w*(c[2]-b[2]); }
                        else { double den=1.0/(va+vb+vc), v=vb*den, w=vc*den; cx=a[0]+ab[0]*v+ac[0]*w;cy=a[1]+ab[1]*v+ac[1]*w;cz=a[2]+ab[2]*v+ac[2]*w; }
                    }
                }
            }
        }
    }
    double dx=p[0]-cx,dy=p[1]-cy,dz=p[2]-cz; return dx*dx+dy*dy+dz*dz;
}

// Triangle soup with a BVH: crack-robust point-in-mesh (7-ray majority) + band-capped distance.
struct MeshQuery {
    std::vector<Tri> tris;
    SpatialBVH bvh;
    void build(const Mesh& m) {
        for (const auto& kv : m.face) {
            const auto& vs = kv.second;
            if (vs.size() < 3) continue;
            Point p0 = m.vertex.at(vs[0]).position();
            for (size_t i = 1; i + 1 < vs.size(); ++i)
                tris.push_back({p0, m.vertex.at(vs[i]).position(), m.vertex.at(vs[i+1]).position()});
        }
        std::vector<AABB> boxes; boxes.reserve(tris.size());
        double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
        for (const auto& t : tris) {
            boxes.push_back(AABB::from_points({t.a, t.b, t.c}, 1e-9));
            for (const Point* p : {&t.a,&t.b,&t.c}) {
                xmn=std::min(xmn,(*p)[0]); ymn=std::min(ymn,(*p)[1]); zmn=std::min(zmn,(*p)[2]);
                xmx=std::max(xmx,(*p)[0]); ymx=std::max(ymx,(*p)[1]); zmx=std::max(zmx,(*p)[2]);
            }
        }
        double diag = std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
        if (diag <= 0) diag = 1.0;
        if (!boxes.empty()) bvh.build_from_aabbs(boxes.data(), boxes.size(), diag * 2.0);
    }
    bool inside(const Point& p) const {
        static const double D[7][3] = {
            {0.5773502691,0.6539124,0.5023147},{0.8506508084,0.5257311121,0.0},
            {0.0,0.8506508084,0.5257311121},{0.5257311121,0.0,0.8506508084},
            {-0.3574067443,0.7844645405,0.5057219851},{0.7844645405,-0.5057219851,0.3574067443},
            {-0.5023147,0.5773502691,-0.6435942529}
        };
        int votes = 0;
        for (const auto& d : D) {
            Vector dir(d[0], d[1], d[2]);
            std::vector<int> cand;
            bvh.ray_cast(p, dir, cand, true);
            int cnt = 0;
            for (int id : cand) {
                const Tri& t = tris[id];
                double tt, u, v; bool par;
                if (Intersection::ray_triangle(p, dir, t.a, t.b, t.c, 1e-9, tt, u, v, par) && tt > 1e-7)
                    ++cnt;
            }
            votes += (cnt & 1);
        }
        return votes * 2 > 7;
    }
    double dist(const Point& p, double band) const {
        double r = band * 0.5, best2 = band * band;
        for (int it = 0; it < 6; ++it) {
            AABB q = AABB::from_points({Point(p[0]-r,p[1]-r,p[2]-r), Point(p[0]+r,p[1]+r,p[2]+r)});
            auto cand = bvh.query_aabb(q);
            double b2 = band * band;
            for (int id : cand) { const Tri& t=tris[id]; b2 = std::min(b2, pt_tri_dist2(p, t.a, t.b, t.c)); }
            best2 = std::min(best2, b2);
            if (best2 <= r * r) break;
            r *= 2.0;
            if (r > band) break;
        }
        return std::sqrt(best2);
    }
    double sdf(const Point& p, double band) const { return (inside(p) ? -1.0 : 1.0) * dist(p, band); }
};

enum class BoolOp { Difference, Union, Intersection };

// combine(sdf_A, sdf_B): difference = A ∩ !B = max(sa,-sb); union = min; intersection = max.
double combine(BoolOp op, double sa, double sb) {
    switch (op) {
        case BoolOp::Difference:   return std::max(sa, -sb);
        case BoolOp::Union:        return std::min(sa, sb);
        default:                   return std::max(sa, sb);   // Intersection
    }
}

Mesh mesh_boolean_op(const Mesh& A, const Mesh& B, BoolOp op, int resolution) {
    if (resolution < 8) resolution = 8;
    MeshQuery qa, qb; qa.build(A); qb.build(B);
    if (qa.tris.empty() || qb.tris.empty()) return Mesh();

    double lo[3] = {1e300,1e300,1e300}, hi[3] = {-1e300,-1e300,-1e300};
    for (const auto* Q : {&qa, &qb})
        for (const auto& t : Q->tris)
            for (const Point* p : {&t.a,&t.b,&t.c})
                for (int k=0;k<3;++k){ lo[k]=std::min(lo[k],(*p)[k]); hi[k]=std::max(hi[k],(*p)[k]); }
    double cell = 0.0;
    for (int k=0;k<3;++k) cell = std::max(cell, (hi[k]-lo[k]) / resolution);
    if (cell <= 0) return Mesh();
    for (int k=0;k<3;++k){ lo[k]-=cell*2; hi[k]+=cell*2; }
    int nx = (int)std::ceil((hi[0]-lo[0])/cell), ny = (int)std::ceil((hi[1]-lo[1])/cell), nz = (int)std::ceil((hi[2]-lo[2])/cell);

    // signed-distance field of the combined solids at grid corners (band-capped)
    double band = cell * 3.0;
    std::vector<float> sdf((size_t)(nx+1)*(ny+1)*(nz+1), (float)band);
    auto gidx = [&](int i,int j,int k){ return ((size_t)i*(ny+1)+j)*(nz+1)+k; };
    for (int i=0;i<=nx;++i)
        for (int j=0;j<=ny;++j)
            for (int k=0;k<=nz;++k){
                Point p(lo[0]+i*cell, lo[1]+j*cell, lo[2]+k*cell);
                sdf[gidx(i,j,k)] = (float)combine(op, qa.sdf(p, band), qb.sdf(p, band));
            }

    // marching cubes with edge-keyed vertex welding + zero-crossing interpolation
    Mesh out;
    const int CORNER[8][3] = {{0,0,0},{1,0,0},{1,1,0},{0,1,0},{0,0,1},{1,0,1},{1,1,1},{0,1,1}};
    std::map<std::pair<size_t,size_t>, size_t> edge_vert;
    auto edge_vertex = [&](int i,int j,int k,int e) -> size_t {
        int a=MC_EDGE[e][0], b=MC_EDGE[e][1];
        size_t ga = gidx(i+CORNER[a][0], j+CORNER[a][1], k+CORNER[a][2]);
        size_t gb = gidx(i+CORNER[b][0], j+CORNER[b][1], k+CORNER[b][2]);
        auto key = std::make_pair(std::min(ga,gb), std::max(ga,gb));
        auto it = edge_vert.find(key);
        if (it != edge_vert.end()) return it->second;
        Point pa(lo[0]+(i+CORNER[a][0])*cell, lo[1]+(j+CORNER[a][1])*cell, lo[2]+(k+CORNER[a][2])*cell);
        Point pb(lo[0]+(i+CORNER[b][0])*cell, lo[1]+(j+CORNER[b][1])*cell, lo[2]+(k+CORNER[b][2])*cell);
        double va = sdf[ga], vb = sdf[gb];
        double t = (std::abs(va - vb) > 1e-12) ? va / (va - vb) : 0.5;
        t = std::min(std::max(t, 0.0), 1.0);
        size_t vk = out.add_vertex(Point(pa[0]+t*(pb[0]-pa[0]), pa[1]+t*(pb[1]-pa[1]), pa[2]+t*(pb[2]-pa[2])));
        edge_vert[key] = vk;
        return vk;
    };
    for (int i=0;i<nx;++i) for (int j=0;j<ny;++j) for (int k=0;k<nz;++k) {
        int ci = 0;
        for (int c=0;c<8;++c) if (sdf[gidx(i+CORNER[c][0], j+CORNER[c][1], k+CORNER[c][2])] < 0.0f) ci |= (1<<c);
        if (ci==0 || ci==255) continue;
        for (int t=0; MC_TRI[ci][t] != -1; t += 3) {
            size_t v0 = edge_vertex(i,j,k, MC_TRI[ci][t]);
            size_t v1 = edge_vertex(i,j,k, MC_TRI[ci][t+1]);
            size_t v2 = edge_vertex(i,j,k, MC_TRI[ci][t+2]);
            if (v0!=v1 && v1!=v2 && v0!=v2) out.add_face({v0, v1, v2});
        }
    }

    // fill the small marching-cubes ambiguity holes: walk naked-edge loops, fan-triangulate
    {
        auto naked = out.naked_edges(true);
        std::map<size_t, std::vector<size_t>> adj;
        std::set<std::pair<size_t,size_t>> eset;
        for (auto& e : naked) {
            adj[e.first].push_back(e.second);
            adj[e.second].push_back(e.first);
            eset.insert(std::minmax(e.first, e.second));
        }
        while (!eset.empty()) {
            auto se = *eset.begin();
            size_t start = se.first, prev = start, cur = se.second;
            eset.erase(eset.begin());
            std::vector<size_t> loop = {start, cur};
            bool ok = true;
            while (cur != start) {
                size_t nxt = SIZE_MAX;
                for (size_t w : adj[cur]) {
                    auto kk = std::minmax(cur, w);
                    if (w != prev && eset.count(kk)) { nxt = w; eset.erase(kk); break; }
                }
                if (nxt == SIZE_MAX) { ok = false; break; }
                if (nxt == start) break;
                loop.push_back(nxt); prev = cur; cur = nxt;
            }
            if (ok && loop.size() >= 3)
                for (size_t i = 1; i + 1 < loop.size(); ++i)
                    out.add_face({loop[0], loop[i], loop[i+1]});
        }
        out.unify_winding();
    }
    return out;
}

}  // namespace

Mesh Mesh::boolean_difference(const Mesh& other, int resolution) const {
    return mesh_boolean_op(*this, other, BoolOp::Difference, resolution);
}
Mesh Mesh::boolean_union(const Mesh& other, int resolution) const {
    return mesh_boolean_op(*this, other, BoolOp::Union, resolution);
}
Mesh Mesh::boolean_intersection(const Mesh& other, int resolution) const {
    return mesh_boolean_op(*this, other, BoolOp::Intersection, resolution);
}

}  // namespace session_cpp
