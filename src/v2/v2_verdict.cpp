#include "v2_verdict.h"

#include "nurbscurve.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <set>
#include <vector>

namespace session_cpp {
namespace v2v {

namespace {

/// Degenerate-edge tolerance, IDENTICAL to BRep::is_solid() (src/brep.cpp): the bounding-box
/// diagonal of the vertex pool scaled by 1e-7, floored at 1e-12. Any divergence here and the
/// harness would disagree with the kernel's own validity rule on exactly the shapes that
/// produced this campaign's measurement errors.
double degeneracy_tolerance(const BRep& b) {
    double xmn = 1e300, ymn = 1e300, zmn = 1e300, xmx = -1e300, ymx = -1e300, zmx = -1e300;
    for (const auto& p : b.m_vertices) {
        xmn = std::min(xmn, p[0]); ymn = std::min(ymn, p[1]); zmn = std::min(zmn, p[2]);
        xmx = std::max(xmx, p[0]); ymx = std::max(ymx, p[1]); zmx = std::max(zmx, p[2]);
    }
    const double diag =
        b.m_vertices.empty()
            ? 1.0
            : std::sqrt((xmx - xmn) * (xmx - xmn) + (ymx - ymn) * (ymx - ymn) +
                        (zmx - zmn) * (zmx - zmn));
    return std::max(diag * 1e-7, 1e-12);
}

/// Extent of an edge's 3D curve, sampled exactly as is_solid() samples it: the largest distance
/// from the domain start to four evenly spaced parameters. An edge with no usable 3D curve is
/// NOT degenerate (is_solid() does not skip it either), so it reports an infinite extent.
double edge_extent(const BRep& b, const BRepEdge& e) {
    const int ci = e.curve_3d_index;
    if (ci < 0 || ci >= (int)b.m_curves_3d.size()) return 1e300;
    const NurbsCurve& c = b.m_curves_3d[ci];
    const auto d = c.domain();
    const Point p0 = c.point_at(d.first);
    double ext = 0.0;
    for (int k = 1; k <= 4; ++k)
        ext = std::max(ext, p0.distance(c.point_at(d.first + (d.second - d.first) * k / 4.0)));
    return ext;
}

int face_of_trim(const BRep& b, int ti) {
    if (ti < 0 || ti >= (int)b.m_trims.size()) return -1;
    const int li = b.m_trims[ti].loop_index;
    return (li >= 0 && li < (int)b.m_loops.size()) ? b.m_loops[li].face_index : -1;
}

/// Counts, shells and edge classes. No quadrature.
V2Verdict topology_verdict(const BRep& b) {
    V2Verdict v;
    v.faces = (int)b.m_faces.size();
    v.has_topology = v.faces > 0 && !b.m_topology_edges.empty();
    if (!v.has_topology) return v;

    const double deg_tol = degeneracy_tolerance(b);

    for (const BRepEdge& e : b.m_topology_edges) {
        const int n = (int)e.trim_indices.size();
        // An ORPHANED record (no face references it) has no topological role -- is_solid()
        // skips it, so it is neither naked nor manifold here.
        if (n == 0) { ++v.orphan; continue; }
        // A DEGENERATE edge is a point: watertight by construction, excluded from the counts.
        if (edge_extent(b, e) < deg_tol) { ++v.degenerate; continue; }
        ++v.edges;
        if (n == 2 && face_of_trim(b, e.trim_indices[0]) == face_of_trim(b, e.trim_indices[1]))
            ++v.seam_edges;
        if (n == 1) ++v.naked_real;
        else if (n > 2) ++v.nonmanifold;
    }

    // Shells: connected components of faces joined across a 2-use edge.
    std::vector<int> uf((size_t)std::max(v.faces, 1));
    for (int i = 0; i < v.faces; ++i) uf[(size_t)i] = i;
    std::function<int(int)> find = [&](int a) {
        while (uf[(size_t)a] != a) { uf[(size_t)a] = uf[(size_t)uf[(size_t)a]]; a = uf[(size_t)a]; }
        return a;
    };
    auto unite = [&](int a, int c) { a = find(a); c = find(c); if (a != c) uf[(size_t)a] = c; };
    for (const BRepEdge& e : b.m_topology_edges) {
        if ((int)e.trim_indices.size() != 2) continue;
        const int f1 = face_of_trim(b, e.trim_indices[0]);
        const int f2 = face_of_trim(b, e.trim_indices[1]);
        if (f1 >= 0 && f2 >= 0 && f1 != f2) unite(f1, f2);
    }
    std::set<int> roots;
    for (int i = 0; i < v.faces; ++i) roots.insert(find(i));
    v.shells = (int)roots.size();
    return v;
}

}  // namespace

std::string V2Verdict::str() const {
    char buf[320];
    std::snprintf(buf, sizeof buf,
                  "F=%d shells=%d solids=%d naked=%d nonman=%d seam=%d degen=%d orphan=%d "
                  "resid=%.2e vol=%.9g%s closed=%d",
                  faces, shells, solids, naked_real, nonmanifold, seam_edges, degenerate, orphan,
                  closure_residual, volume, volume_valid ? "" : "(invalid)", (int)closed());
    return std::string(buf);
}

MassPropsOptions v2_verdict_options() {
    MassPropsOptions o;
    o.rel_tolerance = 1e-10;
    o.max_surface_evals = 4000000;
    o.min_face_evals = 40000;
    return o;
}

V2Verdict v2_verdict_topology_only(const BRep& b) { return topology_verdict(b); }

V2Verdict v2_verdict(const BRep& b, const MassPropsOptions& opt) {
    V2Verdict v = topology_verdict(b);
    if (!v.has_topology) { v.closure_residual = 1.0; return v; }

    const MassProps mp = brep_massprops(b, opt);
    v.closure_residual = mp.closure_residual;
    v.area = mp.area;
    v.volume = mp.volume;
    v.converged = mp.converged;
    // NOTE: MassProps::closed is a NAIVE count (every edge with != 2 trims, degenerates
    // included), so it is deliberately not used here -- only the geometric certificate is.
    v.volume_valid = v.closed() && mp.converged && std::isfinite(mp.volume);
    if (v.closed()) {
        for (double sv : mp.shell_volumes)
            if (sv > 0.0) ++v.solids;
    }
    return v;
}

V2Verdict v2_verdict(const BRep& b) { return v2_verdict(b, v2_verdict_options()); }

}  // namespace v2v
}  // namespace session_cpp
