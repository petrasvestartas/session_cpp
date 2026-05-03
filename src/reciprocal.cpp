#include "reciprocal.h"
#include "intersection.h"
#include "xform.h"
#include <algorithm>
#include <map>
#include <numeric>

namespace session_cpp {

std::vector<Line> Reciprocal::get_lines(
    const std::vector<Line>&             lines,
    const std::vector<Plane>&            lp,
    const std::vector<std::vector<int>>& fe,
    std::vector<std::array<Plane,2>>&    end_planes,
    double move)
{
    int ne = (int)lines.size();
    int nf = (int)fe.size();

    std::vector<Line> moved = lines;
    for (int i = 0; i < ne; i++)
        moved[i] += lp[i].y_axis() * move;

    std::vector<std::vector<Point>> pts(ne);
    std::vector<std::vector<int>>   pid(ne);

    for (int fi = 0; fi < nf; fi++) {
        int n = (int)fe[fi].size();
        for (int j = 0; j < n; j++) {
            int cur  = fe[fi][j];
            int prev = fe[fi][((j - 1) % n + n) % n];
            int next = fe[fi][(j + 1) % n];
            Point p0, p1;
            if (Intersection::line_plane(moved[cur], lp[prev], p0, false)) {
                pts[cur].push_back(p0);
                pid[cur].push_back(prev);
            }
            if (Intersection::line_plane(moved[cur], lp[next], p1, false)) {
                pts[cur].push_back(p1);
                pid[cur].push_back(next);
            }
        }
    }

    std::vector<Line> out(ne);
    end_planes.resize(ne);

    for (int ei = 0; ei < ne; ei++) {
        if ((int)pts[ei].size() < 2) {
            out[ei] = moved[ei];
            end_planes[ei] = {lp[ei], lp[ei]};
            continue;
        }
        int np = (int)pts[ei].size();
        std::vector<int> ids(np);
        std::iota(ids.begin(), ids.end(), 0);
        std::sort(ids.begin(), ids.end(), [&](int a, int b) {
            return moved[ei].closest_point(pts[ei][a], false).first <
                   moved[ei].closest_point(pts[ei][b], false).first;
        });
        int s = ids[0];
        int e = ids.back();
        out[ei] = Line::from_points(pts[ei][s], pts[ei][e]);
        Plane ps(pts[ei][s], lp[pid[ei][s]].x_axis(), lp[pid[ei][s]].y_axis());
        Plane pe(pts[ei][e], lp[pid[ei][e]].x_axis(), lp[pid[ei][e]].y_axis());
        end_planes[ei] = {ps, pe};
    }
    return out;
}

Reciprocal::Result Reciprocal::from_mesh(
    const Mesh& mesh,
    double angle,
    double scale,
    bool   /*use_ngon_normals*/,
    double height)
{
    auto fkeys = mesh.faces();
    auto ekeys = mesh.edges();
    int ne = (int)ekeys.size();
    int nf = (int)fkeys.size();

    std::map<std::pair<size_t,size_t>, int> edge_idx;
    for (int i = 0; i < ne; i++) {
        edge_idx[ekeys[i]] = i;
        edge_idx[{ekeys[i].second, ekeys[i].first}] = i;
    }

    std::map<size_t, Plane> fplane;
    for (size_t fk : fkeys) {
        auto n = mesh.face_normal(fk).value();
        auto c = mesh.face_centroid(fk).value();
        fplane[fk] = Plane::from_point_normal(c, n);
    }

    std::vector<std::vector<int>> fe(nf);
    for (int fi = 0; fi < nf; fi++) {
        for (auto& [u, v] : mesh.face_edges(fkeys[fi]).value()) {
            auto key = std::make_pair(std::min(u, v), std::max(u, v));
            fe[fi].push_back(edge_idx[key]);
        }
    }

    std::vector<Vector> vecs(ne, Vector(0, 0, 0));
    for (int ei = 0; ei < ne; ei++) {
        auto adj = mesh.edge_faces(ekeys[ei].first, ekeys[ei].second);
        if (!adj) continue;
        Vector sum(0, 0, 0);
        for (size_t fk : *adj)
            sum += fplane[fk].z_axis();
        Vector avg = sum / (double)adj->size();
        if (!avg.is_zero())
            vecs[ei] = avg.normalized();
    }

    std::vector<Line> lines(ne);
    for (int ei = 0; ei < ne; ei++)
        lines[ei] = mesh.edge_line(ekeys[ei].first, ekeys[ei].second).value();

    for (int ei = 0; ei < ne; ei++) {
        if (vecs[ei].is_zero()) continue;
        Point mid = lines[ei].center();
        lines[ei].xform = Xform::scale_uniform(mid, scale);
        lines[ei].transform();
        // rotation center is the original midpoint (unchanged by uniform scale about it)
        Point axis_end(mid[0] + vecs[ei][0], mid[1] + vecs[ei][1], mid[2] + vecs[ei][2]);
        Line rot_axis = Line::from_points(mid, axis_end);
        lines[ei].xform = Xform::rotation_around_line(rot_axis, angle);
        lines[ei].transform();
    }

    std::vector<Plane> lp(ne);
    for (int ei = 0; ei < ne; ei++) {
        Point mid = lines[ei].center();
        Vector dir = lines[ei].to_direction();
        if (!vecs[ei].is_zero())
            lp[ei] = Plane(mid, dir, vecs[ei]);
        else
            lp[ei] = Plane::from_point_normal(mid, dir);
    }

    Result result;
    result.lineplanes = lp;
    result.center = get_lines(lines, lp, fe, result.endplanes, 0.0);
    std::vector<std::array<Plane,2>> dummy;
    result.top    = get_lines(lines, lp, fe, dummy,  height);
    result.bottom = get_lines(lines, lp, fe, dummy, -height);
    return result;
}

} // namespace session_cpp
