#include "bop.h"
#include "bop_builder_face.h"
#include "bop_builder_solid.h"
#include "bop_ds.h"
#include "brep_boolean.h"
#include "brep_classify.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <numeric>

namespace session_cpp::bop {
namespace {

bool occupied(bool a, bool b, Operation operation) {
    switch (operation) {
        case Operation::Intersection: return a && b;
        case Operation::Union: return a || b;
        case Operation::Difference: return a && !b;
        case Operation::ReverseDifference: return b && !a;
        case Operation::Xor: return a != b;
    }
    return false;
}

NurbsCurve reversed(NurbsCurve c) {
    c.reverse();
    return c;
}

/// A split face: loops of uses of its DS face, its state against the other operand (1 in, -1 out, 0 on) and its same-domain twin
struct Piece {
    int face = -1;
    std::vector<std::vector<int>> loops;
    std::vector<int> images;
    int state = -1;
    int twin = -1;
    Point uv, point;
    bool has_point = false;
};

struct Builder {
    const DS& ds;
    Report& report;
    std::vector<std::vector<EdgeUse>> uses;
    std::vector<Piece> pieces;
    BRep result;
    std::map<int, int> vertex_image, edge_image;
    std::map<std::pair<int, int>, int> surface_image;
    std::map<std::pair<int, int>, std::array<std::vector<NurbsCurve>, 2>> slots; // (result edge, result surface) -> pcurves of forward and reversed uses

    Builder(const DS& source, Report& out) : ds(source), report(out) {}

    int image(int b) const {
        const std::vector<int> family = ds.family(b);
        return *std::min_element(family.begin(), family.end());
    }

    /// Whether block b runs the same way as block r of its family
    bool along(int b, int r) const {
        const PaveBlock &x = ds.blocks[b], &y = ds.blocks[r];
        if (x.first.vertex != x.last.vertex)
            return x.first.vertex == y.first.vertex;
        const Point p = ds.block_point(x, 0.25);
        return p.distance(ds.block_point(y, 0.25)) <= p.distance(ds.block_point(y, 0.75));
    }

    EdgeUse use_of(int b, bool backwards, NurbsCurve pcurve, bool internal, const Face& f) const {
        const PaveBlock& block = ds.blocks[b];
        const int r = image(b);
        const double gap = 2.0 * std::max(ds.vertices[block.first.vertex].tolerance, ds.vertices[block.last.vertex].tolerance) * f.region.eps / f.tolerance;
        return {r, backwards ? block.last.vertex : block.first.vertex, backwards ? block.first.vertex : block.last.vertex,
            std::move(pcurve), backwards == along(b, r), internal, gap};
    }

    /// FillImagesFaces: each face of both operands cut by its In and Sc edges
    void split_faces() {
        uses.resize(ds.faces.size());
        for (size_t id = 0; id < ds.faces.size(); ++id) {
            const Face& f = ds.faces[id];
            if (f.edges.empty())
                continue;
            const BRep& brep = ds.brep(f.operand);
            std::vector<EdgeUse>& list = uses[id];
            std::vector<std::vector<int>> wires;
            for (const BRepRef& wire : brep.m_faces[f.local].wires) {
                std::vector<int> loop;
                for (const BRepRef& use : brep.wire_edges(wire)) {
                    const Edge& e = ds.edges[ds.edge_offset[f.operand] + use.index];
                    const int index = brep.pcurve_index(use.index, f.local, use.orientation);
                    if (index < 0) {
                        report.log.push_back("edge without pcurve on its face");
                        continue;
                    }
                    const bool back = use.orientation == BRepOrientation::Reversed;
                    std::vector<int> blocks = e.blocks;
                    if (back)
                        std::reverse(blocks.begin(), blocks.end());
                    for (int b : blocks) {
                        const PaveBlock& block = ds.blocks[b];
                        if (!e.degenerated && ds.micro(block))
                            continue;
                        loop.push_back(static_cast<int>(list.size()));
                        list.push_back(use_of(b, back, trimmed(brep.m_curves_2d[index].nurbs(), {e.t0, e.t1},
                            back ? block.last.t : block.first.t, back ? block.first.t : block.last.t), false, f));
                    }
                }
                wires.push_back(std::move(loop));
            }
            const size_t boundary = list.size();
            for (const std::vector<int>* blocks : {&f.in_blocks, &f.sc_blocks})
                for (int b : *blocks) {
                    if (ds.micro(ds.blocks[b]))
                        continue;
                    const NurbsCurve& pcurve = ds.pcurves.at({b, static_cast<int>(id)});
                    list.push_back(use_of(b, false, pcurve, true, f));
                    list.push_back(use_of(b, true, reversed(pcurve), true, f));
                }
            std::vector<std::vector<std::vector<int>>> faces;
            if (list.size() == boundary)
                faces.push_back(wires);
            else
                faces = build_faces(list, f.region, report.log);
            for (auto& loops : faces) {
                Piece piece;
                piece.face = static_cast<int>(id);
                piece.loops = std::move(loops);
                for (const auto& loop : piece.loops)
                    for (int k : loop)
                        piece.images.push_back(list[k].image);
                std::sort(piece.images.begin(), piece.images.end());
                piece.images.erase(std::unique(piece.images.begin(), piece.images.end()), piece.images.end());
                pieces.push_back(std::move(piece));
            }
        }
    }

    bool sample(Piece& piece) {
        if (piece.has_point)
            return true;
        const Face& f = ds.faces[piece.face];
        Region region;
        region.eps = f.region.eps;
        for (const auto& loop : piece.loops)
            region.add_loop(loop_polygon(uses[piece.face], loop));
        if (!region.interior(piece.uv))
            return false;
        piece.point = ds.surface_of(f).point_at(piece.uv[0], piece.uv[1]);
        piece.has_point = true;
        return true;
    }

    Vector normal(const Piece& piece, const Point& p) const {
        const Face& f = ds.faces[piece.face];
        const NurbsSurface& s = ds.surface_of(f);
        const auto [u, v] = s.closest_parameters(p);
        Vector n = s.normal_at(u, v);
        if (ds.brep(f.operand).face_orientation(f.local) == BRepOrientation::Reversed)
            n *= -1.0;
        return n;
    }

    /// FillSameDomainFaces: pieces of faces on one surface bounded by the same edge images
    void same_domain() {
        for (const auto& [fa, fb] : ds.same_domain_faces)
            for (Piece& a : pieces)
                if (a.face == fa && a.twin < 0)
                    for (size_t k = 0; k < pieces.size(); ++k)
                        if (pieces[k].face == fb && pieces[k].twin < 0 && pieces[k].images == a.images) {
                            a.twin = static_cast<int>(k);
                            pieces[k].twin = static_cast<int>(&a - pieces.data());
                            a.state = pieces[k].state = 0;
                            break;
                        }
    }

    /// BOPTools_AlgoTools::IsInternalFace: at an edge shared with the other operand, a piece is inside it when it leaves the edge below the nearest of that operand's faces there; per piece the count of edges that say inside and outside
    std::vector<std::array<int, 2>> edge_states(std::map<int, int>& operands) const {
        struct Side {
            int piece;
            Vector inward, outward;
        };
        std::map<int, std::vector<Side>> around;
        for (size_t k = 0; k < pieces.size(); ++k) {
            const Piece& piece = pieces[k];
            const Face& f = ds.faces[piece.face];
            const NurbsSurface& s = ds.surface_of(f);
            for (const auto& loop : piece.loops)
                for (int i : loop) {
                    const EdgeUse& use = uses[piece.face][i];
                    const PaveBlock& block = ds.blocks[use.image];
                    if (operands[use.image] != 3 || (block.edge >= 0 && ds.edges[block.edge].degenerated))
                        continue;
                    const NurbsCurve& c = ds.block_curve(block);
                    const double t = 0.5 * (block.first.t + block.last.t);
                    const Point p = c.point_at(t);
                    Point uv = use.pcurve.point_at(use.pcurve.domain_middle());
                    if (s.point_at(uv[0], uv[1]).distance(p) > 1e-3 * c.point_at(block.first.t).distance(c.point_at(block.last.t))) {
                        const auto [u, v] = s.closest_parameters(p);
                        uv = Point(u, v, 0);
                    }
                    Vector along = c.tangent_at(t), normal = s.normal_at(uv[0], uv[1]);
                    along.normalize_self();
                    normal.normalize_self();
                    if (use.reversed)
                        along *= -1.0;
                    Vector inward = normal.cross(along);
                    inward.normalize_self();
                    around[use.image].push_back({static_cast<int>(k), inward, ds.brep(f.operand).face_orientation(f.local) == BRepOrientation::Reversed ? normal * -1.0 : normal});
                }
        }
        std::vector<std::array<int, 2>> votes(pieces.size(), {0, 0});
        for (const auto& [unused, sides] : around)
            for (const Side& p : sides) {
                if (pieces[p.piece].twin >= 0)
                    continue;
                const Side* nearest = nullptr;
                for (const Side& q : sides)
                    if (ds.faces[pieces[q.piece].face].operand != ds.faces[pieces[p.piece].face].operand && (!nearest || p.inward.dot(q.inward) > p.inward.dot(nearest->inward)))
                        nearest = &q;
                const double height = nearest ? p.inward.dot(nearest->outward) : 0.0;
                if (std::abs(height) > 1e-3)
                    ++votes[p.piece][height < 0.0 ? 0 : 1];
            }
        return votes;
    }

    /// BOPTools_AlgoTools::ComputeState for blocks of pieces joined across edges the other operand does not touch
    void classify() {
        // An edge lying on a face of the other operand bounds a block of pieces, whether or not that face was split along it.
        std::map<int, int> operands;
        for (const Face& f : ds.faces) {
            for (int e : f.edges)
                for (int b : ds.edges[e].blocks)
                    operands[image(b)] |= 1 << f.operand;
            for (const std::vector<int>* blocks : {&f.in_blocks, &f.sc_blocks})
                for (int b : *blocks)
                    operands[image(b)] |= 1 << f.operand;
        }
        std::vector<int> parent(pieces.size());
        std::iota(parent.begin(), parent.end(), 0);
        const auto root = [&](int k) {
            while (parent[k] != k)
                k = parent[k] = parent[parent[k]];
            return k;
        };
        std::map<int, std::vector<int>> by_image;
        for (size_t k = 0; k < pieces.size(); ++k)
            if (pieces[k].twin < 0)
                for (int i : pieces[k].images)
                    if (operands[i] != 3)
                        by_image[i].push_back(static_cast<int>(k));
        for (const auto& [unused, list] : by_image)
            for (int k : list)
                parent[root(k)] = root(list[0]);
        std::map<int, std::vector<int>> groups;
        for (size_t k = 0; k < pieces.size(); ++k)
            if (pieces[k].twin < 0)
                groups[root(static_cast<int>(k))].push_back(static_cast<int>(k));
        const std::vector<std::array<int, 2>> votes = edge_states(operands);
        std::array<std::unique_ptr<brep_classify::Solid>, 2> solids;
        std::array<std::unique_ptr<Mesh>, 2> meshes;
        for (int o = 0; o < 2; ++o)
            try {
                solids[o] = std::make_unique<brep_classify::Solid>(ds.brep(o));
            } catch (const std::exception& error) {
                report.log.push_back(std::string("classifier: ") + error.what());
            }
        for (const auto& [unused, group] : groups) {
            const int other = 1 - ds.faces[pieces[group[0]].face].operand;
            int state = 2, first = -1;
            for (size_t i = 0; i < group.size() && i < 4 && state == 2; ++i) {
                Piece& piece = pieces[group[i]];
                if (!sample(piece))
                    continue;
                if (first < 0)
                    first = group[i];
                if (!solids[other])
                    break;
                try {
                    const double tolerance = std::max(ds.faces[piece.face].tolerance, 1e-7) + ds.fuzzy;
                    const brep_classify::State s = brep_classify::point(*solids[other], piece.point, tolerance);
                    state = s == brep_classify::State::Inside ? 1 : s == brep_classify::State::Outside ? -1 : 2;
                } catch (const std::exception& error) {
                    report.log.push_back(std::string("classify: ") + error.what());
                }
            }
            if (state == 2 && first >= 0) {
                try {
                    if (!meshes[other])
                        meshes[other] = std::make_unique<Mesh>(ds.brep(other).mesh());
                    state = std::abs(winding(*meshes[other], pieces[first].point)) > 0.5 ? 1 : -1;
                    report.log.push_back("classified by winding number");
                } catch (const std::exception& error) {
                    report.log.push_back(std::string("winding number: ") + error.what());
                }
            }
            if (state == 2)
                report.log.push_back("piece without an interior point");
            int in = 0, out = 0;
            for (int k : group) {
                in += votes[k][0];
                out += votes[k][1];
            }
            if (in > 0 && out > 0)
                report.log.push_back("classification conflict: a block of pieces is inside at " + std::to_string(in) + " shared edges and outside at " + std::to_string(out));
            else if (in + out > 0 && (in > 0) != (state == 1)) {
                report.log.push_back("classified at shared edges against the point classifier");
                state = in > 0 ? 1 : -1;
            }
            for (int k : group)
                pieces[k].state = in > 0 && out > 0 && votes[k][0] != votes[k][1] ? (votes[k][0] > votes[k][1] ? 1 : -1) : state == 1 ? 1 : -1;
        }
    }

    int emit_vertex(int v) {
        const auto found = vertex_image.find(v);
        if (found != vertex_image.end())
            return found->second;
        return vertex_image[v] = result.add_vertex(ds.vertices[v].point, ds.vertices[v].tolerance);
    }

    int emit_edge(int r) {
        const auto found = edge_image.find(r);
        if (found != edge_image.end())
            return found->second;
        const PaveBlock& block = ds.blocks[r];
        double tolerance = 0.0;
        for (int b : ds.family(r))
            tolerance = std::max(tolerance, ds.block_tolerance(ds.blocks[b]));
        const int v0 = emit_vertex(block.first.vertex), v1 = emit_vertex(block.last.vertex);
        if (block.edge >= 0 && ds.edges[block.edge].degenerated)
            return edge_image[r] = result.add_edge(-1, v0, v1, tolerance);
        const NurbsCurve& c = ds.block_curve(block);
        const int curve = result.add_curve_3d(trimmed(c, c.domain(), block.first.t, block.last.t));
        return edge_image[r] = result.add_edge(curve, v0, v1, tolerance);
    }

    int emit_face(const Piece& piece) {
        const Face& f = ds.faces[piece.face];
        const BRep& brep = ds.brep(f.operand);
        const int source = brep.m_faces[f.local].surface_index;
        auto found = surface_image.find({f.operand, source});
        if (found == surface_image.end())
            found = surface_image.emplace(std::make_pair(f.operand, source), result.add_surface(brep.m_surfaces[source].nurbs())).first;
        std::vector<BRepRef> wires;
        for (const auto& loop : piece.loops) {
            std::vector<BRepRef> refs;
            for (int k : loop) {
                const EdgeUse& use = uses[piece.face][k];
                const int edge = emit_edge(use.image);
                refs.push_back({edge, use.reversed ? BRepOrientation::Reversed : BRepOrientation::Forward});
                auto& slot = slots[{edge, found->second}][use.reversed];
                if (slot.empty())
                    slot.push_back(use.reversed ? reversed(use.pcurve) : use.pcurve);
            }
            wires.push_back({result.add_wire(refs), BRepOrientation::Forward});
        }
        return result.add_face(found->second, wires, f.tolerance);
    }

    /// BOPAlgo_BOP::BuildRC and BuildShape: the pieces each operation keeps, oriented, then shells and solids
    void build(Operation operation) {
        std::vector<BRepRef> faces;
        for (const Piece& piece : pieces) {
            const Face& f = ds.faces[piece.face];
            bool minus, plus;
            if (piece.state == 0) {
                if (f.operand == 1)
                    continue;
                Piece copy = piece;
                if (!sample(copy))
                    continue;
                const double sense = normal(piece, copy.point).dot(normal(pieces[piece.twin], copy.point));
                minus = occupied(true, sense > 0.0, operation);
                plus = occupied(false, sense < 0.0, operation);
            } else {
                const bool inside = piece.state == 1;
                minus = f.operand == 0 ? occupied(true, inside, operation) : occupied(inside, true, operation);
                plus = f.operand == 0 ? occupied(false, inside, operation) : occupied(inside, false, operation);
            }
            if (minus == plus)
                continue;
            const BRepOrientation o = ds.brep(f.operand).face_orientation(f.local);
            faces.push_back({emit_face(piece), plus ? brep_reverse(o) : o});
        }
        for (auto& [key, pair] : slots) {
            const auto& [edge, surface] = key;
            std::vector<NurbsCurve> forward = pair[0], back = pair[1];
            if (forward.empty())
                std::swap(forward, back);
            const NurbsSurface& s = result.m_surfaces[surface].nurbs();
            const double span = std::hypot(s.domain(0).second - s.domain(0).first, s.domain(1).second - s.domain(1).first);
            const bool seam = !back.empty() && forward[0].point_at_start().distance(back[0].point_at_start()) > 1e-6 * span;
            result.add_pcurve(edge, surface, result.add_curve_2d(forward[0]), seam ? result.add_curve_2d(back[0]) : -1);
        }
        build_solids(result, faces);
    }
};

bool empty(const BRep& b) {
    return b.m_faces.empty();
}

}

BRep apply(const BRep& a, const BRep& b, Operation operation, double fuzzy, Report* report) {
    Report local;
    Report& out = report ? *report : local;
    if (empty(a) || empty(b)) {
        const bool keep_a = !empty(a) && (operation == Operation::Union || operation == Operation::Xor || operation == Operation::Difference);
        const bool keep_b = !empty(b) && (operation == Operation::Union || operation == Operation::Xor || operation == Operation::ReverseDifference);
        return keep_a ? a : keep_b ? b : BRep();
    }
    const DS ds = fill(a, b, fuzzy);
    out.log.insert(out.log.end(), ds.log.begin(), ds.log.end());
    Builder builder(ds, out);
    builder.split_faces();
    builder.same_domain();
    builder.classify();
    builder.build(operation);
    BRep result = builder.result;
    try {
        std::vector<bool> keep(result.vertex_count(), false);
        for (const auto& [v, image] : builder.vertex_image)
            keep[image] = !ds.vertices[v].joint;
        result = brep_boolean::joined_edges(result, std::max(1e-7, fuzzy), keep);
    } catch (const std::exception& error) {
        out.log.push_back(std::string("joined edges: ") + error.what());
    }
    result.update_tolerances();
    out.solid = result.is_solid();
    if (!out.solid && result.face_count() > 0) {
        std::map<int, int> uses;
        for (const BRepFace& face : result.m_faces)
            for (const BRepRef& wire : face.wires)
                for (const BRepRef& use : result.m_wires[wire.index].edges)
                    if (!result.m_edges[use.index].degenerated)
                        ++uses[use.index];
        int open = 0;
        for (const auto& [edge, count] : uses)
            open += count % 2;
        out.log.push_back("not solid: " + std::to_string(open) + " edges used an odd number of times");
        out.failure = "result is not a closed solid";
    }
    for (const std::string& line : out.log)
        if (out.failure.empty() && line.rfind("classification conflict", 0) == 0)
            out.failure = "pieces of one block are inside and outside the other operand";
    return result;
}

}
