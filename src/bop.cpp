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
    std::map<std::pair<int, int>, std::array<std::vector<Span>, 2>> slots;
    std::map<const Curve*, int> curve_image, pcurve_image; // shared curves of the operands and sections -> result pools // (result edge, result surface) -> pcurves of forward and reversed uses

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

    EdgeUse use_of(int b, bool backwards, Span pcurve, bool internal, const Face& f) const {
        const PaveBlock& block = ds.blocks[b];
        const int r = image(b);
        // The tolerance of its vertices in uv: across the slower parameter of the surface a length is a longer way.
        double slow = 1e300;
        for (double end : {0.0, 1.0}) {
            const Point uv = pcurve.at(end);
            const std::vector<Vector> d = ds.surface_of(f).evaluate(uv[0], uv[1], 1);
            slow = std::min(slow, std::max(std::min(d[1].magnitude(), d[2].magnitude()), 1e-2 * std::max(d[1].magnitude(), d[2].magnitude())));
        }
        const double gap = 2.0 * std::max(ds.vertices[block.first.vertex].tolerance, ds.vertices[block.last.vertex].tolerance) / std::max(slow, 1e-300);
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
                    if (!pcurve_span(brep, use.index, f.local, use.orientation).curve) {
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
                        list.push_back(use_of(b, back, ds.edge_pcurve(e, f.local, use.orientation, back ? block.last.t : block.first.t, back ? block.first.t : block.last.t), false, f));
                    }
                }
                wires.push_back(std::move(loop));
            }
            const size_t boundary = list.size();
            for (const std::vector<int>* blocks : {&f.in_blocks, &f.sc_blocks})
                for (int b : *blocks) {
                    if (ds.micro(ds.blocks[b]))
                        continue;
                    const Span& pcurve = ds.pcurves.at({b, static_cast<int>(id)});
                    list.push_back(use_of(b, false, pcurve, true, f));
                    list.push_back(use_of(b, true, pcurve.reversed(), true, f));
                }
            std::vector<std::vector<std::vector<int>>> faces;
            if (list.size() == boundary)
                faces.push_back(wires);
            else
            {
                const size_t noted = report.log.size();
                faces = build_faces(list, f.region, report.log);
                for (size_t k = noted; k < report.log.size(); ++k)
                    report.log[k] += std::string(" (face ") + "AB"[f.operand] + std::to_string(f.local) + ")";
            }
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

    /// The loops of a piece as polygons, where each arrives and which periodic direction it runs around. On an analytic surface every loop is in one piece, its uses moved by whole periods (turns), and the holes of a piece that closes lie in the turn of its outer loop.
    struct Layout {
        std::vector<std::vector<Point>> polygons;
        std::vector<Point> ends;
        std::vector<int> winds;
        std::vector<std::vector<std::array<double, 2>>> turns;
        bool band = false;
    };

    Layout layout(const Piece& piece) const {
        const Region& whole = ds.faces[piece.face].region;
        Layout l;
        for (const auto& loop : piece.loops) {
            l.ends.emplace_back();
            l.turns.emplace_back();
            l.polygons.push_back(loop.empty() ? std::vector<Point>{} : loop_polygon(uses[piece.face], loop, whole.whole ? whole.period : std::array<double, 2>{0.0, 0.0}, &l.ends.back(), &l.turns.back()));
            l.winds.push_back(loop.empty() ? -1 : whole.winding(l.polygons.back().front(), l.ends.back()));
            l.band = l.band || l.winds.back() >= 0;
        }
        const auto middle = [](const std::vector<Point>& polygon, int d) {
            double lo = 1e300, hi = -1e300;
            for (const Point& p : polygon)
                lo = std::min(lo, p[d]), hi = std::max(hi, p[d]);
            return 0.5 * (lo + hi);
        };
        for (size_t k = 1; k < l.polygons.size() && whole.whole && !l.band && !l.polygons[0].empty() && !l.polygons[k].empty(); ++k)
            for (int d = 0; d < 2; ++d) {
                const double move = whole.period[d] > 0.0 ? std::round((middle(l.polygons[0], d) - middle(l.polygons[k], d)) / whole.period[d]) * whole.period[d] : 0.0;
                for (Point& p : l.polygons[k])
                    p[d] += move;
                for (std::array<double, 2>& turn : l.turns[k])
                    turn[d] += move;
                l.ends[k][d] += move;
            }
        return l;
    }

    bool sample(Piece& piece) {
        if (piece.has_point)
            return true;
        const Face& f = ds.faces[piece.face];
        Region region;
        region.eps = f.region.eps;
        Layout l = layout(piece);
        for (size_t k = 0; k < l.polygons.size(); ++k) {
            if (l.polygons[k].empty())
                continue;
            if (l.winds[k] >= 0) {
                l.polygons[k].push_back(l.ends[k]);
                region.period = f.region.period;
            }
            region.add_loop(std::move(l.polygons[k]), l.winds[k]);
        }
        if (!region.interior(piece.uv))
            return false;
        piece.point = ds.surface_of(f).point_at(piece.uv[0], piece.uv[1]);
        piece.has_point = true;
        return true;
    }

    Vector normal(const Piece& piece, const Point& p) const {
        const Face& f = ds.faces[piece.face];
        const Surface& s = ds.surface_of(f);
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
            const Surface& s = ds.surface_of(f);
            for (const auto& loop : piece.loops)
                for (int i : loop) {
                    const EdgeUse& use = uses[piece.face][i];
                    const PaveBlock& block = ds.blocks[use.image];
                    if (operands[use.image] != 3 || (block.edge >= 0 && ds.edges[block.edge].degenerated))
                        continue;
                    const Curve& c = ds.block_curve(block);
                    const double t = 0.5 * (block.first.t + block.last.t);
                    const Point p = c.point_at(t);
                    Point uv = use.pcurve.at(0.5);
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
        // The point classifier reads NURBS.
        std::array<BRep, 2> plain;
        for (int o = 0; o < 2; ++o)
            try {
                if (!ds.brep(o).is_nurbs())
                    plain[o] = ds.brep(o).as_nurbs();
                solids[o] = std::make_unique<brep_classify::Solid>(ds.brep(o).is_nurbs() ? ds.brep(o) : plain[o]);
            } catch (const std::exception& error) {
                report.log.push_back(std::string("classifier: ") + error.what());
            }
        for (const auto& [unused, group] : groups) {
            const int other = 1 - ds.faces[pieces[group[0]].face].operand;
            int state = 2, first = -1, in = 0, out = 0;
            for (int k : group) {
                in += votes[k][0];
                out += votes[k][1];
            }
            // Shared edges that all say the same decide, as they would against the point classifier: it is asked only where they are silent or split.
            if ((in > 0) != (out > 0))
                state = in > 0 ? 1 : -1;
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

    /// Whether a span is all of its curve
    static bool whole(const Span& span) {
        const auto [t0, t1] = span.curve->domain();
        return std::abs(span.t0 - t0) + std::abs(span.t1 - t1) <= 1e-12 * std::max(1.0, std::abs(t1 - t0));
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
        // A split edge is its curve with a new range: the curve goes into the result once.
        const Span span = ds.block_span(block);
        auto shared = curve_image.find(span.curve.get());
        if (shared == curve_image.end())
            shared = curve_image.emplace(span.curve.get(), result.add_curve_3d(*span.curve)).first;
        return edge_image[r] = result.add_edge(shared->second, v0, v1, tolerance, whole(span) ? std::array<double, 2>{0.0, 0.0} : std::array<double, 2>{span.t0, span.t1});
    }

    int emit_face(const Piece& piece) {
        const Face& f = ds.faces[piece.face];
        const BRep& brep = ds.brep(f.operand);
        const int source = brep.m_faces[f.local].surface_index;
        // A piece that closes on a periodic analytic surface is written in one turn: the one its outer loop starts in from the surface's rectangle, and where it runs past that, on the surface over a rectangle of its own.
        const Surface& s = brep.m_surfaces[source];
        Layout l;
        Surface own;
        if (f.region.whole && (s.period(0) > 0.0 || s.period(1) > 0.0) && !(l = layout(piece)).band && !l.polygons.empty() && !l.polygons[0].empty())
            for (int d = 0; d < 2; ++d) {
                const double period = s.period(d), e = 1e-9 * period;
                if (!(period > 0.0))
                    continue;
                double lo = 1e300, hi = -1e300;
                for (const Point& p : l.polygons[0])
                    lo = std::min(lo, p[d]), hi = std::max(hi, p[d]);
                const double move = -std::floor((lo - s.domain(d).first + e) / period) * period;
                for (auto& loop : l.turns)
                    for (std::array<double, 2>& turn : loop)
                        turn[d] += move;
                if (hi + move > s.domain(d).second + e) {
                    if (!own.is_valid())
                        own = s;
                    own.set_domain(d, lo + move, hi + move);
                }
            }
        else
            l.turns.clear();
        int surface = 0;
        if (own.is_valid())
            surface = result.add_surface(own);
        else {
            auto found = surface_image.find({f.operand, source});
            if (found == surface_image.end())
                found = surface_image.emplace(std::make_pair(f.operand, source), result.add_surface(s)).first;
            surface = found->second;
        }
        std::vector<BRepRef> wires;
        for (size_t i = 0; i < piece.loops.size(); ++i) {
            std::vector<BRepRef> refs;
            for (size_t k = 0; k < piece.loops[i].size(); ++k) {
                const EdgeUse& use = uses[piece.face][piece.loops[i][k]];
                const int edge = emit_edge(use.image);
                refs.push_back({edge, use.reversed ? BRepOrientation::Reversed : BRepOrientation::Forward});
                auto& slot = slots[{edge, surface}][use.reversed];
                if (!slot.empty())
                    continue;
                slot.push_back(use.reversed ? use.pcurve.reversed() : use.pcurve);
                for (int d = 0; d < 2 && !l.turns.empty(); ++d)
                    slot.back().shift[d] += l.turns[i][k][d];
            }
            wires.push_back({result.add_wire(refs), BRepOrientation::Forward});
        }
        return result.add_face(surface, wires, f.tolerance);
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
            std::vector<Span> forward = pair[0], back = pair[1];
            if (forward.empty())
                std::swap(forward, back);
            const Surface& s = result.m_surfaces[surface];
            const double span = std::hypot(s.domain(0).second - s.domain(0).first, s.domain(1).second - s.domain(1).first);
            const bool seam = !back.empty() && forward[0].at(0.0).distance(back[0].at(0.0)) > 1e-6 * span;
            // A pcurve running with its edge and not moved by a period is shared with its range; one that runs against it or was moved gets a curve of its own.
            std::array<int, 2> index{-1, -1};
            std::array<std::array<double, 2>, 2> range{};
            for (int k = 0; k < (seam ? 2 : 1); ++k) {
                const Span& p = k ? back[0] : forward[0];
                if (p.t0 < p.t1 && p.shift[0] == 0.0 && p.shift[1] == 0.0) {
                    auto shared = pcurve_image.find(p.curve.get());
                    if (shared == pcurve_image.end())
                        shared = pcurve_image.emplace(p.curve.get(), result.add_curve_2d(*p.curve)).first;
                    index[k] = shared->second;
                    range[k] = whole(p) ? std::array<double, 2>{0.0, 0.0} : std::array<double, 2>{p.t0, p.t1};
                } else
                    index[k] = result.add_curve_2d(p.copy());
            }
            result.add_pcurve(edge, surface, index[0], index[1], range[0], range[1]);
        }
        build_solids(result, faces);
    }
};

/// BRepLib::UpdateTolerances as BRep::update_tolerances, on the edges' and pcurves' ranges so that the shared curves stay shared
void grow_tolerances(BRep& b) {
    for (int i = 0; i < b.edge_count(); ++i) {
        BRepEdge& e = b.m_edges[i];
        BRepVertex& vs = b.m_vertices[e.start_vertex];
        BRepVertex& ve = b.m_vertices[e.end_vertex];
        double tol = e.tolerance;
        if (e.curve_3d_index >= 0) {
            const Span c = edge_span(b, i);
            tol = std::max({tol, c.at(0.0).distance(vs.point), c.at(1.0).distance(ve.point)});
        }
        for (const BRepCurveOnSurface& pc : e.pcurves)
            for (bool second : {false, true}) {
                if (second && pc.curve_2d_index_2 < 0)
                    continue;
                const auto [t0, t1] = b.pcurve_range(pc, second);
                const Curve2d& p = b.m_curves_2d[second ? pc.curve_2d_index_2 : pc.curve_2d_index];
                const Point a = p.point_at(t0), z = p.point_at(t1);
                const Surface& s = b.m_surfaces[pc.surface_index];
                tol = std::max({tol, s.point_at(a[0], a[1]).distance(vs.point), s.point_at(z[0], z[1]).distance(ve.point)});
            }
        e.tolerance = tol;
        vs.tolerance = std::max(vs.tolerance, tol);
        ve.tolerance = std::max(ve.tolerance, tol);
    }
}

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
        // Joining refits curves, so it runs only where pieces of one section met: any other result keeps its shared curves and ranges.
        std::vector<bool> keep(result.vertex_count(), false);
        std::vector<int> ends(result.vertex_count(), 0);
        for (const BRepEdge& e : result.m_edges) {
            ++ends[e.start_vertex];
            ++ends[e.end_vertex];
        }
        bool joints = false;
        for (const auto& [v, image] : builder.vertex_image) {
            keep[image] = !ds.vertices[v].joint;
            joints = joints || (ds.vertices[v].joint && ends[image] == 2);
        }
        if (joints)
            result = brep_boolean::joined_edges(result, std::max(1e-7, fuzzy), keep);
    } catch (const std::exception& error) {
        out.log.push_back(std::string("joined edges: ") + error.what());
    }
    grow_tolerances(result);
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
    // Where the analytic operands do not give a solid, their NURBS may: the result is then NURBS.
    if (!out.failure.empty() && (!a.is_freeform() || !b.is_freeform())) {
        Report plain;
        const BRep through = apply(a.as_nurbs(false), b.as_nurbs(false), operation, fuzzy, &plain);
        if (plain.failure.empty()) {
            plain.log.insert(plain.log.begin(), "analytic operands: " + out.failure + "; result from their NURBS");
            out = plain;
            return through;
        }
    }
    return result;
}

}
