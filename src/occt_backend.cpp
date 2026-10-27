#include "occt_backend.h"
#include "intersection.h"
#include <cmath>
#include <stdexcept>

#ifdef SESSION_WITH_OCCT
#include <BOPAlgo_Operation.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepAlgoAPI_BooleanOperation.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepLib.hxx>
#include <BRepOffsetAPI_MakeOffsetShape.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom2dConvert.hxx>
#include <GeomConvert.hxx>
#include <GeomConvert_ApproxCurve.hxx>
#include <GeomConvert_ApproxSurface.hxx>
#include <Geom_OffsetCurve.hxx>
#include <Geom_OffsetSurface.hxx>
#include <GeomInt_IntSS.hxx>
#include <GProp_GProps.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <gp_Pnt2d.hxx>
#include <ShapeFix_Shape.hxx>
#include <ShapeBuild_ReShape.hxx>
#include <ShapeUpgrade_ShapeDivideContinuity.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <NCollection_IndexedMap.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <NCollection_List.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Solid.hxx>

namespace session_cpp::occt {
namespace {

struct Knots {
    std::vector<double> values;
    std::vector<int> counts;

    explicit Knots(const std::vector<double>& shortened) {
        std::vector<double> full{shortened.front()};
        full.insert(full.end(), shortened.begin(), shortened.end());
        full.push_back(shortened.back());
        for (double knot : full) {
            if (values.empty() || values.back() != knot) {
                values.push_back(knot);
                counts.push_back(1);
            } else {
                ++counts.back();
            }
        }
    }

    NCollection_Array1<double> knots() const {
        NCollection_Array1<double> result(1, (int)values.size());
        for (int i = 0; i < (int)values.size(); ++i)
            result.SetValue(i + 1, values[i]);
        return result;
    }

    NCollection_Array1<int> multiplicities() const {
        NCollection_Array1<int> result(1, (int)counts.size());
        for (int i = 0; i < (int)counts.size(); ++i)
            result.SetValue(i + 1, counts[i]);
        return result;
    }
};

Handle(Geom_BSplineCurve) to_curve(const NurbsCurve& curve) {
    if (!curve.is_valid())
        throw std::invalid_argument("OCCT requires a valid NURBS curve");
    NCollection_Array1<gp_Pnt> poles(1, curve.cv_count());
    NCollection_Array1<double> weights(1, curve.cv_count());
    for (int i = 0; i < curve.cv_count(); ++i) {
        const Point point = curve.get_cv(i);
        poles.SetValue(i + 1, gp_Pnt(point[0], point[1], point[2]));
        weights.SetValue(i + 1, curve.weight(i));
    }
    const Knots knots(curve.m_nurbsknot);
    return new Geom_BSplineCurve(poles, weights, knots.knots(), knots.multiplicities(), curve.degree());
}

Handle(Geom2d_BSplineCurve) to_pcurve(const NurbsCurve& curve) {
    NCollection_Array1<gp_Pnt2d> poles(1, curve.cv_count());
    NCollection_Array1<double> weights(1, curve.cv_count());
    for (int i = 0; i < curve.cv_count(); ++i) {
        const Point point = curve.get_cv(i);
        poles.SetValue(i + 1, gp_Pnt2d(point[0], point[1]));
        weights.SetValue(i + 1, curve.weight(i));
    }
    const Knots knots(curve.m_nurbsknot);
    return new Geom2d_BSplineCurve(poles, weights, knots.knots(), knots.multiplicities(), curve.degree());
}

Handle(Geom_BSplineSurface) to_surface(const NurbsSurface& surface) {
    if (!surface.is_valid())
        throw std::invalid_argument("OCCT requires a valid NURBS surface");
    NCollection_Array2<gp_Pnt> poles(1, surface.cv_count(0), 1, surface.cv_count(1));
    NCollection_Array2<double> weights(1, surface.cv_count(0), 1, surface.cv_count(1));
    for (int i = 0; i < surface.cv_count(0); ++i)
        for (int j = 0; j < surface.cv_count(1); ++j) {
            double x, y, z, w;
            surface.get_cv_4d(i, j, x, y, z, w);
            poles.SetValue(i + 1, j + 1, gp_Pnt(x / w, y / w, z / w));
            weights.SetValue(i + 1, j + 1, w);
        }
    const Knots u(surface.m_nurbsknot[0]);
    const Knots v(surface.m_nurbsknot[1]);
    return new Geom_BSplineSurface(
        poles, weights, u.knots(), v.knots(), u.multiplicities(), v.multiplicities(),
        surface.degree(0), surface.degree(1)
    );
}

TopoDS_Shape to_shape(const BRep& brep, double tolerance, std::vector<TopoDS_Face>* face_map = nullptr) {
    if (brep.vertex_count() == 0 && brep.edge_count() == 0 && brep.face_count() == 0) {
        BRep_Builder builder;
        TopoDS_Compound empty;
        builder.MakeCompound(empty);
        return empty;
    }
    if (!brep.is_valid() || (brep.solid_count() > 0 && !brep.is_solid()))
        throw std::invalid_argument("OCCT requires a valid session BRep");

    BRep_Builder builder;
    std::vector<Handle(Geom_BSplineSurface)> surfaces;
    for (const NurbsSurface& surface : brep.m_surfaces)
        surfaces.push_back(to_surface(surface));

    std::vector<TopoDS_Vertex> vertices(brep.vertex_count());
    for (int i = 0; i < brep.vertex_count(); ++i) {
        const BRepVertex& vertex = brep.m_vertices[i];
        const Point& point = vertex.point;
        builder.MakeVertex(vertices[i], gp_Pnt(point[0], point[1], point[2]), std::max(tolerance, vertex.tolerance));
    }

    std::vector<TopoDS_Edge> edges(brep.edge_count());
    for (int i = 0; i < brep.edge_count(); ++i) {
        const BRepEdge& edge = brep.m_edges[i];
        builder.MakeEdge(edges[i]);
        builder.Degenerated(edges[i], edge.degenerated);
        builder.Add(edges[i], vertices[edge.start_vertex].Oriented(TopAbs_FORWARD));
        builder.Add(edges[i], vertices[edge.end_vertex].Oriented(TopAbs_REVERSED));
        std::pair<double, double> range{0.0, 1.0};
        if (!edge.degenerated) {
            const NurbsCurve& curve = brep.m_curves_3d[edge.curve_3d_index];
            range = curve.domain();
            builder.UpdateEdge(edges[i], to_curve(curve), std::max(tolerance, edge.tolerance));
        } else if (!edge.pcurves.empty()) {
            range = brep.m_curves_2d[edge.pcurves[0].curve_2d_index].domain();
        }

        for (const BRepCurveOnSurface& pcurve : edge.pcurves) {
            NurbsCurve first = brep.m_curves_2d[pcurve.curve_2d_index];
            first.set_domain(range.first, range.second);
            const Handle(Geom2d_BSplineCurve) forward = to_pcurve(first);
            if (pcurve.curve_2d_index_2 >= 0) {
                NurbsCurve second = brep.m_curves_2d[pcurve.curve_2d_index_2];
                second.set_domain(range.first, range.second);
                builder.UpdateEdge(edges[i], forward, to_pcurve(second), surfaces[pcurve.surface_index], TopLoc_Location(), tolerance);
            } else {
                builder.UpdateEdge(edges[i], forward, surfaces[pcurve.surface_index], TopLoc_Location(), tolerance);
            }
        }
        builder.Range(edges[i], range.first, range.second);
        builder.SameParameter(edges[i], false);
    }

    std::vector<TopoDS_Wire> wires(brep.wire_count());
    for (int i = 0; i < brep.wire_count(); ++i) {
        builder.MakeWire(wires[i]);
        for (const BRepRef& edge : brep.m_wires[i].edges)
            builder.Add(wires[i], edges[edge.index].Oriented((TopAbs_Orientation)edge.orientation));
    }
    std::vector<TopoDS_Face> faces(brep.face_count());
    for (int i = 0; i < brep.face_count(); ++i) {
        const BRepFace& face = brep.m_faces[i];
        builder.MakeFace(faces[i], surfaces[face.surface_index], std::max(tolerance, face.tolerance));
        for (const BRepRef& wire : face.wires)
            builder.Add(faces[i], wires[wire.index].Oriented((TopAbs_Orientation)wire.orientation));
    }
    std::vector<TopoDS_Shell> shells(brep.shell_count());
    for (int i = 0; i < brep.shell_count(); ++i) {
        builder.MakeShell(shells[i]);
        for (const BRepRef& face : brep.m_shells[i].faces)
            builder.Add(shells[i], faces[face.index].Oriented((TopAbs_Orientation)face.orientation));
        shells[i].Closed(brep.is_closed(i));
    }

    TopoDS_Compound result;
    builder.MakeCompound(result);
    for (const BRepSolid& solid : brep.m_solids) {
        TopoDS_Solid shape;
        builder.MakeSolid(shape);
        for (const BRepRef& shell : solid.shells)
            builder.Add(shape, shells[shell.index].Oriented((TopAbs_Orientation)shell.orientation));
        builder.Add(result, shape);
    }
    if (brep.solid_count() == 0) {
        if (brep.shell_count() > 0)
            for (const TopoDS_Shell& shell : shells)
                builder.Add(result, shell);
        else
            for (const TopoDS_Face& face : faces)
                builder.Add(result, face);
    }
    BRepLib::SameParameter(result, tolerance, true);
    ShapeFix_Shape repair(result);
    repair.SetPrecision(tolerance);
    repair.SetMaxTolerance(tolerance * 10.0);
    repair.Perform();
    TopoDS_Shape shape = repair.Shape();
    if (shape.ShapeType() == TopAbs_COMPOUND) {
        TopoDS_Iterator iterator(shape);
        if (iterator.More()) {
            const TopoDS_Shape child = iterator.Value();
            iterator.Next();
            if (!iterator.More())
                shape = child;
        }
    }
    if (!BRepCheck_Analyzer(shape).IsValid())
        throw std::runtime_error("Session BRep conversion produced invalid OCCT topology");
    if (face_map) {
        for (const TopoDS_Face& face : faces) {
            const TopoDS_Shape mapped = repair.Context()->Apply(face);
            if (mapped.ShapeType() != TopAbs_FACE)
                throw std::runtime_error("OCCT repair split a requested face");
            face_map->push_back(TopoDS::Face(mapped));
        }
    }
    return shape;
}

NurbsCurve from_curve(const Handle(Geom_Curve)& geometry, double first, double last) {
    Handle(Geom_BSplineCurve) curve = Handle(Geom_BSplineCurve)::DownCast(geometry);
    if (!curve.IsNull()) {
        curve = Handle(Geom_BSplineCurve)::DownCast(curve->Copy());
        curve->Segment(first, last);
    } else {
        const Handle(Geom_TrimmedCurve) trimmed = new Geom_TrimmedCurve(geometry, first, last);
        curve = GeomConvert::CurveToBSplineCurve(trimmed);
    }
    if (curve->IsPeriodic())
        curve->SetNotPeriodic();
    NurbsCurve result(3, curve->IsRational(), curve->Degree() + 1, curve->NbPoles());
    std::vector<double> knots;
    for (int i = 1; i <= curve->NbKnots(); ++i)
        for (int j = 0; j < curve->Multiplicity(i); ++j)
            knots.push_back(curve->Knot(i));
    result.m_nurbsknot.assign(knots.begin() + 1, knots.end() - 1);
    for (int i = 1; i <= curve->NbPoles(); ++i) {
        const gp_Pnt point = curve->Pole(i);
        const double weight = curve->Weight(i);
        if (result.is_rational())
            result.set_cv_4d(i - 1, point.X() * weight, point.Y() * weight, point.Z() * weight, weight);
        else
            result.set_cv(i - 1, {point.X(), point.Y(), point.Z()});
    }
    return result;
}

NurbsCurve from_pcurve(const Handle(Geom2d_Curve)& geometry, double first, double last) {
    Handle(Geom2d_BSplineCurve) curve = Handle(Geom2d_BSplineCurve)::DownCast(geometry);
    if (!curve.IsNull()) {
        curve = Handle(Geom2d_BSplineCurve)::DownCast(curve->Copy());
        curve->Segment(first, last);
    } else {
        const Handle(Geom2d_TrimmedCurve) trimmed = new Geom2d_TrimmedCurve(geometry, first, last);
        curve = Geom2dConvert::CurveToBSplineCurve(trimmed);
    }
    if (curve->IsPeriodic())
        curve->SetNotPeriodic();
    NurbsCurve result(3, curve->IsRational(), curve->Degree() + 1, curve->NbPoles());
    std::vector<double> knots;
    for (int i = 1; i <= curve->NbKnots(); ++i)
        for (int j = 0; j < curve->Multiplicity(i); ++j)
            knots.push_back(curve->Knot(i));
    result.m_nurbsknot.assign(knots.begin() + 1, knots.end() - 1);
    for (int i = 1; i <= curve->NbPoles(); ++i) {
        const gp_Pnt2d point = curve->Pole(i);
        const double weight = curve->Weight(i);
        if (result.is_rational())
            result.set_cv_4d(i - 1, point.X() * weight, point.Y() * weight, 0.0, weight);
        else
            result.set_cv(i - 1, {point.X(), point.Y(), 0.0});
    }
    return result;
}

NurbsSurface from_surface(const Handle(Geom_BSplineSurface)& original) {
    const Handle(Geom_BSplineSurface) surface = Handle(Geom_BSplineSurface)::DownCast(original->Copy());
    if (surface->IsUPeriodic())
        surface->SetUNotPeriodic();
    if (surface->IsVPeriodic())
        surface->SetVNotPeriodic();
    NurbsSurface result(
        3, surface->IsURational() || surface->IsVRational(),
        surface->UDegree() + 1, surface->VDegree() + 1, surface->NbUPoles(), surface->NbVPoles()
    );
    for (int direction = 0; direction < 2; ++direction) {
        std::vector<double> knots;
        const int count = direction == 0 ? surface->NbUKnots() : surface->NbVKnots();
        for (int i = 1; i <= count; ++i) {
            const int multiplicity = direction == 0 ? surface->UMultiplicity(i) : surface->VMultiplicity(i);
            const double knot = direction == 0 ? surface->UKnot(i) : surface->VKnot(i);
            knots.insert(knots.end(), multiplicity, knot);
        }
        result.m_nurbsknot[direction].assign(knots.begin() + 1, knots.end() - 1);
    }
    for (int i = 1; i <= surface->NbUPoles(); ++i)
        for (int j = 1; j <= surface->NbVPoles(); ++j) {
            const gp_Pnt point = surface->Pole(i, j);
            const double weight = surface->Weight(i, j);
            if (result.is_rational())
                result.set_cv_4d(i - 1, j - 1, point.X() * weight, point.Y() * weight, point.Z() * weight, weight);
            else
                result.set_cv(i - 1, j - 1, {point.X(), point.Y(), point.Z()});
        }
    return result;
}

NurbsSurface from_surface(const TopoDS_Face& face) {
    Handle(Geom_Surface) geometry = BRep_Tool::Surface(face);
    Handle(Geom_BSplineSurface) surface = Handle(Geom_BSplineSurface)::DownCast(geometry);
    if (surface.IsNull()) {
        double u0, u1, v0, v1;
        BRepTools::UVBounds(face, u0, u1, v0, v1);
        geometry = new Geom_RectangularTrimmedSurface(geometry, u0, u1, v0, v1);
        surface = GeomConvert::SurfaceToBSplineSurface(geometry);
    }
    return from_surface(surface);
}

BRep from_shape(const TopoDS_Shape& source) {
    const TopoDS_Shape shape = source;
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> vertices, edges, faces, shells, solids;
    TopExp::MapShapes(shape, TopAbs_VERTEX, vertices);
    TopExp::MapShapes(shape, TopAbs_EDGE, edges);
    TopExp::MapShapes(shape, TopAbs_FACE, faces);
    TopExp::MapShapes(shape, TopAbs_SHELL, shells);
    TopExp::MapShapes(shape, TopAbs_SOLID, solids);
    BRep result;
    for (int i = 1; i <= vertices.Extent(); ++i) {
        const TopoDS_Vertex vertex = TopoDS::Vertex(vertices(i));
        const gp_Pnt point = BRep_Tool::Pnt(vertex);
        result.add_vertex({point.X(), point.Y(), point.Z()}, BRep_Tool::Tolerance(vertex));
    }
    for (int i = 1; i <= edges.Extent(); ++i) {
        const TopoDS_Edge edge = TopoDS::Edge(edges(i).Oriented(TopAbs_FORWARD));
        TopoDS_Vertex first, last;
        TopExp::Vertices(edge, first, last);
        int curve = -1;
        if (!BRep_Tool::Degenerated(edge)) {
            double start, end;
            const Handle(Geom_Curve) geometry = BRep_Tool::Curve(edge, start, end);
            if (geometry.IsNull())
                throw std::runtime_error("OCCT result has no 3D edge curve");
            curve = result.add_curve_3d(from_curve(geometry, start, end));
        }
        result.add_edge(curve, vertices.FindIndex(first) - 1, vertices.FindIndex(last) - 1, BRep_Tool::Tolerance(edge));
    }
    for (int i = 1; i <= faces.Extent(); ++i) {
        const TopoDS_Face face = TopoDS::Face(faces(i).Oriented(TopAbs_FORWARD));
        const int surface = result.add_surface(from_surface(face));
        std::vector<BRepRef> wires;
        const TopoDS_Wire outer = BRepTools::OuterWire(face);
        for (TopoDS_Iterator iterator(face); iterator.More(); iterator.Next()) {
            if (iterator.Value().ShapeType() != TopAbs_WIRE)
                continue;
            const TopoDS_Wire wire = TopoDS::Wire(iterator.Value());
            std::vector<BRepRef> uses;
            for (BRepTools_WireExplorer explorer(TopoDS::Wire(wire.Oriented(TopAbs_FORWARD)), face); explorer.More(); explorer.Next()) {
                const TopoDS_Edge edge = explorer.Current();
                const int index = edges.FindIndex(edge) - 1;
                uses.push_back({index, (BRepOrientation)edge.Orientation()});
                double start, end;
                const TopoDS_Edge forward = TopoDS::Edge(edge.Oriented(TopAbs_FORWARD));
                const Handle(Geom2d_Curve) first = BRep_Tool::CurveOnSurface(forward, face, start, end);
                if (first.IsNull())
                    throw std::runtime_error("OCCT result has no face pcurve");
                const int pcurve = result.add_curve_2d(from_pcurve(first, start, end));
                int reverse = -1;
                if (BRep_Tool::IsClosed(edge, face)) {
                    const TopoDS_Edge reversed = TopoDS::Edge(edge.Oriented(TopAbs_REVERSED));
                    const Handle(Geom2d_Curve) second = BRep_Tool::CurveOnSurface(reversed, face, start, end);
                    reverse = result.add_curve_2d(from_pcurve(second, start, end));
                }
                result.add_pcurve(index, surface, pcurve, reverse);
            }
            const BRepRef reference{result.add_wire(uses), (BRepOrientation)wire.Orientation()};
            if (wire.IsSame(outer))
                wires.insert(wires.begin(), reference);
            else
                wires.push_back(reference);
        }
        result.add_face(surface, wires, BRep_Tool::Tolerance(face));
    }
    for (int i = 1; i <= shells.Extent(); ++i) {
        std::vector<BRepRef> uses;
        for (TopoDS_Iterator iterator(shells(i).Oriented(TopAbs_FORWARD)); iterator.More(); iterator.Next())
            uses.push_back({faces.FindIndex(iterator.Value()) - 1, (BRepOrientation)iterator.Value().Orientation()});
        result.add_shell(uses);
    }
    for (int i = 1; i <= solids.Extent(); ++i) {
        std::vector<BRepRef> uses;
        for (TopoDS_Iterator iterator(solids(i).Oriented(TopAbs_FORWARD)); iterator.More(); iterator.Next())
            uses.push_back({shells.FindIndex(iterator.Value()) - 1, (BRepOrientation)iterator.Value().Orientation()});
        result.add_solid(uses);
    }
    for (size_t i = 0; i < result.m_curves_3d.size(); ++i)
        if (!result.m_curves_3d[i].is_valid())
            throw std::runtime_error("Invalid OCCT result curve " + std::to_string(i));
    for (size_t i = 0; i < result.m_curves_2d.size(); ++i)
        if (!result.m_curves_2d[i].is_valid())
            throw std::runtime_error("Invalid OCCT result pcurve " + std::to_string(i));
    for (size_t i = 0; i < result.m_surfaces.size(); ++i)
        if (!result.m_surfaces[i].is_valid())
            throw std::runtime_error("Invalid OCCT result surface " + std::to_string(i));
    if (!result.is_valid())
        throw std::runtime_error("Invalid OCCT result wire topology");
    if (result.face_count() > 0 && solids.Extent() > 0 && !result.is_solid())
        throw std::runtime_error("OCCT result could not be represented as valid session topology");
    return result;
}

TopoDS_Shape compute_boolean(
    const TopoDS_Shape& first,
    const TopoDS_Shape& second,
    BOPAlgo_Operation operation,
    double tolerance
) {
    NCollection_List<TopoDS_Shape> arguments, tools;
    arguments.Append(first);
    tools.Append(second);
    BRepAlgoAPI_BooleanOperation builder;
    builder.SetArguments(arguments);
    builder.SetTools(tools);
    builder.SetOperation(operation);
    builder.SetNonDestructive(true);
    builder.SetFuzzyValue(tolerance);
    builder.Build();
    if (!builder.IsDone() || builder.HasErrors())
        throw std::runtime_error("OCCT Boolean operation failed");
    const TopoDS_Shape result = builder.Shape();
    if (!BRepCheck_Analyzer(result).IsValid())
        throw std::runtime_error("OCCT Boolean result is invalid");
    return result;
}

void check_tolerance(double tolerance) {
    if (!std::isfinite(tolerance) || tolerance <= 0.0)
        throw std::invalid_argument("OCCT operations require a finite positive tolerance");
}

SectionResult compute_section(const TopoDS_Shape& a, const TopoDS_Shape& b, double tolerance) {
    check_tolerance(tolerance);
    BRepAlgoAPI_Section operation(a, b, false);
    operation.SetFuzzyValue(tolerance);
    operation.SetNonDestructive(true);
    operation.Approximation(true);
    operation.Build();
    if (!operation.IsDone() || operation.HasErrors())
        throw std::runtime_error("OCCT section failed");

    const TopoDS_Shape shape = operation.Shape();
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> edges, vertices, endpoints;
    TopExp::MapShapes(shape, TopAbs_EDGE, edges);
    TopExp::MapShapes(shape, TopAbs_VERTEX, vertices);
    SectionResult result;
    for (int i = 1; i <= edges.Extent(); ++i) {
        const TopoDS_Edge edge = TopoDS::Edge(edges(i));
        TopExp::MapShapes(edge, TopAbs_VERTEX, endpoints);
        if (BRep_Tool::Degenerated(edge))
            continue;
        double first, last;
        const Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
        result.curves.push_back(from_curve(curve, first, last));
    }
    for (int i = 1; i <= vertices.Extent(); ++i) {
        if (endpoints.Contains(vertices(i)))
            continue;
        const gp_Pnt point = BRep_Tool::Pnt(TopoDS::Vertex(vertices(i)));
        result.points.push_back({point.X(), point.Y(), point.Z()});
    }
    return result;
}

} // namespace

BRep boolean_operation(const BRep& first, const BRep& second, BooleanOperation operation, double tolerance) {
    check_tolerance(tolerance);
    for (const BRep* shape : {&first, &second})
        if (shape->face_count() > 0 && !shape->is_solid())
            throw std::invalid_argument("Solid Booleans require closed input solids");
    try {
        const TopoDS_Shape a = to_shape(first, tolerance);
        const TopoDS_Shape b = to_shape(second, tolerance);
        TopoDS_Shape result;
        if (operation == BooleanOperation::Xor) {
            const TopoDS_Shape left = compute_boolean(a, b, BOPAlgo_CUT, tolerance);
            const TopoDS_Shape right = compute_boolean(b, a, BOPAlgo_CUT, tolerance);
            result = compute_boolean(left, right, BOPAlgo_FUSE, tolerance);
        } else {
            const BOPAlgo_Operation kind = operation == BooleanOperation::Union ? BOPAlgo_FUSE
                : operation == BooleanOperation::Intersection ? BOPAlgo_COMMON : BOPAlgo_CUT;
            result = compute_boolean(a, b, kind, tolerance);
        }
        NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> solids;
        TopExp::MapShapes(result, TopAbs_SOLID, solids);
        if (solids.IsEmpty())
            return BRep();
        BRep converted = from_shape(result);
        converted.surfacecolor = first.surfacecolor;
        converted.name = first.name;
        return converted;
    } catch (const Standard_Failure& error) {
        throw std::runtime_error(std::string("OCCT Boolean: ") + error.what());
    }
}

BRep offset(const BRep& shape, double distance, double tolerance) {
    check_tolerance(tolerance);
    if (!std::isfinite(distance))
        throw std::invalid_argument("OCCT offset requires a finite distance");
    try {
        TopoDS_Shape source = to_shape(shape, tolerance);
        if (distance == 0.0)
            return shape;
        ShapeUpgrade_ShapeDivideContinuity division(source);
        division.SetTolerance(tolerance);
        division.SetTolerance2d(tolerance);
        division.SetSurfaceCriterion(GeomAbs_C1);
        const bool divided = division.Perform();
        if (division.Status(ShapeExtend_FAIL))
            throw std::runtime_error("OCCT could not split offset geometry into C1 patches");
        if (divided)
            source = division.Result();
        BRepOffsetAPI_MakeOffsetShape builder;
        builder.PerformByJoin(source, distance, tolerance);
        if (!builder.IsDone())
            throw std::runtime_error("OCCT offset failed with status " + std::to_string(builder.MakeOffset().Error()));
        if (builder.Shape().IsNull())
            throw std::runtime_error("OCCT offset returned no shape");
        ShapeFix_Shape repair(builder.Shape());
        repair.SetPrecision(tolerance);
        repair.SetMaxTolerance(tolerance * 10);
        repair.Perform();
        if (!BRepCheck_Analyzer(repair.Shape()).IsValid())
            throw std::runtime_error("OCCT offset produced an invalid shape");
        return from_shape(repair.Shape());
    } catch (const Standard_Failure& error) {
        throw std::runtime_error(std::string("OCCT offset: ") + error.what());
    }
}

BRep hollow(const BRep& brep, const std::vector<int>& openings, double thickness, double tolerance) {
    check_tolerance(tolerance);
    if (!brep.is_solid() || !std::isfinite(thickness) || thickness == 0.0)
        throw std::invalid_argument("Hollowing requires a valid solid and finite nonzero thickness");
    try {
        std::vector<TopoDS_Face> faces;
        const TopoDS_Shape source = to_shape(brep, tolerance, &faces);
        NCollection_List<TopoDS_Shape> removed;
        for (int index : openings) {
            if (index < 0 || index >= brep.face_count())
                throw std::out_of_range("Hollowing face index is out of range");
            removed.Append(faces[index]);
        }
        BRepOffsetAPI_MakeThickSolid builder;
        builder.MakeThickSolidByJoin(source, removed, thickness, tolerance);
        if (!builder.IsDone() || !BRepCheck_Analyzer(builder.Shape()).IsValid())
            throw std::runtime_error("OCCT hollowing failed or produced an invalid shape");
        return from_shape(builder.Shape());
    } catch (const Standard_Failure& error) {
        throw std::runtime_error(std::string("OCCT hollowing: ") + error.what());
    }
}

NurbsCurve offset(const NurbsCurve& curve, double distance, const Vector& normal, double tolerance) {
    check_tolerance(tolerance);
    if (!curve.is_valid() || !std::isfinite(distance))
        throw std::invalid_argument("Curve offsets require valid geometry and a finite distance");
    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(normal[i]))
            throw std::invalid_argument("Curve offsets require a finite normal");
    if (distance == 0.0)
        return curve;
    try {
        const gp_Dir direction(normal[0], normal[1], normal[2]);
        const Handle(Geom_OffsetCurve) geometry = new Geom_OffsetCurve(to_curve(curve), distance, direction);
        const GeomConvert_ApproxCurve approximation(geometry, tolerance, GeomAbs_C1, 1000, 14);
        if (!approximation.IsDone() || approximation.MaxError() > tolerance)
            throw std::runtime_error("Curve offset approximation did not meet tolerance");
        const Handle(Geom_BSplineCurve) result = approximation.Curve();
        return from_curve(result, result->FirstParameter(), result->LastParameter());
    } catch (const Standard_Failure& error) {
        throw std::runtime_error(std::string("OCCT curve offset: ") + error.what());
    }
}

NurbsSurface offset(const NurbsSurface& surface, double distance, double tolerance) {
    check_tolerance(tolerance);
    if (!surface.is_valid() || !std::isfinite(distance))
        throw std::invalid_argument("Surface offsets require valid geometry and a finite distance");
    if (distance == 0.0)
        return surface;
    try {
        const Handle(Geom_OffsetSurface) geometry = new Geom_OffsetSurface(to_surface(surface), distance);
        const GeomConvert_ApproxSurface approximation(geometry, tolerance, GeomAbs_C1, GeomAbs_C1, 14, 14, 1000, 1);
        if (!approximation.IsDone() || approximation.MaxError() > tolerance)
            throw std::runtime_error("Surface offset approximation did not meet tolerance");
        return from_surface(approximation.Surface());
    } catch (const Standard_Failure& error) {
        throw std::runtime_error(std::string("OCCT surface offset: ") + error.what());
    }
}

double volume(const BRep& shape, double tolerance) {
    check_tolerance(tolerance);
    try {
        GProp_GProps properties;
        BRepGProp::VolumeProperties(to_shape(shape, tolerance), properties, tolerance);
        return std::abs(properties.Mass());
    } catch (const Standard_Failure& error) {
        throw std::runtime_error(std::string("OCCT volume: ") + error.what());
    }
}

Mesh tessellate(const BRep& brep, double deflection, double angle) {
    check_tolerance(deflection);
    check_tolerance(angle);
    try {
        const TopoDS_Shape shape = to_shape(brep, 1e-7);
        const BRepMesh_IncrementalMesh mesher(shape, deflection, false, angle, false);
        if (!mesher.IsDone())
            throw std::runtime_error("OCCT tessellation failed");

        NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> faces;
        TopExp::MapShapes(shape, TopAbs_FACE, faces);
        Mesh result;
        for (int i = 1; i <= faces.Extent(); ++i) {
            const TopoDS_Face face = TopoDS::Face(faces(i));
            TopLoc_Location location;
            const Handle(Poly_Triangulation) mesh = BRep_Tool::Triangulation(face, location);
            if (mesh.IsNull())
                throw std::runtime_error("OCCT tessellation omitted a face");

            std::vector<size_t> vertices;
            for (int j = 1; j <= mesh->NbNodes(); ++j) {
                const gp_Pnt point = mesh->Node(j).Transformed(location.Transformation());
                vertices.push_back(result.add_vertex({point.X(), point.Y(), point.Z()}));
            }
            for (int j = 1; j <= mesh->NbTriangles(); ++j) {
                int a, b, c;
                mesh->Triangle(j).Get(a, b, c);
                if (face.Orientation() == TopAbs_REVERSED)
                    std::swap(b, c);
                if (!result.add_face({vertices[a - 1], vertices[b - 1], vertices[c - 1]}))
                    throw std::runtime_error("OCCT tessellation produced an invalid triangle");
            }
        }
        return result.weld(1e-7);
    } catch (const Standard_Failure& error) {
        throw std::runtime_error(std::string("OCCT tessellation: ") + error.what());
    }
}

SectionResult section(const NurbsCurve& a, const NurbsCurve& b, double tolerance) {
    try {
        return compute_section(BRepBuilderAPI_MakeEdge(to_curve(a)), BRepBuilderAPI_MakeEdge(to_curve(b)), tolerance);
    } catch (const Standard_Failure& error) {
        throw std::runtime_error(std::string("OCCT curve section: ") + error.what());
    }
}

SectionResult section(const NurbsCurve& curve, const NurbsSurface& surface, double tolerance) {
    try {
        const TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(to_curve(curve));
        const TopoDS_Face face = BRepBuilderAPI_MakeFace(to_surface(surface), tolerance);
        return compute_section(edge, face, tolerance);
    } catch (const Standard_Failure& error) {
        throw std::runtime_error(std::string("OCCT curve-surface section: ") + error.what());
    }
}

SectionResult section(const NurbsCurve& curve, const BRep& brep, double tolerance) {
    try {
        return compute_section(BRepBuilderAPI_MakeEdge(to_curve(curve)), to_shape(brep, tolerance), tolerance);
    } catch (const Standard_Failure& error) {
        throw std::runtime_error(std::string("OCCT curve-BRep section: ") + error.what());
    }
}

SectionResult section(const BRep& a, const BRep& b, double tolerance) {
    try {
        return compute_section(to_shape(a, tolerance), to_shape(b, tolerance), tolerance);
    } catch (const Standard_Failure& error) {
        throw std::runtime_error(std::string("OCCT BRep section: ") + error.what());
    }
}

std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> surface_surface(
    const NurbsSurface& a,
    const NurbsSurface& b,
    double tolerance
) {
    check_tolerance(tolerance);
    try {
        const GeomInt_IntSS intersection(to_surface(a), to_surface(b), tolerance, true, true, true);
        if (!intersection.IsDone())
            throw std::runtime_error("OCCT surface-surface intersection failed");

        std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> result;
        for (int i = 1; i <= intersection.NbLines(); ++i) {
            if (!intersection.HasLineOnS1(i) || !intersection.HasLineOnS2(i))
                throw std::runtime_error("OCCT surface intersection omitted a pcurve");
            const Handle(Geom_Curve) geometry = intersection.Line(i);
            const double first = geometry->FirstParameter();
            const double last = geometry->LastParameter();
            NurbsCurve curve = from_curve(geometry, first, last);
            NurbsCurve pa = from_pcurve(intersection.LineOnS1(i), first, last);
            NurbsCurve pb = from_pcurve(intersection.LineOnS2(i), first, last);
            curve.set_domain(0.0, 1.0);
            pa.set_domain(0.0, 1.0);
            pb.set_domain(0.0, 1.0);
            result.emplace_back(curve, pa, pb);
        }
        return result;
    } catch (const Standard_Failure& error) {
        throw std::runtime_error(std::string("OCCT surface intersection: ") + error.what());
    }
}

} // namespace session_cpp::occt
#else
namespace session_cpp::occt {

BRep boolean_operation(const BRep&, const BRep&, BooleanOperation, double) {
    throw std::runtime_error("BRep Booleans require a build with SESSION_WITH_OCCT=ON");
}

BRep offset(const BRep&, double, double) {
    throw std::runtime_error("General BRep offsets require a build with SESSION_WITH_OCCT=ON");
}

BRep hollow(const BRep&, const std::vector<int>&, double, double) {
    throw std::runtime_error("BRep hollowing requires a build with SESSION_WITH_OCCT=ON");
}

NurbsCurve offset(const NurbsCurve&, double, const Vector&, double) {
    throw std::runtime_error("Curve offsets require a build with SESSION_WITH_OCCT=ON");
}

NurbsSurface offset(const NurbsSurface&, double, double) {
    throw std::runtime_error("Surface offsets require a build with SESSION_WITH_OCCT=ON");
}

double volume(const BRep&, double) {
    throw std::runtime_error("Exact BRep volume requires a build with SESSION_WITH_OCCT=ON");
}

Mesh tessellate(const BRep&, double, double) {
    throw std::runtime_error("Controlled BRep tessellation requires a build with SESSION_WITH_OCCT=ON");
}

SectionResult section(const NurbsCurve&, const NurbsCurve&, double) {
    throw std::runtime_error("Curve intersections require a build with SESSION_WITH_OCCT=ON");
}

SectionResult section(const NurbsCurve&, const NurbsSurface&, double) {
    throw std::runtime_error("Curve-surface intersections require a build with SESSION_WITH_OCCT=ON");
}

SectionResult section(const NurbsCurve&, const BRep&, double) {
    throw std::runtime_error("Curve-BRep intersections require a build with SESSION_WITH_OCCT=ON");
}

SectionResult section(const BRep&, const BRep&, double) {
    throw std::runtime_error("BRep sections require a build with SESSION_WITH_OCCT=ON");
}

std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> surface_surface(
    const NurbsSurface&,
    const NurbsSurface&,
    double
) {
    throw std::runtime_error("OCCT surface intersections require a build with SESSION_WITH_OCCT=ON");
}

}
#endif
