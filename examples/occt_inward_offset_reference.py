"""Independent OCCT oracle; never imported or linked by the Session kernel."""

from pathlib import Path
import json
import time

import OCP
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeSolid, BRepBuilderAPI_Sewing
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.BRepLib import BRepLib
from OCP.BRepOffsetAPI import BRepOffsetAPI_MakeOffsetShape
from OCP.Geom import Geom_BSplineSurface
from OCP.GProp import GProp_GProps
from OCP.TColgp import TColgp_Array2OfPnt
from OCP.TColStd import TColStd_Array1OfInteger, TColStd_Array1OfReal
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_SHELL, TopAbs_SOLID, TopAbs_VERTEX
from OCP.TopExp import TopExp
from OCP.TopTools import TopTools_IndexedMapOfShape
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Pnt


ROOT = Path(__file__).resolve().parents[1]
FOLDER = ROOT / "build/native-geometry-tests/viewer"


def knots(values):
    result = TColStd_Array1OfReal(1, 2)
    for index, value in enumerate(values, 1):
        result.SetValue(index, value)
    return result


def bilinear_support(data):
    if (data["dimension"], data["is_rational"], data["order_u"], data["order_v"],
            data["cv_count_u"], data["cv_count_v"]) != (3, False, 2, 2, 2, 2):
        raise ValueError("This oracle only reconstructs the preserved exact bilinear fixture")
    poles = TColgp_Array2OfPnt(1, 2, 1, 2)
    coordinates = data["control_points"]
    for u in range(2):
        for v in range(2):
            start = (u * 2 + v) * 3
            poles.SetValue(u + 1, v + 1, gp_Pnt(*coordinates[start:start + 3]))
    multiplicities = TColStd_Array1OfInteger(1, 2)
    multiplicities.SetValue(1, 2)
    multiplicities.SetValue(2, 2)
    return Geom_BSplineSurface(poles, knots(data["nurbsknots_u"]), knots(data["nurbsknots_v"]),
                               multiplicities, multiplicities, 1, 1)


def natural_boundary(data, face):
    if len(face["wires"]) != 1 or face["wires"][0]["orientation"] != "forward":
        raise ValueError("Oracle requires one forward natural outer wire")
    surface = data["surfaces"][face["surface_index"]]
    u0, u1 = surface["nurbsknots_u"]
    v0, v1 = surface["nurbsknots_v"]
    corners = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
    uses = data["wires"][face["wires"][0]["index"]]["edges"]
    if len(uses) != 4:
        raise ValueError("Expected four complete natural rectangle sides")
    sides = []
    for use in uses:
        edge = data["edges"][use["index"]]
        relation = next(item for item in edge["pcurves"] if item["surface_index"] == face["surface_index"])
        curve = data["curves_2d"][relation["curve_2d_index"]]
        if curve["is_rational"] or curve["order"] != 2 or curve["cv_count"] != 2:
            raise ValueError("Expected exact linear natural pcurves")
        endpoints = [corners.index(tuple(point[:2])) for point in curve["control_points"]]
        if use["orientation"] == "reversed":
            endpoints.reverse()
        elif use["orientation"] != "forward":
            raise ValueError("Unsupported boundary orientation")
        if (endpoints[1] - endpoints[0]) % 4 != 1:
            raise ValueError("Boundary does not follow the complete positive natural rectangle")
        sides.append(endpoints)
    if len({a for a, _ in sides}) != 4 or any(sides[i][1] != sides[(i + 1) % 4][0] for i in range(4)):
        raise ValueError("Boundary sides do not form one natural rectangle")


def source_solid(path):
    data = json.loads(path.read_text(encoding="utf-8"))
    if len(data["solids"]) != 1 or len(data["solids"][0]["shells"]) != 1:
        raise ValueError("Expected one source shell")
    shell_use = data["solids"][0]["shells"][0]
    if shell_use["orientation"] != "forward":
        raise ValueError("Unexpected source shell orientation")
    sewing = BRepBuilderAPI_Sewing(1e-9)
    for use in data["shells"][shell_use["index"]]["faces"]:
        face = data["faces"][use["index"]]
        natural_boundary(data, face)
        support = bilinear_support(data["surfaces"][face["surface_index"]])
        shape = BRepBuilderAPI_MakeFace(support, 1e-9).Face()
        if use["orientation"] == "reversed":
            shape.Reverse()
        elif use["orientation"] != "forward":
            raise ValueError("Unsupported source face orientation")
        sewing.Add(shape)
    sewing.Perform()
    sewn = sewing.SewedShape()
    if sewn.ShapeType() != TopAbs_SHELL:
        raise RuntimeError("OCCT did not reconstruct one sewn shell")
    result = BRepBuilderAPI_MakeSolid(TopoDS.Shell_s(sewn)).Solid()
    if not BRepLib.OrientClosedSolid_s(result) or not BRepCheck_Analyzer(result).IsValid():
        raise RuntimeError("OCCT reconstructed an invalid source solid")
    return result


def volume(shape):
    properties = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, properties, 1e-9)
    return properties.Mass()


def topology(shape):
    result = {}
    for name, kind in (("solids", TopAbs_SOLID), ("shells", TopAbs_SHELL), ("faces", TopAbs_FACE),
                       ("edges", TopAbs_EDGE), ("vertices", TopAbs_VERTEX)):
        unique = TopTools_IndexedMapOfShape()
        TopExp.MapShapes_s(shape, kind, unique)
        result[name] = unique.Extent()
    return result


def main():
    FOLDER.mkdir(parents=True, exist_ok=True)
    output = FOLDER / "occt_inward_offset_reference.json"
    report = {"source": "Standalone OCCT reconstruction of exact saved bilinear supports",
              "warp_coefficients": {"x_plus_yz": 0.12, "y_plus_xz": 0.09, "z_plus_xy": 0.16},
              "ocp_version": OCP.__version__, "offset_tolerance": 1e-8, "inputs": [], "cases": []}
    for variant, fixture in (("standard", "native_offset_inward_freeform_source.brep.json"),
                             ("affine", "native_offset_inward_freeform_nonzero_domains_source.brep.json")):
        source_path = ROOT / "tests/fixtures" / fixture
        source = source_solid(source_path)
        data = json.loads(source_path.read_text(encoding="utf-8"))
        source_volume = volume(source)
        supports = [{key: surface[key] for key in ("control_points", "nurbsknots_u", "nurbsknots_v")}
                    for surface in data["surfaces"]]
        report["inputs"].append({"variant": variant, "volume": source_volume, "valid": True,
                                 "topology": topology(source), "supports": supports,
                                 "shells": data["shells"], "solids": data["solids"]})
        if abs(source_volume - 7.8816) > 1e-8:
            raise RuntimeError("Reconstructed source disagrees with the analytic Jacobian volume")
        for distance, label in ((-0.04, "small"), (-0.16, "large")):
            row = {"variant": variant, "distance": distance, "valid": False}
            started = time.perf_counter()
            try:
                operation = BRepOffsetAPI_MakeOffsetShape()
                operation.PerformByJoin(source, distance, 1e-8)
                row["done"] = operation.IsDone()
                if operation.IsDone():
                    result = operation.Shape()
                    row["valid"] = not result.IsNull() and BRepCheck_Analyzer(result).IsValid()
                    if row["valid"]:
                        row["volume"] = volume(result)
                        row["topology"] = topology(result)
                native_path = FOLDER / f"native_offset_inward_{variant}_{label}.assertions.json"
                if native_path.exists():
                    native = json.loads(native_path.read_text(encoding="utf-8"))
                    row["native_passed"] = native.get("passed", False)
                    if "volume" in native and "volume" in row:
                        row["native_volume"] = native["volume"]
                        row["volume_difference"] = abs(native["volume"] - row["volume"])
            except Exception as error:
                row["error"] = str(error)
            row["seconds"] = time.perf_counter() - started
            report["cases"].append(row)
            output.write_text(json.dumps(report, indent=2), encoding="utf-8")
            print(json.dumps(row), flush=True)
    return 0 if all(row["valid"] for row in report["cases"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
