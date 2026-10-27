"""Independent OCCT oracle on the native chair support surfaces, never a backend."""
import json
from collections import Counter
from pathlib import Path

import OCP
from OCP.Geom import Geom_BSplineSurface
from OCP.GeomAPI import GeomAPI_IntSS
from OCP.TColgp import TColgp_Array2OfPnt
from OCP.TColStd import TColStd_Array1OfInteger, TColStd_Array1OfReal, TColStd_Array2OfReal
from OCP.gp import gp_Pnt


def knots(compact):
    counts = Counter([compact[0], *compact, compact[-1]])
    values = TColStd_Array1OfReal(1, len(counts))
    multiplicities = TColStd_Array1OfInteger(1, len(counts))
    for index, (value, count) in enumerate(sorted(counts.items()), 1):
        values.SetValue(index, value)
        multiplicities.SetValue(index, count)
    return values, multiplicities


def surface(data):
    nu, nv = data["cv_count_u"], data["cv_count_v"]
    stride = data["dimension"] + int(data["is_rational"])
    poles = TColgp_Array2OfPnt(1, nu, 1, nv)
    weights = TColStd_Array2OfReal(1, nu, 1, nv)
    for u in range(nu):
        for v in range(nv):
            start = (u * nv + v) * stride
            point = data["control_points"][start:start + stride]
            weight = point[-1] if data["is_rational"] else 1.0
            poles.SetValue(u + 1, v + 1, gp_Pnt(*(x / weight for x in point[:3])))
            weights.SetValue(u + 1, v + 1, weight)
    uk, um = knots(data["nurbsknots_u"])
    vk, vm = knots(data["nurbsknots_v"])
    return Geom_BSplineSurface(poles, weights, uk, vk, um, vm,
                              data["order_u"] - 1, data["order_v"] - 1)


def main():
    folder = Path(__file__).resolve().parents[1] / "build/native-geometry-tests"
    chairs = [json.loads((folder / f"chair_{i}.brep.json").read_text()) for i in range(2)]
    report = {"occt_version": OCP.__version__, "scope": "untrimmed finite support surfaces",
              "tolerance": 1e-6, "pairs": []}
    for a, b in [(8, 17), (9, 17), (14, 0), (15, 18), (16, 2), (16, 17), (19, 0), (19, 14), (19, 19)]:
        first = surface(chairs[0]["surfaces"][chairs[0]["faces"][a]["surface_index"]])
        second = surface(chairs[1]["surfaces"][chairs[1]["faces"][b]["surface_index"]])
        operation = GeomAPI_IntSS(first, second, 1e-6)
        row = {"a": a, "b": b, "done": operation.IsDone(), "lines": []}
        if operation.IsDone():
            for index in range(1, operation.NbLines() + 1):
                curve = operation.Line(index)
                lo, hi = curve.FirstParameter(), curve.LastParameter()
                row["lines"].append({"domain": [lo, hi], "samples": [
                    list(curve.Value(lo + (hi - lo) * i / 200).Coord()) for i in range(201)]})
        report["pairs"].append(row)
        print(a, b, row["done"], len(row["lines"]), flush=True)
        (folder / "occt_chair_support_sections.json").write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
