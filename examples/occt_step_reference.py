from pathlib import Path
import json

import OCP
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.BRepOffsetAPI import BRepOffsetAPI_MakeOffsetShape
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer


ROOT = Path(__file__).resolve().parents[1]
FOLDER = ROOT / "build" / "geometry-tests"


def volume(shape):
    properties = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, properties, 1e-7)
    return abs(properties.Mass())


def solids(shape):
    result = []
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    while explorer.More():
        result.append(explorer.Current())
        explorer.Next()
    return result


def main():
    reader = STEPControl_Reader()
    if reader.ReadFile(str(ROOT / "tests/fixtures/solidbrep.stp")) != IFSelect_RetDone:
        raise RuntimeError("OCCT could not read the chair STEP file")
    if reader.TransferRoots() == 0:
        raise RuntimeError("OCCT did not transfer STEP geometry")
    a, b = solids(reader.OneShape())
    report = {
        "occt_version": OCP.__version__,
        "input_volumes": [volume(a), volume(b)],
        "input_valid": [BRepCheck_Analyzer(a).IsValid(), BRepCheck_Analyzer(b).IsValid()],
        "booleans": [],
        "offsets": [],
    }
    difference_ab = BRepAlgoAPI_Cut(a, b).Shape()
    difference_ba = BRepAlgoAPI_Cut(b, a).Shape()
    results = {
        "intersection": BRepAlgoAPI_Common(a, b).Shape(),
        "union": BRepAlgoAPI_Fuse(a, b).Shape(),
        "difference_ab": difference_ab,
        "difference_ba": difference_ba,
        "xor": BRepAlgoAPI_Fuse(difference_ab, difference_ba).Shape(),
    }
    native = json.loads((FOLDER / "rhino_chairs_report.json").read_text())
    for name, shape in results.items():
        native_result = next(row for row in native["results"] if row["operation"] == name)
        expected = native_result["volume"]
        measured = volume(shape)
        valid = BRepCheck_Analyzer(shape).IsValid()
        error = abs(measured - expected) / max(measured, expected)
        count = len(solids(shape))
        report["booleans"].append({"operation": name, "volume": measured, "cpp_volume": expected,
            "relative_error": error, "valid": valid, "solids": count,
            "passed": valid and error < 1e-6 and count == native_result["solids"]})
    for index, shape in enumerate((a, b)):
        for distance in (-0.01, 0.01):
            row = {"chair": index, "distance": distance}
            try:
                operation = BRepOffsetAPI_MakeOffsetShape()
                operation.PerformByJoin(shape, distance, 1e-7)
                row["done"] = operation.IsDone()
                offset = operation.Shape() if operation.IsDone() else None
                row["valid"] = offset is not None and not offset.IsNull() and BRepCheck_Analyzer(offset).IsValid()
                if row["valid"]:
                    row["volume"] = volume(offset)
            except Exception as error:
                row["valid"] = False
                row["error"] = str(error)
            report["offsets"].append(row)
    (FOLDER / "occt_step_reference.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
