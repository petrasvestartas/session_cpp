from pathlib import Path
import json

from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_IN, TopAbs_OUT, TopAbs_ON, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Pnt


def main():
    root = Path(__file__).resolve().parent.parent
    folder = root / "build/native-geometry-tests"
    reader = STEPControl_Reader()
    if reader.ReadFile(str(root / "tests/fixtures/solidbrep.stp")) != IFSelect_RetDone:
        raise RuntimeError("STEP import failed")
    if reader.TransferRoots() != 1:
        raise RuntimeError("Unexpected STEP root count")
    solids = []
    explorer = TopExp_Explorer(reader.OneShape(), TopAbs_SOLID)
    while explorer.More():
        solids.append(TopoDS.Solid_s(explorer.Current()))
        explorer.Next()
    if len(solids) != 2:
        raise RuntimeError("Expected the two Rhino chair solids")
    states = {TopAbs_OUT: 0, TopAbs_IN: 1, TopAbs_ON: 2}
    results = []
    for row in json.loads((folder / "chair_classification.json").read_text()):
        for label, sign in (("inside", -1), ("outside", 1)):
            coordinates = [p + sign * n * 1e-3 for p, n in zip(row["point"], row["normal"])]
            classifier = BRepClass3d_SolidClassifier(solids[row["chair"]], gp_Pnt(*coordinates), 1e-6)
            reference = states.get(classifier.State(), 3)
            results.append({"chair": row["chair"], "face": row["face"], "side": label,
                            "native": row[label], "reference": reference, "matched": row[label] == reference})
    report = {"source": "Direct OCP STEP import; independent of the session importer and classifier",
              "cases": results, "matched": sum(row["matched"] for row in results), "total": len(results)}
    (folder / "occt_classification_reference.json").write_text(json.dumps(report, indent=2))
    print(f"Direct STEP classification comparison: {report['matched']}/{report['total']}")
    return 0 if report["matched"] == 80 and report["total"] == 80 else 1


if __name__ == "__main__":
    raise SystemExit(main())
