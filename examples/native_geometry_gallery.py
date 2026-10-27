from pathlib import Path
import hashlib
import html
import json
import shutil

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from geometry_gallery import draw_mesh


def main():
    root = Path(__file__).resolve().parent.parent
    folder = root / "build/native-geometry-tests"
    tests = json.loads((folder / "classification_tests.json").read_text())
    probes = json.loads((folder / "chair_classification.json").read_text())
    reference = json.loads((folder / "occt_classification_reference.json").read_text())
    intersections = json.loads((root.parent / "session_tests/session_cpp/intersection_test.json").read_text())
    cache = (root / "build/CMakeCache.txt").read_text()
    if "SESSION_WITH_OCCT:BOOL=OFF" not in cache:
        raise RuntimeError("Native evidence requires SESSION_WITH_OCCT=OFF")
    images = []
    for path in sorted(folder.glob("*.sections.json")):
        report = json.loads(path.read_text())
        figure = plt.figure(figsize=(10, 8))
        axis = figure.add_subplot(111, projection="3d")
        for grid, color in zip(report.get("surface_samples", []), ("#3075a8", "#c88b36")):
            points = np.array(grid)
            axis.plot_surface(points[:, :, 0], points[:, :, 1], points[:, :, 2],
                              color=color, alpha=0.25, linewidth=0)
        for section in report["sections"]:
            points = np.array(section["samples"])
            axis.plot(*points.T, color="#c62d4c", linewidth=2.5)
        gap = max((s["max_sampled_gap"] for s in report["sections"]), default=0)
        label = path.name.removesuffix(".sections.json").replace("_", " ")
        axis.set_box_aspect((1, 1, 1))
        figure.suptitle(f"Native surface intersection: {label}", fontsize=16)
        figure.text(0.5, 0.04, f"{len(report['sections'])} sections; 257 samples per section\n"
                    f"Maximum sampled world/pcurve gap: {gap:.3g}. OCCT disabled.", ha="center")
        name = path.name.replace(".sections.json", ".png")
        figure.savefig(folder / name, dpi=160, bbox_inches="tight")
        plt.close(figure)
        images.append(name)
    for ci in range(2):
        selected = [row for row in probes if row["chair"] == ci]
        points = np.array([row["point"] for row in selected])
        for view, azimuth in (("front", -58), ("back", 122)):
            figure = plt.figure(figsize=(10, 8))
            axis = figure.add_subplot(111, projection="3d")
            draw_mesh(axis, folder / f"chair_{ci}.mesh.json")
            axis.view_init(elev=24, azim=azimuth)
            for row, point in zip(selected, points):
                good = row["inside"] == 1 and row["outside"] == 0
                axis.scatter(*point, color="#17933c" if good else "#cf2737", s=22, depthshade=False)
                axis.text(*point, str(row["face"]), fontsize=7, color="#13243c")
            figure.suptitle(f"Native session classifier — Rhino chair {ci + 1}", fontsize=17)
            figure.text(0.5, 0.04,
                        f"{len(selected)} face locations; probes at ±0.001 along oriented normals\n"
                        "Original solid with classification locations — not a Boolean result. OCCT disabled.",
                        ha="center", fontsize=10)
            image = f"chair_{ci}_classification_{view}.png"
            figure.savefig(folder / image, dpi=160, bbox_inches="tight")
            plt.close(figure)
            images.append(image)
    for source in ("brep_classify.h", "brep_classify.cpp", "brep_classify_test.cpp"):
        shutil.copy2(root / "src" / source, folder / source)
    shutil.copy2(root / "docs/native_brep_plan.md", folder / "native_brep_plan.md")
    shutil.copy2(root / "build/native-suite-test.log", folder / "native-suite-test.log")
    shutil.copy2(root.parent / "session_tests/session_cpp/intersection_test.json", folder / "intersection_tests.json")
    shutil.copy2(root.parent / "session_tests/session_cpp/nurbscurve_test.json", folder / "nurbscurve_tests.json")
    manifest = {"scope": "Native classification and surface intersection regressions",
                "occt_enabled": False, "native_boolean_operations_complete": False,
                "known_issues": ["Native solid Booleans and general offsets remain unfinished",
                                 "Torus/plane paired-curve sampled gap remains about 1.35e-5",
                                 "Intersection completeness and approximation certificates remain unfinished"],
                "fixture_sha256": hashlib.sha256((root / "tests/fixtures/solidbrep.stp").read_bytes()).hexdigest(),
                "native_executable_sha256": hashlib.sha256((root / "build/Release/native_brep_minitest.exe").read_bytes()).hexdigest(),
                "classification_groups_passed": sum(test["passed"] for test in tests),
                "classification_groups_total": len(tests),
                "intersection_groups_passed": sum(test["passed"] for test in intersections),
                "intersection_groups_total": len(intersections),
                "chair_classifications_matched": reference["matched"],
                "chair_classifications_total": reference["total"], "images": images}
    (folder / "coverage.json").write_text(json.dumps(manifest, indent=2))
    cards = "".join(f'<a href="{name}"><img src="{name}" alt="Native geometry regression"></a>' for name in images)
    rows = "".join(f"<li>{html.escape(test['test_name'])}: {'PASS' if test['passed'] else 'FAIL'}</li>" for test in tests)
    page = f"""<!doctype html><meta charset="utf-8"><title>Native session BRep progress</title>
<style>body{{font:17px system-ui;max-width:1200px;margin:40px auto;padding:20px;color:#142337}}
img{{width:49%;vertical-align:top}}a{{color:#1768a3}}.notice{{background:#fff0cf;padding:20px}}</style>
<h1>Native session BRep translation: classification and intersections</h1>
<p class="notice">These are native classification and intersection tests, not completed native Boolean operations.
The full translation remains in progress. The executable is built with OCCT disabled.</p>
<p>{manifest['classification_groups_passed']}/{len(tests)} classification groups passed.
{reference['matched']}/{reference['total']} chair probes agree with independently imported OCCT STEP solids.</p>
<p>{manifest['intersection_groups_passed']}/{len(intersections)} native intersection groups passed.
The plotted residuals are sampled checks, not certified error bounds or a completeness proof.</p>
<p class="notice">Unresolved: the torus/plane paired-curve gap is about 1.35e-5.
Its separate cut-curve test passes, but that does not establish the requested paired-curve tolerance.
Native solid Booleans and general offsets remain unfinished.</p>
<ul>{rows}</ul>{cards}
<p><a href="native_brep_plan.md">Implementation plan and remaining gates</a> ·
<a href="classification_tests.json">Assertions</a> · <a href="chair_classification.json">Raw probes</a> ·
<a href="intersection_tests.json">Intersection assertions</a> · <a href="nurbscurve_tests.json">Curve assertions</a> ·
<a href="occt_classification_reference.json">Independent comparison</a> ·
<a href="native-suite-test.log">Full native C++ suite log</a> · <a href="coverage.json">Coverage</a></p>
<p><a href="brep_classify.cpp">Implementation</a> · <a href="brep_classify_test.cpp">Tests</a> ·
<a href="chair_0.brep.json">Chair A geometry</a> · <a href="chair_1.brep.json">Chair B geometry</a></p>"""
    (folder / "index.html").write_text(page, encoding="utf-8")
    print(f"Saved {len(images)} native geometry images to {folder}")


if __name__ == "__main__":
    main()
