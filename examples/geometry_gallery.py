from pathlib import Path
import html
import json
import hashlib
import shutil

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
FOLDER = ROOT / "build" / "geometry-tests"
RESULTS = ROOT.parent / "session_tests" / "session_cpp"
CURVES = {
    "curve_plane_tangency": "Curve Plane Tangency",
    "curve_plane_hidden_crossings": "Curve Plane Hidden Crossings",
    "curve_plane_small_domain": "Curve Plane Hidden Crossings",
    "curve_plane_rational_tangency": "Curve Plane Rational Tangency",
    "curve_plane_coplanar_span": "Curve Plane Coplanar Span",
}


def test_name(name):
    if name.startswith("rhino_offset_"):
        return "Rhino Freeform Chairs Inward Offsets"
    if name.startswith("rhino_chair_"):
        return "Rhino Freeform Chairs Import"
    if name.startswith("rhino_chairs_"):
        return "Rhino Freeform Chairs Booleans"
    if name == "boolean_box_cavity":
        return "Boolean Solid Configurations"
    if name == "boolean_hollow_box":
        return "Hollow Solid With Opening"
    if name.startswith("boolean_occt_"):
        return "Boolean OCCT Box Configurations"
    if name.startswith("boolean_platonic_"):
        return "Boolean Platonic Solids"
    if name.startswith("boolean_spheres_") or name.startswith("boolean_torus_"):
        return "Boolean Curved Seams And Poles"
    if name.startswith("offset_sphere_"):
        return "Offset Curved Solids"
    if name in CURVES:
        return CURVES[name]
    if name.startswith("offset_box_"):
        return "Offset Convex Box"
    if name.startswith("offset_bevel_"):
        return "Offset Convex Face Disappears"
    if name.startswith("offset_"):
        return "Offset Convex Platonic Solids"
    if name in {"sphere_poles", "cone_apex", "torus_seams", "thin_bore_walls", "thin_rational_shell"}:
        return "Singular And Periodic Solid Boundaries"
    raise ValueError(f"No test mapping for {name}")


def mesh_data(path):
    data = json.loads(path.read_text())
    vertices = data["vertex"]
    polygons = []
    for face in data["face"].values():
        polygons.append([[vertices[str(i)][axis] for axis in ("x", "y", "z")] for i in face])
    points = np.array([[v[axis] for axis in ("x", "y", "z")] for v in vertices.values()])
    return np.array(polygons), points


def draw_mesh(axis, path, limits=None, color="#60a5c7", translation=None):
    polygons, points = mesh_data(path)
    if translation is not None:
        polygons += translation
        points += translation
    axis.set_facecolor("#f5f7fb")
    normals = np.cross(polygons[:, 1] - polygons[:, 0], polygons[:, 2] - polygons[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    normals /= np.maximum(lengths[:, None], 1e-30)
    light = np.array([0.3, -0.6, 0.74])
    intensity = 0.45 + 0.55 * np.abs(normals @ light)
    colors = np.array(matplotlib.colors.to_rgb(color))[None, :] * intensity[:, None]
    collection = Poly3DCollection(polygons, facecolors=colors, edgecolor="none")
    axis.add_collection3d(collection)
    low, high = limits if limits is not None else (points.min(axis=0), points.max(axis=0))
    center = (low + high) / 2
    radius = max(high - low) * 0.6
    axis.set_xlim(center[0] - radius, center[0] + radius)
    axis.set_ylim(center[1] - radius, center[1] + radius)
    axis.set_zlim(center[2] - radius, center[2] + radius)
    axis.set_box_aspect((1, 1, 1))
    axis.view_init(elev=24, azim=-58)
    axis.set_xlabel("X")
    axis.set_ylabel("Y")
    axis.set_zlabel("Z")


def render_mesh(path):
    fig = plt.figure(figsize=(8, 6), facecolor="#f5f7fb")
    axis = fig.add_subplot(projection="3d")
    draw_mesh(axis, path)
    return fig


def mesh_path(name):
    controlled = FOLDER / f"{name}.controlled.mesh.json"
    return controlled if controlled.exists() else FOLDER / f"{name}.mesh.json"


def render_chairs(tests):
    names = ["rhino_chair_0", "rhino_chair_1", "rhino_chairs_intersection", "rhino_chairs_union",
             "rhino_chairs_difference_ab", "rhino_chairs_difference_ba", "rhino_chairs_xor"]
    if not all(mesh_path(name).exists() for name in names):
        return None
    points = np.concatenate([mesh_data(mesh_path(name))[1] for name in names[:2]])
    limits = points.min(axis=0), points.max(axis=0)
    figure = plt.figure(figsize=(18, 17), facecolor="#f5f7fb")
    labels = ["Input A", "Input B", "A intersection B", "A union B", "A minus B", "B minus A", "A xor B"]
    report_path = FOLDER / "rhino_chairs_report.json"
    report = json.loads(report_path.read_text()) if report_path.exists() else {}
    for i, (name, label) in enumerate(zip(names, labels)):
        axis = figure.add_subplot(3, 3, i + 1, projection="3d")
        draw_mesh(axis, mesh_path(name), limits)
        if i >= 2 and report:
            row = report["results"][i - 2]
            label += f'\n{row["solids"]} solid(s), {row["faces"]} faces, volume {row["volume"]:.5f}'
        axis.set_title(label, fontsize=12)
        axis.tick_params(labelsize=7)
    axis = figure.add_subplot(3, 3, 8, projection="3d")
    draw_mesh(axis, mesh_path(names[0]), limits, "#63b2d1")
    draw_mesh(axis, mesh_path(names[1]), limits, "#e6a26b")
    axis.set_title("Inputs together")
    axis = figure.add_subplot(3, 3, 9)
    axis.axis("off")
    passed = tests["Rhino Freeform Chairs Booleans"]["passed"]
    axis.text(0.04, 0.95, f'{"PASS" if passed else "FAIL"}: actual Rhino STEP fixture\n\n'
              'Same scale and viewpoint in every panel.\n\n'
              'C++ -> OCCT -> native C++ BRep\nTrimmed NURBS faces retained\nSolid topology checked\nVolume identities checked\nProtobuf round trips checked\n\n'
              'Controlled-deflection meshes are used\nwhen available; raw meshes are retained.', va="top", fontsize=12, linespacing=1.7)
    figure.suptitle("Freeform solid chair Boolean regression", fontsize=23)
    figure.savefig(FOLDER / "rhino_chairs_comparison.png", dpi=140, bbox_inches="tight")
    plt.close(figure)
    return '<article><h2>Chair operations at the same scale</h2><a href="rhino_chairs_comparison.png"><img src="rhino_chairs_comparison.png"></a><p><a href="rhino_chairs_report.json">Volumes and topology counts</a></p></article>'


def render_offsets():
    cards = []
    for solid in ("box", "tetrahedron", "cube", "octahedron", "dodecahedron", "icosahedron", "bevel"):
        names = [f"offset_{solid}_{direction}" for direction in ("original", "outward", "inward")]
        names = [name for name in names if mesh_path(name).exists()]
        points = np.concatenate([mesh_data(mesh_path(name))[1] for name in names])
        limits = points.min(axis=0), points.max(axis=0)
        figure = plt.figure(figsize=(6 * len(names), 6), facecolor="#f5f7fb")
        for i, name in enumerate(names):
            axis = figure.add_subplot(1, len(names), i + 1, projection="3d")
            draw_mesh(axis, mesh_path(name), limits)
            axis.set_title(name.rsplit("_", 1)[1].capitalize())
        figure.suptitle(f"{solid.capitalize()} miter offset — shared scale", fontsize=18)
        image = f"offset_comparison_{solid}.png"
        figure.savefig(FOLDER / image, dpi=120, bbox_inches="tight")
        plt.close(figure)
        cards.append(f'<article><h2>{solid.capitalize()} offset comparison</h2><a href="{image}"><img src="{image}"></a></article>')
    return cards


def render_components():
    path = FOLDER / "rhino_xor_components.json"
    if not path.exists():
        return None
    components = json.loads(path.read_text())
    paths = [FOLDER / (row["name"] + ".component.json") for row in components]
    points = np.concatenate([mesh_data(path)[1] for path in paths])
    low, high = points.min(axis=0), points.max(axis=0)
    center = (low + high) / 2
    limits = low - 4, high + 4
    figure = plt.figure(figsize=(22, 6), facecolor="#f5f7fb")
    colors = ["#65a7c8", "#d58b56", "#83ad75"]
    exploded = figure.add_subplot(1, len(paths) + 1, len(paths) + 1, projection="3d")
    for i, (path, row) in enumerate(zip(paths, components)):
        axis = figure.add_subplot(1, len(paths) + 1, i + 1, projection="3d")
        draw_mesh(axis, path, limits, colors[i % len(colors)])
        axis.set_title(f'Solid {i + 1}\nVolume {row["volume"]:.5f}')
        component_points = mesh_data(path)[1]
        direction = (component_points.min(axis=0) + component_points.max(axis=0)) / 2 - center
        direction *= 4 / max(np.linalg.norm(direction), 1e-9)
        draw_mesh(exploded, path, limits, colors[i % len(colors)], direction)
    exploded.set_title("Separated by 4 units for visibility\nDisplay translation only")
    figure.suptitle("XOR: three separate solid components, with their volumes checked", fontsize=20)
    image = "rhino_xor_components.png"
    figure.savefig(FOLDER / image, dpi=140, bbox_inches="tight")
    plt.close(figure)
    return f'<article><h2>XOR solid components</h2><a href="{image}"><img src="{image}"></a><p><a href="rhino_xor_components.json">Component volumes</a>. The separated panel applies display translations only.</p></article>'


def render_curve(path):
    data = json.loads(path.read_text())
    samples = np.array(data["samples"])
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), facecolor="#f5f7fb")
    for axis in axes:
        axis.set_facecolor("#f5f7fb")
        axis.axhline(0, color="#a64732", linewidth=1, label="Plane z = 0")
        axis.grid(alpha=0.2)
        axis.set_ylabel("Signed distance")
    axes[0].plot(samples[:, 1], samples[:, 3], color="#21618c", label="C++ evaluated curve")
    axes[1].plot(samples[:, 0], samples[:, 3], color="#21618c")
    if data["points"]:
        points = np.array(data["points"])
        axes[0].scatter(points[:, 0], points[:, 2], color="#c94728", zorder=3)
        axes[1].scatter(data["roots"], points[:, 2], color="#c94728", zorder=3)
    axes[0].set_xlabel("X coordinate")
    axes[1].set_xlabel("Curve parameter")
    axes[0].legend(fontsize=8)
    return fig


def render_offset(data):
    samples = np.array(data["samples"])
    original, offset = samples[:, :3], samples[:, 3:]
    distances = np.linalg.norm(offset - original, axis=1)
    figure = plt.figure(figsize=(11, 5), facecolor="#f5f7fb")
    if data["kind"] == "curve":
        axis = figure.add_subplot(1, 2, 1)
        axis.plot(original[:, 0], original[:, 1], label="Original", color="#21618c")
        axis.plot(offset[:, 0], offset[:, 1], label="Offset", color="#d07036")
        axis.set_aspect("equal", adjustable="datalim")
        axis.legend()
        axis.set_xlabel("X")
        axis.set_ylabel("Y")
    else:
        axis = figure.add_subplot(1, 2, 1, projection="3d")
        size = int(np.sqrt(len(samples)))
        points = original.reshape(size, size, 3)
        shifted = offset.reshape(size, size, 3)
        axis.plot_wireframe(points[:, :, 0], points[:, :, 1], points[:, :, 2], color="#7c8c99", alpha=0.6, linewidth=0.5)
        axis.plot_surface(shifted[:, :, 0], shifted[:, :, 1], shifted[:, :, 2], color="#60a5c7", alpha=0.85)
        axis.set_xlabel("X")
        axis.set_ylabel("Y")
        axis.set_zlabel("Z")
        axis.set_title("Original grid and offset surface")
    axis = figure.add_subplot(1, 2, 2)
    residual = distances - abs(data["distance"])
    axis.plot(residual, color="#21618c")
    axis.axhline(0, color="#b66a42", linewidth=0.8)
    axis.set_xlabel("C++ evaluation sample")
    axis.set_ylabel("Offset distance residual")
    axis.set_title(f"Maximum distance residual: {max(abs(residual)):.2g}")
    axis.grid(alpha=0.2)
    figure.tight_layout(rect=(0, 0, 1, 0.92))
    return figure


def main():
    sources = FOLDER / "test-sources"
    sources.mkdir(exist_ok=True)
    for name in ("brep", "intersection", "nurbscurve", "nurbssurface"):
        shutil.copyfile(ROOT / "src" / f"{name}_test.cpp", sources / f"{name}_test.cpp")
    tests = {}
    for name in ("brep", "intersection"):
        for result in json.loads((RESULTS / f"{name}_test.json").read_text()):
            tests[result["test_name"]] = result
    cards = []
    manifest = []
    reference_path = FOLDER / "occt_reference.json"
    reference = json.loads(reference_path.read_text()) if reference_path.exists() else []
    paths = sorted(path for path in FOLDER.glob("*.mesh.json") if ".controlled." not in path.name) + sorted(FOLDER.glob("*.curve.json"))
    for path in paths:
        name = path.name.rsplit(".", 2)[0]
        test = tests[test_name(name)]
        passed = test["passed"]
        status = "PASS" if passed else "FAIL"
        title = name.replace("_", " ").capitalize()
        geometry = mesh_path(name) if path.name.endswith(".mesh.json") else path
        figure = render_mesh(geometry) if path.name.endswith(".mesh.json") else render_curve(path)
        figure.suptitle(f"{title} | {status}", fontsize=13)
        figure.text(0.5, 0.02, test["test_name"], ha="center", fontsize=9)
        image = name + ".png"
        figure.savefig(FOLDER / image, dpi=150, bbox_inches="tight")
        plt.close(figure)
        checks = test.get("checks", [])
        assertions = "".join(f"<li>{html.escape(c['code_line'])}</li>" for c in checks)
        cards.append(f'<article><h2>{html.escape(title)}</h2><p>{status} — {len(checks)} assertions in this test</p>'
                     f'<a href="{image}"><img src="{image}"></a><details><summary>Assertions and raw geometry</summary>'
                     f'<a href="{path.name}">C++ geometry JSON</a><ul>{assertions}</ul></details></article>')
        manifest.append({"image": image, "geometry": geometry.name, "test": test["test_name"], "passed": passed})
    for path in sorted(FOLDER.glob("*.offset.json")):
        data = json.loads(path.read_text())
        results = json.loads((RESULTS / (data["group"].lower() + "_test.json")).read_text())
        test = next(result for result in results if result["test_name"] == data["test"])
        status = "PASS" if test["passed"] else "FAIL"
        figure = render_offset(data)
        name = path.name.removesuffix(".offset.json")
        title = name.replace("_", " ").capitalize()
        figure.suptitle(f'{title} | distance {data["distance"]} | {status}', fontsize=14)
        image = name + ".png"
        figure.savefig(FOLDER / image, dpi=140, bbox_inches="tight")
        plt.close(figure)
        cards.append(f'<article><h2>{title}</h2><p>{status}</p><a href="{image}"><img src="{image}"></a><a href="{path.name}">C++ geometry and evaluated samples</a></article>')
        manifest.append({"image": image, "geometry": path.name, "test": data["group"] + "/" + data["test"], "passed": test["passed"]})
    comparison = render_chairs(tests)
    if comparison:
        cards.insert(0, comparison)
    components = render_components()
    if components:
        cards.insert(1, components)
    cards.extend(render_offsets())
    gaps = [
        "Freeform Boolean results, when present, use the configured OCCT backend and the exact Rhino STEP chair fixture.",
        "Curve/surface offsets, curved solid offsets, and hollowing have separate assertion results below; images alone do not validate them.",
        "Both inward chair offsets have dedicated tests. Outward chair offsets at +0.01 fail; the independent OCCT STEP check also returns invalid outward offsets.",
        "Planar wire offsets, arbitrary self-intersection removal, and the full OCCT corpus remain incomplete.",
        "Sphere/cone/torus/thin-shell images validate topology and serialization; those images do not demonstrate Booleans or offsets.",
        "Convex offsets use intersecting shifted planes (miter joins), not rounded joins.",
        "Images show evaluated geometry; assertions, rather than appearance, establish the listed properties.",
        "This is not the complete OCCT regression corpus or proof of every numerical edge case.",
    ]
    page = '<!doctype html><meta charset="utf-8"><title>C++ geometry regression evidence</title>'
    page += '<style>body{font:16px system-ui;background:#f5f7fb;color:#172d3a;margin:32px auto;max-width:1200px;padding:0 20px}h1{font-size:30px}h2{font-size:19px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));gap:20px}article{background:white;padding:18px;border:1px solid #d5e0e7;border-radius:10px}img{width:100%}li{margin:8px 0}details{font-size:13px;overflow-wrap:anywhere}.gaps{background:#fff1d8;padding:18px;margin:24px 0;border-radius:10px}</style>'
    page += '<h1>C++ geometry regression evidence</h1><p>Images exported from the actual C++ regression fixtures. Open a card to inspect its assertions and geometry.</p>'
    page += '<p>Saved C++ test sources: <a href="test-sources/brep_test.cpp">BRep</a>, <a href="test-sources/intersection_test.cpp">intersections</a>, <a href="test-sources/nurbscurve_test.cpp">curves</a>, <a href="test-sources/nurbssurface_test.cpp">surfaces</a>.</p>'
    all_results = []
    for path in sorted(RESULTS.glob("*_test.json")):
        all_results.extend(json.loads(path.read_text()))
    passed = sum(result["passed"] for result in all_results)
    page += f'<p><strong>Latest saved C++ results: {passed}/{len(all_results)} passed.</strong> <a href="test_results.json">All assertion results</a></p>'
    focused = [result for result in all_results if result["group"] in {"BRep", "Intersection", "NurbsCurve", "NurbsSurface"}
               and (result["test_name"].startswith(("Boolean ", "Offset", "Rhino ", "Hollow ", "Curve Curve", "Curve Surface", "Curve BRep", "Surface Surface Freeform", "Surface Surface Small")))]
    page += '<details><summary>Operation test results</summary><table><tr><th>Test</th><th>Result</th><th>Time</th></tr>'
    for result in focused:
        label = html.escape(result["group"] + " / " + result["test_name"])
        status = "PASS" if result["passed"] else "FAIL"
        page += f'<tr><td>{label}</td><td>{status}</td><td>{result["time_ms"] / 1000:.3f} s</td></tr>'
    page += '</table></details>'
    page += '<div class="gaps"><strong>Coverage limits</strong><ul>' + ''.join(f'<li>{x}</li>' for x in gaps) + '</ul></div>'
    if reference:
        matched = sum(row["passed"] for row in reference)
        page += f'<h2>Independent planar-offset comparison: {matched}/{len(reference)} matched</h2>'
        page += '<p><a href="occt_reference.json">Complete OCCT comparison report</a>. The disappearing-bevel reference returned an invalid OCCT shape; its discrepancy remains recorded.</p>'
    step_path = FOLDER / "occt_step_reference.json"
    if step_path.exists():
        step = json.loads(step_path.read_text())
        matched = sum(row["passed"] for row in step["booleans"])
        maximum = max(row["relative_error"] for row in step["booleans"])
        page += f'<h2>Independent STEP import and chair Booleans: {matched}/5 matched</h2><p>Direct OCCT {step["occt_version"]} STEP import, bypassing the session importer. Largest relative volume difference: {maximum:.3g}. <a href="occt_step_reference.json">Full reference and offset results</a>.</p>'
    probes = FOLDER / "cpp_outward_offset_probes.json"
    if probes.exists():
        failures = sum(not row["valid"] for row in json.loads(probes.read_text()))
        page += f'<p><strong>Separate outward chair-offset probes: {failures}/2 failed.</strong> These are outside the passing unit-test count. <a href="cpp_outward_offset_probes.json">C++ failure details</a>.</p>'
    page += '<div class="grid">' + ''.join(cards) + '</div>'
    (FOLDER / "index.html").write_text(page, encoding="utf-8")
    (FOLDER / "test_results.json").write_text(json.dumps(all_results, indent=2))
    fixture_hash = hashlib.sha256((ROOT / "tests/fixtures/solidbrep.stp").read_bytes()).hexdigest()
    (FOLDER / "coverage.json").write_text(json.dumps({"images": manifest, "limits": gaps, "occt_reference": reference,
        "cpp_passed": passed, "cpp_tests": len(all_results), "chair_step_sha256": fixture_hash}, indent=2))
    print(f"Saved {len(manifest)} images to {FOLDER}")


if __name__ == "__main__":
    main()
