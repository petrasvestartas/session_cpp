import hashlib
import html
import json
import os
from pathlib import Path
import subprocess

from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
FOLDER = ROOT / "build/native-geometry-tests/viewer"
RENDERER = ROOT.parent / "session_viewer/target/x86_64-pc-windows-msvc/release/examples/selftest.exe"
SETTINGS = {
    "VIEWER_W": "1600", "VIEWER_H": "1200", "VIEWER_VIEW": "iso",
    "VIEWER_LIT": "1", "VIEWER_OUTLINES": "1", "VIEWER_MSAA": "4",
    "VIEWER_NO_GRID": "1", "BENCH_NO_MARKERS": "1", "VIEWER_OPACITY": "1",
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def assertions():
    records = {}
    manifest = FOLDER / "native_geometry_cases.json"
    if manifest.exists():
        for case in json.loads(manifest.read_text())["cases"]:
            if case.get("scene"):
                records[case["scene"]] = case
            result = case.get("result_scene")
            if result:
                records[result] = case
    for path in FOLDER.glob("*.assertions.json"):
        prefix = path.name.removesuffix(".assertions.json")
        report = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(report, list):
            for case in report:
                if "operation" in case:
                    records[f"{prefix}_{case['operation']}.pb"] = case
                elif case.get("scene"):
                    records[case["scene"]] = case
            if all("operation" not in case and "scene" not in case for case in report):
                records[f"{prefix}.pb"] = {"cases": report}
            continue
        if "cases" not in report:
            result = report.get("result_scene") or report.get("scene")
            if not result:
                result = f"{prefix}_result.pb" if (FOLDER / f"{prefix}_result.pb").exists() else f"{prefix}.pb"
            records[result] = report
        for case in report.get("cases", []):
            context = {key: value for key, value in case.items() if key != "operations"}
            for operation in case.get("operations", []):
                records[f"{case['name']}_{operation['operation']}.pb"] = {
                    "backend": report.get("backend"), "oracle": report.get("oracle"),
                    "occt_commit": report.get("occt_commit"), "case": context, **operation,
                }
            for side in range(2):
                records[f"{case['name']}_operand_{side}.pb"] = {
                    "role": "operand", "side": side, "case": context,
                }
    return records


def save_gallery(records):
    (FOLDER / "viewer_captures.json").write_text(json.dumps(records, indent=2))
    cards = []
    for record in records:
        name = html.escape(record["scene"])
        status = html.escape(record["status"])
        passed = record.get("assertions", {}).get("passed")
        acceptance = "failed" if passed is False else "passed" if passed is True else "not recorded"
        evidence = html.escape(json.dumps(record.get("assertions", {}), indent=2))
        picture = ""
        if record["status"] == "rendered":
            png = html.escape(record["png"])
            picture = f'<a href="{png}"><img loading="lazy" src="{png}" alt="{name}"></a>'
        link = f'<p><a href="{name}">Native Session scene</a></p>' if record.get("scene_sha256") else ""
        cards.append(f'<article><h2>{name}</h2><p>{status} · Test result: {acceptance}</p>{picture}'
                     f'<details><summary>Recorded numerical evidence</summary><pre>{evidence}</pre></details>'
                     f'{link}</article>')
    document = '''<!doctype html><html lang="en"><meta charset="utf-8">
<title>Native kernel — Session Viewer evidence</title>
<style>body{margin:32px;background:#f4f5f7;color:#18232e;font:16px system-ui}
main{display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));gap:24px}
article{background:white;border:1px solid #ddd;padding:18px;border-radius:8px}
h2{font-size:17px;overflow-wrap:anywhere}img{width:100%;height:auto}pre{white-space:pre-wrap;font-size:12px}
header{max-width:1000px;margin-bottom:28px}a{color:#235c8c}</style>
<header><h1>Native kernel evidence</h1>
<p>Images are rendered by the actual Session Viewer GPU renderer from native Session scenes,
at 1600 × 1200 with 4× MSAA. PNG conversion is lossless; geometry is unchanged.</p>
<p>Images show geometry, not a proof of correctness. Expand the recorded numerical evidence
for each scene. Missing assertions mean that scene has no linked acceptance record.
The complete native chair Boolean and general BRep offset acceptance gates remain unfinished.</p>
<p><a href="viewer_captures.json">Capture manifest, hashes and settings</a> ·
<a href="native_geometry_cases.json">Curve and surface test manifest</a></p></header><main>'''
    (FOLDER / "index.html").write_text(document + "".join(cards) + "</main></html>", encoding="utf-8")


def main():
    os.environ.pop("VIEWER_AO", None)
    renderer_hash = digest(RENDERER)
    evidence = assertions()
    previous_path = FOLDER / "viewer_captures.json"
    previous = json.loads(previous_path.read_text()) if previous_path.exists() else []
    cached = {record["scene"]: record for record in previous}
    records = []
    scenes = {path.name: path for path in FOLDER.glob("*.pb")}
    for name in evidence:
        scenes.setdefault(name, FOLDER / name)
    for scene in sorted(scenes.values()):
        if not scene.exists():
            assertion = evidence.get(scene.name, {})
            status = "operation failed" if assertion.get("failure") else "scene not exported"
            records.append({"scene": scene.name, "status": status, "assertions": assertion})
            save_gallery(records)
            continue
        record = {"scene": scene.name, "scene_sha256": digest(scene),
                  "renderer": str(RENDERER), "renderer_sha256": renderer_hash,
                  "settings": SETTINGS, "assertions": evidence.get(scene.name, {})}
        old = cached.get(scene.name, {})
        png = scene.with_suffix(".png")
        if record["assertions"].get("faces") == 0:
            record["status"] = "empty result"
        elif old.get("status") == "rendered" and all(old.get(key) == record[key] for key in ("scene_sha256", "renderer_sha256", "settings")) and png.exists():
            record.update(status="rendered", png=png.name, png_sha256=digest(png))
        else:
            ppm = scene.with_suffix(".ppm")
            try:
                with scene.with_suffix(".render.log").open("wb") as log:
                    subprocess.run([str(RENDERER), str(ppm), str(scene)], check=True, timeout=60,
                                   env={**os.environ, **SETTINGS}, stdout=log, stderr=subprocess.STDOUT,
                                   creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
                with Image.open(ppm) as picture:
                    picture.save(png)
                record.update(status="rendered", png=png.name, png_sha256=digest(png))
            except (OSError, subprocess.SubprocessError) as error:
                record.update(status="capture failed", error=str(error))
        records.append(record)
        save_gallery(records)
        print(f"{scene.name}: {record['status']}", flush=True)


if __name__ == "__main__":
    main()
