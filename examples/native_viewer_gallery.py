import hashlib
import html
import json
import os
from pathlib import Path
import subprocess
import sys

from PIL import Image


ROOT = Path(__file__).resolve().parents[1]


def gallery_folder():
    override = os.environ.get("VIEWER_GALLERY_FOLDER")
    if not override:
        return ROOT / "build/native-geometry-tests/viewer"
    folder = Path(override)
    if not folder.is_absolute():
        raise ValueError("VIEWER_GALLERY_FOLDER must be an absolute directory path")
    return folder.resolve()


def renderer_path():
    override = os.environ.get("VIEWER_RENDERER")
    if override is None:
        return ROOT.parent / "session_viewer/target/course-cargo/x86_64-pc-windows-msvc/release/examples/selftest.exe"
    renderer = Path(override)
    if not renderer.is_absolute():
        raise ValueError("VIEWER_RENDERER must be an absolute file path")
    renderer = renderer.resolve()
    if not renderer.is_file():
        raise ValueError("VIEWER_RENDERER must name an existing renderer file")
    return renderer


FOLDER = gallery_folder()
RENDERER = renderer_path()
SETTINGS = {
    "VIEWER_W": "1600", "VIEWER_H": "1200", "VIEWER_VIEW": "iso",
    "VIEWER_LIT": "1", "VIEWER_OUTLINES": "1", "VIEWER_MSAA": "4",
    "VIEWER_NO_GRID": "1", "BENCH_NO_MARKERS": "1", "VIEWER_OPACITY": "1",
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def artifact_snapshot(path, content):
    sha256 = hashlib.sha256(content).hexdigest()
    folder = FOLDER / "viewer_artifacts"
    folder.mkdir(exist_ok=True)
    suffix = ".pb" if path.suffix.lower() == ".pb" else ".json"
    target = folder / (sha256 + suffix)
    try:
        with target.open("xb") as output:
            output.write(content)
    except FileExistsError:
        pass
    if target.read_bytes() != content:
        raise ValueError("Immutable viewer snapshot differs: " + str(target))
    return sha256, target.relative_to(FOLDER).as_posix()


def retained_record(record):
    result = dict(record)
    scene = FOLDER / record["scene"]
    stale = bool(record.get("scene_sha256")) and (not scene.exists() or digest(scene) != record["scene_sha256"])
    if record.get("scene_snapshot"):
        scene = FOLDER / record["scene_snapshot"]
        stale = not scene.exists() or digest(scene) != record["scene_sha256"]
    for source in record.get("assertions", {}).get("assertion_sources", []):
        path = FOLDER / source["snapshot"] if source.get("snapshot") else Path(source["path"])
        stale |= not path.exists() or digest(path) != source["sha256"]
    if record.get("status") == "rendered":
        png = FOLDER / record["png"]
        stale |= not png.exists() or digest(png) != record.get("png_sha256")
    if stale:
        result.update(previous_status=record["status"], status="stale artifacts; recapture required")
    elif not record.get("scene_snapshot"):
        result["provenance_state"] = "legacy capture without immutable snapshots"
    return result


def json_snapshot(path):
    content = path.read_bytes()
    report = json.loads(content)
    sha256, snapshot = artifact_snapshot(path, content)
    return report, {"path": str(path.resolve()), "sha256": sha256, "snapshot": snapshot}


def assertions():
    records = {}

    def put(scene, report, source):
        records[scene] = {**report, "assertion_sources": [source],
                          "scene_binding": "unverified filename association"}

    manifest = FOLDER / "native_geometry_cases.json"
    if manifest.exists():
        report, source = json_snapshot(manifest)
        for case in report["cases"]:
            for scene in (case.get("scene"), case.get("result_scene")):
                if scene:
                    put(scene, case, source)
    for path in sorted(FOLDER.glob("*.assertions.json")):
        prefix = path.name.removesuffix(".assertions.json")
        report, source = json_snapshot(path)
        if isinstance(report, list):
            for case in report:
                if "operation" in case:
                    put(f"{prefix}_{case['operation']}.pb", case, source)
                elif case.get("scene"):
                    put(case["scene"], case, source)
            if all("operation" not in case and "scene" not in case for case in report):
                put(f"{prefix}.pb", {"cases": report}, source)
            continue
        if "cases" not in report:
            result = report.get("result_scene") or report.get("scene") or report.get("session_file")
            if not result:
                result = f"{prefix}_result.pb" if (FOLDER / f"{prefix}_result.pb").exists() else f"{prefix}.pb"
            put(result, report, source)
        for case in report.get("cases", []):
            context = {key: value for key, value in case.items() if key != "operations"}
            for operation in case.get("operations", []):
                put(f"{case['name']}_{operation['operation']}.pb", {
                    "backend": report.get("backend"), "oracle": report.get("oracle"),
                    "occt_commit": report.get("occt_commit"), "case": context, **operation}, source)
            for side in range(2):
                put(f"{case['name']}_operand_{side}.pb", {"role": "operand", "side": side, "case": context}, source)
    for path in sorted(FOLDER.glob("*.acceptance.json")):
        report, source = json_snapshot(path)
        put(path.name.removesuffix(".acceptance.json") + ".pb", report, source)
    bindings_path = os.environ.get("VIEWER_ASSERTION_BINDINGS")
    if bindings_path:
        binding_file = Path(bindings_path).resolve()
        bindings, binding_source = json_snapshot(binding_file)
        for scene, binding in bindings.items():
            if Path(scene).name != scene or not scene.endswith(".pb"):
                raise ValueError("Invalid bound scene name")
            report_file = Path(binding["report"])
            if not report_file.is_absolute():
                report_file = binding_file.parent / report_file
            report, source = json_snapshot(report_file)
            if source["sha256"] != binding["report_sha256"] or digest(FOLDER / scene) != binding["scene_sha256"]:
                raise ValueError(f"Fresh assertion binding hash mismatch: {scene}")
            put(scene, report, source)
            records[scene]["assertion_sources"].append(binding_source)
            records[scene].update(scene_binding="explicit SHA256 binding", scene_sha256=binding["scene_sha256"],
                                  binding_file=str(binding_file), binding_sha256=binding_source["sha256"])
    return records


def save_gallery(records):
    (FOLDER / "viewer_captures.json").write_text(json.dumps(records, indent=2))
    cards = []
    for record in records:
        name = html.escape(record["scene"])
        status = html.escape(record["status"])
        passed = record.get("assertions", {}).get("passed")
        acceptance = "failed" if passed is False else "reported pass" if passed is True else "not recorded"
        if passed is True and record.get("assertions", {}).get("scene_binding") != "explicit SHA256 binding":
            acceptance += " (scene binding unverified)"
        evidence = html.escape(json.dumps(record.get("assertions", {}), indent=2))
        picture = ""
        if record["status"] == "rendered":
            png = html.escape(record["png"])
            picture = f'<a href="{png}"><img loading="lazy" src="{png}" alt="{name}"></a>'
        scene_link = html.escape(record.get("scene_snapshot", record["scene"]))
        link = f'<p><a href="{scene_link}">Native Session scene</a></p>' if record.get("scene_sha256") else ""
        cards.append(f'<article><h2>{name}</h2><p>{status} - Test result: {acceptance}</p>{picture}'
                     f'<details><summary>Recorded numerical evidence</summary><pre>{evidence}</pre></details>'
                     f'{link}</article>')
    document = '''<!doctype html><html lang="en"><meta charset="utf-8">
<title>Native kernel - Session Viewer evidence</title>
<style>body{margin:32px;background:#f4f5f7;color:#18232e;font:16px system-ui}
main{display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));gap:24px}
article{background:white;border:1px solid #ddd;padding:18px;border-radius:8px}
h2{font-size:17px;overflow-wrap:anywhere}img{width:100%;height:auto}pre{white-space:pre-wrap;font-size:12px}
header{max-width:1000px;margin-bottom:28px}a{color:#235c8c}</style>
<header><h1>Native kernel evidence</h1>
<p>Images are rendered by the actual Session Viewer GPU renderer from native Session scenes,
at 1600 x 1200 with 4x MSAA. PNG conversion is lossless; geometry is unchanged.</p>
<p>Images show geometry, not a proof of correctness. Expand the recorded numerical evidence
for each scene. Missing assertions mean that scene has no linked acceptance record.
The complete native chair Boolean and general BRep offset acceptance gates remain unfinished.</p>
<p><a href="viewer_captures.json">Capture manifest, hashes and settings</a> -
<a href="native_geometry_cases.json">Curve and surface test manifest</a></p></header><main>'''
    (FOLDER / "index.html").write_text(document + "".join(cards) + "</main></html>", encoding="utf-8")


def main():
    opacity = os.environ.get("VIEWER_OPACITY", SETTINGS["VIEWER_OPACITY"])
    if not 0 < float(opacity) <= 1:
        raise ValueError("VIEWER_OPACITY must be finite and greater than 0 and at most 1")
    environment = {**os.environ, **SETTINGS}
    environment["VIEWER_OPACITY"] = opacity
    environment.pop("VIEWER_AO", None)
    environment.pop("VIEWER_GALLERY_FOLDER", None)
    environment.pop("VIEWER_RENDERER", None)
    effective = {key: value for key, value in environment.items() if key.startswith(("VIEWER_", "BENCH_"))}
    effective.pop("VIEWER_ASSERTION_BINDINGS", None)
    effective.update({key: environment.get(key) for key in ("VIEWER_TARGET", "VIEWER_DISTANCE")})
    renderer_hash = digest(RENDERER)
    evidence = assertions()
    previous_path = FOLDER / "viewer_captures.json"
    previous = json.loads(previous_path.read_text()) if previous_path.exists() else []
    if previous_path.exists():
        archive = FOLDER / ("viewer_captures.previous." + digest(previous_path) + ".json")
        if archive.exists() and archive.read_bytes() != previous_path.read_bytes():
            raise ValueError("Capture history hash collision")
        if not archive.exists():
            with archive.open("xb") as history:
                history.write(previous_path.read_bytes())
    cached = {record["scene"]: record for record in previous}
    selected = set(sys.argv[1:])
    if any(Path(name).name != name or not name.endswith(".pb") for name in selected):
        raise ValueError("Pass scene filenames ending in .pb")
    records = [retained_record(record) for record in previous if selected and record["scene"] not in selected]
    scenes = {path.name: path for path in FOLDER.glob("*.pb")}
    for name in evidence:
        scenes.setdefault(name, FOLDER / name)
    if selected:
        scenes = {name: FOLDER / name for name in selected}
    for scene in sorted(scenes.values()):
        if not scene.exists():
            assertion = evidence.get(scene.name, {})
            status = "operation failed" if assertion.get("failure") else "scene not exported"
            if assertion.get("passed") is True and assertion.get("faces") == 0:
                status = "empty result reported (scene absent)"
            records.append({"scene": scene.name, "status": status, "assertions": assertion})
            save_gallery(records)
            continue
        scene_bytes = scene.read_bytes()
        scene_hash, scene_snapshot = artifact_snapshot(scene, scene_bytes)
        record = {"scene": scene.name, "scene_sha256": scene_hash, "scene_snapshot": scene_snapshot,
                  "renderer": str(RENDERER), "renderer_sha256": renderer_hash,
                  "settings": effective, "assertions": evidence.get(scene.name, {})}
        assertion = record["assertions"]
        if (assertion.get("scene_binding") == "explicit SHA256 binding"
                and assertion.get("scene_sha256") != record["scene_sha256"]):
            raise ValueError(f"Bound scene changed before capture: {scene.name}")
        old = cached.get(scene.name, {})
        png = scene.with_suffix(".png")
        if (record["assertions"].get("faces") == 0 and record["assertions"].get("passed") is True
                and record["assertions"].get("scene_binding") == "explicit SHA256 binding"):
            record["status"] = "empty result"
        elif old.get("status") == "rendered" and all(old.get(key) == record[key] for key in ("scene_sha256", "renderer_sha256", "settings")) and png.exists() and old.get("png_sha256") == digest(png):
            record.update(status="rendered", png=png.name, png_sha256=digest(png))
        else:
            ppm = scene.with_suffix(".ppm")
            if png.exists():
                old_image = png.with_name(png.stem + ".previous." + digest(png) + ".png")
                if not old_image.exists():
                    with old_image.open("xb") as archived:
                        archived.write(png.read_bytes())
            try:
                with scene.with_suffix(".render.log").open("wb") as log:
                    subprocess.run([str(RENDERER), str(ppm), str(FOLDER / scene_snapshot)], check=True,
                                   env=environment, stdout=log, stderr=subprocess.STDOUT,
                                   creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
                with Image.open(ppm) as picture:
                    picture.save(png)
                record.update(status="rendered", png=png.name, png_sha256=digest(png))
            except (OSError, subprocess.SubprocessError) as error:
                record.update(status="capture failed", error=str(error))
        if (digest(scene) != record["scene_sha256"] or digest(RENDERER) != renderer_hash
                or digest(FOLDER / scene_snapshot) != record["scene_sha256"]):
            record.update(status="capture inputs changed", error="Scene or renderer bytes changed during capture")
        for source in record["assertions"].get("assertion_sources", []):
            if (digest(Path(source["path"])) != source["sha256"]
                    or digest(FOLDER / source["snapshot"]) != source["sha256"]):
                record.update(status="capture inputs changed", error="Linked numerical report changed during capture")
        records.append(record)
        save_gallery(records)
        print(f"{scene.name}: {record['status']}", flush=True)


if __name__ == "__main__":
    main()
