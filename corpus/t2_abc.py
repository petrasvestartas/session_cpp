"""T2 harvest: ABC-dataset boolean-pair sampler + manifest schema.

NO BULK DOWNLOAD. This tool only works against a LOCAL mirror directory that someone
has already populated (see T2_HARVEST_PLAN.md for the fetch recipe and licensing). All
commands are safe to run with an empty/missing mirror: they report what they would do.

Pipeline
  index   walk the mirror, probe every STEP with step_probe, keep only importable valid
          closed solids, fingerprint + dedup    -> corpus/t2/index.jsonl
  sample  deterministic seeded pairing: pick two distinct models, place B by a rigid
          motion that makes the bounding boxes overlap, keep the pose only if B is
          genuinely PARTIALLY inside A (>=1 control point IN and >=1 OUT of A), record
          the OCCT truth for the three ops plus the inclusion-exclusion consistency
          verdict of that truth                 -> corpus/t2/manifest.jsonl + pair dirs
  stats   summarize an existing manifest

A sampled pair dir is directly runnable by the kernel: it contains chair0.stp (=A) and
chair1.stp (=B posed), so
    env SESSION_CHAIRS=<pair dir> SESSION_OP=cut ./build/main_7 zzzz
runs it exactly like the chairs_base corpus cell (corpus/runner.py run_chairs_base).

Manifest record (one JSON object per line):
  {"id": "abc_0000_00001234__abc_0000_00007777__p03",
   "dir": "corpus/t2/pairs/<id>",
   "a": {"src", "sha1", "solids", "faces", "volume", "bbox"},
   "b": {"src", "sha1", "solids", "faces", "volume", "bbox"},
   "pose": {"axis": [x,y,z], "deg": d, "trans": [x,y,z]},
   "interference": {"pts_in": n, "pts_out": n, "class": "partial"},
   "truth": {"cut": {"vol","solids","faces","valid"}, "common": {...}, "fuse": {...},
             "identity": {"implied_common_from_cut", "implied_common_from_fuse",
                          "verdict": "CONSISTENT|SELF-INCONSISTENT", "odd_cell"}},
   "provenance": {"dataset": "ABC", "chunk", "model_a", "model_b", "license": "..."},
   "created": "<iso8601>", "sampler_seed": n}

Usage:
  python corpus/t2_abc.py plan
  python corpus/t2_abc.py index  --mirror /data/abc/step
  python corpus/t2_abc.py sample --pairs 200 --seed 1
  python corpus/t2_abc.py stats
"""
import argparse
import datetime
import glob
import hashlib
import json
import math
import os
import random
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import runner                                    # noqa: E402
import step_rigid                                # noqa: E402

T2 = os.path.join(HERE, "t2")
INDEX = os.path.join(T2, "index.jsonl")
MANIFEST = os.path.join(T2, "manifest.jsonl")
PAIRS = os.path.join(T2, "pairs")
PT3 = re.compile(r"CARTESIAN_POINT\s*\(\s*'[^']*'\s*,\s*\(([^)]*)\)\s*\)")


def bbox_of(path):
    """Loose but valid bound: the convex hull of the 3D control points contains the
    NURBS geometry, so their bbox contains the model."""
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    for m in PT3.finditer(open(path, errors="replace").read()):
        parts = [p.strip() for p in m.group(1).split(",")]
        if len(parts) != 3:
            continue
        try:
            v = [float(p) for p in parts]
        except ValueError:
            continue
        for i in range(3):
            lo[i] = min(lo[i], v[i])
            hi[i] = max(hi[i], v[i])
    if lo[0] == float("inf"):
        return None
    return [lo, hi]


def sha1_of(path):
    h = hashlib.sha1()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def control_points(path, limit=400):
    pts = []
    for m in PT3.finditer(open(path, errors="replace").read()):
        parts = [p.strip() for p in m.group(1).split(",")]
        if len(parts) != 3:
            continue
        try:
            pts.append([float(p) for p in parts])
        except ValueError:
            pass
    if len(pts) <= limit:
        return pts
    step = len(pts) / float(limit)
    return [pts[int(i * step)] for i in range(limit)]


# ------------------------------------------------------------------- commands

def cmd_plan(args):
    print(__doc__.split("Usage:")[0].strip())
    print()
    print(open(os.path.join(HERE, "T2_HARVEST_PLAN.md")).read())


def cmd_index(args):
    os.makedirs(T2, exist_ok=True)
    if not args.mirror or not os.path.isdir(args.mirror):
        print("no mirror at %r -- nothing indexed. See T2_HARVEST_PLAN.md for how to "
              "populate one (no bulk download is performed by this tool)."
              % args.mirror)
        return
    files = sorted(glob.glob(os.path.join(args.mirror, "**", "*.st*p"),
                             recursive=True))[:args.limit or None]
    seen, kept, rejected = set(), 0, {}
    with open(INDEX, "w") as out:
        for i, f in enumerate(files):
            try:
                s = runner.probe_summary(f)
            except Exception as e:
                rejected["probe-error"] = rejected.get("probe-error", 0) + 1
                continue
            if s is None:
                rejected["import-fail"] = rejected.get("import-fail", 0) + 1
                continue
            if s["solids"] < 1 or s["valid"] != 1 or abs(s["volume"]) < 1e-9:
                rejected["not-valid-solid"] = rejected.get("not-valid-solid", 0) + 1
                continue
            fp = (s["solids"], s["shells"], s["faces"], round(s["volume"], 6))
            if fp in seen:
                rejected["duplicate"] = rejected.get("duplicate", 0) + 1
                continue
            seen.add(fp)
            bb = bbox_of(f)
            rec = {"src": f, "sha1": sha1_of(f), "solids": s["solids"],
                   "shells": s["shells"], "faces": s["faces"],
                   "volume": s["volume"], "bbox": bb,
                   "fingerprint": list(fp)}
            out.write(json.dumps(rec) + "\n")
            kept += 1
            if args.progress and i % args.progress == 0:
                print("%d/%d kept %d" % (i, len(files), kept), flush=True)
    print("indexed %d/%d models -> %s ; rejected: %s"
          % (kept, len(files), INDEX,
             ", ".join("%s %d" % kv for kv in sorted(rejected.items())) or "none"))


def pose_for(a, b, rng):
    """Rigid motion putting B's bbox center at a random point inside A's bbox."""
    alo, ahi = a["bbox"]
    blo, bhi = b["bbox"]
    axis = [rng.uniform(-1, 1) for _ in range(3)]
    n = math.sqrt(sum(x * x for x in axis)) or 1.0
    axis = [x / n for x in axis]
    deg = rng.uniform(0, 360)
    R = step_rigid.rot_matrix(axis, deg)
    bc = [(blo[i] + bhi[i]) * 0.5 for i in range(3)]
    bc_r = step_rigid.apply(R, bc)
    # target: random point in the middle half of A's bbox (biased to real overlap)
    tgt = [alo[i] + (ahi[i] - alo[i]) * rng.uniform(0.25, 0.75) for i in range(3)]
    trans = [tgt[i] - bc_r[i] for i in range(3)]
    return axis, deg, trans


def interference(a_path, b_path):
    """Classify by classifying B's control points against solid A."""
    pts = control_points(b_path, limit=120)
    if not pts:
        return {"pts_in": 0, "pts_out": 0, "class": "unknown"}
    n_in = n_out = 0
    CH = 200
    for i in range(0, len(pts), CH):
        args = ["--inside", a_path]
        for p in pts[i:i + CH]:
            args += ["%.9g" % c for c in p]
        out = runner.probe_out(args)
        for s in re.findall(r"-> (IN|OUT|ON)$", out, re.M):
            if s == "IN":
                n_in += 1
            elif s == "OUT":
                n_out += 1
    cls = ("partial" if n_in and n_out else
           "contained" if n_in and not n_out else "disjoint-or-outside")
    return {"pts_in": n_in, "pts_out": n_out, "class": cls}


def truth_for(a_path, b_path):
    ops = {op: runner.probe_bool(op, a_path, b_path) for op in runner.OPS}
    vA = runner.probe_summary(a_path)["volume"]
    vB = runner.probe_summary(b_path)["volume"]
    c_cut = vA - ops["cut"]["volume"]
    c_fuse = vA + vB - ops["fuse"]["volume"]
    tol = max(0.005 * abs(vA), 1e-6)
    ok_cut = abs(c_cut - ops["common"]["volume"]) <= tol
    ok_fuse = abs(c_fuse - ops["common"]["volume"]) <= tol
    if ok_cut and ok_fuse:
        verdict, odd = "CONSISTENT", ""
    elif ok_cut:
        verdict, odd = "SELF-INCONSISTENT", "fuse"
    elif ok_fuse:
        verdict, odd = "SELF-INCONSISTENT", "cut"
    else:
        verdict, odd = "SELF-INCONSISTENT", ("common"
                                             if abs(c_cut - c_fuse) <= tol else
                                             "unresolved")
    return {op: {"vol": ops[op]["volume"], "solids": ops[op]["solids"],
                 "faces": ops[op]["faces"], "valid": ops[op]["valid"]}
            for op in runner.OPS} | {
        "identity": {"implied_common_from_cut": c_cut,
                     "implied_common_from_fuse": c_fuse,
                     "verdict": verdict, "odd_cell": odd, "volA": vA, "volB": vB}}


def cmd_sample(args):
    if not os.path.exists(INDEX):
        print("no index at %s -- run `t2_abc.py index --mirror <dir>` first. "
              "Sampler design is documented in T2_HARVEST_PLAN.md; nothing to do."
              % INDEX)
        return
    models = [json.loads(l) for l in open(INDEX)]
    models = [m for m in models if m.get("bbox")]
    if len(models) < 2:
        print("index has %d usable models -- need >= 2" % len(models))
        return
    os.makedirs(PAIRS, exist_ok=True)
    rng = random.Random(args.seed)
    made, tries = 0, 0
    with open(MANIFEST, "a" if args.append else "w") as out:
        while made < args.pairs and tries < args.pairs * args.max_tries:
            tries += 1
            a, b = rng.sample(models, 2)
            axis, deg, trans = pose_for(a, b, rng)
            pid = "%s__%s__p%02d" % (
                os.path.splitext(os.path.basename(a["src"]))[0],
                os.path.splitext(os.path.basename(b["src"]))[0], made)
            pdir = os.path.join(PAIRS, pid)
            os.makedirs(pdir, exist_ok=True)
            a_dst = os.path.join(pdir, "chair0.stp")
            b_dst = os.path.join(pdir, "chair1.stp")
            if not os.path.exists(a_dst):
                open(a_dst, "w").write(open(a["src"], errors="replace").read())
            step_rigid.transform_file(b["src"], b_dst, axis, deg, trans)
            inter = interference(a_dst, b_dst)
            if inter["class"] != "partial" and not args.keep_all:
                continue
            truth = truth_for(a_dst, b_dst)
            # The control-point prefilter is loose (poles need not lie on the solid):
            # confirm real interference with the oracle's own common volume.
            inter["occt_common"] = truth["common"]["vol"]
            if abs(inter["occt_common"]) < 1e-9:
                inter["class"] = "empty-common"
                if not args.keep_all:
                    continue
            rec = {"id": pid, "dir": pdir,
                   "a": {k: a[k] for k in ("src", "sha1", "solids", "faces",
                                           "volume", "bbox")},
                   "b": {k: b[k] for k in ("src", "sha1", "solids", "faces",
                                           "volume", "bbox")},
                   "pose": {"axis": axis, "deg": deg, "trans": trans},
                   "interference": inter,
                   "truth": truth,
                   "provenance": {"dataset": args.dataset,
                                  "model_a": a["src"], "model_b": b["src"],
                                  "license": "ABC/Onshape public-document terms; "
                                             "keep provenance, internal testing"},
                   "created": datetime.datetime.now().isoformat(timespec="seconds"),
                   "sampler_seed": args.seed}
            out.write(json.dumps(rec) + "\n")
            made += 1
    print("sampled %d pairs (%d poses tried) -> %s" % (made, tries, MANIFEST))


def cmd_stats(args):
    if not os.path.exists(MANIFEST):
        print("no manifest at %s" % MANIFEST)
        return
    recs = [json.loads(l) for l in open(MANIFEST)]
    cls, ident = {}, {}
    for r in recs:
        c = r["interference"]["class"]
        cls[c] = cls.get(c, 0) + 1
        v = r["truth"]["identity"]["verdict"]
        ident[v] = ident.get(v, 0) + 1
    print("pairs: %d" % len(recs))
    print("interference: " + ", ".join("%s %d" % kv for kv in sorted(cls.items())))
    print("oracle identity: " + ", ".join("%s %d" % kv for kv in sorted(ident.items())))


def main():
    ap = argparse.ArgumentParser(description="T2 ABC pair sampler (no bulk download)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("plan"); p.set_defaults(fn=cmd_plan)
    p = sub.add_parser("index"); p.set_defaults(fn=cmd_index)
    p.add_argument("--mirror", default="")
    p.add_argument("--limit", type=int, default=0)
    p.add_argument("--progress", type=int, default=200)
    p = sub.add_parser("sample"); p.set_defaults(fn=cmd_sample)
    p.add_argument("--pairs", type=int, default=100)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--max-tries", type=int, default=6)
    p.add_argument("--dataset", default="ABC")
    p.add_argument("--append", action="store_true")
    p.add_argument("--keep-all", action="store_true")
    p = sub.add_parser("stats"); p.set_defaults(fn=cmd_stats)
    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
