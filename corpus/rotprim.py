"""T0R — rotated-primitive tier: the minimal reproduction of the rotated-chair failure.

WHY THIS TIER EXISTS
main_7.cpp's primitive matrix carries an ORIENTED battery (cylR/boxR/torR/coneR, 6 pairs
= 18 cells) that `SESSION_NO_ROT=1` silently skips — and that flag is set in almost every
documented repro. "matrix 45/45" is therefore 15 general-position pairs only; the rotated
cells were never in any headline number. This tier replaces that hidden battery with a
seeded, reproducible, MUCH larger one driven entirely from Python, so nothing can be
masked by an env var: operands are written as STEP and rotated by us, the kernel is
driven through its `SESSION_CHAIRS=<dir>` entry point (A=chair0.stp, B=chair1.stp), and
every skipped cell is recorded as a cell, never dropped.

Primitives are OCCT-exact and identical to the matrix battery's placements
(box 4^3 = 64, cyl r1.5 h6 = 42.4115, cone r2 h4 = 16.7552, torus 2/0.8 = 25.2662,
sphere 2.5 = 65.4498), plus the four platonic solids (planar-faced, non-axis-aligned —
the geometry class the chairs represent; the kernel has NO BRep constructors for them,
so they are built through step_probe --poly).

ORACLE-FREE CHECKS (the point of the tier — no OCCT needed, so they cannot inherit an
oracle defect):
  equivariance   rotate BOTH operands by the same R: every volume and face count must be
                 unchanged
  sweep          rotate B through a fine angle sweep: the result volume must be
                 CONTINUOUS in the angle; a jump localises the exact breaking angle
  partition      vol(cut) + vol(common) = vol(A) at every angle
  self           A op R(A) with the same R applied to both copies: cut EMPTY,
                 common = A, fuse = A
  analytic       a sphere is invariant under rotation about its centre; a cylinder/cone/
                 torus is invariant under rotation about its own axis; two axis-aligned
                 boxes have an exact closed-form intersection volume. These are EXACT
                 truths, stronger than any oracle.

Usage:
  python corpus/rotprim.py operands                 # write primitive + platonic STEPs
  python corpus/rotprim.py run --rotations 20       # main tier -> ledger + tickets
  python corpus/rotprim.py sweep --pair box,cyl     # angle sweep continuity
  python corpus/rotprim.py equivariance             # both-operand rotation
  python corpus/rotprim.py selfop                   # A op R(A)
  python corpus/rotprim.py analytic                 # exact invariants
"""
import argparse
import concurrent.futures
import datetime
import json
import math
import os
import random
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import runner                                     # noqa: E402
import step_rigid                                 # noqa: E402

T0R = os.path.join(HERE, "t0r")
OPERANDS = os.path.join(T0R, "operands")
CELLS = os.path.join(T0R, "cells")
LEDGER = os.path.join(T0R, "ledger")
CELL_TIMEOUT = 180          # primitives are small; a longer run is a defect signal

# name -> (step_probe --prim args). Identical to the matrix battery's primitives.
PRIMS = {
    "box":  ["box", "4", "4", "4"],
    "sph":  ["sph", "2.5"],
    "cyl":  ["cyl", "1.5", "6"],
    "cone": ["cone", "2", "4"],
    "tor":  ["tor", "2", "0.8"],
}
PAIRS = [(a, b) for i, a in enumerate(sorted(PRIMS)) for b in sorted(PRIMS)[i:]]
OPS = ["cut", "common", "fuse"]
PHI = (1 + math.sqrt(5)) / 2


# ------------------------------------------------------------------- platonics

def platonic(kind):
    """(vertices, faces) with outward CCW faces, circumradius ~2."""
    if kind == "tetra":
        v = [(1, 1, 1), (1, -1, -1), (-1, 1, -1), (-1, -1, 1)]
        f = [(0, 1, 2), (0, 3, 1), (0, 2, 3), (1, 3, 2)]
    elif kind == "octa":
        v = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
        f = [(0, 2, 4), (2, 1, 4), (1, 3, 4), (3, 0, 4),
             (2, 0, 5), (1, 2, 5), (3, 1, 5), (0, 3, 5)]
    elif kind == "icosa":
        a, b = 1.0, PHI
        v = [(0, a, b), (0, -a, b), (0, a, -b), (0, -a, -b),
             (a, b, 0), (-a, b, 0), (a, -b, 0), (-a, -b, 0),
             (b, 0, a), (b, 0, -a), (-b, 0, a), (-b, 0, -a)]
        f = [(0, 1, 8), (0, 8, 4), (0, 4, 5), (0, 5, 10), (0, 10, 1),
             (1, 10, 7), (1, 7, 6), (1, 6, 8), (8, 6, 9), (8, 9, 4),
             (4, 9, 2), (4, 2, 5), (5, 2, 11), (5, 11, 10), (10, 11, 7),
             (3, 7, 11), (3, 11, 2), (3, 2, 9), (3, 9, 6), (3, 6, 7)]
    elif kind == "dodeca":
        p, q = PHI, 1 / PHI
        v = [(1, 1, 1), (1, 1, -1), (1, -1, 1), (1, -1, -1),
             (-1, 1, 1), (-1, 1, -1), (-1, -1, 1), (-1, -1, -1),
             (0, q, p), (0, q, -p), (0, -q, p), (0, -q, -p),
             (q, p, 0), (q, -p, 0), (-q, p, 0), (-q, -p, 0),
             (p, 0, q), (p, 0, -q), (-p, 0, q), (-p, 0, -q)]
        f = [(0, 8, 10, 2, 16), (0, 16, 17, 1, 12), (0, 12, 14, 4, 8),
             (8, 4, 18, 6, 10), (10, 6, 15, 13, 2), (2, 13, 3, 17, 16),
             (17, 3, 11, 9, 1), (1, 9, 5, 14, 12), (14, 5, 19, 18, 4),
             (18, 19, 7, 15, 6), (15, 7, 11, 3, 13), (9, 11, 7, 19, 5)]
    else:
        raise ValueError(kind)
    return [tuple(float(c) for c in p) for p in v], [list(x) for x in f]


def poly_volume(verts, faces):
    """Exact volume by the divergence theorem over the (planar) faces."""
    vol = 0.0
    for f in faces:
        a = verts[f[0]]
        for i in range(1, len(f) - 1):
            b, c = verts[f[i]], verts[f[i + 1]]
            vol += (a[0] * (b[1] * c[2] - b[2] * c[1])
                    - a[1] * (b[0] * c[2] - b[2] * c[0])
                    + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
    return abs(vol)


def orient_outward(verts, faces):
    """Flip any face whose plane normal points at the centroid (built data is
    hand-written; this makes the outward convention machine-checked)."""
    cx = [sum(p[i] for p in verts) / len(verts) for i in range(3)]
    out = []
    for f in faces:
        a, b, c = verts[f[0]], verts[f[1]], verts[f[2]]
        u = [b[i] - a[i] for i in range(3)]
        w = [c[i] - a[i] for i in range(3)]
        n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2],
             u[0] * w[1] - u[1] * w[0]]
        d = sum(n[i] * (a[i] - cx[i]) for i in range(3))
        out.append(list(f) if d > 0 else list(reversed(f)))
    return out


# ------------------------------------------------------------------- operands

def write_operands(args):
    os.makedirs(OPERANDS, exist_ok=True)
    made = {}
    for name, spec in PRIMS.items():
        dst = os.path.join(OPERANDS, "%s.step" % name)
        subprocess.run([runner.PROBE, "--prim", spec[0], dst] + spec[1:],
                       capture_output=True, text=True, timeout=CELL_TIMEOUT)
        s = runner.probe_summary(dst)
        made[name] = {"file": dst, "volume": s["volume"], "faces": s["faces"],
                      "kind": "primitive"}
    for kind in ("tetra", "octa", "icosa", "dodeca"):
        verts, faces = platonic(kind)
        # normalise to circumradius 2.5 (the sphere's radius) so a platonic actually
        # interferes with the 4x4x4 box instead of sitting inside it
        r = max(math.sqrt(sum(c * c for c in p)) for p in verts)
        k = 2.5 / r
        verts = [tuple(c * k for c in p) for p in verts]
        faces = orient_outward(verts, faces)
        txt = os.path.join(OPERANDS, "%s.poly" % kind)
        with open(txt, "w") as f:
            for p in verts:
                f.write("V %.17g %.17g %.17g\n" % p)
            for fc in faces:
                f.write("F " + " ".join(str(i) for i in fc) + "\n")
        dst = os.path.join(OPERANDS, "%s.step" % kind)
        subprocess.run([runner.PROBE, "--poly", txt, dst],
                       capture_output=True, text=True, timeout=CELL_TIMEOUT)
        s = runner.probe_summary(dst)
        exact = poly_volume(verts, faces)
        made[kind] = {"file": dst, "volume": s["volume"], "faces": s["faces"],
                      "kind": "platonic", "analytic_volume": exact,
                      "rel_vs_analytic": abs(s["volume"] - exact) / exact}
    json.dump(made, open(os.path.join(OPERANDS, "index.json"), "w"), indent=1)
    for k in sorted(made):
        m = made[k]
        extra = ("  analytic %.6f (rel %.1e)" % (m["analytic_volume"],
                                                 m["rel_vs_analytic"])
                 if "analytic_volume" in m else "")
        print("%-7s faces %-3d vol %12.6f%s" % (k, m["faces"], m["volume"], extra))
    return made


def operand_index():
    p = os.path.join(OPERANDS, "index.json")
    if not os.path.exists(p):
        sys.exit("no operands -- run: python corpus/rotprim.py operands")
    return json.load(open(p))


# ------------------------------------------------------------------- rotations

def rotations(n, seed):
    """Seeded axis/angle list; deterministic across runs and machines."""
    rng = random.Random(seed)
    out = []
    for k in range(n):
        while True:
            ax = [rng.uniform(-1, 1) for _ in range(3)]
            m = math.sqrt(sum(x * x for x in ax))
            if m > 1e-3:
                break
        out.append(([x / m for x in ax], rng.uniform(5.0, 175.0)))
    return out


def place(src, dst, axis, deg, trans=(0.0, 0.0, 0.0), centre=None):
    """Rotate about `centre` (default: the world origin) then translate."""
    if centre is not None:
        R = step_rigid.rot_matrix(axis, deg)
        rc = step_rigid.apply(R, list(centre))
        trans = tuple(centre[i] - rc[i] + trans[i] for i in range(3))
    step_rigid.transform_file(src, dst, axis, deg, trans)
    return dst


def point_mean(path):
    """Mean of the 3D control points — the same quantity the kernel's chairs battery
    uses to place its rotation centre (vmean over the BRep's vertices)."""
    tot = [0.0, 0.0, 0.0]
    n = 0
    for m in re.finditer(r"CARTESIAN_POINT\s*\(\s*'[^']*'\s*,\s*\(([^)]*)\)\s*\)",
                         open(path, errors="replace").read()):
        parts = [p.strip() for p in m.group(1).split(",")]
        if len(parts) != 3:
            continue
        try:
            v = [float(p) for p in parts]
        except ValueError:
            continue
        for i in range(3):
            tot[i] += v[i]
        n += 1
    return [c / max(1, n) for c in tot]


# -------------------------------------------------------------------- probing

def probe_topo(path):
    out = subprocess.run([runner.PROBE, path, "-n"], capture_output=True, text=True,
                         timeout=CELL_TIMEOUT).stdout
    d = {}
    for k in ("EDGES", "NAKED", "SEAM", "SHARED", "DEGEN", "NONMANIFOLD",
              "SHELLS_CLOSED", "SHELLS_OPEN"):
        m = re.search(r"^%s (\d+)" % k, out, re.M)
        d[k.lower()] = int(m.group(1)) if m else None
    return d


def run_cell(cell_id, a_file, b_file, op, truth, timeout=CELL_TIMEOUT):
    """Run one A op B through the kernel's STEP entry point; five-verdict + closure."""
    wd = os.path.join(CELLS, cell_id)
    os.makedirs(wd, exist_ok=True)
    shutil.copyfile(a_file, os.path.join(wd, "chair0.stp"))
    shutil.copyfile(b_file, os.path.join(wd, "chair1.stp"))
    out_step = os.path.join(wd, "chair0_%s_chair1.step" % op)
    if os.path.exists(out_step):
        os.remove(out_step)
    exe = os.path.join(runner.WORK, "main_7_snapshot")
    rc, out, dt, to = runner.run_kernel(exe, ["zzzz"],
                                        {"SESSION_CHAIRS": wd, "SESSION_OP": op},
                                        timeout=timeout)
    rec = {"cell": cell_id, "op": op, "secs": round(dt, 1),
           "truth_vol": truth["vol"], "truth_solids": truth.get("solids")}
    if to:
        rec["verdict"] = "timeout"
        return rec
    if rc != 0 or re.search(r"^chairs %s THREW" % op, out, re.M):
        rec.update(verdict="crash", tail=out[-300:])
        return rec
    m = re.search(r"^chairs\s+%s\s*: faces (\d+) solid (\d+)(?: shells (\d+))?"
                  r" naked (\d+) vol ([-\d.]+)" % op, out, re.M)
    if m:
        rec.update(kfaces=int(m.group(1)), ksolid=int(m.group(2)),
                   knaked=int(m.group(4)))
    volA, volB = truth["volA"], truth["volB"]
    if not os.path.exists(out_step):
        rec["verdict"] = "exact" if abs(truth["vol"]) < runner.EMPTY else "open"
        rec["our_vol"] = 0.0
        return rec
    meas = runner.measure_result(out_step, volA, volB)
    topo = probe_topo(out_step)
    # operand shells are closed and manifold, so any naked/non-manifold edge in the
    # combined file belongs to OUR result
    rec.update(our_vol=meas["vol"], our_solids=meas["solids"], our_valid=meas["valid"],
               naked=topo["naked"], nonmanifold=topo["nonmanifold"],
               shells_open=topo["shells_open"], shells_closed=topo["shells_closed"])
    rec["closed"] = (topo["naked"] == 0 and topo["nonmanifold"] == 0
                     and topo["shells_open"] == 0)
    tv = truth["vol"]
    if tv is None:              # --no-oracle: continuity/partition are oracle-free, and
        rec["verdict"] = ("exact" if rec["closed"] and meas["valid"] and meas["solids"]
                          else "open")           # an OCCT boolean per angle is minutes
        return rec
    if abs(tv) < runner.EMPTY:
        rec["verdict"] = "exact" if abs(meas["vol"]) < runner.EMPTY else "closed-wrong"
        return rec
    rec["rel"] = abs(meas["vol"] - tv) / abs(tv)
    if not rec["closed"] or meas["valid"] == 0 or meas["solids"] == 0:
        rec["verdict"] = "open"
    elif rec["rel"] > runner.VOLTOL:
        rec["verdict"] = "closed-wrong"
    else:
        rec["verdict"] = "exact"
    return rec


def truth_for(a_file, b_file, op=None):
    va = runner.probe_summary(a_file)["volume"]
    vb = runner.probe_summary(b_file)["volume"]
    ops = OPS if op is None else [op]
    t = {}
    for o in ops:
        r = runner.probe_bool(o, a_file, b_file)
        t[o] = {"vol": r["volume"], "solids": r["solids"], "faces": r["faces"],
                "valid": r["valid"], "volA": va, "volB": vb}
    return t


# ------------------------------------------------------------------- main tier

def save_ledger(name, payload):
    """LEDGER HONESTY: every ledger records the exact kernel binary (sha1), the FULL
    SESSION_* environment the cells ran under, and anything skipped with a reason.
    The oriented primitive battery stayed invisible for a whole campaign because a
    SESSION_NO_ROT skip was never written down anywhere."""
    os.makedirs(LEDGER, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    p = os.path.join(LEDGER, "%s_%s.json" % (name, ts))
    exe = os.path.join(runner.WORK, "main_7_snapshot")
    payload = dict(payload)
    payload["ts"] = ts
    payload["exe_meta"] = runner.exe_id(exe) if os.path.exists(exe) else None
    payload["session_env"] = {k: v for k, v in os.environ.items()
                              if k.startswith("SESSION_")}
    payload["session_env_per_cell"] = {"SESSION_CHAIRS": "<cell dir>",
                                       "SESSION_OP": "<op>"}
    payload.setdefault("skipped", [])
    json.dump(payload, open(p, "w"), indent=1)
    print("ledger: %s  (kernel %s, %d skipped)"
          % (p, (payload["exe_meta"] or {}).get("sha1", "?")[:12],
             len(payload["skipped"])))
    return p


def counts_of(recs):
    c = {v: 0 for v in runner.VERDICTS}
    for r in recs:
        c[r["verdict"]] += 1
    return c


def ticket_for(rec, kind, note, chairs_dir):
    t = {"battery": "t0r", "op": rec["op"], "cfg": rec["cell"],
         "chairs_dir": os.path.relpath(chairs_dir, runner.CPP),
         "vol": rec.get("truth_vol"), "solids": rec.get("truth_solids"),
         "gate_solids": False, "vol_derived": False}
    r = dict(rec, truth=t, our_vol=rec.get("our_vol"), verdict=rec["verdict"])
    return runner.write_ticket("t0r/%s" % rec["cell"], r, "n/a (%s)" % kind, {},
                               note=note, kind=kind)


def cmd_run(args):
    idx = operand_index()
    names = [n for n in sorted(idx) if not args.only or n in args.only.split(",")]
    pairs = [(a, b) for i, a in enumerate(names) for b in names[i:]]
    rots = rotations(args.rotations, args.seed)
    os.makedirs(CELLS, exist_ok=True)
    jobs = []
    for (a, b) in pairs:
        for ri, (axis, deg) in enumerate(rots):
            jobs.append((a, b, ri, axis, deg))
    print("t0r: %d pairs x %d rotations x %d ops = %d cells"
          % (len(pairs), len(rots), len(OPS), len(jobs) * len(OPS)))

    def one(job):
        a, b, ri, axis, deg = job
        cid = "%s_x_%s_r%02d" % (a, b, ri)
        wd = os.path.join(CELLS, cid)
        os.makedirs(wd, exist_ok=True)
        bf = place(idx[b]["file"], os.path.join(wd, "B_rot.step"), axis, deg)
        try:
            truth = truth_for(idx[a]["file"], bf)
        except Exception as e:
            return [{"cell": cid, "op": o, "verdict": "crash",
                     "tail": "oracle failure: %r" % e} for o in OPS]
        recs = []
        for op in OPS:
            r = run_cell(cid, idx[a]["file"], bf, op, truth[op], args.timeout)
            r.update(pair=[a, b], rot=ri, axis=[round(x, 6) for x in axis],
                     deg=round(deg, 4))
            recs.append(r)
        # oracle-free partition identity at this angle
        vols = {r["op"]: r.get("our_vol") for r in recs}
        if all(v is not None for v in vols.values()):
            vA, vB = truth["cut"]["volA"], truth["cut"]["volB"]
            r1 = vols["cut"] + vols["common"] - vA
            r2 = vols["fuse"] - (vA + vB - vols["common"])
            for r in recs:
                r["partition_r1"], r["partition_r2"] = r1, r2
                r["partition_ok"] = (abs(r1) <= 0.01 * vA and abs(r2) <= 0.01 * vA)
        return recs

    out = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as ex:
        for recs in ex.map(one, jobs):
            out.extend(recs)
            r = recs[0]
            bad = [x for x in recs if x["verdict"] != "exact"]
            print("%-22s %s%s" % (r["cell"],
                                  " ".join("%s:%s" % (x["op"], x["verdict"])
                                           for x in recs),
                                  "" if not bad else
                                  ("  partition_ok=%s" % r.get("partition_ok"))),
                  flush=True)
    counts = counts_of(out)
    npart = sum(1 for r in out if r.get("partition_ok") is False) // len(OPS)
    print("\nT0R ledger (%d cells): %s" %
          (len(out), "  ".join("%s %d" % (v, counts[v]) for v in runner.VERDICTS)))
    print("closure failures: %d, partition-identity violations: %d configs"
          % (sum(1 for r in out if r.get("closed") is False), npart))
    save_ledger("t0r", {"counts": counts, "rotations": args.rotations,
                        "seed": args.seed, "cells": out})
    # tickets: worst cell per (pair, op) class
    seen = set()
    for r in sorted(out, key=lambda x: -runner.RANK[x["verdict"]]):
        if r["verdict"] == "exact":
            continue
        key = (tuple(r.get("pair", [])), r["op"])
        if key in seen or not args.tickets:
            continue
        seen.add(key)
        note = ("Rotated-primitive tier: %s %s at rotation r%02d "
                "(axis %s, %.2f deg). our vol %s vs OCCT %s; naked %s, non-manifold %s, "
                "open shells %s. Partition identity at this angle: cut+common-A=%s, "
                "fuse-(A+B-com)=%s. Operands are OCCT-exact primitives, so this is a "
                "MINIMAL reproduction of the rotated-chair class."
                % (r.get("pair"), r["op"], r.get("rot", -1), r.get("axis"),
                   r.get("deg", 0.0), r.get("our_vol"), r.get("truth_vol"),
                   r.get("naked"), r.get("nonmanifold"), r.get("shells_open"),
                   None if r.get("partition_r1") is None else round(r["partition_r1"], 4),
                   None if r.get("partition_r2") is None else round(r["partition_r2"], 4)))
        ticket_for(r, "rotated-primitive failure", note,
                   os.path.join(CELLS, r["cell"]))
    return out


# ---------------------------------------------------------------- sweep tier

def cmd_sweep(args):
    idx = operand_index()
    a, b = args.pair.split(",")
    axis = [float(x) for x in args.axis.split(",")]
    wd = os.path.join(CELLS, "sweep_%s_x_%s" % (a, b))
    os.makedirs(wd, exist_ok=True)
    rows = []
    angles = [args.start + i * args.step
              for i in range(int((args.end - args.start) / args.step) + 1)]
    print("sweep %s x %s about %s: %d angles, ops %s"
          % (a, b, axis, len(angles), args.ops))

    def one(deg):
        cid = "sweep_%s_x_%s_%07.3f" % (a, b, deg)
        cdir = os.path.join(CELLS, cid)
        os.makedirs(cdir, exist_ok=True)
        bf = place(idx[b]["file"], os.path.join(cdir, "B_rot.step"), axis, deg)
        truth = truth_for(idx[a]["file"], bf)
        row = {"deg": deg}
        for op in args.ops.split(","):
            r = run_cell(cid, idx[a]["file"], bf, op, truth[op], args.timeout)
            row[op] = {"our": r.get("our_vol"), "occt": truth[op]["vol"],
                       "verdict": r["verdict"], "naked": r.get("naked"),
                       "nonmanifold": r.get("nonmanifold"),
                       "closed": r.get("closed")}
        return row

    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as ex:
        for row in ex.map(one, angles):
            rows.append(row)
            print("  %7.3f deg  %s" % (row["deg"], "  ".join(
                "%s our %s occt %.4f [%s]" % (op, "None" if row[op]["our"] is None
                                              else "%.4f" % row[op]["our"],
                                              row[op]["occt"], row[op]["verdict"])
                for op in args.ops.split(","))), flush=True)
    rows.sort(key=lambda r: r["deg"])
    # continuity: the TRUE volume is continuous in the angle, so a jump in ours that
    # the oracle curve does not show localises the breaking angle.
    jumps = []
    for op in args.ops.split(","):
        for i in range(1, len(rows)):
            p, q = rows[i - 1][op], rows[i][op]
            if p["our"] is None or q["our"] is None:
                continue
            d_our = abs(q["our"] - p["our"])
            d_occt = (abs(q["occt"] - p["occt"])
                      if p.get("occt") is not None and q.get("occt") is not None
                      else 0.0)          # --no-oracle: continuity alone is the test
            if d_our > max(args.jump, 3 * d_occt + args.jump):
                jumps.append({"op": op, "from_deg": rows[i - 1]["deg"],
                              "to_deg": rows[i]["deg"], "d_our": d_our,
                              "d_occt": d_occt,
                              "our": [p["our"], q["our"]],
                              "occt": [p.get("occt"), q.get("occt")]})
    print("\ndiscontinuities: %d" % len(jumps))
    for j in jumps:
        print("  %s: %.3f -> %.3f deg, our jumps %.4f (oracle moves %.4f): %.4f -> %.4f"
              % (j["op"], j["from_deg"], j["to_deg"], j["d_our"], j["d_occt"],
                 j["our"][0], j["our"][1]))
    save_ledger("sweep_%s_x_%s" % (a, b),
                {"pair": [a, b], "axis": axis, "ops": args.ops, "rows": rows,
                 "discontinuities": jumps})
    return rows, jumps


def cmd_sweepchairs(args):
    """Fine angle sweep on the CHAIRS (freeform NURBS operands). This is the sweep that
    matters: the chairs' STEP path works (base cut/common/fuse are exact), so a
    discontinuity here is the rotated-chair defect itself, localised to an angle.
    Rotation is about the midpoint of the two operands' control-point means — the same
    centre the kernel's own rotated battery uses."""
    wd = os.path.join(runner.WORK, "chairs")
    A = os.path.join(wd, "chair0.stp")
    B = os.path.join(wd, "chair1.stp")
    if not os.path.exists(A):
        runner.prep_work(runner.KERNEL)
    vA = runner.probe_summary(A)["volume"]
    vB = runner.probe_summary(B)["volume"]
    ca, cb = point_mean(A), point_mean(B)
    centre = [(ca[i] + cb[i]) / 2 for i in range(3)]
    axis = [float(x) for x in args.axis.split(",")]
    angles = [args.start + i * args.step
              for i in range(int(round((args.end - args.start) / args.step)) + 1)]
    print("chairs sweep about %s, centre %s: %d angles x ops %s"
          % (axis, [round(c, 4) for c in centre], len(angles), args.ops))

    def one(deg):
        cid = "chairsweep_%07.3f" % deg
        cdir = os.path.join(CELLS, cid)
        os.makedirs(cdir, exist_ok=True)
        bf = place(B, os.path.join(cdir, "B_rot.step"), axis, deg, centre=centre)
        row = {"deg": deg}
        for op in args.ops.split(","):
            if args.no_oracle:
                t = {"vol": None, "volA": vA, "volB": vB, "solids": None}
            else:
                t = truth_for(A, bf, op)[op]
            r = run_cell(cid, A, bf, op, t, args.timeout)
            row[op] = {"our": r.get("our_vol"), "occt": t["vol"],
                       "occt_solids": t["solids"], "verdict": r["verdict"],
                       "faces": r.get("kfaces"), "naked": r.get("naked"),
                       "nonmanifold": r.get("nonmanifold"),
                       "closed": r.get("closed"),
                       "err": (None if r.get("our_vol") is None
                               or t["vol"] is None
                               else abs(r["our_vol"] - t["vol"]))}
        return row

    rows = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as ex:
        for row in ex.map(one, angles):
            rows.append(row)
            print("  %7.3f  %s" % (row["deg"], "  ".join(
                "%s our %s occt %s err %s naked %s [%s]"
                % (op, "None" if row[op]["our"] is None else "%.4f" % row[op]["our"],
                   "n/a" if row[op]["occt"] is None else "%.4f" % row[op]["occt"],
                   "None" if row[op]["err"] is None else "%.4f" % row[op]["err"],
                   row[op]["naked"], row[op]["verdict"])
                for op in args.ops.split(","))), flush=True)
    rows.sort(key=lambda r: r["deg"])
    analysis = analyse_sweep(rows, args.ops.split(","), args.jump)
    save_ledger("sweep_chairs", {"centre": centre, "axis": axis, "rows": rows,
                                 "analysis": analysis})
    return rows, analysis


def analyse_sweep(rows, ops, jump):
    """Discontinuity detection + CLIFF vs RAMP classification.
    cliff = the error is ~0 up to an angle then jumps (a predicate with an angular
    tolerance); ramp = the error grows smoothly from zero (accumulated numerics)."""
    out = {}
    for op in ops:
        errs = [(r["deg"], r[op]["err"]) for r in rows if r[op]["err"] is not None]
        jumps = []
        for i in range(1, len(rows)):
            p, q = rows[i - 1][op], rows[i][op]
            if p["our"] is None or q["our"] is None:
                continue
            d_our = abs(q["our"] - p["our"])
            d_occt = (abs(q["occt"] - p["occt"])
                      if p.get("occt") is not None and q.get("occt") is not None
                      else 0.0)          # --no-oracle: continuity alone is the test
            if d_our > max(jump, 3 * d_occt + jump):
                jumps.append({"from_deg": rows[i - 1]["deg"], "to_deg": rows[i]["deg"],
                              "d_our": d_our, "d_occt": d_occt,
                              "our": [p["our"], q["our"]],
                              "occt": [p.get("occt"), q.get("occt")]})
        kind, detail = "clean", ""
        if errs:
            e = [x[1] for x in errs]
            emax = max(e)
            small = [x for x in errs if x[1] <= 0.05 * max(emax, 1e-9)]
            first_big = next((d for d, v in errs if v > 0.05 * max(emax, 1e-9)), None)
            if emax < 1e-3:
                kind = "clean"
            elif jumps:
                kind = "cliff"
                detail = ("error stays <=5%% of max up to %.3f deg (%d angles), then "
                          "jumps at %.3f->%.3f deg"
                          % (small[-1][0] if small else -1, len(small),
                             jumps[0]["from_deg"], jumps[0]["to_deg"]))
            else:
                # monotone growth from ~0 => ramp
                kind = "ramp" if e[0] < 0.2 * emax else "offset"
                detail = ("error %.4f at %.2f deg -> %.4f at %.2f deg with no jump"
                          % (e[0], errs[0][0], emax,
                             max(errs, key=lambda x: x[1])[0]))
            out[op] = {"max_err": emax, "first_big_deg": first_big,
                       "kind": kind, "detail": detail, "jumps": jumps}
        else:
            out[op] = {"max_err": None, "kind": "no-data", "jumps": jumps}
        print("\n[%s] %s  max err %s  %s" %
              (op, out[op]["kind"],
               "None" if out[op]["max_err"] is None else "%.4f" % out[op]["max_err"],
               out[op].get("detail", "")))
        for j in jumps:
            print("   jump %.3f -> %.3f deg: ours %.4f -> %.4f (oracle moves %.4f)"
                  % (j["from_deg"], j["to_deg"], j["our"][0], j["our"][1], j["d_occt"]))
    return out


# ------------------------------------------------------- oracle-free invariants

def cmd_equivariance(args):
    """Rotate BOTH operands by the same R: volumes and face counts must not move."""
    idx = operand_index()
    names = [n for n in sorted(idx) if not args.only or n in args.only.split(",")]
    pairs = [(a, b) for i, a in enumerate(names) for b in names[i:]]
    rots = rotations(args.rotations, args.seed + 7717)
    rows, viol = [], []

    def one(job):
        a, b, ri, axis, deg = job
        base_id = "equi_%s_x_%s_ref" % (a, b)
        wd = os.path.join(CELLS, base_id)
        os.makedirs(wd, exist_ok=True)
        # reference: B offset by a fixed "interference" rotation so the pair actually
        # intersects, then BOTH operands moved by R for the test run
        bf = place(idx[b]["file"], os.path.join(wd, "B_ref.step"), [0, 0, 1], 25.0)
        t0 = truth_for(idx[a]["file"], bf)
        ref = {op: run_cell(base_id, idx[a]["file"], bf, op, t0[op], args.timeout)
               for op in OPS}
        mid = "equi_%s_x_%s_r%02d" % (a, b, ri)
        md = os.path.join(CELLS, mid)
        os.makedirs(md, exist_ok=True)
        af2 = place(idx[a]["file"], os.path.join(md, "A_mov.step"), axis, deg)
        bf2 = place(bf, os.path.join(md, "B_mov.step"), axis, deg)
        t1 = truth_for(af2, bf2)
        mov = {op: run_cell(mid, af2, bf2, op, t1[op], args.timeout) for op in OPS}
        out = []
        for op in OPS:
            r0, r1 = ref[op], mov[op]
            v0, v1 = r0.get("our_vol"), r1.get("our_vol")
            f0, f1 = r0.get("kfaces"), r1.get("kfaces")
            rel = (None if v0 in (None, 0) or v1 is None
                   else abs(v1 - v0) / max(abs(v0), 1e-9))
            bad = (r0["verdict"] != r1["verdict"] or f0 != f1
                   or (rel is not None and rel > 0.005))
            out.append({"pair": [a, b], "rot": ri, "op": op, "axis": axis, "deg": deg,
                        "ref_vol": v0, "mov_vol": v1, "rel": rel,
                        "ref_faces": f0, "mov_faces": f1,
                        "ref_verdict": r0["verdict"], "mov_verdict": r1["verdict"],
                        "violation": bool(bad), "rec": r1})
        return out

    jobs = [(a, b, ri, ax, dg) for (a, b) in pairs
            for ri, (ax, dg) in enumerate(rots)]
    print("equivariance: %d pairs x %d rotations" % (len(pairs), len(rots)))
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as ex:
        for res in ex.map(one, jobs):
            for r in res:
                rows.append({k: v for k, v in r.items() if k != "rec"})
                mark = "VIOLATION" if r["violation"] else "ok"
                print("  %-14s r%02d %-6s ref %s / moved %s  faces %s/%s  %s"
                      % ("%s x %s" % (r["pair"][0], r["pair"][1]), r["rot"], r["op"],
                         "None" if r["ref_vol"] is None else "%.4f" % r["ref_vol"],
                         "None" if r["mov_vol"] is None else "%.4f" % r["mov_vol"],
                         r["ref_faces"], r["mov_faces"], mark), flush=True)
                if r["violation"]:
                    viol.append(r)
    print("\nequivariance violations: %d / %d" % (len(viol), len(rows)))
    save_ledger("equivariance", {"rows": rows, "violations": len(viol)})
    if args.tickets:
        for r in viol[:20]:
            note = ("Rigid-motion equivariance broken on primitives: %s %s, both "
                    "operands rotated %.2f deg about %s -> volume %s vs %s unmoved "
                    "(rel %s), faces %s vs %s, verdict %s vs %s. No oracle involved: "
                    "the same solid pair in a different orientation must give the same "
                    "answer." % (r["pair"], r["op"], r["deg"],
                                 [round(x, 4) for x in r["axis"]],
                                 r["mov_vol"], r["ref_vol"],
                                 None if r["rel"] is None else round(r["rel"], 6),
                                 r["mov_faces"], r["ref_faces"],
                                 r["mov_verdict"], r["ref_verdict"]))
            rec = dict(r["rec"] if "rec" in r else {}, op=r["op"],
                       cell="equi_%s_x_%s_r%02d" % (r["pair"][0], r["pair"][1],
                                                    r["rot"]),
                       verdict="closed-wrong")
            ticket_for(rec, "equivariance violation", note,
                       os.path.join(CELLS, rec["cell"]))
    return rows, viol


def cmd_selfop(args):
    """A op R(A) with R applied to BOTH copies: cut EMPTY, common = A, fuse = A."""
    idx = operand_index()
    names = [n for n in sorted(idx) if not args.only or n in args.only.split(",")]
    rots = rotations(args.rotations, args.seed + 31337)
    rows, viol = [], []
    for name in names:
        vA = idx[name]["volume"]
        nfA = idx[name]["faces"]
        for ri, (axis, deg) in enumerate(rots):
            cid = "self_%s_r%02d" % (name, ri)
            wd = os.path.join(CELLS, cid)
            os.makedirs(wd, exist_ok=True)
            af = place(idx[name]["file"], os.path.join(wd, "A_rot.step"), axis, deg)
            bf = os.path.join(wd, "B_rot.step")
            shutil.copyfile(af, bf)
            exp = {"cut": 0.0, "common": vA, "fuse": vA}
            expf = {"cut": 0, "common": nfA, "fuse": nfA}
            for op in OPS:
                truth = {"vol": exp[op], "volA": vA, "volB": vA, "solids": 1}
                r = run_cell(cid, af, bf, op, truth, args.timeout)
                ov = r.get("our_vol")
                kf = r.get("kfaces")
                bad = (ov is None or abs(ov - exp[op]) > 0.005 * vA
                       or (kf is not None and kf != expf[op]))
                row = {"solid": name, "rot": ri, "op": op, "deg": round(deg, 3),
                       "expected_vol": exp[op], "expected_faces": expf[op],
                       "our_vol": ov, "our_faces": kf, "verdict": r["verdict"],
                       "naked": r.get("naked"), "violation": bool(bad)}
                rows.append(row)
                print("  %-7s r%02d %-6s want %8.4f/%2d got %s/%s [%s] %s"
                      % (name, ri, op, exp[op], expf[op],
                         "None" if ov is None else "%.4f" % ov, kf, r["verdict"],
                         "VIOLATION" if bad else "ok"), flush=True)
                if bad:
                    viol.append(row)
                    if args.tickets and len(viol) <= 20:
                        note = ("A-op-A under rotation: %s rotated %.2f deg and "
                                "booleaned against an identical copy. %s must give vol "
                                "%.4f with %d faces; got vol %s with %s faces "
                                "(verdict %s, naked %s). Exact face-on-face coincidence "
                                "in a NON-axis-aligned pose — same class as the chairs, "
                                "on a %d-face primitive."
                                % (name, deg, op, exp[op], expf[op], ov, kf,
                                   r["verdict"], r.get("naked"), nfA))
                        ticket_for(dict(r, cell=cid), "self-boolean violation", note, wd)
    print("\nA-op-A violations: %d / %d" % (len(viol), len(rows)))
    save_ledger("selfop", {"rows": rows, "violations": len(viol)})
    return rows, viol


def cmd_analytic(args):
    """Exact, oracle-free checks.
    1. rotation-invariant operands: a sphere rotated about its centre, and a cylinder /
       cone / torus rotated about its own axis, are the SAME solid -- so any boolean
       against a fixed partner must be bit-stable across those rotations;
    2. axis-aligned box x box: the intersection volume is a closed-form product."""
    idx = operand_index()
    rows, viol = [], []
    # ---- 1. self-symmetric rotations
    sym = {"sph": [[1, 0, 0], [0, 1, 0], [0, 0, 1], [0.3, 0.5, 0.81]],
           "cyl": [[0, 0, 1]], "cone": [[0, 0, 1]], "tor": [[0, 0, 1]]}
    for name, axes in sym.items():
        if args.only and name not in args.only.split(","):
            continue
        partner = idx["box"]["file"]
        base = run_cell("anal_%s_ref" % name, partner, idx[name]["file"], "common",
                        dict(truth_for(partner, idx[name]["file"], "common")["common"]),
                        args.timeout)
        for ai, axis in enumerate(axes):
            for deg in (17.0, 90.0, 143.0):
                cid = "anal_%s_a%d_%03.0f" % (name, ai, deg)
                wd = os.path.join(CELLS, cid)
                os.makedirs(wd, exist_ok=True)
                bf = place(idx[name]["file"], os.path.join(wd, "B_sym.step"), axis, deg)
                t = truth_for(partner, bf, "common")["common"]
                r = run_cell(cid, partner, bf, "common", t, args.timeout)
                v0, v1 = base.get("our_vol"), r.get("our_vol")
                rel = (None if v0 in (None, 0) or v1 is None
                       else abs(v1 - v0) / abs(v0))
                bad = rel is None or rel > 0.005
                rows.append({"check": "symmetry", "solid": name, "axis": axis,
                             "deg": deg, "ref_vol": v0, "rot_vol": v1, "rel": rel,
                             "violation": bool(bad)})
                print("  symmetry %-5s axis %s %5.1f deg: ref %s rot %s rel %s %s"
                      % (name, axis, deg,
                         "None" if v0 is None else "%.4f" % v0,
                         "None" if v1 is None else "%.4f" % v1,
                         "None" if rel is None else "%.2e" % rel,
                         "VIOLATION" if bad else "ok"), flush=True)
                if bad:
                    viol.append(rows[-1])
    # ---- 1b. sphere x sphere: closed-form lens volume
    #      V = pi (R+r-d)^2 (d^2 + 2d(R+r) - 3(R-r)^2) / (12 d)
    if not args.only or "sph" in args.only.split(","):
        R = 2.5
        for d in (1.0, 2.5, 4.0, 4.9):
            cid = "anal_sphsph_%g" % d
            wd = os.path.join(CELLS, cid)
            os.makedirs(wd, exist_ok=True)
            bf = place(idx["sph"]["file"], os.path.join(wd, "B_shift.step"),
                       [0, 0, 1], 0.0, (d, 0.0, 0.0))
            lens = (math.pi * (2 * R - d) ** 2 * (d * d + 4 * d * R) / (12 * d)
                    if d < 2 * R else 0.0)
            vs = 4.0 / 3.0 * math.pi * R ** 3
            exact = {"common": lens, "cut": vs - lens, "fuse": 2 * vs - lens}
            for op in OPS:
                t = {"vol": exact[op], "volA": vs, "volB": vs, "solids": 1}
                r = run_cell(cid, idx["sph"]["file"], bf, op, t, args.timeout)
                got = r.get("our_vol")
                bad = got is None or abs(got - exact[op]) > max(1e-9, 1e-4 * vs)
                rows.append({"check": "sphsph", "d": d, "op": op,
                             "exact": exact[op], "our": got,
                             "verdict": r["verdict"], "violation": bool(bad)})
                print("  sphsph d=%-4g %-6s exact %8.4f got %s [%s] %s"
                      % (d, op, exact[op],
                         "None" if got is None else "%.4f" % got, r["verdict"],
                         "VIOLATION" if bad else "ok"), flush=True)
                if bad:
                    viol.append(rows[-1])
    # ---- 2. axis-aligned box x box, exact closed form
    if not args.only or "box" in args.only.split(","):
        for dx, dy, dz in ((1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (1.5, 0.75, 0.5),
                           (3.0, 0.0, 0.0)):
            cid = "anal_boxbox_%g_%g_%g" % (dx, dy, dz)
            wd = os.path.join(CELLS, cid)
            os.makedirs(wd, exist_ok=True)
            bf = place(idx["box"]["file"], os.path.join(wd, "B_shift.step"),
                       [0, 0, 1], 0.0, (dx, dy, dz))
            ov = 1.0
            for d in (dx, dy, dz):
                ov *= max(0.0, 4.0 - abs(d))
            exact = {"common": ov, "cut": 64.0 - ov, "fuse": 128.0 - ov}
            for op in OPS:
                t = {"vol": exact[op], "volA": 64.0, "volB": 64.0, "solids": 1}
                r = run_cell(cid, idx["box"]["file"], bf, op, t, args.timeout)
                got = r.get("our_vol")
                bad = got is None or abs(got - exact[op]) > max(1e-6, 1e-4 * 64.0)
                rows.append({"check": "boxbox", "offset": [dx, dy, dz], "op": op,
                             "exact": exact[op], "our": got,
                             "verdict": r["verdict"], "violation": bool(bad)})
                print("  boxbox %-14s %-6s exact %8.4f got %s [%s] %s"
                      % ([dx, dy, dz], op, exact[op],
                         "None" if got is None else "%.4f" % got, r["verdict"],
                         "VIOLATION" if bad else "ok"), flush=True)
                if bad:
                    viol.append(rows[-1])
    print("\nanalytic violations: %d / %d" % (len(viol), len(rows)))
    save_ledger("analytic", {"rows": rows, "violations": len(viol)})
    return rows, viol


# ------------------------------------------------------------------------ main

def main():
    ap = argparse.ArgumentParser(description="rotated-primitive corpus tier (T0R)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        p.add_argument("--jobs", type=int, default=6)
        p.add_argument("--seed", type=int, default=20260725)
        p.add_argument("--rotations", type=int, default=20)
        p.add_argument("--timeout", type=int, default=CELL_TIMEOUT)
        p.add_argument("--only", default="", help="comma list of operand names")
        p.add_argument("--tickets", action="store_true", default=True)
        p.add_argument("--no-tickets", dest="tickets", action="store_false")

    p = sub.add_parser("operands"); p.set_defaults(fn=write_operands)
    p = sub.add_parser("run"); common(p); p.set_defaults(fn=cmd_run)
    p = sub.add_parser("sweep"); common(p); p.set_defaults(fn=cmd_sweep)
    p.add_argument("--pair", default="box,box")
    p.add_argument("--axis", default="0,0,1")
    p.add_argument("--start", type=float, default=0.0)
    p.add_argument("--end", type=float, default=90.0)
    p.add_argument("--step", type=float, default=3.0)
    p.add_argument("--ops", default="cut,common,fuse")
    p.add_argument("--jump", type=float, default=0.5,
                   help="absolute volume jump that counts as a discontinuity")
    p = sub.add_parser("sweepchairs"); common(p); p.set_defaults(fn=cmd_sweepchairs)
    p.add_argument("--axis", default="0,0,1")
    p.add_argument("--start", type=float, default=0.0)
    p.add_argument("--end", type=float, default=45.0)
    p.add_argument("--step", type=float, default=1.5)
    p.add_argument("--ops", default="cut")
    p.add_argument("--jump", type=float, default=1.0)
    p.add_argument("--no-oracle", action="store_true",
                   help="skip the per-angle OCCT boolean (minutes per chairs cell); "
                        "continuity and partition identity need no oracle")
    p = sub.add_parser("equivariance"); common(p); p.set_defaults(fn=cmd_equivariance)
    p = sub.add_parser("selfop"); common(p); p.set_defaults(fn=cmd_selfop)
    p = sub.add_parser("analytic"); common(p); p.set_defaults(fn=cmd_analytic)
    args = ap.parse_args()
    if not os.path.exists(os.path.join(runner.WORK, "main_7_snapshot")):
        runner.prep_work(runner.KERNEL)
    args.fn(args)


if __name__ == "__main__":
    main()
