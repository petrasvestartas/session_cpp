"""Metamorphic invariant engine over T0 — the PRIMARY acceptance oracle.

Raw OCCT-matching is not sufficient: the oracle itself is wrong on some cells (chairs
z15 fuse, see validation/OCCT_TRUTH.md). These invariants are properties of the boolean
algebra itself, so they hold no matter what any oracle says:

  partition     vol(A cut B) + vol(A common B) = vol(A)
                vol(A fuse B) = vol(A) + vol(B) - vol(A common B)
                (computed from OUR OWN results; needs no external truth)
  equivariance  rigid motion M applied to BOTH operands leaves every measured quantity
                (volume, solids, faces) unchanged: results(M A, M B) == results(A, B)
  idempotence   A cut A = empty, A common A = A, A fuse A = A

Every violation becomes a ticket in corpus/tickets/ (actionable for the kernel session:
config, op, verdict, minimal repro).

Costs: partition is free (it reads a ledger); equivariance and idempotence each run the
three base chair booleans (~8 min a batch, 3 ops in parallel).

Usage:
  python corpus/invariants.py partition                  # newest FULL T0 ledger
  python corpus/invariants.py partition --ledger <path>
  python corpus/invariants.py equivariance --motions 3   # chairs base under rigid motions
  python corpus/invariants.py idempotence                # A op A on the real chair BRep
  python corpus/invariants.py all [--append]             # --append keeps prior sections
"""
import argparse
import concurrent.futures
import datetime
import glob
import json
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import runner                                    # noqa: E402
import step_rigid                                # noqa: E402

REPORT = os.path.join(HERE, "invariants_report.md")
# tolerances: relative to vol(A). The operand round-trip through the STEP rewriter
# costs ~5e-7 relative, the kernel's own trim resampling ~1e-3 on freeform cells.
PART_TOL = 0.01          # 1% of vol(A) — a partition break is a topology defect
EQUI_TOL = 0.005         # 0.5% — same geometry, only re-oriented
IDEM_TOL = 0.005

MOTIONS = [                     # deterministic "random" rigid motions (axis, deg, trans)
    ((0.3, 0.5, 0.81), 37.0, (5.0, -3.0, 2.0)),
    ((-0.7, 0.2, 0.68), 113.0, (-12.0, 7.0, 4.5)),
    ((0.577, 0.577, 0.577), 240.0, (0.0, 0.0, 0.0)),
    ((1.0, 0.0, 0.0), 90.0, (3.0, 3.0, 3.0)),
    ((0.12, -0.93, 0.35), 61.0, (-2.0, 8.0, -6.0)),
]


def newest_ledger():
    """Newest ledger with the MOST cells: single-cell debug runs are newer than the
    nightly full run and would silently empty the partition table."""
    ls = sorted(glob.glob(os.path.join(runner.LEDGER_DIR, "ledger_*.json")))
    if not ls:
        sys.exit("no ledger in %s -- run: python corpus/runner.py run" % runner.LEDGER_DIR)
    best, best_key = None, None
    for p in ls:
        try:
            n = len(json.load(open(p))["cells"])
        except Exception:
            continue
        key = (n, os.path.getmtime(p))
        if best_key is None or key > best_key:
            best, best_key = p, key
    return best or ls[-1]


def ticket(cell, rec, note):
    return runner.write_ticket(cell, rec, "n/a (invariant)", {}, note=note,
                               kind="invariant violation")


# ----------------------------------------------------------------- partition

def partition(args, L):
    """vol(cut)+vol(common)=vol(A) and vol(fuse)=vol(A)+vol(B)-vol(common) over OUR
    results, per config. Operand volumes: chairs from the manifest, primitives derived
    from the (trusted, analytic) OCCT truth of the same config."""
    lpath = args.ledger or newest_ledger()
    led = json.load(open(lpath))
    man = json.load(open(runner.MANIFEST))
    groups = {}
    for cell, rec in led["cells"].items():
        t = rec["truth"]
        key = ("chairs_base" if t["battery"] == "chairs_base"
               else "chairs_rot/%s" % t["cfg"] if t["battery"] == "chairs_rot"
               else "%s/%s" % (t["battery"], t["label"].strip()))
        groups.setdefault(key, {})[t["op"]] = rec
    L.append("## partition identity (ledger %s)" % os.path.basename(lpath))
    L.append("")
    L.append("| config | vol(A) | our cut | our common | our fuse | cut+com-A | "
             "fuse-(A+B-com) | verdict |")
    L.append("|---|---|---|---|---|---|---|---|")
    viol = []
    for key in sorted(groups):
        g = groups[key]
        if set(g) != {"cut", "common", "fuse"}:
            continue
        if any(g[op]["verdict"] in ("crash", "timeout") for op in g):
            L.append("| %s | - | - | - | - | - | - | SKIPPED (crash/timeout) |" % key)
            continue
        if key.startswith("chairs"):
            vA, vB = man["volA"], man["volB"]
        else:                       # analytic primitives: OCCT truth is exact here
            tc, tk, tf = (g["cut"]["truth"]["vol"], g["common"]["truth"]["vol"],
                          g["fuse"]["truth"]["vol"])
            vA, vB = tc + tk, tf - tc
        our = {op: g[op].get("our_vol") for op in ("cut", "common", "fuse")}
        if any(v is None for v in our.values()):
            L.append("| %s | %.4f | - | - | - | - | - | SKIPPED (no volume) |" % (key, vA))
            continue
        r1 = our["cut"] + our["common"] - vA
        r2 = our["fuse"] - (vA + vB - our["common"])
        tol = PART_TOL * max(abs(vA), 1e-9)
        bad = abs(r1) > tol or abs(r2) > tol
        L.append("| %s | %.4f | %.4f | %.4f | %.4f | %+.4f | %+.4f | %s |" %
                 (key, vA, our["cut"], our["common"], our["fuse"], r1, r2,
                  "**VIOLATION**" if bad else "ok"))
        if bad:
            rep = "cut" if abs(r1) >= abs(r2) else "fuse"   # representative cell
            rec = dict(g[rep])
            note = ("Representative cell: %s (largest residual). Partition identity "
                    "broken on our own results: cut+common-A = %+.4f, "
                    "fuse-(A+B-common) = %+.4f (tol %.4f). vol(A)=%.4f vol(B)=%.4f; "
                    "our cut=%.4f common=%.4f fuse=%.4f. This is oracle-independent — "
                    "at least one of the three ops on this config is wrong regardless "
                    "of what OCCT says."
                    % (rep, r1, r2, tol, vA, vB, our["cut"], our["common"],
                       our["fuse"]))
            neg = [op for op in our if our[op] < 0]
            if neg:
                note += (" NOTE: %s volume is NEGATIVE — an inside-out shell "
                         "(orientation/classification inversion), not a sewing gap; "
                         "chase face orientation before naked-edge welding."
                         % "/".join(neg))
            viol.append((key, ticket("invariant/partition/%s" % key, rec, note), note))
    L.append("")
    L.append("partition violations: %d" % len(viol))
    return viol


# -------------------------------------------------------------- equivariance

def measure_operands(wd):
    a = runner.probe_summary(os.path.join(wd, "chair0.stp"))
    b = runner.probe_summary(os.path.join(wd, "chair1.stp"))
    return {"volA": a["volume"], "volB": b["volume"]}


def run_base_ops(exe, wd, man, flags, expected):
    """Run the three base chair ops in wd (in parallel; they write distinct files);
    returns {op: (verdict, rec)}."""
    def one(op):
        t = {"battery": "chairs_base", "op": op, "vol": expected[op], "chairs_dir": wd,
             "solids": 1, "occt_valid": 1, "gate_solids": False, "vol_derived": False}
        v, x = runner.run_chairs_base(exe, wd, t, flags, man)
        return op, (v, dict(x, verdict=v, truth=t))

    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as ex:
        return dict(ex.map(one, runner.OPS))


def equivariance(args, L):
    exe = os.path.join(runner.WORK, "main_7_snapshot")
    if not os.path.exists(exe):
        shutil.copyfile(args.exe, exe)
        os.chmod(exe, 0o755)
    ref_dir = os.path.join(runner.WORK, "equi_ref")
    os.makedirs(ref_dir, exist_ok=True)
    for f in ("chair0.stp", "chair1.stp"):
        shutil.copyfile(os.path.join(runner.CHAIRS, f), os.path.join(ref_dir, f))
    man0 = measure_operands(ref_dir)
    # Reference truth = OCCT on the untransformed pair. Reuse the T0 manifest when it
    # has the base cells (each OCCT chair boolean costs minutes).
    ref_truth = None
    if os.path.exists(runner.MANIFEST):
        cells = json.load(open(runner.MANIFEST))["cells"]
        if all("chairs_base/%s" % op in cells for op in runner.OPS):
            ref_truth = {op: cells["chairs_base/%s" % op]["vol"] for op in runner.OPS}
    if ref_truth is None:
        ref_truth = {op: runner.probe_bool(op, os.path.join(ref_dir, "chair0.stp"),
                                           os.path.join(ref_dir, "chair1.stp"))["volume"]
                     for op in runner.OPS}
    ref = run_base_ops(exe, ref_dir, man0, {}, ref_truth)
    L.append("## rigid-motion equivariance (chairs base A op B)")
    L.append("")
    L.append("reference (identity): " + ", ".join(
        "%s %.4f [%s]" % (op, ref[op][1].get("our_vol", float("nan")), ref[op][0])
        for op in runner.OPS))
    L.append("")
    L.append("| motion | op | our vol | ref vol | rel diff | our solids/faces | verdict |")
    L.append("|---|---|---|---|---|---|---|")
    viol = []
    for i, (axis, deg, trans) in enumerate(MOTIONS[:args.motions]):
        wd = os.path.join(runner.WORK, "equi_m%d" % i)
        os.makedirs(wd, exist_ok=True)
        for f in ("chair0.stp", "chair1.stp"):
            step_rigid.transform_file(os.path.join(runner.CHAIRS, f),
                                      os.path.join(wd, f), axis, deg, trans)
        man = measure_operands(wd)
        got = run_base_ops(exe, wd, man, {}, ref_truth)
        lab = "axis(%.2f,%.2f,%.2f) %.0fdeg t(%.0f,%.0f,%.0f)" % (
            axis + (deg,) + trans)
        for op in runner.OPS:
            v, rec = got[op]
            rv = ref[op][1].get("our_vol")
            ov = rec.get("our_vol")
            if rv is None or ov is None:
                rel = float("nan")
            else:
                rel = abs(ov - rv) / max(abs(rv), 1e-9)
            bad = (v != ref[op][0]) or not (rel <= EQUI_TOL)
            L.append("| %s | %s | %.4f | %.4f | %.2e | %s/%s | %s |" %
                     (lab, op, ov or 0.0, rv or 0.0, rel,
                      rec.get("our_solids"), rec.get("kfaces"),
                      "**VIOLATION**" if bad else "ok"))
            if bad:
                note = ("Rigid-motion equivariance broken: the SAME boolean under a "
                        "whole-scene rigid motion %s gives volume %.4f vs %.4f "
                        "unmoved (rel %.2e, tol %.1e); verdict %s vs %s. Operands were "
                        "transformed with corpus/step_rigid.py (rewrites 3D "
                        "CARTESIAN_POINT/DIRECTION only; operand volume drift is "
                        "~5e-7). Transformed inputs kept at %s."
                        % (lab, ov or 0.0, rv or 0.0, rel, EQUI_TOL, v, ref[op][0], wd))
                viol.append(("equi_m%d/%s" % (i, op),
                             ticket("invariant/equivariance/m%d/%s" % (i, op), rec, note),
                             note))
    L.append("")
    L.append("equivariance violations: %d" % len(viol))
    return viol


# ---------------------------------------------------------------- idempotence

def idempotence(args, L):
    exe = os.path.join(runner.WORK, "main_7_snapshot")
    if not os.path.exists(exe):
        shutil.copyfile(args.exe, exe)
        os.chmod(exe, 0o755)
    wd = os.path.join(runner.WORK, "idem")
    os.makedirs(wd, exist_ok=True)
    src = os.path.join(runner.CHAIRS, "chair0.stp")
    shutil.copyfile(src, os.path.join(wd, "chair0.stp"))
    shutil.copyfile(src, os.path.join(wd, "chair1.stp"))     # B := A
    man = measure_operands(wd)
    vA = man["volA"]
    expected = {"cut": 0.0, "common": vA, "fuse": vA}
    got = run_base_ops(exe, wd, man, {}, expected)
    L.append("## A-op-A idempotence (B := chair0.stp)")
    L.append("")
    L.append("vol(A) = %.6f" % vA)
    L.append("")
    L.append("| op | expected vol/faces | our vol | our faces | our solids | verdict |")
    L.append("|---|---|---|---|---|---|")
    viol = []
    # Face expectations: A cut A is EMPTY (no faces at all); A common A and A fuse A are
    # A itself (same face count). A zero-volume answer is NOT enough -- an open shell
    # also measures 0, so the topology has to be checked too.
    exp_faces = {"cut": 0, "common": None, "fuse": None}
    nfA = runner.probe_summary(os.path.join(wd, "chair0.stp"))["faces"]
    for op in runner.OPS:
        v, rec = got[op]
        ov = rec.get("our_vol") or 0.0
        kf = rec.get("kfaces")
        exp = expected[op]
        tol = IDEM_TOL * vA
        bad = abs(ov - exp) > tol
        want_f = exp_faces[op] if exp_faces[op] is not None else nfA
        if kf is not None and kf != want_f:
            bad = True
        L.append("| %s | %.4f / %d | %.4f | %s | %s | %s |" %
                 (op, exp, want_f, ov, kf, rec.get("our_solids"),
                  "**VIOLATION**" if bad else "ok"))
        if bad:
            note = ("A-op-A idempotence broken: with B an exact copy of A, %s must give "
                    "vol %.4f with %d faces but gave vol %.4f with %s faces (vol tol "
                    "%.4f). A zero volume alone does not clear it: an open shell also "
                    "measures zero. Repro dir %s holds chair1.stp = chair0.stp; this is "
                    "the same-domain (P3) regime — every face pair is exactly "
                    "coincident." % (op, exp, want_f, ov, kf, tol, wd))
            viol.append(("idem/%s" % op,
                         ticket("invariant/idempotence/%s" % op, rec, note), note))
    L.append("")
    L.append("idempotence violations: %d" % len(viol))
    return viol


# ----------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description="metamorphic invariant engine over T0")
    ap.add_argument("what", choices=["partition", "equivariance", "idempotence", "all"])
    ap.add_argument("--ledger", default="")
    ap.add_argument("--exe", default=runner.KERNEL)
    ap.add_argument("--motions", type=int, default=2)
    ap.add_argument("--report", default=REPORT)
    ap.add_argument("--append", action="store_true",
                    help="keep the existing report and append these sections")
    args = ap.parse_args()
    L = ["# Metamorphic invariants over T0",
         "",
         "generated %s" % datetime.datetime.now().isoformat(timespec="seconds"),
         "",
         "These identities are oracle-independent: a violation is a kernel defect even "
         "where OCCT itself is wrong (see validation/OCCT_TRUTH.md, chairs z15 fuse).",
         ""]
    viol = []
    if args.what in ("partition", "all"):
        viol += partition(args, L)
        L.append("")
    if args.what in ("equivariance", "all"):
        viol += equivariance(args, L)
        L.append("")
    if args.what in ("idempotence", "all"):
        viol += idempotence(args, L)
        L.append("")
    L.append("## tickets")
    L.append("")
    if viol:
        for key, path, _n in viol:
            L.append("- `%s` -> `%s`" % (key, os.path.relpath(path, HERE)))
    else:
        L.append("no violations")
    text = "\n".join(L) + "\n"
    if args.append and os.path.exists(args.report):
        text = open(args.report).read().rstrip("\n") + "\n\n---\n\n" + text
    open(args.report, "w").write(text)
    print(text)
    print("report: %s" % args.report, file=sys.stderr)


if __name__ == "__main__":
    main()
