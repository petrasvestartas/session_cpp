"""T0 corpus runner: chairs (base + 10 rotated configs) + matrix + edge batteries as
STEP pairs with recorded OCCT truth, five-verdict nightly diff loop.

Verdicts (rank order, higher is worse):
  exact        result closed + OCCT-valid + volume within tol of truth (and solid count
               where the oracle is itself trusted on that cell)
  closed-wrong closed but wrong volume / solid count (or non-empty where truth is empty)
  open         not a closed valid solid (naked edges / unsewn shells / BRepCheck invalid)
  timeout      kernel exceeded the per-cell cap
  crash        kernel THREW or exited non-zero

Truth sources:
  chairs rot   validation/occt_truth.json — identity-corrected re-derivation (task #10);
               cells where OCCT contradicts itself carry the identity-DERIVED volume and
               are excluded from solid-count/OP_VALID gating
  chairs base  live step_probe --op chair0.stp chair1.stp at manifest build
  matrix/edge  validation/occt_cache.txt (the kernel's own OCCT reference cache)

Usage (paths are absolute; run from anywhere):
  python corpus/runner.py truth                  # (re)build corpus/t0_manifest.json
  python corpus/runner.py run                    # run T0, write ledger, diff baseline
  python corpus/runner.py run --cells chairs_rot # substring cell filter
  python corpus/runner.py run --set-baseline     # accept this run as the baseline
  python corpus/runner.py run --flags "SESSION_AUTO=1 SESSION_M3=1" --jobs 8
"""
import argparse
import concurrent.futures
import datetime
import glob
import hashlib
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
CPP = os.path.dirname(HERE)
ROOT = os.path.dirname(CPP)
PROBE = os.path.join(ROOT, "validation", "step_probe", "build", "step_probe")
KERNEL = os.path.join(CPP, "build", "main_7")
MAIN7_SRC = os.path.join(CPP, "main_7.cpp")
CHAIRS = os.path.join(CPP, "serialization", "boolean_steps", "chairs")
TRUTH_JSON = os.path.join(ROOT, "validation", "occt_truth.json")
CACHE = os.path.join(ROOT, "validation", "occt_cache.txt")
MANIFEST = os.path.join(HERE, "t0_manifest.json")
BASELINE = os.path.join(HERE, "baseline.json")
BASELINE_META = os.path.join(HERE, "baseline_meta.json")
LEDGER_DIR = os.path.join(HERE, "ledger")
TICKETS = os.path.join(HERE, "tickets")
WORK = os.path.join(HERE, "work")

CFGS = ["z15", "z30", "z45", "z90", "x20", "y30", "z30x20", "z37", "x13y29", "z63"]
OPS = ["cut", "common", "fuse"]
VERDICTS = ["exact", "closed-wrong", "open", "timeout", "crash"]
RANK = {v: i for i, v in enumerate(VERDICTS)}
VOLTOL = 0.01          # 1% relative volume tolerance vs truth
EMPTY = 0.01           # |vol| below this counts as an empty result
CELL_TIMEOUT = 600     # hard per-cell cap (10 min)
RSS_CAP_MB = 0         # 0 = uncapped; set via --rss-cap-mb (needs prlimit)


# --------------------------------------------------------------------- probing

def probe_out(args, timeout=CELL_TIMEOUT):
    return subprocess.run([PROBE] + args, capture_output=True, text=True,
                          timeout=timeout).stdout


def probe_summary(path):
    out = probe_out([path])
    d = {}
    for k in ("SOLIDS", "SHELLS", "FACES", "VOLUME", "VALID"):
        m = re.search(r"^%s ([-\d.]+)" % k, out, re.M)
        if not m:
            return None
        d[k.lower()] = float(m.group(1)) if k == "VOLUME" else int(m.group(1))
    return d


def probe_topo(path):
    """step_probe -n edge typing: naked/nonmanifold/open shells of the exported artifact.
    DEGEN excludes OCCT-typed degenerate pole edges, matching the kernel's verdict()."""
    out = probe_out([path, "-n"])
    d = {}
    for k in ("EDGES", "NAKED", "SEAM", "SHARED", "DEGEN", "NONMANIFOLD",
              "SHELLS_CLOSED", "SHELLS_OPEN"):
        m = re.search(r"^%s (\d+)" % k, out, re.M)
        d[k.lower()] = int(m.group(1)) if m else None
    return d


def probe_bool(op, a, b):
    out = probe_out(["--" + op, a, b])
    d = {}
    for k in ("OP_SOLIDS", "OP_FACES", "OP_VOLUME", "OP_VALID"):
        m = re.search(r"^%s ([-\d.]+)" % k, out, re.M)
        if not m:
            raise RuntimeError("oracle gave no %s for --%s" % (k, op))
        d[k[3:].lower()] = float(m.group(1)) if "VOL" in k else int(m.group(1))
    return d


UNIT = re.compile(r"MANIFOLD_SOLID_BREP\s*\(|SHELL_BASED_SURFACE_MODEL\s*\(")


def measure_result(path, volA, volB):
    """OCCT measurement of OUR result in `path`. The kernel exports inspection files
    that may hold A + B + result (operands in place, result in green) or the result
    alone -- decide from the STEP's shape-unit count, never from a fixed assumption:
    session A changed the rot export from 1 unit to 3 mid-campaign."""
    s = probe_summary(path)
    if s is None:
        return None
    units = len(UNIT.findall(open(path, errors="replace").read()))
    if units >= 3:
        return {"vol": s["volume"] - volA - volB, "solids": max(0, s["solids"] - 2),
                "valid": s["valid"], "units": units}
    return {"vol": s["volume"], "solids": s["solids"], "valid": s["valid"],
            "units": units}


def run_kernel(exe, args, env_extra, timeout=None):
    """Returns (rc, out, secs, timed_out, meta). Hermetic: ambient SESSION_* would
    silently alter every cell while only --flags is ledgered, so the inherited env is
    stripped before the cell's own vars go on. The kernel runs in its own session so a
    timeout kills the whole process tree (a bare child-kill leaves grandchildren
    running into later cells); wait4 harvests per-cell peak RSS."""
    if timeout is None:      # bind at CALL time: a def-time default freezes the
        timeout = CELL_TIMEOUT   # module constant and silently ignores --timeout
    env = {k: v for k, v in os.environ.items() if not k.startswith("SESSION_")}
    env.update(env_extra)
    cmd = [exe] + args
    if RSS_CAP_MB:
        cmd = ["prlimit", "--as=%d" % (RSS_CAP_MB << 20), "--"] + cmd
    t0 = time.time()
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, cwd=CPP, env=env, start_new_session=True)
    timed_out = [False]

    def _kill():
        timed_out[0] = True
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    tm = threading.Timer(timeout, _kill)
    tm.start()
    try:
        out = p.stdout.read()
    finally:
        tm.cancel()
    _, status, ru = os.wait4(p.pid, 0)
    p.returncode = rc = (-os.WTERMSIG(status) if os.WIFSIGNALED(status)
                         else os.WEXITSTATUS(status))
    meta = {"maxrss_kb": ru.ru_maxrss}
    if rc < 0:
        meta["signal"] = -rc
    return rc, out, time.time() - t0, timed_out[0], meta


# ------------------------------------------------------------------ cell lists

def source_pairs(fn):
    """[(label, keyA, keyB)] parsed from main_7.cpp's pairs()/edge_pairs()."""
    src = open(MAIN7_SRC, errors="replace").read()
    m = re.search(r"std::vector<std::array<std::string,3>> %s\(\) \{(.*?)\n\}" % fn,
                  src, re.S)
    if not m:
        sys.exit("cannot parse %s() from %s" % (fn, MAIN7_SRC))
    return re.findall(r'\{"([^"]*)",\s*"([^"]*)",\s*"([^"]*)"\}', m.group(1))


def load_cache():
    truth = {}
    for ln in open(CACHE):
        if ln.startswith("#") or "|" not in ln:
            continue
        parts = ln.rstrip("\n").split("\t")
        if len(parts) != 3:
            continue
        truth[parts[0]] = {"vol": float(parts[1]), "faces": int(parts[2])}
    return truth


# ----------------------------------------------------------------- truth build

def build_truth(args):
    if not os.path.exists(TRUTH_JSON):
        sys.exit("missing %s -- run: python validation/rederive_truth.py" % TRUTH_JSON)
    rot = json.load(open(TRUTH_JSON))
    a = os.path.join(CHAIRS, "chair0.stp")
    b = os.path.join(CHAIRS, "chair1.stp")
    vA, vB = rot["volA"], rot["volB"]
    cells = {}

    # chairs base: live oracle + its own identity check
    base = {op: probe_bool(op, a, b) for op in OPS}
    ietol = 0.005 * abs(vA)
    c_cut = vA - base["cut"]["volume"]
    c_fuse = vA + vB - base["fuse"]["volume"]
    base_ok = (abs(c_cut - base["common"]["volume"]) <= ietol
               and abs(c_fuse - base["common"]["volume"]) <= ietol)
    for op in OPS:
        r = base[op]
        cells["chairs_base/%s" % op] = {
            "battery": "chairs_base", "op": op, "vol": r["volume"],
            "solids": r["solids"], "occt_valid": r["valid"],
            "gate_solids": bool(base_ok and r["valid"] == 1
                                and abs(r["volume"]) >= EMPTY),
            "vol_derived": False}

    # chairs rot: identity-corrected truth; OCCT's own bad cells are gated softly
    for cfg in CFGS:
        c = rot["configs"][cfg]
        for op in OPS:
            r = c["ops"][op]
            vol, derived = r["volume"], False
            if c["odd_cell"] == op:
                vol, derived = c["derived"][op], True
            cells["chairs_rot/%s/%s" % (cfg, op)] = {
                "battery": "chairs_rot", "cfg": cfg, "op": op, "vol": vol,
                "solids": r["solids"], "occt_valid": r["valid"],
                "gate_solids": bool(not derived and r["valid"] == 1
                                    and op not in c["degenerate_ops"]),
                "vol_derived": derived}

    # matrix + edge: cell list from source, truth from the kernel's OCCT cache.
    # A pair whose operand key ends in 'R' is part of the ORIENTED battery that
    # SESSION_NO_ROT skips ("their marcher cells can hang"). Those 18 cells were
    # therefore absent from every headline number for the whole campaign. They get
    # their own battery here and are RUN (SESSION_NO_ROT is never set for them):
    # a hang is recorded as `timeout`, which is a verdict, not an excuse to hide it.
    cache = load_cache()
    for battery, fn in (("matrix", "pairs"), ("edge", "edge_pairs")):
        for label, ka, kb in source_pairs(fn):
            rot = ka.endswith("R") or kb.endswith("R")
            bat = "matrix_rot" if rot else battery
            for op in OPS:
                key = "%s|%s" % (label, op)
                if key not in cache:
                    continue
                name = "%s/%s/%s" % (bat, label.strip().replace(" ", "_"), op)
                cells[name] = {"battery": bat, "label": label, "op": op,
                               "vol": cache[key]["vol"], "faces": cache[key]["faces"]}
    man = {"generated": datetime.datetime.now().isoformat(timespec="seconds"),
           "volA": vA, "volB": vB, "probe": PROBE, "cells": cells}
    json.dump(man, open(MANIFEST, "w"), indent=1)
    n = {}
    for c in cells.values():
        n[c["battery"]] = n.get(c["battery"], 0) + 1
    print("manifest %s: %d cells (%s)" %
          (MANIFEST, len(cells), ", ".join("%s %d" % kv for kv in sorted(n.items()))))


# ------------------------------------------------------------------- cell runs

def chairs_verdict(meas, t, extra):
    """meas = OCCT measurement of OUR result (None when nothing was exported)."""
    tvol = t["vol"]
    if meas is None:
        if abs(tvol) < EMPTY:
            # Empty-and-nothing-exported grades "exact" only with kernel-row evidence:
            # a kernel that exits 0 having printed nothing must not grade as correct.
            if "kvol" in extra:
                return "exact", dict(extra, our_vol=0.0)
            return "crash", dict(extra, our_vol=0.0, cause="no-evidence")
        return "open", dict(extra, our_vol=0.0)
    d = dict(extra, our_vol=meas["vol"], our_solids=meas["solids"],
             our_valid=meas["valid"])
    for k in ("naked", "nonmanifold", "shells_open"):
        if meas.get(k) is not None:
            d[k] = meas[k]
    # Kernel-self-report vs exported-artifact drift, recorded (not yet gated): in-memory
    # 45/45 vs exported 23/45 was a whole campaign's blind spot.
    if "kvol" in extra:
        d["self_report_rel"] = abs(extra["kvol"] - meas["vol"]) / max(abs(tvol), EMPTY)
    if extra.get("knaked") == 0 and (meas.get("naked") or 0) > 0:
        d["kernel_export_disagree"] = True
    # A result with naked/non-manifold edges or open shells is open even when it volumes
    # correctly and BRepCheck's VALID bit passes (rotprim has gated on this all along).
    topo_open = any((meas.get(k) or 0) > 0
                    for k in ("naked", "nonmanifold", "shells_open"))
    if abs(tvol) < EMPTY:
        return ("exact" if abs(meas["vol"]) < EMPTY and not topo_open
                else "closed-wrong"), d
    d["rel"] = abs(meas["vol"] - tvol) / abs(tvol)
    if meas["valid"] == 0 or meas["solids"] == 0 or topo_open:
        return "open", d
    if d["rel"] > VOLTOL:
        return "closed-wrong", d
    if t.get("gate_solids") and meas["solids"] != t["solids"]:
        return "closed-wrong", d
    return "exact", d


def run_chairs_base(exe, wd, t, flags, man):
    op = t["op"]
    out_step = os.path.join(wd, "chair0_%s_chair1.step" % op)
    if os.path.exists(out_step):
        os.remove(out_step)
    rc, out, dt, to, km = run_kernel(exe, ["zzzz"],
                                     dict(flags, SESSION_CHAIRS=wd, SESSION_OP=op))
    extra = dict(km, secs=dt, rc=rc)
    if to:
        return "timeout", extra
    threw = re.search(r"^chairs %s THREW" % op, out, re.M)
    if rc != 0 or threw:
        return "crash", dict(extra, tail=out[-400:],
                             cause="threw" if threw else "exit")
    m = re.search(r"^chairs\s+%s\s*: faces (\d+) solid (\d+)(?: shells (\d+))?"
                  r" naked (\d+) vol ([-\d.]+)" % op, out, re.M)
    if m:
        extra.update(kfaces=int(m.group(1)), ksolid=int(m.group(2)),
                     knaked=int(m.group(4)), kvol=float(m.group(5)))
    meas = None
    if os.path.exists(out_step):
        meas = measure_result(out_step, man["volA"], man["volB"])
        # 2-unit export = operands only; with a 0-face kernel row the result is EMPTY
        # (reporting the file total minted phantom 'closed-wrong' verdicts).
        if meas is not None and meas.get("units") == 2 and extra.get("kfaces") == 0:
            meas = {"vol": 0.0, "solids": 0, "valid": meas["valid"], "units": 2}
        if meas is not None:
            meas.update(probe_topo(out_step))
    return chairs_verdict(meas, t, extra)


def run_chairs_rot(exe, wd, t, flags, man):
    cfg, op = t["cfg"], t["op"]
    res = os.path.join(wd, "rot", "res_%s_%s.step" % (cfg, op))
    if os.path.exists(res):
        os.remove(res)
    env = dict(flags, SESSION_CHAIRS=wd, SESSION_CHAIRS_ROT="1",
               SESSION_ROT_ONLY=cfg, SESSION_OP=op, SESSION_ROT_STEP="1",
               SESSION_FAST="1")
    rc, out, dt, to, km = run_kernel(exe, ["zzzz"], env)
    extra = dict(km, secs=dt, rc=rc)
    if to:
        return "timeout", extra
    threw = re.search(r"^chairsROT\s+%s\s+%s\s*: THREW" % (cfg, op), out, re.M)
    if rc != 0 or threw:
        return "crash", dict(extra, tail=out[-400:],
                             cause="threw" if threw else "exit")
    m = re.search(r"^chairsROT\s+%s\s+%s\s*: faces (\d+) solid (\d+)(?: shells (\d+))?"
                  r" naked (\d+)" % (cfg, op), out, re.M)
    if m:
        extra.update(kfaces=int(m.group(1)), ksolid=int(m.group(2)),
                     knaked=int(m.group(4)))
    meas = None
    if os.path.exists(res):
        meas = measure_result(res, man["volA"], man["volB"])
        if meas is not None and meas.get("units") == 2 and extra.get("kfaces") == 0:
            meas = {"vol": 0.0, "solids": 0, "valid": meas["valid"], "units": 2}
        if meas is not None:
            meas.update(probe_topo(res))
    return chairs_verdict(meas, t, extra)


ROW = re.compile(r"^(?P<lab>.+?)\s+(?P<op>cut|common|fuse)\s+\|(?P<body>.*)$", re.M)


def run_battery_cell(exe, t, flags):
    label, op = t["label"], t["op"]
    # Per-cell export dir: the kernel row alone graded 117/150 cells with no external
    # oracle on any artifact; SESSION_STEP_DIR makes matrix/edge cells ship their
    # A/B/result STEP so the exported artifact gets probed too. matrix_rot is EXCLUDED
    # for now: exporting rotated marched results is 17-50x (tor 19.5s->330s, coneRx_cyl
    # 12s->over the 600s cap) -- that battery keeps kernel-row-only verdicts until the
    # writer's section-curve compression is off the critical path (measured 2026-08-14).
    cell_dir = None
    env = dict(flags, SESSION_OP=op)
    if t["battery"] != "matrix_rot":
        cell_dir = os.path.join(WORK, "battery",
                                re.sub(r"[^A-Za-z0-9_.-]+", "_", label.strip()) + "_" + op)
        shutil.rmtree(cell_dir, ignore_errors=True)
        os.makedirs(cell_dir, exist_ok=True)
        env["SESSION_STEP_DIR"] = cell_dir
    if t["battery"] != "matrix_rot":     # the oriented battery must NOT be skipped;
        env["SESSION_NO_ROT"] = "1"      # its hangs are recorded as `timeout`
    if t["battery"] == "edge":
        env["SESSION_EDGE"] = "1"
    rc, out, dt, to, km = run_kernel(exe, [label.strip()], env)
    ex0 = dict(km, secs=dt, rc=rc)
    if to:
        return "timeout", ex0
    row = None
    for m in ROW.finditer(out):
        if m.group("lab").strip() == label.strip() and m.group("op") == op:
            row = m
            break
    if rc != 0 or row is None:
        return "crash", dict(ex0, tail=out[-400:],
                             cause="exit" if rc != 0 else "no-row")
    body = row.group("body")
    extra = dict(ex0, row=(label.strip() + " " + op + " |" + body).rstrip())
    # External oracle on the EXPORTED artifact (operands are closed solids, so any
    # naked/non-manifold edge or open shell in the combined file belongs to OUR result;
    # solids-2 = result solid count). No volume gate here: the manifest carries no
    # per-operand volumes for battery pairs -- topology/validity is the new signal.
    steps = glob.glob(os.path.join(cell_dir, "*.step")) if cell_dir else []
    ext_open = False
    if len(steps) == 1:
        s = probe_summary(steps[0])
        topo = probe_topo(steps[0])
        if s is not None:
            extra.update(ext_total_vol=s["volume"], ext_valid=s["valid"],
                         ext_solids=max(0, s["solids"] - 2))
        for k in ("naked", "nonmanifold", "shells_open"):
            if topo.get(k) is not None:
                extra["ext_" + k] = topo[k]
        ext_open = (any((topo.get(k) or 0) > 0
                        for k in ("naked", "nonmanifold", "shells_open"))
                    or (s is not None and s["valid"] == 0)
                    or (s is not None and abs(t["vol"]) >= EMPTY
                        and s["solids"] - 2 <= 0))
    if "THREW" in body:
        return "crash", dict(extra, cause="threw")
    m = re.match(r"\s*(?P<v>[-\d.eE+]+)\s+(?P<o>[-\d.eE+]+|-)\s+\S+\s*\|"
                 r"\s*(?P<nf>\d+)\s+(?P<of>\d+|-)\s*\|\s*(?P<s>\d)\s*\|", body)
    if m:                     # volumes are recorded for OK rows too (invariant engine)
        extra.update(our_vol=float(m.group("v")), our_solids=int(m.group("s")),
                     kfaces=int(m.group("nf")))
    if body.rstrip().endswith("OK"):
        return ("open" if ext_open else "exact"), extra
    if not m:
        return "crash", dict(extra, cause="parse")
    our_vol, solid, nf = extra["our_vol"], extra["our_solids"], extra["kfaces"]
    tvol = t["vol"]
    if abs(tvol) < EMPTY:
        return ("exact" if abs(our_vol) < EMPTY and nf == 0 and not ext_open
                else "closed-wrong"), extra
    extra["rel"] = abs(our_vol - tvol) / abs(tvol)
    if solid == 0 or ext_open:
        return "open", extra
    return "closed-wrong", extra


# --------------------------------------------------------------------- tickets

def repro_cmd(t, flags):
    fl = " ".join("%s=%s" % kv for kv in sorted(flags.items()))
    fl = (fl + " ") if fl else ""
    # invariant runs use a purpose-built operand dir (rigid-moved / A-copied operands);
    # the repro must point at THAT dir, not at the shipped chairs.
    ch = t.get("chairs_dir", "serialization/boolean_steps/chairs")
    if t["battery"] == "chairs_base":
        return ("env %sSESSION_CHAIRS=%s SESSION_OP=%s ./build/main_7 zzzz"
                % (fl, ch, t["op"]))
    if t["battery"] == "chairs_rot":
        return ("env %sSESSION_CHAIRS=%s SESSION_CHAIRS_ROT=1 SESSION_ROT_ONLY=%s "
                "SESSION_OP=%s SESSION_ROT_STEP=1 SESSION_FAST=1 ./build/main_7 zzzz"
                % (fl, ch, t["cfg"], t["op"]))
    if "label" not in t:            # t0r and other STEP-pair batteries: chairs entry point
        return ("env %sSESSION_CHAIRS=%s SESSION_OP=%s ./build/main_7 zzzz"
                % (fl, t.get("chairs_dir", "<pair dir>"), t["op"]))
    edge = "SESSION_EDGE=1 " if t["battery"] == "edge" else ""
    # matrix_rot cells are the ORIENTED battery: never pass SESSION_NO_ROT for them.
    norot = "" if t["battery"] == "matrix_rot" else "SESSION_NO_ROT=1 "
    return ("env %s%s%sSESSION_OP=%s ./build/main_7 '%s'"
            % (fl, edge, norot, t["op"], t["label"].strip()))


def write_ticket(cell, rec, base_verdict, flags, note="", kind="regression"):
    os.makedirs(TICKETS, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d-%H%M")
    slug = cell.replace("/", "-").replace(" ", "")
    path = os.path.join(TICKETS, "%s_%s_%s.md" % (ts, slug, rec["verdict"]))
    t = rec.get("truth", {})
    L = ["# %s — %s (%s)" % (cell, rec["verdict"], kind),
         "",
         "| field | value |",
         "|---|---|",
         "| battery | %s |" % t.get("battery"),
         "| config | %s |" % t.get("cfg", t.get("label", "base")),
         "| op | %s |" % t.get("op"),
         "| verdict | **%s** (baseline: %s) |" % (rec["verdict"], base_verdict),
         "| truth vol | %s%s |" % (t.get("vol"),
                                   " **identity-derived — OCCT itself is wrong on this "
                                   "cell**" if t.get("vol_derived") else ""),
         "| truth solids | %s (gated: %s) |" % (t.get("solids"), t.get("gate_solids")),
         "| our vol | %s |" % rec.get("our_vol"),
         "| our solids / valid | %s / %s |" % (rec.get("our_solids"),
                                               rec.get("our_valid")),
         "| rel vol err | %s |" % rec.get("rel"),
         "| kernel faces / naked | %s / %s |" % (rec.get("kfaces"), rec.get("knaked")),
         "| runtime | %.1f s |" % rec.get("secs", 0.0),
         "",
         "## Minimal repro (cwd session_cpp)",
         "```bash",
         repro_cmd(t, flags),
         "```",
         "",
         "Check the exported result:",
         "```bash",
         "%s <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID" % PROBE,
         "%s <result.step> -c     # per-subshape BRepCheck failures" % PROBE,
         "```"]
    if rec.get("row"):
        L += ["", "Battery row: `%s`" % rec["row"]]
    if rec.get("tail"):
        L += ["", "Kernel output tail:", "```", rec["tail"], "```"]
    if note:
        L += ["", note]
    open(path, "w").write("\n".join(L) + "\n")
    return path


# ------------------------------------------------------------------ ledger run

def exe_id(path):
    h = hashlib.sha1()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    st = os.stat(path)
    return {"sha1": h.hexdigest(), "size": st.st_size,
            "mtime": datetime.datetime.fromtimestamp(st.st_mtime).isoformat(
                timespec="seconds")}


def git_state(repo):
    """A binary sha1 alone cannot be mapped back to source (build/main_7 is rebuilt
    continuously); the ledger needs the source state too."""
    def g(*a):
        return subprocess.run(["git", "-C", repo] + list(a),
                              capture_output=True, text=True).stdout
    diff = g("diff", "HEAD")
    return {"commit": g("rev-parse", "HEAD").strip(),
            "dirty": bool(g("status", "--porcelain").strip()),
            "diff_sha1": (hashlib.sha1(diff.encode()).hexdigest()
                          if diff.strip() else None)}


def prep_work(exe, reuse=False, refresh_operands=False):
    """Operands are PINNED on first use: the shipped chairs dir is rewritten live by the
    kernel session (rot results and B_<cfg>.step are regenerated mid-campaign), and the
    recorded truth belongs to one specific set of operand files. --refresh-operands
    re-pins deliberately."""
    wd = os.path.join(WORK, "chairs")
    os.makedirs(os.path.join(wd, "rot"), exist_ok=True)
    want = [("chair0.stp", os.path.join(CHAIRS, "chair0.stp")),
            ("chair1.stp", os.path.join(CHAIRS, "chair1.stp"))]
    want += [(os.path.join("rot", "B_%s.step" % c),
              os.path.join(CHAIRS, "rot", "B_%s.step" % c)) for c in CFGS]
    for rel, src in want:
        dst = os.path.join(wd, rel)
        if os.path.exists(dst) and not refresh_operands:
            continue
        if os.path.exists(src):
            shutil.copyfile(src, dst)
    # Stable snapshot (session A rebuilds build/ under us). Publish it ATOMICALLY:
    # a plain copyfile leaves a 0644, half-written binary visible to a concurrent
    # runner, which then dies with EACCES/ETXTBSY mid-run.
    snap = os.path.join(WORK, "main_7_snapshot")
    if not (reuse and os.path.exists(snap)):
        tmp = snap + ".tmp%d" % os.getpid()
        shutil.copyfile(exe, tmp)
        os.chmod(tmp, 0o755)
        os.replace(tmp, snap)
    return wd, snap


def run_cell(cell, t, exe, wd, flags, man):
    for attempt in (0, 1):                    # one retry: harness hiccups are not
        try:                                  # kernel verdicts
            if t["battery"] == "chairs_base":
                v, x = run_chairs_base(exe, wd, t, flags, man)
            elif t["battery"] == "chairs_rot":
                v, x = run_chairs_rot(exe, wd, t, flags, man)
            else:
                v, x = run_battery_cell(exe, t, flags)
            break
        except Exception as e:
            v, x = "crash", {"secs": 0.0, "cause": "harness",
                             "tail": "runner exception: %r" % e}
            if attempt == 0:
                time.sleep(2.0)
    if (v == "crash" and RSS_CAP_MB
            and x.get("maxrss_kb", 0) >= RSS_CAP_MB * 1024 * 0.95):
        x["oom"] = True
    return cell, dict(x, verdict=v, truth=t)


def run_all(args):
    global CELL_TIMEOUT, RSS_CAP_MB
    CELL_TIMEOUT = args.timeout
    RSS_CAP_MB = args.rss_cap_mb
    if RSS_CAP_MB and not shutil.which("prlimit"):
        print("WARNING: prlimit not found -- --rss-cap-mb ignored")
        RSS_CAP_MB = 0
    if not os.path.exists(MANIFEST):
        sys.exit("no manifest -- run: python corpus/runner.py truth")
    man = json.load(open(MANIFEST))
    cells = {k: v for k, v in man["cells"].items() if not args.cells or args.cells in k}
    if not cells:
        sys.exit("no cells match %r" % args.cells)
    wd, exe = prep_work(args.exe, reuse=args.reuse_exe,
                        refresh_operands=args.refresh_operands)
    # Kernel-binary provenance: session A rebuilds build/main_7 continuously, so two
    # runs minutes apart can measure DIFFERENT kernels. Every ledger records the exact
    # binary; a baseline diff against a different binary is flagged, not silently
    # reported as a regression. --reuse-exe pins the snapshot already in work/.
    # The verdict harness may not score anything before it is validated on the known-
    # degenerate shapes (sphere poles, cone apex, pyramid apex rows) + STEP round trips.
    rc, sout, _dt, _to, _km = run_kernel(exe, ["verdict_selftest"], {}, timeout=120)
    if rc != 0 or "SELFTEST PASS" not in sout:
        sys.exit("verdict_selftest FAILED -- refusing to score:\n" + sout[-800:])
    exe_meta = exe_id(exe)
    probe_meta = exe_id(PROBE) if os.path.exists(PROBE) else None
    git = git_state(CPP)
    print("kernel: %s sha1 %s (built %s)" %
          (exe, exe_meta["sha1"][:12], exe_meta["mtime"]))
    print("source: %s%s" % (git["commit"][:12], " DIRTY" if git["dirty"] else ""))
    ambient = {k: os.environ[k] for k in sorted(os.environ)
               if k.startswith("SESSION_")}
    if ambient:
        print("WARNING: ambient %s stripped from cell envs (hermetic runs)"
              % " ".join(sorted(ambient)))
    flags = dict(kv.split("=", 1) for kv in args.flags.split()) if args.flags else {}
    reps = max(1, args.repeats)
    if reps > 1:
        args.jobs = 1        # the noise band needs contention-free serial timing
    ledger, t0 = {}, time.time()

    def show(cell, rec):
        print("%-34s %-12s %6.1fs %s" %
              (cell, rec["verdict"], rec.get("secs", 0.0),
               "" if rec.get("rel") is None else "rel %.2e" % rec["rel"]),
              flush=True)

    if reps > 1:
        for c in sorted(cells):
            recs = [run_cell(c, cells[c], exe, wd, flags, man)[1]
                    for _ in range(reps)]
            worst = max(recs, key=lambda r: RANK[r["verdict"]])
            worst["repeat_verdicts"] = [r["verdict"] for r in recs]
            worst["secs_range"] = [round(min(r.get("secs", 0.0) for r in recs), 1),
                                   round(max(r.get("secs", 0.0) for r in recs), 1)]
            ledger[c] = worst
            show(c, worst)
    else:
        with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as ex:
            futs = [ex.submit(run_cell, c, cells[c], exe, wd, flags, man)
                    for c in sorted(cells)]
            for fut in concurrent.futures.as_completed(futs):
                cell, rec = fut.result()
                ledger[cell] = rec
                show(cell, rec)
        # A cell can blow the cap purely from CPU contention (chairs_base/fuse needs
        # ~430 s solo but >600 s at 8-way). Re-run every timeout once, alone, before
        # recording it.
        slow = sorted(c for c, r in ledger.items() if r["verdict"] == "timeout")
        for cell in slow:
            print("retry (solo): %s" % cell, flush=True)
            _c, rec = run_cell(cell, cells[cell], exe, wd, flags, man)
            rec["retried_solo"] = True
            ledger[cell] = rec
            show(cell, rec)

    counts = {v: 0 for v in VERDICTS}
    for rec in ledger.values():
        counts[rec["verdict"]] += 1
    ts = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    os.makedirs(LEDGER_DIR, exist_ok=True)
    out = {"ts": ts, "exe": args.exe, "exe_meta": exe_meta,
           "probe_meta": probe_meta, "git": git, "flags": flags,
           "ambient_session": ambient, "jobs": args.jobs, "repeats": reps,
           "rss_cap_mb": RSS_CAP_MB,
           "wall_secs": time.time() - t0, "counts": counts, "cells": ledger}
    lpath = os.path.join(LEDGER_DIR, "ledger_%s.json" % ts)
    json.dump(out, open(lpath, "w"), indent=1)
    print("\nT0 ledger (%d cells, %.0f s): %s" %
          (len(ledger), out["wall_secs"],
           "  ".join("%s %d" % (v, counts[v]) for v in VERDICTS)))
    if reps > 1:
        noisy = sorted(c for c, r in ledger.items()
                       if len(set(r["repeat_verdicts"])) > 1)
        print("noise band (%d repeats): %d/%d cells nondeterministic%s"
              % (reps, len(noisy), len(ledger),
                 " -- " + " ".join(noisy) if noisy else ""))
    for bat in sorted({c["truth"]["battery"] for c in ledger.values()}):
        sub = {v: 0 for v in VERDICTS}
        for rec in ledger.values():
            if rec["truth"]["battery"] == bat:
                sub[rec["verdict"]] += 1
        print("  %-12s %s" % (bat, "  ".join("%s %d" % (v, sub[v]) for v in VERDICTS)))
    print("ledger: %s" % lpath)

    if os.path.exists(BASELINE) and not args.no_diff:
        base = json.load(open(BASELINE))
        bmeta = json.load(open(BASELINE_META)) if os.path.exists(BASELINE_META) else {}
        if bmeta.get("exe_meta", {}).get("sha1") not in (None, exe_meta["sha1"]):
            print("WARNING: baseline was set with kernel sha1 %s (built %s), this run "
                  "used %s (built %s) -- verdict deltas below may be kernel changes, "
                  "not harness noise."
                  % (bmeta["exe_meta"]["sha1"][:12], bmeta["exe_meta"]["mtime"],
                     exe_meta["sha1"][:12], exe_meta["mtime"]))
        if bmeta.get("flags") not in (None, flags):
            print("WARNING: baseline flags %s != this run's %s"
                  % (bmeta["flags"] or "{}", flags or "{}"))
        regs, imps = [], []
        for cell, rec in sorted(ledger.items()):
            bv = base.get(cell)
            if bv is None:
                continue
            if RANK[rec["verdict"]] > RANK[bv]:
                regs.append((cell, bv, rec))
            elif RANK[rec["verdict"]] < RANK[bv]:
                imps.append((cell, bv, rec["verdict"]))
        for cell, bv, rec in regs:
            p = write_ticket(cell, rec, bv, flags)
            print("REGRESSION %s: %s -> %s  ticket %s"
                  % (cell, bv, rec["verdict"], os.path.basename(p)))
        for cell, bv, nv in imps:
            print("improved   %s: %s -> %s" % (cell, bv, nv))
        if not regs:
            print("no regressions vs baseline (%s)" % BASELINE)
    elif not os.path.exists(BASELINE):
        print("no baseline yet -- rerun with --set-baseline to accept this run")
    if args.set_baseline:
        base = json.load(open(BASELINE)) if os.path.exists(BASELINE) else {}
        base.update({c: r["verdict"] for c, r in ledger.items()})   # merge: a filtered
        json.dump(dict(sorted(base.items())), open(BASELINE, "w"), indent=1)  # run only
        json.dump({"ts": ts, "exe": args.exe, "exe_meta": exe_meta,
                   "probe_meta": probe_meta, "git": git, "flags": flags,
                   "cells": len(base)}, open(BASELINE_META, "w"), indent=1)
        print("baseline set: %s (%d cells, kernel %s)"
              % (BASELINE, len(base), exe_meta["sha1"][:12]))   # updates its own
    return out


def main():
    ap = argparse.ArgumentParser(description="T0 corpus runner (five-verdict ledger)")
    ap.add_argument("cmd", choices=["truth", "run"])
    ap.add_argument("--exe", default=KERNEL)
    ap.add_argument("--cells", default="", help="substring filter on cell name")
    ap.add_argument("--flags", default="", help='e.g. "SESSION_AUTO=1 SESSION_M3=1"')
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--timeout", type=int, default=CELL_TIMEOUT,
                    help="per-cell cap in seconds (a hang is recorded as `timeout`, "
                         "never skipped)")
    ap.add_argument("--repeats", type=int, default=1,
                    help="run each cell N times serially (forces --jobs 1) and "
                         "record the verdict spread -- the noise band")
    ap.add_argument("--rss-cap-mb", type=int, default=0,
                    help="cap each cell's address space via prlimit; peak RSS is "
                         "recorded per cell either way")
    ap.add_argument("--set-baseline", action="store_true")
    ap.add_argument("--no-diff", action="store_true")
    ap.add_argument("--refresh-operands", action="store_true",
                    help="re-copy the operand STEPs from the shipped chairs dir "
                         "(they are pinned in work/ on first use)")
    ap.add_argument("--reuse-exe", action="store_true",
                    help="pin the snapshot already in work/ instead of re-copying "
                         "build/main_7 (session A rebuilds it continuously)")
    args = ap.parse_args()
    if args.cmd == "truth":
        build_truth(args)
    else:
        run_all(args)


if __name__ == "__main__":
    main()
