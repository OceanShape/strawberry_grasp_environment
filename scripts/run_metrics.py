#!/usr/bin/env python3
"""T4d 런 지표 — 로그 4개(+Kit 브릿지 줄)에서 런 하나의 수치를 뽑아 JSON/CSV 로 남긴다.

    python3 scripts/run_metrics.py <런 디렉터리> [--seed N] [--csv log/m3/random/runs.csv]
    python3 scripts/run_metrics.py --aggregate log/m3/random/runs.csv

런 디렉터리는 run_nodes.sh 의 run_logs/<stamp>/ (planner.log·scan.log·bridge.log·vision.log) 또는
log/m3/<run_id>/ (curobo_planner.log·scan_executor.log·sim_executor_bridge.log·fake_vision.log) 둘 다 받는다.
Kit 쪽 사건(PLACED/DROPPED/DROP_REST)은 디렉터리의 kit_bridge.log(run_batch.sh 가 잘라 둔 브릿지 줄) 또는 kit_*.log 에서 읽는다.

지표 정의 (SUBMISSION_PLAN T4d):
  grasp_rate     = GRASP_JUDGE CONTACT 수 / 익은 과실 수(8)          — 파지 판정 통과
  detach_rate    = PICK COMPLETE 수 / 8                              — 분리까지
  reject_rate    = PLACE_BLOCKED 수 / PICK COMPLETE 수               — 이송·배치 계획 거부 (분리된 것 중)
  place_rate     = Kit PLACED 수 / 8                                 — 트레이 안 릴리스 (최종)
  drop           = Kit DROPPED 수 (= 플래너 released-here 수여야 한다)
분할: SUBDIVIDE 분면 목록, 세부 자세 단계(lab_plane / parent_y), SUBDIVIDE_REJECTED, SUBCELL_EMPTY, VIEWING 수.
숫자는 로그에 있는 그대로 센다. 보정 없음.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import re
import sys

RIPE = 8

NAMES = {
    "planner": ("planner.log", "curobo_planner.log"),
    "scan": ("scan.log", "scan_executor.log"),
    "bridge": ("bridge.log", "sim_executor_bridge.log"),
    "vision": ("vision.log", "fake_vision.log"),
}

_TS = re.compile(r"^\[[A-Z]+\] \[(\d+)\.\d+\]")


def _read(d: str, role: str) -> list[str]:
    for n in NAMES[role]:
        p = os.path.join(d, n)
        if os.path.exists(p):
            return open(p, encoding="utf-8", errors="replace").read().splitlines()
    return []


def _kit_lines(d: str) -> list[str]:
    p = os.path.join(d, "kit_bridge.log")
    if os.path.exists(p):
        return open(p, encoding="utf-8", errors="replace").read().splitlines()
    out = []
    for k in sorted(glob.glob(os.path.join(d, "kit_*.log"))):
        for line in open(k, encoding="utf-8", errors="replace"):
            if "[bridge] " in line:
                out.append(line.split("[py stdout]: ")[-1].rstrip())
    return out


def _ts(line: str) -> int | None:
    m = _TS.match(line)
    return int(m.group(1)) if m else None


def _reason(prev_lines: list[str]) -> str:
    """PLACE_BLOCKED 직전 몇 줄에서 거부 이유를 분류한다."""
    for l in reversed(prev_lines[-6:]):
        if "Cartesian plan rejected" in l:
            m = re.search(r"rejected: (J\d) (swing|spline jump)", l)
            if m:
                return "%s_%s" % (m.group(1), m.group(2).replace(" ", "_"))
            return "plan_rejected"
        if "IK_FAIL" in l:
            return "IK_FAIL"
    return "other"


def metrics(d: str, seed=None) -> dict:
    pl, sc, br, kit = _read(d, "planner"), _read(d, "scan"), _read(d, "bridge"), _kit_lines(d)
    m: dict = {"dir": os.path.relpath(d), "seed": seed, "ripe": RIPE}

    # --- scan: 시간, 1차 스캔, 분할 ---
    t0 = next((_ts(l) for l in sc if "OVERVIEW_SCAN_STARTED" in l), None)
    t1 = next((_ts(l) for l in reversed(sc) if "READY_FOR_NEXT_START" in l), None)
    m["duration_s"] = (t1 - t0) if (t0 and t1) else None
    ov = next((l for l in sc if re.search(r"OVERVIEW_SCAN\s+nw:", l)), "")
    m["overview"] = {q: int(v) for q, v in re.findall(r"(nw|ne|se|sw):(\d+)", ov)} if ov else {}
    pr = next((l for l in sc if "TRAVERSAL_PRUNED" in l), "")
    m["pruned"] = re.findall(r"root/(nw|ne|se|sw)", pr)
    m["subdivide"] = re.findall(r"SUBDIVIDE root/(nw|ne|se|sw) candidates", "\n".join(sc))
    m["subdivide_skip"] = len([l for l in sc if "SUBDIVIDE_SKIP" in l])
    tiers = re.findall(r"SUBCELL_POSE root/\w+/\w+ tier=(\w+)", "\n".join(sc))
    m["subcell_lab_plane"] = tiers.count("lab_plane")
    m["subcell_parent_y"] = tiers.count("parent_y")
    m["subcell_rejected"] = len([l for l in sc if "SUBDIVIDE_REJECTED" in l])
    m["subcell_empty"] = len([l for l in sc if "SUBCELL_EMPTY" in l])
    vi = _read(d, "vision")
    m["viewing"] = len([l for l in vi if "(VIEWING)" in l])   # fake_vision 이 세부 칸 시야로 좁힌 횟수

    # --- planner: 시도, 분리, 거부, 중단 ---
    m["pick_attempts"] = len([l for l in pl if "=== PICK 딸기" in l])
    m["pick_complete"] = len([l for l in pl if "PICK COMPLETE" in l])
    m["aborts"] = [l.split("ABORT:")[-1].strip()[:60] for l in pl if "ABORT:" in l]
    blocked, released = [], 0
    for i, l in enumerate(pl):
        if "_PLACE_BLOCKED" in l:
            blocked.append(_reason(pl[max(0, i - 6):i]))
        if "released fruit here" in l:
            released += 1
    m["place_blocked"] = len(blocked)
    m["place_blocked_reasons"] = blocked
    m["released_here"] = released
    m["guard_j3_swing"] = sum(1 for r in blocked if r == "J3_swing")
    m["guard_j6_spline"] = sum(1 for r in blocked if r == "J6_spline_jump")

    # --- bridge(ROS): 파지 판정, 방어선 ---
    m["grasp_contact"] = len([l for l in br if "GRASP_JUDGE" in l and "CONTACT" in l and "MODEL" not in l])
    m["grasp_judged"] = len([l for l in br if "GRASP_JUDGE 줄기기준" in l or ("GRASP_JUDGE" in l and "MODEL" not in l)])
    for key in ("JOINT_COMMAND_REJECTED", "ARM_ARRIVAL_TIMEOUT", "STALLED", "clamped", "EXEC_TIMEOUT"):
        m[key.lower()] = len([l for l in br + pl if key in l])
    m["movej_over_moveit"] = len([l for l in br if "MOVEJ_OVER_DOOSAN_MOVEIT" in l])

    # --- Kit: 트레이 안/밖, 낙하 정지 ---
    m["placed"] = len([l for l in kit if "PLACED in tray" in l])
    m["dropped"] = len([l for l in kit if "DROPPED outside tray" in l])
    rest = [l for l in kit if "DROP_REST" in l]
    m["drop_on_floor"] = len([l for l in rest if "on floor" in l])
    m["drop_caught"] = len([l for l in rest if "caught" in l])
    m["drop_below_floor"] = len([l for l in rest if "BELOW FLOOR" in l])
    m["kit_lines"] = len(kit)

    # --- 비율 ---
    m["grasp_rate"] = m["grasp_contact"] / RIPE
    m["detach_rate"] = m["pick_complete"] / RIPE
    m["reject_rate"] = (m["place_blocked"] / m["pick_complete"]) if m["pick_complete"] else None
    m["place_rate"] = m["placed"] / RIPE
    m["consistent"] = (m["dropped"] == m["released_here"]) and (m["placed"] + m["dropped"] == m["pick_complete"])
    return m


CSV_COLS = ["seed", "dir", "duration_s", "pick_attempts", "grasp_contact", "pick_complete", "place_blocked",
            "guard_j6_spline", "guard_j3_swing", "placed", "dropped", "drop_on_floor", "drop_caught", "drop_below_floor",
            "grasp_rate", "detach_rate", "reject_rate", "place_rate", "subdivide", "subcell_lab_plane",
            "subcell_parent_y", "subcell_rejected", "pruned", "joint_command_rejected", "arm_arrival_timeout",
            "aborts", "consistent"]


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (c - h, c + h)


def aggregate(csv_path: str) -> None:
    if not os.path.exists(csv_path):
        print("no CSV: %s (완주한 런이 없다)" % csv_path)
        return
    rows = list(csv.DictReader(open(csv_path, encoding="utf-8")))
    n = len(rows)
    if n == 0:
        print("no rows")
        return
    tot = lambda k: sum(int(r[k]) for r in rows)
    ripe = RIPE * n
    print("runs %d  ripe %d" % (n, ripe))
    for label, k, den in (("grasp", "grasp_contact", ripe), ("detach", "pick_complete", ripe),
                          ("place", "placed", ripe)):
        v = tot(k)
        lo, hi = wilson(v, den)
        print("  %-7s %3d/%d = %.3f  (95%% Wilson %.3f~%.3f)" % (label, v, den, v / den, lo, hi))
    det, blk = tot("pick_complete"), tot("place_blocked")
    lo, hi = wilson(blk, det)
    print("  reject  %3d/%d = %.3f  (95%% Wilson %.3f~%.3f)  J6 spline %d · J3 swing %d" % (
        blk, det, blk / det if det else float("nan"), lo, hi, tot("guard_j6_spline"), tot("guard_j3_swing")))
    print("  dropped %d  on floor %d  caught %d  below floor %d" % (
        tot("dropped"), tot("drop_on_floor"), tot("drop_caught"), tot("drop_below_floor")))
    print("  subdivide per run: %s" % [r["subdivide"] for r in rows])
    print("  subcell lab_plane %d  parent_y %d  rejected %d" % (
        tot("subcell_lab_plane"), tot("subcell_parent_y"), tot("subcell_rejected")))
    durs = [float(r["duration_s"]) for r in rows if r["duration_s"] not in ("", "None")]
    if durs:
        print("  duration s: min %.0f  mean %.0f  max %.0f" % (min(durs), sum(durs) / len(durs), max(durs)))
    bad = [r["dir"] for r in rows if r["consistent"] != "True"]
    if bad:
        print("  !! 불일치(placed+dropped != pick_complete 또는 dropped != released): %s" % bad)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", nargs="?")
    ap.add_argument("--seed", type=int)
    ap.add_argument("--csv")
    ap.add_argument("--aggregate")
    a = ap.parse_args()
    if a.aggregate:
        aggregate(a.aggregate)
        return 0
    if not a.run_dir:
        ap.error("run_dir 또는 --aggregate")
    m = metrics(a.run_dir, a.seed)
    print(json.dumps(m, ensure_ascii=False, indent=1))
    json.dump(m, open(os.path.join(a.run_dir, "metrics.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    if a.csv:
        new = not os.path.exists(a.csv)
        os.makedirs(os.path.dirname(a.csv) or ".", exist_ok=True)
        with open(a.csv, "a", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=CSV_COLS, extrasaction="ignore")
            if new:
                w.writeheader()
            row = {k: m.get(k) for k in CSV_COLS}
            row["subdivide"] = "+".join(m["subdivide"])
            row["pruned"] = "+".join(m["pruned"])
            row["aborts"] = " | ".join(m["aborts"])
            w.writerow(row)
    return 0


if __name__ == "__main__":
    sys.exit(main())
