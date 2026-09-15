"""런의 실행기 로그(run_logs/<stamp>/scan.log 또는 log/m3/<run_id>/scan_executor.log) 의 MoveJoint 순서를 J6 등가각 창 두 가지로 재생한다.
- old: ±359.4 (종전 _JOINT_LIMITS_RAD)   - new: ±225 (플래너 OPERATIONAL 창)
픽 모델: 픽 시작 J6 가 ±225 밖이면 플래너가 궤적을 창 안으로 고쳐 써 브릿지가 360° 회전 → 복귀 후 J6 는 창 안 등가값.
"""
import re, sys
log = sys.argv[1]
PLANNER_WIN = (-225.0, 225.0)
OVERVIEW_J6 = 93.4

def pick_equiv(base, cur, lo, hi):
    cands = [base + 360.0 * k for k in range(-2, 3)]
    cands = [c for c in cands if lo <= c <= hi]
    return min(cands, key=lambda c: abs(c - cur)) if cands else base

def into(v, lo, hi):
    return pick_equiv(v, 0.0 if lo < 0 < hi else lo, lo, hi) if not (lo <= v <= hi) else v

events = []
for line in open(log, encoding="utf-8", errors="replace"):
    m = re.search(r"MOVING_TO (\S+)\s+endpoint_deg=\[([^\]]+)\]", line)
    if m:
        events.append(("move", m.group(1), float(m.group(2).split()[5]))); continue
    if "TRANSIT_VIA_OVERVIEW" in line:
        events.insert(len(events) - 1, ("move", "overview", OVERVIEW_J6)); continue
    m = re.search(r"PICK_SEQUENCE_START (\S+)", line)
    if m:
        events.append(("pick", m.group(1), None))

for name, (lo, hi) in (("old ±359.4", (-359.4, 359.4)), ("new ±225", PLANNER_WIN)):
    cur = OVERVIEW_J6
    spins, parked_out, movej_total, rows = 0, 0, 0.0, []
    for kind, cell, base in events:
        if kind == "move":
            tgt = pick_equiv(base, cur, lo, hi)
            rot = abs(tgt - cur); movej_total += rot
            flag = " ★창밖" if not (PLANNER_WIN[0] <= tgt <= PLANNER_WIN[1]) else ""
            if flag: parked_out += 1
            rows.append("  MoveJoint %-12s J6 %7.1f -> %7.1f  (%5.1f°)%s" % (cell, cur, tgt, rot, flag))
            cur = tgt
        else:
            if not (PLANNER_WIN[0] <= cur <= PLANNER_WIN[1]):
                new = pick_equiv(cur, 0.0, *PLANNER_WIN)
                rows.append("  PICK      %-12s J6 %7.1f -> 플래너 재작성 %7.1f  ★360° 회전" % (cell, cur, new))
                spins += 1; cur = new
            else:
                rows.append("  PICK      %-12s J6 %7.1f (창 안, 회전 없음)" % (cell, cur))
    print("=== %s: 360° 회전 %d회, 창 밖 정차 %d회, MoveJoint J6 누적 회전 %.0f°" % (name, spins, parked_out, movej_total))
    print("\n".join(rows))
