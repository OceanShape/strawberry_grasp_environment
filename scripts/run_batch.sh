#!/usr/bin/env bash
# T4d 무작위 배치 배치 실행 — 런 N 회를 사람 손 없이 돈다.
#
#   bash scripts/run_batch.sh <런 수> [--seed-start S] [--tag pilot]
#
# 전제: Isaac Sim 이 떠 있고 Script Editor 에서 strawberry_harvest/scripts/isaac_batch_orchestrator.py 를 한 번 Run 한 상태.
# 런마다:  gen_random_layout.py --seed s --apply  →  request.json(reload)  →  [Isaac: Stop·재로드·브릿지·HUD·Play]  →
#          run_nodes.sh(새 프로세스)  →  "전부 정합" 대기  →  트리거  →  READY_FOR_NEXT_START 대기  →  --kill  →
#          로그·Kit 브릿지 줄·배치 JSON 을 log/m3/random/<tag>/run_<i>_seed_<s>/ 로  →  run_metrics.py (CSV 누적)
# 끝나면 base 배치를 파일에 되돌린다(씬은 다음 재로드 때 반영). 지표 요약은 run_metrics.py --aggregate.
#
# 왜 런마다 노드를 새로 띄우나: 재트리거는 트레이 슬롯 포인터를 이어 간다(런 13 교훈, run_guide). 런당 조건을 같게 하려면 재기동.
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO" || exit 1

N="${1:-}"; shift || true
SEED_START=1
TAG="batch"
while [ $# -gt 0 ]; do
    case "$1" in
        --seed-start) SEED_START="$2"; shift 2 ;;
        --tag) TAG="$2"; shift 2 ;;
        *) echo "unknown arg: $1"; exit 1 ;;
    esac
done
[ -n "$N" ] || { echo "usage: bash scripts/run_batch.sh <런 수> [--seed-start S] [--tag pilot]"; exit 1; }

BATCH_DIR=/tmp/harvest_batch
REQ="$BATCH_DIR/request.json"
STATE="$BATCH_DIR/isaac_state.json"
GEN="strawberry_harvest/scripts/scene_tools/gen_random_layout.py"
OUT_ROOT="log/m3/random/$TAG"
CSV="$OUT_ROOT/runs.csv"
mkdir -p "$BATCH_DIR" "$OUT_ROOT"

ISAAC_READY_TIMEOUT=300     # 재로드 + 브릿지 + Play
NODES_READY_TIMEOUT=600     # cuRobo warmup 포함
RUN_TIMEOUT=1200            # 한 런 완주 (런 14 292초)
CONSEC_FAIL_LIMIT=2

log() { printf '[run_batch %s] %s\n' "$(date +%H:%M:%S)" "$*"; }

state_field() {  # state_field <키>
    python3 - "$STATE" "$1" <<'EOF'
import json, sys
try:
    print(json.load(open(sys.argv[1])).get(sys.argv[2], ""))
except Exception:
    print("")
EOF
}

wait_isaac_ready() {  # wait_isaac_ready <run>
    local run="$1" t=0
    while [ $t -lt $ISAAC_READY_TIMEOUT ]; do
        if [ "$(state_field run)" = "$run" ]; then
            case "$(state_field state)" in
                ready) return 0 ;;
                error) log "Isaac 오케스트레이터 오류: $(state_field msg)"; return 1 ;;
            esac
        fi
        sleep 1; t=$((t + 1))
    done
    log "Isaac ready 대기 시간 초과 (${ISAAC_READY_TIMEOUT}s) — 오케스트레이터가 Run 돼 있는지 확인"
    return 1
}

[ -f "$STATE" ] || { log "$STATE 가 없다 — Isaac Script Editor 에서 isaac_batch_orchestrator.py 를 먼저 Run 할 것"; exit 1; }
[ -f log/m3/random/layouts/base_layout.json ] || python3 "$GEN" --snapshot-base

set +u
source /opt/ros/humble/setup.bash || exit 1
source install/setup.bash || exit 1
set -u

bash scripts/run_nodes.sh --kill >/dev/null 2>&1 || true
FAILS=0
for i in $(seq 1 "$N"); do
    SEED=$((SEED_START + i - 1))
    OUT="$OUT_ROOT/run_${i}_seed_${SEED}"
    mkdir -p "$OUT"
    log "===== run $i/$N  seed $SEED  -> $OUT"

    python3 "$GEN" --seed "$SEED" --apply | tee "$OUT/layout.txt" | tail -3
    cp "log/m3/random/layouts/seed_${SEED}.json" "$OUT/layout.json"

    printf '{"action": "reload", "run": %d, "seed": %d, "ts": %s}\n' "$i" "$SEED" "$(date +%s)" > "$REQ.tmp" && mv "$REQ.tmp" "$REQ"
    if ! wait_isaac_ready "$i"; then FAILS=$((FAILS + 1)); [ $FAILS -ge $CONSEC_FAIL_LIMIT ] && break; continue; fi
    KIT_LOG="$(state_field kit_log)"; KIT_OFF="$(state_field kit_offset)"
    log "Isaac ready (kit log $KIT_LOG @${KIT_OFF})"

    bash scripts/run_nodes.sh > "$OUT/run_nodes.out" 2>&1 &
    RN_PID=$!
    t=0; OK=0
    while [ $t -lt $NODES_READY_TIMEOUT ]; do
        grep -q "전부 정합" "$OUT/run_nodes.out" 2>/dev/null && { OK=1; break; }
        grep -q "대조 실패\|설정 불일치\|이전 실행의 노드가 살아 있다" "$OUT/run_nodes.out" 2>/dev/null && break
        kill -0 $RN_PID 2>/dev/null || break
        sleep 2; t=$((t + 2))
    done
    if [ $OK -ne 1 ]; then
        log "노드 기동 실패 — $OUT/run_nodes.out"; bash scripts/run_nodes.sh --kill >/dev/null 2>&1
        FAILS=$((FAILS + 1)); [ $FAILS -ge $CONSEC_FAIL_LIMIT ] && break; continue
    fi
    sleep 3
    log "trigger"
    ros2 service call /strawberry/scan/start std_srvs/srv/Trigger > "$OUT/trigger.out" 2>&1
    t=0; DONE=0
    while [ $t -lt $RUN_TIMEOUT ]; do
        grep -q "READY_FOR_NEXT_START" run_logs/latest/scan.log 2>/dev/null && { DONE=1; break; }
        sleep 5; t=$((t + 5))
    done
    sleep 6      # 마지막 낙하의 DROP_REST(릴리스 +3s) 까지 Kit 로그에 남게
    [ $DONE -eq 1 ] && log "완주 (${t}s)" || log "!! 완주 신호 없음 (${RUN_TIMEOUT}s) — 그대로 수집하고 다음으로"
    bash scripts/run_nodes.sh --kill >/dev/null 2>&1

    cp run_logs/latest/planner.log run_logs/latest/scan.log run_logs/latest/bridge.log run_logs/latest/vision.log "$OUT/" 2>/dev/null
    if [ -n "$KIT_LOG" ] && [ -f "$KIT_LOG" ]; then
        tail -c +"$((KIT_OFF + 1))" "$KIT_LOG" | grep -a "\[bridge\] " | sed 's/^.*\[py stdout\]: //' > "$OUT/kit_bridge.log"
    fi
    python3 scripts/run_metrics.py "$OUT" --seed "$SEED" --csv "$CSV" > "$OUT/metrics.txt"
    python3 - "$OUT/metrics.json" <<'EOF'
import json, sys
m = json.load(open(sys.argv[1]))
print("  grasp %d  detach %d  blocked %d(%s)  placed %d  dropped %d(floor %d caught %d below %d)  subdivide %s  %ds  consistent=%s" % (
    m["grasp_contact"], m["pick_complete"], m["place_blocked"], ",".join(m["place_blocked_reasons"]), m["placed"], m["dropped"],
    m["drop_on_floor"], m["drop_caught"], m["drop_below_floor"], "+".join(m["subdivide"]), m["duration_s"] or -1, m["consistent"]))
EOF
    [ $DONE -eq 1 ] && FAILS=0 || FAILS=$((FAILS + 1))
    [ $FAILS -ge $CONSEC_FAIL_LIMIT ] && { log "연속 실패 $FAILS — 중단"; break; }
done

printf '{"action": "done", "ts": %s}\n' "$(date +%s)" > "$REQ.tmp" && mv "$REQ.tmp" "$REQ"
python3 "$GEN" --restore
log "base 배치를 파일에 되돌렸다 (Isaac 씬은 다음 재로드 때 반영). 요약:"
python3 scripts/run_metrics.py --aggregate "$CSV"
