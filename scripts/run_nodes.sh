#!/usr/bin/env bash
# 수확 파이프라인 노드 4개를 한 터미널에서 한 번에 띄운다. (docs/run_guide.md T2)
#
#   bash scripts/run_nodes.sh            띄운다
#   bash scripts/run_nodes.sh --kill     남은 노드만 정리하고 끝낸다
#   bash scripts/run_nodes.sh --no-check check_params.py 게이트를 건너뛴다
#
# 로그는 터미널에 [태그] 접두사로 섞여 보이고, 동시에 노드별 파일로 따로 남는다.
#   run_logs/latest/{vision,bridge,planner,scan}.log
# Ctrl+C 한 번이면 4개 다 죽는다 (비대화형 셸이라 전부 같은 프로세스 그룹이다).

set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO" || exit 1

# 로그 파일 이름에 노드 이름을 쓰지 않는다 — pkill -f 패턴에 tee 가 걸린다.
NODE_NAMES=(fake_vision_node sim_executor_bridge_node curobo_planner_node
            scan_executor_node status_monitor_node)
NODE_PAT="$(IFS='|'; echo "${NODE_NAMES[*]}")"

kill_nodes() {
    for n in "${NODE_NAMES[@]}"; do pkill -f "$n" >/dev/null 2>&1; done
    sleep 2
    # planner 는 MotionGen warmup 중 SIGTERM 을 늦게 받는다 → -9 필요
    for n in "${NODE_NAMES[@]}"; do pkill -9 -f "$n" >/dev/null 2>&1; done
    sleep 1
}

if [ "${1:-}" = "--kill" ]; then
    echo "[run_nodes] 노드 정리 중..."
    kill_nodes
    left="$(pgrep -af "$NODE_PAT")"
    if [ -n "$left" ]; then echo "!! 아직 남았다:"; echo "$left"; exit 1; fi
    echo "[run_nodes] 정리 완료. Isaac Sim 은 건드리지 않았다 (씬 재로드 필요)."
    exit 0
fi

# ── 1. 남은 노드 점검 ─────────────────────────────────────────────
# 2026-09-08: 전날 띄운 노드 4개가 11시간째 살아 옛 설정으로 계획하고 있었다.
LEFT="$(pgrep -af "$NODE_PAT")"
if [ -n "$LEFT" ]; then
    echo "!! 이전 실행의 노드가 살아 있다. 새 노드와 토픽·서비스가 겹친다."
    echo "$LEFT"
    echo
    echo "   정리:  bash scripts/run_nodes.sh --kill"
    exit 1
fi

# ── 2. 설정 정합 (보드 y 는 6개 파일에 중복돼 있다) ────────────────
if [ "${1:-}" != "--no-check" ]; then
    if ! python3 check_params.py; then
        echo
        echo "!! 설정 불일치. docs/parameters.md 와 대조해 고치고 다시 띄운다."
        echo "   그래도 강행하려면: bash scripts/run_nodes.sh --no-check"
        exit 1
    fi
    echo
fi

# ── 3. source ────────────────────────────────────────────────────
# ROS 의 setup.bash 는 미정의 변수를 읽으므로 이 구간에서만 set -u 를 끈다.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash || { echo "!! source /opt/ros/humble/setup.bash 실패"; exit 1; }
# shellcheck disable=SC1091
source install/setup.bash || { echo "!! source install/setup.bash 실패 — colcon build 를 했는가?"; exit 1; }
set -u

# ── 2.5 HUD 스냅샷 잔재 제거 ─────────────────────────────────────
# 노드 4개는 /tmp/harvest_hud_<role>.json 을 0.1초마다 덮어쓰고, 죽으면 마지막 값이 남는다.
# 그대로 두면 planner warmup(수십 초) 동안 HUD 가 직전 런의 '수확 완료 6/6' 과 새 노드의
# 빈 상태를 섞어 보여준다. 여기서 지우면 기동 즉시 대기 상태로 시작한다.
# 완주 후 트리거 대기 중에는 노드가 살아 있어 이 경로를 타지 않는다 (--kill 도 지우지 않는다 — 엔딩 유지).
rm -f /tmp/harvest_hud_*.json

STAMP="$(date +%Y%m%d_%H%M%S)"
LOGDIR="$REPO/run_logs/$STAMP"
mkdir -p "$LOGDIR"
ln -sfn "$LOGDIR" "$REPO/run_logs/latest"

export PYTHONUNBUFFERED=1

cleanup() {
    trap - INT TERM EXIT
    echo
    echo "[run_nodes] 종료 중 — 노드 4개를 모두 정리한다."
    kill_nodes
    left="$(pgrep -af "$NODE_PAT")"
    if [ -n "$left" ]; then
        echo "!! 아직 남은 노드가 있다. 직접 확인할 것:"; echo "$left"
    else
        echo "[run_nodes] 남은 노드 없음. 로그: $LOGDIR"
    fi
    echo "[run_nodes] Isaac Sim 은 그대로다. 재실행 전 Stop → 씬 재로드 → 브릿지 Run → Play."
}
trap cleanup INT TERM EXIT

echo "[run_nodes] 로그: $LOGDIR  (run_logs/latest 로도 열린다)"
echo

# ── 4. 기동 ──────────────────────────────────────────────────────
# 노드 사이에 기동 순서 의존성은 없다 (초기화 시점에 blocking wait 이 하나도 없다).
# 다만 bridge 와 planner 가 둘 다 CUDA 를 잡으므로 IK 초기화가 끝난 뒤 planner 를 올린다.

stdbuf -oL -eL ros2 run strawberry_sim_core fake_vision_node \
    > >(tee "$LOGDIR/vision.log" | stdbuf -oL sed 's/^/[vision]  /') 2>&1 &

stdbuf -oL -eL ros2 run strawberry_sim_core sim_executor_bridge_node \
    > >(tee "$LOGDIR/bridge.log" | stdbuf -oL sed 's/^/[bridge]  /') 2>&1 &

# bridge 의 cuRobo IK 초기화를 기다린다 (최대 120초). 못 기다려도 계속 진행한다.
for _ in $(seq 1 240); do
    grep -q "cuRobo IK Solver successfully initialized" "$LOGDIR/bridge.log" 2>/dev/null && break
    sleep 0.5
done

# planner 는 flat import 구조라 scripts/ 에서 실행해야 한다.
# taught_slot_sequence (T4-1, 2026-09-10): 0,0,0,0,0,0 -> 0,1,3,4,6,7 -> **0,1,6,7,12,13**.
#   1차(0,1,3,4,6,7): slot 0 고정을 풀었다. T2 로 과실이 실제 이송되자 여섯 개가 한 칸에
#     겹치는 것이 드러났고, 이는 시뮬이 아니라 실기 노드 설정의 결함이다 (같은 값이면 실기도
#     한 칸에 떨어뜨린다; 부트캠프 최종은 과실 1~2개라 안 드러났다).
#   2차(0,1,6,7,12,13): **행을 한 칸씩 건너뛴다.** 13:12 런에서 이웃한 행끼리 과실이 닿았다.
#     원인은 배치가 아니라 티칭 격자 자체다 — 행 피치 51.2mm 인데 트레이에 놓인 과실의
#     y 전폭이 53.8mm (파지 자세에서 수직축 기준 82° 돌아 x·y 전폭이 바뀐다; 장축은 그대로 수직). **산포가 0이어도 행 이웃 간격은 +0.3mm**,
#     즉 완벽히 실행해도 닿는다. 행을 건너뛰면 피치가 102.4mm 가 되어 이 쌍이 사라지고,
#     남는 열 이웃은 피치 59.8mm / 간격 +15.3mm 다.
#     ⚠️ 5행 × 열 2개(열 2·5·8·11·14 는 IK_FAIL)에서 과실 6개를 행 이웃 없이 놓으려면
#        (0,2,4행) × (0,1열) 조합밖에 없다 — 즉 slot 13 은 뺄 수 없다. slot 13 은 사전 검증에서
#        5/7 이라 **시퀀스 맨 뒤**에 둔다 (실패해도 이미 5개가 놓인 뒤이고, hold_on_place_failure
#        =false 라 그 자리에 놓고 끝낸다). 사전 검증: check_tray_slot_reachability.py.
# orthogonalize_taught_grid (T4-3 2차, 2026-09-11): 실기 slot 0·1·3 세 점이 만드는 배치 격자는 사이각 84.26°
#   평행사변형에 행당 z -2.5mm 기울기다 — 강체 계란판은 그럴 수 없으니 수동 티칭 오차다. 시뮬 계란판은
#   수평·직사각(사용자 결정)이므로 피치 크기만 남기고 축을 -x/-y, z 를 수평으로 둔다. 실기 기본값 false 는 보존.
( cd "$REPO/src/strawberry_motion/scripts" && exec stdbuf -oL -eL python3 curobo_planner_node.py --ros-args \
    -p tool_model_profile:=legacy_160mm \
    -p ee_to_tcp_offset_m:=0.236 \
    -p enable_open_stem_descent:=true \
    -p enable_straight_reverse_retreat:=true \
    -p pick_target_z_bias_m:=0.035 \
    -p allow_generated_tray_slot_release:=true \
    -p enable_marker_place_sequence:=true \
    -p use_taught_slot0_place_reference:=true \
    -p execute_marker_place_release:=true \
    -p hold_after_taught_slot0_place:=false \
    -p taught_slot_sequence:=0,1,6,7,12,13 \
    -p orthogonalize_taught_grid:=true \
) > >(tee "$LOGDIR/planner.log" | stdbuf -oL sed 's/^/[planner] /') 2>&1 &

# overview_prescan: 원안 1·2단계(overview 1차 스캔 → 익은 과실 있는 분면만 순회). 실기 기본 false.
# scan_dwell_sec: 실기 12초는 fusion 다중 프레임 안정화용. fake_vision 은 2Hz 결정적 좌표라 3초면 된다.
stdbuf -oL -eL python3 -m strawberry_motion.execution.scan_executor_node --ros-args \
    -p execute_motion:=true \
    -p target_cell:=all \
    -p overview_prescan:=true \
    -p scan_dwell_sec:=3.0 \
    > >(tee "$LOGDIR/scan.log" | stdbuf -oL sed 's/^/[scan]    /') 2>&1 &

# ── 5. planner warmup 대기 후 기동 로그 자동 대조 ──────────────────
echo
echo "[run_nodes] cuRobo Planner warmup 대기 중 (수십 초)..."
READY=0
for _ in $(seq 1 600); do
    grep -q "cuRobo Planner Ready!" "$LOGDIR/planner.log" 2>/dev/null && { READY=1; break; }
    sleep 0.5
done

FAILS=0
need() {  # need <로그파일> <패턴> <설명>
    if grep -q "$2" "$LOGDIR/$1" 2>/dev/null; then
        printf '  OK   %s\n' "$3"
    else
        printf '  !!   %s   (없음: %s → %s)\n' "$3" "$2" "$1"
        FAILS=$((FAILS + 1))
    fi
}

echo
echo "──────────────────────────────────────────────────────────"
echo " 기동 로그 대조"
echo "──────────────────────────────────────────────────────────"
need bridge.log  "MOVELINE_COLLISION_WORLD:"          "MoveLine 충돌월드 로드 (없으면 이동 중 보드 관통)"
need bridge.log  "tool_tcp_offset=236mm"              "브릿지 TCP 오프셋 236mm (다르면 전 타겟 GRASP_EMPTY)"
need bridge.log  "arm_arrival_tol=0.30deg"            "도착 판정 허용오차 0.30deg (1.50 이면 배치·파지가 손끝 RMS 23.6mm 로 흔들린다)"
need planner.log "EE_TO_TCP_OFFSET_OVERRIDE"          "플래너 TCP 오프셋 160→236mm (없으면 툴을 짧게 보고 관통)"
need planner.log "open_stem_descent=True"             "열린 조우 하강 단계"
need planner.log "straight_reverse_retreat=True"      "진입 역순 후퇴 단계"
need planner.log "slot_sequence=\[0, 1, 6, 7, 12, 13\]" "배치 슬롯 진행 0,1,6,7,12,13 (행 건너뛰기 — 이웃 행이면 과실이 닿는다)"
need planner.log "orthogonalize_taught_grid=True"     "배치 격자 직교화 (false 면 티칭 평행사변형 84.26° 그대로 — 계란판과 어긋난다)"
need scan.log    "scan_executor_node ready"           "scan_executor 기동"
[ "$READY" = "1" ] && printf '  OK   %s\n' "cuRobo Planner Ready!" \
                   || { printf '  !!   %s\n' "cuRobo Planner Ready! 가 5분 안에 안 떴다"; FAILS=$((FAILS + 1)); }
echo "──────────────────────────────────────────────────────────"

if [ "$FAILS" -gt 0 ]; then
    echo " 대조 실패 $FAILS 건 — 트리거하지 말고 원인을 잡는다."
    echo " 자세히: less $LOGDIR/planner.log"
else
    echo " 전부 정합. Isaac Sim 이 Play 상태이고 로봇이 overview 자세인지 확인한 뒤,"
    echo " 터미널 3 에서:"
    echo
    echo "   ros2 service call /strawberry/scan/start std_srvs/srv/Trigger"
fi
echo "──────────────────────────────────────────────────────────"
echo

wait
