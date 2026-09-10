#!/usr/bin/env bash
# 이식 변경 추적 — PLANNER_POLICY_v2.md
#   v2에서 플래너 수정은 전면 허용된다. 이 스크립트는 "안 고쳤다"를 증명하는 도구가 아니라,
#   원본 대비 무엇을 바꿨는지 언제든 다시 뽑아볼 수 있게 하는 보조 도구다.
# 사용: bash scripts/check_planner.sh
set -u
cd "$(dirname "$0")/.." || exit 1

BASE=_baseline
EV=evidence
X="--exclude=__pycache__ --exclude=*.pyc"
mkdir -p "$EV"

# ---------- 등급 A: 실기 플래너 (원본 대조본 확보) ----------
{
  diff -ru $X "$BASE/A_strawberry_motion/scripts"   "src/strawberry_motion/scripts"
  diff -ru $X "$BASE/A_strawberry_motion/execution" "src/strawberry_motion/strawberry_motion/execution"
} > "$EV/planner_diff.txt" 2>&1

# ---------- 등급 B: 설정 자산 (원본 미확보, logs 제외) ----------
{
  diff -ru $X "$BASE/B_e0509_gripper_description/config"     "src/e0509_gripper_description/config"
  diff -u     "$BASE/B_e0509_gripper_description/setup.py"   "src/e0509_gripper_description/setup.py"
  diff -u     "$BASE/B_e0509_gripper_description/package.xml" "src/e0509_gripper_description/package.xml"
} > "$EV/config_asset_diff.txt" 2>&1

changed_lines() { grep -cE '^[+-]' "$1" 2>/dev/null | tr -d '\n'; }
# 경로 치환(이미 승인된 이관 작업)을 제외한 '로직' 변경 라인 수
logic_lines() {
  grep -E '^[+-]' "$1" 2>/dev/null \
    | grep -vE '^(\+\+\+|---)' \
    | grep -vE 'doosan_ws|strawberry_grasp_environment|/home/user/' \
    | wc -l | tr -d ' '
}

echo "==================================================================="
echo " 이식 변경 추적  ($(date '+%Y-%m-%d %H:%M'))"
echo "==================================================================="
echo
echo "[등급 A] 실기 플래너  src/strawberry_motion/   (원본 대조본 확보)"
echo "  diff 라인      : $(wc -l < "$EV/planner_diff.txt")"
echo "  변경 라인(+/-) : $(changed_lines "$EV/planner_diff.txt")"
echo "  경로 외 변경   : $(logic_lines "$EV/planner_diff.txt")   <- PLANNER_CHANGES.md 에 한 줄씩 있어야 함"
echo "  증거 파일      : $EV/planner_diff.txt"
echo
echo "[등급 B] 설정 자산  src/e0509_gripper_description/   (원본 미확보)"
echo "  diff 라인      : $(wc -l < "$EV/config_asset_diff.txt")"
echo "  변경 라인(+/-) : $(changed_lines "$EV/config_asset_diff.txt")"
echo "  증거 파일      : $EV/config_asset_diff.txt"
echo "  ※ 수치 변경이 보이면 SIM 오염을 의심하고 대장을 확인할 것"
echo
echo "[코드 내 태그]"
grep -rn "PLANNER-FIX\|PLANNER-PARAM" src/strawberry_motion src/e0509_gripper_description \
     --include=*.py --include=*.yml --include=*.yaml 2>/dev/null || echo "  (없음)"
echo
echo "[감사 대상 == 실행 대상 정합]"
# scan_executor_node는 python3 -m 으로 실행되며, src/ 가 아니라 build/ 또는 install/ 의
# 사본에서 로드된다 (어느 쪽이 이기는지는 PYTHONPATH 순서에 달려 있다).
# src/만 감사하면 "감사한 코드"와 "실제 도는 코드"가 갈라질 수 있으므로 사본 전부를 대조한다.
# strawberry_sim_core(가상 제어기)도 같이 본다. symlink-install 이 아니라 복사 설치라
# src/만 고치고 build 를 안 하면 ros2 run 은 옛 코드를 띄운다.
_mismatch=0
_found=0
_bad_pkgs=""
for PKG in strawberry_motion strawberry_sim_core; do
  case "$PKG" in
    strawberry_motion)  SRC_PKG="src/strawberry_motion/strawberry_motion" ;;
    strawberry_sim_core) SRC_PKG="src/strawberry_sim_core/strawberry_sim_core" ;;
  esac
  COPIES="build/$PKG/$PKG install/$PKG/lib/python3.10/site-packages/$PKG"
  for C in $COPIES; do
    [ -d "$C" ] || continue
    _found=$((_found+1))
    if diff -rq $X "$C" "$SRC_PKG" > /dev/null 2>&1; then
      echo "  OK   $C"
    else
      _mismatch=$((_mismatch+1))
      case "$_bad_pkgs" in *"$PKG"*) ;; *) _bad_pkgs="$_bad_pkgs $PKG" ;; esac
      echo "  !!   $C  ← src/와 불일치"
      diff -rq $X "$C" "$SRC_PKG" 2>&1 | sed 's/^/          /'
    fi
  done
done
if [ "$_found" -eq 0 ]; then
  echo "  (사본 없음 — 미빌드 상태)"
elif [ "$_mismatch" -gt 0 ]; then
  echo "  → src/를 고치고 colcon build를 안 했다. 실행되는 코드는 사본 쪽이다."
  echo "     조치: colcon build --packages-select$_bad_pkgs"
else
  echo "  → 사본 $_found개 모두 src/와 일치 (실행되는 코드 = 감사한 코드)"
fi
echo
# v2 로그 형식은 '- [파일:함수] 한 문장' 이다 (v1의 번호 표가 아니다).
echo "[수정 로그 건수] $(grep -cE '^- \[' PLANNER_CHANGES.md 2>/dev/null || echo 0) 건  (PLANNER_CHANGES.md)"
echo
echo "-> diff에 나타난 변경이 PLANNER_CHANGES.md 에 한 줄씩 있는지 확인할 것."
echo "-> 문서·이력서에 \"0줄 수정 / 무수정\" 표현을 쓰지 말 것 (PLANNER_POLICY_v2.md §5.1)."
