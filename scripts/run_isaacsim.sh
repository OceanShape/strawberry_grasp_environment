#!/usr/bin/env bash
# Isaac Sim 을 녹화 준비된 상태로 띄운다. (docs/run_guide.md 터미널 1)
#
#   bash scripts/run_isaacsim.sh                    띄운다
#   bash scripts/run_isaacsim.sh --/exts/strawberry.sim.setup/pin_persp_camera=false
#                                                   카메라 고정만 끄고 띄운다
#   추가 인자는 그대로 Kit 에 넘어간다.
#
# 기존 run_guide 의 긴 한 줄과 env 는 동일하다. 여기에 붙은 것은 두 줄뿐:
#   --ext-folder strawberry_harvest/kit_ext --enable strawberry.sim.setup
# 이 확장이 부팅 때 (1) 뷰포트 HUD(FPS/메모리/해상도) 끄기, (2) Script Editor 를
# Render Settings 탭 모음에 도킹, (3) Perspective 카메라 고정을 해 준다.
# 값은 strawberry_harvest/kit_ext/strawberry.sim.setup/config/extension.toml.

set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${VENV:-$HOME/.venv}"
EXT_FOLDER="$REPO/strawberry_harvest/kit_ext"

if [[ ! -f "$VENV/bin/activate" ]]; then
    echo "[run_isaacsim] venv 가 없다: $VENV (VENV=... 로 지정 가능)" >&2
    exit 1
fi

# shellcheck disable=SC1091
source "$VENV/bin/activate"

# ROS2 브릿지가 쓰는 LD_LIBRARY_PATH 만 남기고 ROS 환경변수는 걷어낸다
# (걷어내지 않으면 Isaac 이 시스템 python 을 잡는다 — run_guide 참고).
exec env -u PYTHONPATH -u AMENT_PREFIX_PATH -u ROS_VERSION -u ROS_PYTHON_VERSION \
    ROS_DISTRO=humble \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    LD_LIBRARY_PATH="$VENV/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib" \
    isaacsim \
    --ext-folder "$EXT_FOLDER" \
    --enable strawberry.sim.setup \
    "$@"
