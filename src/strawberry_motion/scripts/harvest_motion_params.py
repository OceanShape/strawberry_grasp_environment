#!/usr/bin/env python3
"""Runtime constants for the current strawberry harvest baseline.

These values are experiment-tuned. Moving them out of curobo_planner_node.py
does not make them stable product parameters; it only keeps the planner logic
readable while preserving the current debug branch behavior.
"""

import os


# [FIX 2026-09-10] 사다리 상한을 **조우가 실제로 무는 구간**으로 자른다.
#
# 실측 파츠 기하: 조우가 무는 구간은 ee+230.5~262.5mm, 플래너 TCP 는 ee+236mm.
# 즉 TCP 기준 -5.5 ~ **+26.5mm** 안에 줄기가 들어와야 물린다.
# 오프셋은 TCP 를 목표점에서 그만큼 뒤로 물리는 값이므로, 오프셋 > 26.5mm 는
# **물리적으로 파지가 불가능**하다 — 조우가 줄기보다 앞에서 닫힌다.
#
# 실측 (2026-09-09 런): ripe_02 / ripe_03 이 15mm·30mm 에서 IK_FAIL 하고
# **40mm 로 밀려** 파지된 것처럼 진행됐다 (along +42.9 / +42.5mm).
# 종전 사다리는 40/50/70mm 를 "성공"으로 받아들여 이걸 감췄다.
# 이제 도달 못 하면 정직하게 ABORT 하고 scan pose 로 복귀한다.
GRASP_RETRY_OFFSETS = [0.015, 0.020, 0.025]
MEASURED_TCP_FINAL_STANDOFF_M = -0.120
Y_DETECTION_BIAS_M = 0.000
# [FIX 2026-09-10] 종전 값 [30,35,40,45,50,70]mm 은 **전부 물림 구간(+26.5mm) 밖**이라
# 레거시 툴에서는 하나도 파지가 안 된다. 구간 안으로 자른다.
LEFTMOST_GRASP_RETRY_OFFSETS = [0.015, 0.020, 0.025]
LEFTMOST_GRASP_X_CORR_M = 0.005
# Keep per-target approach geometry uniform by default. Extra advance is a
# manual experiment parameter, not part of the normal harvest policy.
LEFTMOST_EXTRA_ADVANCE_REQUEST_M = 0.000
LEFTMOST_WALL_SAFETY_MARGIN_M = -0.030
LEFTMOST_EXTRA_ADVANCE_VEL_MM_S = 20.0
GRASP_Z_BIAS = 0.000
PRE_APPROACH_OFFSET = 0.06
PRE_APPROACH_SETTLE_SEC = 0.3
FINAL_APPROACH_VEL_MM_S = 60.0
FINAL_APPROACH_ACC_MM_S2 = 120.0
DIRECT_CUROBO_FINAL_APPROACH_FOR_MEASURED_TCP = False
ENABLE_CUROBO_FINAL_APPROACH_FALLBACK = True
MEASURED_TCP_TARGET_Z_MAX_M = 1.050
MEASURED_TCP_MAX_APPROACH_M = 0.180
MEASURED_TCP_MAX_APPROACH_CEILING_M = 0.220
MEASURED_TCP_J3_GOOD_ENOUGH_DEG = 45.0
NW_HIGH_TARGET_J3_GOOD_ENOUGH_DEG = 40.0
NW_HIGH_TARGET_MIN_FLAT_BRANCH_J3_DEG = 20.0
MEASURED_TCP_MIN_PRUNE_DEPTH_M = 0.090
NW_HIGH_TARGET_PROBE_DEPTHS_M = [0.090, 0.070, 0.060]
NW_EXPERIMENTAL_MAX_APPROACH_M = 0.150
RETREAT_VEL_MM_S = 80.0
RETREAT_ACC_MM_S2 = 120.0
STRAIGHT_RETREAT_SETTLE_SEC = 0.3
NEIGHBOR_SPHERE_RADIUS_M = 0.030

CRANE_Z_OFFSET_M = 0.030
CRANE_DESCENT_VEL_MM_S = 40.0
CRANE_ASCENT_VEL_MM_S = 80.0
NW_HIGH_TARGET_Z_THRESHOLD_M = 0.750
NW_HIGH_TARGET_FINAL_EXTRA_M = 0.015
NW_HIGH_TARGET_BASE_Y_NUDGE_M = 0.000
NW_HIGH_TARGET_Y_PLANE_RELAX_M = 0.010
NW_HIGH_TARGET_CRANE_Z_OFFSET_M = 0.005
NW_HIGH_TARGET_DESCENT_EXTRA_BELOW_KP1_M = 0.000
DETACH_PULL_DOWN_MM = 40.0
DETACH_PULL_VEL_MM_S = 50.0

LEGACY_EE_TO_TCP_OFFSET_M = 0.160
MEASURED_FLANGE_TO_GRIPPER_M = 0.160
MEASURED_FLANGE_TO_PART_TIP_M = 0.270
MEASURED_FLANGE_TO_GRASP_CENTER_M = 0.260
TCP_MODEL_SHORTFALL_M = (
    MEASURED_FLANGE_TO_GRASP_CENTER_M - LEGACY_EE_TO_TCP_OFFSET_M
)
# [2026-09-09] 0.672 -> 0.810. 1차 출처는 씬 에셋이었다:
# strawberry_harvest/scenes/lab_environment.usd (df0165f) translate.y = 0.81.
# 보드는 두께 0 평면이라 translate.y 가 곧 표면 = 810.0mm.
# 이 값은 파지 목표 y 의 상한 클램프이므로 딸기(782.8mm)보다 뒤에 있어야 한다.
WALL_SURFACE_Y_M = 0.810
WALL_QUAT_WXYZ = [0.497, -0.497, 0.503, 0.503]

GRASP_QUAT_RETRY_VARIANTS = [
    ("base", [1, 0, 0], -10.0),
    ("base", [1, 0, 0], -5.0),
    ("base", [1, 0, 0], 0.0),
    ("base", [1, 0, 0], +5.0),
]
MEASURED_TCP_GRASP_QUAT_RETRY_VARIANTS = [
    ("base", [1, 0, 0], 0.0),
    ("base", [1, 0, 0], +5.0),
    ("base", [1, 0, 0], -5.0),
    ("base", [1, 0, 0], +10.0),
    ("base", [1, 0, 0], -10.0),
    ("base", [1, 0, 0], +15.0),
]
NW_HIGH_TARGET_GRASP_QUAT_RETRY_VARIANTS = [
    ("base", [1, 0, 0], +15.0),
    ("base", [1, 0, 0], 0.0),
    ("base", [1, 0, 0], +5.0),
    ("base", [1, 0, 0], -5.0),
    ("base", [1, 0, 0], +10.0),
    ("base", [1, 0, 0], -10.0),
]

CARTESIAN_PLAN_MAX_ATTEMPTS = 1
CARTESIAN_PLAN_TIMEOUT_SEC = 0.8
DIRECT_GRASP_TARGET_X_RANGE_M = (-0.45, 0.45)

GRIPPER_APPROACH_POS = 600
GRIPPER_PLACE_RELEASE_POS = 600
TAUGHT_SLOT0_ABOVE_CLEARANCE_M = 0.120
TAUGHT_SLOT0_VERTICAL_VEL_MM_S = 40.0

# RH-P12 position is not perfectly quantized at the mechanical end stop.
# Empty closes often report 697~700, while the calibrated mock stem contact
# was observed around the 670s. Keep a deadband between contact and empty.
GRASP_CONTACT_POSITION_THRESHOLD = 685
GRASP_EMPTY_POSITION_THRESHOLD = 695
GRASP_EMPTY_CURRENT_MAX_RAW = 20
GRASP_VERIFY_TIMEOUT_SEC = 5.0
GRASP_UNVERIFIED_CLOSE_RETRY = 0
GRIPPER_CLOSE_SETTLE_SEC = 0.3

HOME_JOINTS_DEG = [88.0, -80.0, 130.0, 0.0, 20.0, -90.0]
OVERVIEW_JOINTS_DEG = [87.98, -94.92, 129.89, 175.94, -31.34, 93.42]
TRAY_VIEW_JOINTS_DEG = [-1.02, 0.11, 97.09, 175.94, -31.34, 93.42]
TRAY_VIEW_POSX_MM_DEG = [
    505.56, -15.35, 423.49, 176.29, -128.45, 88.27,
]
TAUGHT_SLOT0_PLACE_REFERENCE_JOINTS_DEG = [
    4.43, 51.79, 119.38, 175.95, 80.84, 93.42,
]
TAUGHT_SLOT0_PLACE_REFERENCE_POSX_MM_DEG = [
    519.95, 52.39, 65.58, 8.43, 90.35, -87.20,
]
TAUGHT_SLOT1_PLACE_REFERENCE_POSX_MM_DEG = [
    460.24, 55.83, 66.47, 8.43, 90.36, 87.20,
]
TAUGHT_SLOT3_PLACE_REFERENCE_POSX_MM_DEG = [
    511.91, 1.83, 63.12, 8.43, 90.37, -87.20,
]
TAUGHT_TRAY_SLOT_COUNT = 15
DEFAULT_TRAY_CELLS_GLOB = os.path.expanduser(
    "~/Downloads/share_tray/output/tray_cells_*.json")

# [FIX 2026-09-08] J6 운용 한계 ±225 -> ±360 (robot.urdf 의 실제 한계와 동일).
#
# 파지에 성공하고도 트레이 이송이 `spline_jump` 로 거부돼 보드 앞에서 과실을 놓는
# 일이 반복됐다. 원인은 손목 롤의 랩(wrap) 이다: cuRobo 가 J6=273.7deg 해를 내면
# normalize_equivalents 가 ±360 등가값 중 **운용 한계 안에 있는 것**만 고를 수 있어
# −86.3deg 를 택하고, 176.6deg 에서 −86.3deg 로 가는 263deg 점프가 되어 가드가 거부한다.
# 273.7deg 를 그대로 쓸 수 있으면 연속 경로가 된다.
# robot.urdf 의 joint_6 한계는 lower/upper = ∓6.2832 rad = **∓360deg** 이므로
# 기구적으로 문제 없다. ±225 는 보수적으로 잡아둔 프로젝트 값이었다.
# J1 도 URDF 상 ±360 이지만 지금 문제되지 않으므로 건드리지 않는다.
OPERATIONAL_JOINT_LIMITS_DEG = [
    (-225.0, 225.0),
    (-95.0, 95.0),
    (-135.0, 135.0),
    (-360.0, 360.0),
    (-130.0, 130.0),
    (-360.0, 360.0),
]
WRAP_EQUIVALENT_JOINT_IDX = {3, 5}
# [FIX 2026-09-08] J1 상한 75 -> 95도.
# ripe_03(x=+350mm)이 이 가드 때문에 통째로 수확 불가였다. 실측 런:
#   Cartesian plan rejected: J1 swing 86.9deg > 75.0deg (start=119.9 -> end=206.7)
# 목표 J1=206.7도 는 OPERATIONAL_JOINT_LIMITS_DEG[0]=(-225,225) 안이라 기구적으로
# 안전하고, 막고 있던 것은 "한 픽에서 허용하는 이동량" 가드뿐이었다.
# 보드가 이제 충돌 월드에 있으므로 큰 스윙이 보드를 지나가면 계획 단계에서 거부된다.
# [REVERT 2026-09-14] J1 95 -> 75 (원본). 시뮬이 원본보다 관대하면 안 된다 — 실기 노드는
# '핵심 시퀀스의 설계와 맞지 않을 때'만 고친다(PLANNER_POLICY_v2 §0-1). 가드 상향은 그 사유가 아니다.
MAX_HARVEST_JOINT_DELTA_DEG = [75.0, 90.0, 120.0, 150.0, 130.0, 120.0]
# [FIX 2026-09-08] J3 상한 120 -> 175도.
# 파지에 성공해도 트레이 이송이 여기서 막혀 과실을 그 자리(보드 앞)에 놓아버렸다. 실측 런:
#   Cartesian plan rejected: J3 swing 162.8deg > 120.0deg (start=38.9 -> end=-123.9)
#   TAUGHT_TRAY_SLOT0_PLACE_BLOCKED: above plan failed; holding fruit
#   PICK_SEQUENCE_CONTINUE place_status=failed: released fruit here
# 목표 J3=-123.9도 는 OPERATIONAL_JOINT_LIMITS_DEG[2]=(-135,135) 안이다.
# 보드 앞(z=880mm)에서 트레이 위(z=186mm)로 가려면 큰 J3 이동이 불가피하다.
# [FIX 2026-09-10] J2 상한 100 -> 130도.
# 파지에 성공하고 분리까지 끝냈는데 트레이 이송이 여기서 막혀 과실을 보드 앞에
# 놓아버리는 일이 잦았다. 실측(보드 810mm 복원 후, 파지 자세 -> 트레이 above):
#   ripe_01 J2 114도 > 100  /  ripe_02 J2 113도 > 100   -> TAUGHT_TRAY_SLOT0_PLACE_BLOCKED
#   나머지 4개는 J2 6~22도로 여유
# 보드가 뒤로 138mm 물러나면서 위쪽(nw) 딸기를 잡을 때 팔이 더 펴져 J2 스윙이 커졌다.
# 목표 J2 는 OPERATIONAL_JOINT_LIMITS_DEG[1]=(-95,95) 안이고, 경로는 보드가 든
# 충돌 월드에서 cuRobo 가 계획하므로 스윙 상한만 풀면 된다 (J3 를 120->175 로 푼 것과 같은 건).
MAX_TAUGHT_PLACE_TRANSFER_JOINT_DELTA_DEG = [
    # [REVERT 2026-09-14] J2 130 -> 100, J3 175 -> 120 (원본). 위 J1 과 같은 사유.
    170.0, 100.0, 120.0, 150.0, 130.0, 180.0,
]

MAX_SPLINE_POINTS = 12
SPLINE_TIME_SCALE = 0.87
SPLINE_MIN_TIME = 0.58
SPLINE_VEL_DEG_S = 100.0
SPLINE_ACC_DEG_S2 = 180.0

USE_CUROBO_SELF_COLLISION = False
# cuRobo 충돌 활성거리(m). 실기 제어기는 벽이 월드에 없어 여유 0 이었다.
# 시뮬은 보드 큐보이드를 두므로 명시해야 한다 — 기본값 25mm 는 벽 앞 파지를 막는다.
COLLISION_ACTIVATION_DISTANCE_M = 0.005
DEBUG_START_COLLISION = True
