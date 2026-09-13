#!/usr/bin/env python3
"""One-time CuroboPlanner construction helpers.

Both functions here run exactly once, from CuroboPlanner.__init__, and have
no runtime behavior to encapsulate (no plan()/execute_*() calls) — so they
are kept as plain functions rather than client/executor classes. Moved
verbatim out of __init__; parameter names, defaults, and validation are
unchanged.
"""

import os

import torch
import yaml
from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig

from harvest_motion_params import (
    COLLISION_ACTIVATION_DISTANCE_M,
    DIRECT_CUROBO_FINAL_APPROACH_FOR_MEASURED_TCP,
    GRASP_Z_BIAS,
    LEFTMOST_EXTRA_ADVANCE_REQUEST_M,
    LEFTMOST_WALL_SAFETY_MARGIN_M,
    MEASURED_TCP_MAX_APPROACH_M,
    NW_HIGH_TARGET_BASE_Y_NUDGE_M,
    NW_HIGH_TARGET_CRANE_Z_OFFSET_M,
    NW_HIGH_TARGET_DESCENT_EXTRA_BELOW_KP1_M,
    NW_HIGH_TARGET_FINAL_EXTRA_M,
    NW_HIGH_TARGET_Z_THRESHOLD_M,
    TAUGHT_SLOT0_ABOVE_CLEARANCE_M,
    TAUGHT_TRAY_SLOT_COUNT,
    USE_CUROBO_SELF_COLLISION,
)


def build_curobo_motion_gen(measured_tcp_model: bool, static_cuboids) -> MotionGen:
    """Load robot/world config and construct+warm up a cuRobo MotionGen."""
    config_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config", "curobo"
    )
    if not os.path.exists(config_dir):
        from ament_index_python.packages import get_package_share_directory
        config_dir = os.path.join(
            get_package_share_directory("e0509_gripper_description"),
            "config", "curobo"
        )

    tensor_args = TensorDeviceType(device=torch.device("cuda:0"))
    robot_config_name = (
        "e0509_gripper_measured_tcp.yml"
        if measured_tcp_model
        else "e0509_gripper.yml"
    )
    with open(os.path.join(config_dir, robot_config_name), "r", encoding="utf-8") as f:
        robot_cfg_data = yaml.safe_load(f)
    robot_kin = robot_cfg_data["robot_cfg"]["kinematics"]
    robot_kin["urdf_path"] = os.path.join(config_dir, "e0509_gripper.urdf")
    robot_kin["collision_spheres"] = os.path.join(config_dir, "e0509_spheres.yml")
    robot_cfg = RobotConfig.from_dict(robot_cfg_data, tensor_args=tensor_args)
    world_cfg = WorldConfig(cuboid=static_cuboids)
    motion_gen_cfg = MotionGenConfig.load_from_robot_config(
        robot_cfg, world_cfg, tensor_args=tensor_args,
        num_trajopt_seeds=16, num_graph_seeds=16,
        collision_cache={"obb": 30, "mesh": 10, "sphere": 30},
        use_cuda_graph=False,
        self_collision_check=USE_CUROBO_SELF_COLLISION,
        self_collision_opt=USE_CUROBO_SELF_COLLISION,
        # [FIX 2026-09-10] 충돌 활성거리를 명시한다 (cuRobo 기본은 trajopt 25mm /
        # particle_ik 35~50mm).
        #
        # 실기 제어기의 cuRobo 월드는 10m 밖 더미 큐브 하나 — **벽이 없었다**
        # (_baseline/B_e0509_gripper_description/config/environment.yaml).
        # 벽 회피는 전부 WALL_SURFACE_Y_M 클램프 + pre-approach + 오프셋 사다리였다.
        # 즉 실기 플래너의 벽 여유는 0 이었고, 그래서 벽에 붙은 딸기를 잡을 수 있었다.
        #
        # 시뮬은 이송·배치 중 관통을 막으려고 보드 큐보이드를 넣었다(시뮬 측 안전망).
        # 그런데 기본 활성거리 25mm 가 같이 따라와, 오프셋 15mm 파지(손끝~벽 약 10mm)를
        # 실기에는 없던 이유로 IK_FAIL 시켰다. 실측: ripe_02/ripe_03 이 40mm 로 밀림.
        #
        # 5mm 는 실기(0)보다 엄격하고, collision_sphere_buffer 5mm 는 별도로 유지된다.
        # 딸기는 실험실 그대로 벽에 붙여 두고, 시뮬 측 여유만 실기 쪽으로 되돌린 것이다.
        collision_activation_distance=COLLISION_ACTIVATION_DISTANCE_M,
    )
    motion_gen = MotionGen(motion_gen_cfg)
    motion_gen.warmup(warmup_js_trajopt=False)
    motion_gen.detach_object_from_robot()
    return motion_gen


def declare_and_load_params(node, safe_grasp_available: bool) -> None:
    """Declare and read the place/row2/NW-high/leftmost/published-orientation
    ROS parameters, assigning them as node._x attributes (same names/defaults
    as the original __init__ body)."""
    node.declare_parameter("enable_marker_place_sequence", False)
    node.declare_parameter("execute_marker_place_release", False)
    node.declare_parameter("use_taught_slot0_place_reference", False)
    node.declare_parameter("hold_after_taught_slot0_place", True)
    # place가 실패/차단됐을 때 시퀀스 전체를 잠글지. False면 그 자리에서
    # 과실을 놓고 다음 타겟으로 계속한다.
    # [2026-09-14] 기본값 False -> True(실기 원본의 fail-closed 래치). 부수 규칙 1: 실기 기본 동작은
    # 기본값으로 보존한다. 시뮬은 run_nodes.sh 에서 false 를 명시한다 — 설계 시퀀스(PROJECT_GOAL §3
    # "익은 딸기를 모두 파지/배치")상 과실 하나의 배치 실패로 런을 멈추는 것은 설계에 어긋난다는 판단.
    node.declare_parameter("hold_on_place_failure", True)
    # [Stage2 2026-09-07] open-stem descent (crane z offset + 열린 채 하강).
    # 설계 시퀀스: 줄기 위로 crane_z_offset 만큼 올라가 수평 진입 -> 열린 조우로
    # BASE -Z 하강해 KP1 정렬 -> 닫기 -> BASE -Z 당겨 분리.
    # 종전에는 measured_tcp 프로파일에 묶여 있었는데 그 프로파일은 ee_link
    # "grasp_tcp_link" 가 URDF 에 없어 로드 자체가 안 된다. 그래서 실행에 쓰는
    # legacy_160mm 에서는 이 단계가 통째로 죽어 있었다.
    # 기본 False = 실기 legacy 동작 불변. 시뮬은 true 로 켠다.
    node.declare_parameter("enable_open_stem_descent", False)
    # [2026-09-10] 설계 시퀀스 5단계 "진입 경로의 역순(-Z)으로 이동".
    # 실기 코드(execute_detach_and_retreat / build_straight_retreat_steps)는
    # 제대로 구현돼 있지만 measured_tcp 프로파일에만 걸려 있다. legacy 는
    # reverse_distance = extra_advance(기본 0) 라 역진 스텝이 빈 리스트가 된다.
    # 기본 False = 실기 legacy 동작 불변. 시뮬은 true 로 켠다.
    node.declare_parameter("enable_straight_reverse_retreat", False)
    node.declare_parameter("initial_place_slot_index", 0)
    node.declare_parameter("taught_slot_sequence", "")
    node.declare_parameter("taught_slot_index_step", 1)
    node.declare_parameter("skip_row2_place_slots", False)
    # [2026-09-11] 티칭 격자 직교화. 기본 False = 실기 그대로(slot0/1/3 세 점이 만드는 84.26° 평행사변형,
    # 행당 z -2.5mm 기울기). True 면 같은 세 점의 **피치 크기만** 쓰고 축을 -x/-y, z 를 수평으로 둔다.
    # 시뮬은 규칙적인 계란판(수평·직사각) 위에 놓으므로 켠다. 실기 값은 건드리지 않는다.
    node.declare_parameter("orthogonalize_taught_grid", False)
    # [2026-09-11] 배치 격자 전체를 world y 로 평행이동 (m). 기본 0.0 = 실기 그대로. 시뮬은 계란판의 수평 중점을
    # 테이블 중심축(y=0)에 맞추려고 +0.0108 을 준다 (값의 출처: scene_tools/egg_carton_geom.GRID_SHIFT_Y_M).
    node.declare_parameter("taught_grid_shift_y_m", 0.0)
    # [2026-09-11, T4-3 4차] 배치 격자 두 축 피치를 이 값(m)으로 바꾼다 (정사각, 축 -x/-y, z 수평). 기본 0.0 = 실기
    # 티칭 피치 그대로. 시뮬은 0.066 — 과실 애셋 y 전폭 53.8 > 실기 행 피치 51.2 라 인접 칸에 못 들어간다.
    # 값의 출처: scene_tools/egg_carton_geom.PITCH_M (컵 격자와 같은 값이어야 한다).
    node.declare_parameter("taught_grid_pitch_override_m", 0.0)
    node.declare_parameter("allow_generated_tray_slot_release", False)
    node.declare_parameter("allow_unverified_grasp_place", False)
    node.declare_parameter("grasp_current_contact_threshold_raw", -1)
    node.declare_parameter("tray_cells_json", "")
    node.declare_parameter("marker_place_max_age_sec", 3600.0)
    node.declare_parameter("marker_place_above_clearance_m", 0.100)
    node.declare_parameter(
        "taught_slot_above_clearance_m", TAUGHT_SLOT0_ABOVE_CLEARANCE_M)
    node.declare_parameter("row2_place_pitch_tilt_deg", 15.0)
    node.declare_parameter("row2_release_correction_mm", [0.0, 0.0, 0.0])
    node.declare_parameter("row2_max_line_deviation_mm", 20.0)
    # Default demo verification is position-only. SafeGrasp/current-control is
    # kept as an explicit opt-in calibration path, not the normal pick check.
    node.declare_parameter("use_safe_grasp_action", False)
    node.declare_parameter("safe_grasp_max_current", 400)
    # Thin strawberry stems often do not create the large current spike used
    # for cans/bags. Keep the default sensitive enough to stop near the 670s
    # instead of driving all the way to the 700 target.
    node.declare_parameter("safe_grasp_current_delta_threshold", 40)
    node.declare_parameter("safe_grasp_timeout_sec", 5.0)
    node.declare_parameter(
        "direct_curobo_final_approach_for_measured_tcp",
        DIRECT_CUROBO_FINAL_APPROACH_FOR_MEASURED_TCP)
    node.declare_parameter(
        "measured_tcp_max_approach_m",
        MEASURED_TCP_MAX_APPROACH_M)
    node.declare_parameter(
        "measured_tcp_tool_line_after_curobo_fallback",
        True)
    node.declare_parameter("use_published_grasp_orientation", False)
    node.declare_parameter("published_grasp_roll_align_axis", "x")
    node.declare_parameter("published_grasp_roll_max_abs_deg", 75.0)
    node.declare_parameter("flat_grasp_only", False)
    node.declare_parameter("flat_grasp_target_plane_margin_m", 0.090)
    node.declare_parameter("pick_target_x_bias_m", 0.0)
    node.declare_parameter("pick_target_z_bias_m", GRASP_Z_BIAS)
    node.declare_parameter(
        "nw_high_target_z_threshold_m", NW_HIGH_TARGET_Z_THRESHOLD_M)
    node.declare_parameter(
        "nw_high_target_final_extra_m", NW_HIGH_TARGET_FINAL_EXTRA_M)
    node.declare_parameter(
        "nw_high_target_base_y_nudge_m", NW_HIGH_TARGET_BASE_Y_NUDGE_M)
    node.declare_parameter(
        "nw_high_target_crane_z_offset_m", NW_HIGH_TARGET_CRANE_Z_OFFSET_M)
    node.declare_parameter(
        "nw_high_target_descent_extra_below_kp1_m",
        NW_HIGH_TARGET_DESCENT_EXTRA_BELOW_KP1_M)
    node.declare_parameter("debug_dump_plan_calls", False)
    node._debug_dump_plan_calls = bool(
        node.get_parameter("debug_dump_plan_calls").value)
    node.declare_parameter(
        "leftmost_extra_advance_request_m",
        LEFTMOST_EXTRA_ADVANCE_REQUEST_M)
    node.declare_parameter(
        "leftmost_wall_safety_margin_m", LEFTMOST_WALL_SAFETY_MARGIN_M)
    node.declare_parameter("leftmost_allow_wall_model_override", False)
    node._enable_marker_place = bool(
        node.get_parameter("enable_marker_place_sequence").value)
    node._execute_marker_place_release = bool(
        node.get_parameter("execute_marker_place_release").value)
    node._use_taught_slot0_place_reference = bool(
        node.get_parameter("use_taught_slot0_place_reference").value)
    node._hold_after_taught_slot0_place = bool(
        node.get_parameter("hold_after_taught_slot0_place").value)
    node._hold_on_place_failure = bool(
        node.get_parameter("hold_on_place_failure").value)
    node._enable_open_stem_descent = bool(
        node.get_parameter("enable_open_stem_descent").value)
    node._enable_straight_reverse_retreat = bool(
        node.get_parameter("enable_straight_reverse_retreat").value)
    node._marker_place_slot_idx = int(
        node.get_parameter("initial_place_slot_index").value)
    taught_slot_sequence_raw = str(
        node.get_parameter("taught_slot_sequence").value).strip()
    node._taught_slot_sequence = []
    node._taught_slot_sequence_pos = 0
    if taught_slot_sequence_raw:
        node._taught_slot_sequence = [
            int(item.strip())
            for item in taught_slot_sequence_raw.split(",")
            if item.strip()
        ]
        for slot_index in node._taught_slot_sequence:
            if not 0 <= slot_index < TAUGHT_TRAY_SLOT_COUNT:
                raise ValueError(
                    f"taught_slot_sequence contains out-of-range slot {slot_index}; "
                    f"valid range is 0..{TAUGHT_TRAY_SLOT_COUNT - 1}")
        node._marker_place_slot_idx = node._taught_slot_sequence[0]
    node._taught_slot_index_step = max(
        1, int(node.get_parameter("taught_slot_index_step").value))
    node._skip_row2_place_slots = bool(
        node.get_parameter("skip_row2_place_slots").value)
    node._orthogonalize_taught_grid = bool(
        node.get_parameter("orthogonalize_taught_grid").value)
    node._taught_grid_shift_y_m = float(
        node.get_parameter("taught_grid_shift_y_m").value)
    if abs(node._taught_grid_shift_y_m) > 0.05:
        raise ValueError(
            f"taught_grid_shift_y_m={node._taught_grid_shift_y_m:.4f} m: 계란판 정렬용 평행이동은 수 cm 이내다 "
            "(|값| ≤ 0.05 m). 자리(m/mm) 착오가 아닌지 확인할 것")
    node._taught_grid_pitch_override_m = float(
        node.get_parameter("taught_grid_pitch_override_m").value)
    if node._taught_grid_pitch_override_m != 0.0 and not (
            0.03 <= node._taught_grid_pitch_override_m <= 0.15):
        raise ValueError(
            f"taught_grid_pitch_override_m={node._taught_grid_pitch_override_m:.4f} m: 트레이 칸 피치는 "
            "0.03~0.15 m 범위다 (0 = 실기 티칭 피치). 자리(m/mm) 착오가 아닌지 확인할 것")
    node._allow_generated_tray_slot_release = bool(
        node.get_parameter("allow_generated_tray_slot_release").value)
    if not 0 <= node._marker_place_slot_idx < TAUGHT_TRAY_SLOT_COUNT:
        raise ValueError(
            f"initial_place_slot_index must be 0..{TAUGHT_TRAY_SLOT_COUNT - 1}")
    node._allow_unverified_grasp_place = bool(
        node.get_parameter("allow_unverified_grasp_place").value)
    node._grasp_current_contact_threshold_raw = int(
        node.get_parameter("grasp_current_contact_threshold_raw").value)
    node._tray_cells_json = os.path.expanduser(
        str(node.get_parameter("tray_cells_json").value))
    node._marker_place_max_age_sec = float(
        node.get_parameter("marker_place_max_age_sec").value)
    node._marker_place_above_clearance_m = float(
        node.get_parameter("marker_place_above_clearance_m").value)
    node._taught_slot_above_clearance_m = float(
        node.get_parameter("taught_slot_above_clearance_m").value)
    node._row2_place_pitch_tilt_deg = float(
        node.get_parameter("row2_place_pitch_tilt_deg").value)
    node._row2_release_correction_mm = list(
        node.get_parameter("row2_release_correction_mm").value)
    node._row2_max_line_deviation_mm = float(
        node.get_parameter("row2_max_line_deviation_mm").value)
    node._use_safe_grasp_action = bool(
        node.get_parameter("use_safe_grasp_action").value) and safe_grasp_available
    node._safe_grasp_max_current = int(
        node.get_parameter("safe_grasp_max_current").value)
    node._safe_grasp_current_delta_threshold = int(
        node.get_parameter("safe_grasp_current_delta_threshold").value)
    node._safe_grasp_timeout_sec = float(
        node.get_parameter("safe_grasp_timeout_sec").value)
    node._direct_curobo_final_approach_for_measured_tcp = bool(
        node.get_parameter(
            "direct_curobo_final_approach_for_measured_tcp").value)
    node._measured_tcp_max_approach_m = float(
        node.get_parameter("measured_tcp_max_approach_m").value)
    node._measured_tcp_tool_line_after_curobo_fallback = bool(
        node.get_parameter(
            "measured_tcp_tool_line_after_curobo_fallback").value)
    node._use_published_grasp_orientation = bool(
        node.get_parameter("use_published_grasp_orientation").value)
    node._published_grasp_roll_align_axis = str(
        node.get_parameter("published_grasp_roll_align_axis").value
    ).strip().lower()
    if node._published_grasp_roll_align_axis not in {"x", "y"}:
        raise ValueError("published_grasp_roll_align_axis must be 'x' or 'y'")
    node._published_grasp_roll_max_abs_deg = max(
        0.0, float(
            node.get_parameter("published_grasp_roll_max_abs_deg").value))
    node._flat_grasp_only = bool(
        node.get_parameter("flat_grasp_only").value)
    node._flat_grasp_target_plane_margin_m = max(
        0.0, float(
            node.get_parameter("flat_grasp_target_plane_margin_m").value))
    node._pick_target_x_bias_m = float(
        node.get_parameter("pick_target_x_bias_m").value)
    node._pick_target_z_bias_m = float(
        node.get_parameter("pick_target_z_bias_m").value)
    node._nw_high_target_z_threshold_m = float(
        node.get_parameter("nw_high_target_z_threshold_m").value)
    node._nw_high_target_final_extra_m = max(
        0.0, float(node.get_parameter("nw_high_target_final_extra_m").value))
    node._nw_high_target_base_y_nudge_m = max(
        0.0, float(
            node.get_parameter("nw_high_target_base_y_nudge_m").value))
    node._nw_high_target_crane_z_offset_m = max(
        0.0, float(
            node.get_parameter("nw_high_target_crane_z_offset_m").value))
    node._nw_high_target_descent_extra_below_kp1_m = max(
        0.0, float(node.get_parameter(
            "nw_high_target_descent_extra_below_kp1_m").value))
    node._leftmost_extra_advance_request_m = max(
        0.0, float(node.get_parameter("leftmost_extra_advance_request_m").value))
    node._leftmost_wall_safety_margin_m = float(
        node.get_parameter("leftmost_wall_safety_margin_m").value)
    node._leftmost_allow_wall_model_override = bool(
        node.get_parameter("leftmost_allow_wall_model_override").value)
