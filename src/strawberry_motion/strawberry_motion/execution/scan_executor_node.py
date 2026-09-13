"""Fail-closed workspace scan executor for validated scan candidates.

Cell state published to /strawberry/exploration/set_cell_state:
  SCANNING      while robot is moving to the cell
  SCAN_POSE_REACHED after dwell; perception has not classified the cell yet
  PLANNING_FAIL if cuRobo or execution fails

This node never starts from joint-state arrival. Motion requires all of:
  - launch/parameter opt-in: execute_motion:=true
  - YAML flags: use_for_automated_motion=true AND collision_world_validated_for_motion=true,
    or manual_validation_mode:=true for one explicitly selected single cell
  - an explicit /strawberry/scan/start Trigger request
  - a live joint state matching the manually verified overview pose
  - one explicitly selected initial-validation cell (root/nw/root/ne/root/se/root/sw)

The collision backend uses the validated scene (RUN-20260527-012):
  robot/tool collision spheres + registered whiteboard cuboid + self_collision.
Motion remains blocked by use_for_automated_motion in the candidates YAML.
To authorize: set use_for_automated_motion=true after physical E-stop verification.

Run:
  ros2 launch strawberry_motion workspace_scan.launch.py  # preview only
Status monitoring:
  ros2 topic echo /strawberry/scan/status
"""

import threading
import time
from copy import deepcopy
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import torch
import yaml
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
import rclpy.callback_groups
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64MultiArray, Int32, String
from std_srvs.srv import Trigger

from ament_index_python.packages import get_package_share_directory
from dsr_msgs2.srv import MoveJoint, MoveSplineJoint

from curobo.geom.types import Cuboid, WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState as CuroboJointState, RobotConfig
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

from strawberry_motion.execution.cell_traversal import resolve_traversal_order
from strawberry_motion.execution.scan_transit import plan_joint_space
from strawberry_motion.execution.subcell_pose import (
    SubdivideSolver,
    WRAP_EQUIVALENT_JOINT_IDX as _SUBCELL_WRAP_IDX,
    derive_subcell_joints_deg,
    subcell_center_offset_m,
)
from strawberry_motion.execution.scan_safety import (
    joints_within_tolerance_deg,
    motion_start_allowed,
    single_cell_request_allowed,
)

_CUROBO_DIR = Path("/home/oceanshape/strawberry_grasp_environment/src/e0509_gripper_description/config/curobo")
_URDF_PATH = _CUROBO_DIR / "e0509_gripper.urdf"
_ROBOT_YML = _CUROBO_DIR / "e0509_gripper.yml"
_SPHERES_PATH = _CUROBO_DIR / "e0509_spheres.yml"
_CANDIDATES_FNAME = "scan_pose_candidates_refit_candidate.yaml"
_COLLISION_WORLD_FNAME = "scan_collision_world.yaml"
_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# [FIX 2026-09-08] 서브셀 격자는 **보드 기준 고정 좌표**다.
#
# 종전 _group_poses_by_subcell 은 탐지된 딸기들의 bbox 중점을
#   x_mid = (max(xs)+min(xs))/2 ,  z_mid = (max(zs)+min(zs))/2
# 로 매번 다시 계산했다. 그래서 딸기를 하나씩 수확·블랙리스트할 때마다 남은
# 것들의 bbox 가 줄며 **경계선이 움직였고**, 같은 딸기가 패스마다 다른 서브셀로
# 분류됐다. 보드는 로봇 원점 기준 절대 좌표에 고정돼 있으므로 격자도 거기서
# 나와야 한다. 딸기 배치와는 무관해야 한다.
#
# 값의 출처 (합성 USD 실측, /World/lab_environment/whiteboard/board):
#   x [-495.0, +595.0]mm -> 중심 +50.0mm
#   z [ 265.0, 1055.0]mm -> 중심 +660.0mm
#   (로봇 base_link 는 월드 원점, 보드는 (50, 810, 660)mm — 2026-09-09)
# 보드를 옮기면 layout_layer.usd 와 함께 이 두 값도 고쳐야 한다.
# 보드 앞면 y (m). harvest_motion_params.WALL_SURFACE_Y_M 과 **같은 값이어야 한다.**
# scripts/ 는 이 모듈의 import 경로에 없으므로 상수를 여기 둔다 (BOARD_SUBCELL_* 과 같은 이유).
BOARD_SURFACE_Y_M = 0.810
BOARD_SUBCELL_X_MID_M = 0.050
BOARD_SUBCELL_Z_MID_M = 0.660
# [FIX 2026-09-10] 분면 안을 다시 4등분할 때 쓰는 보드 외곽 (m). 위 중심선과 같은 출처.
# 종전 _group_poses_by_subcell 은 분면 안에서도 보드 전체 중심선을 써서, 탐지가 항상
# 부모와 같은 이름의 구석(root/nw/nw 등)으로 몰렸다 — 2차 분할이 퇴화해 있었다.
BOARD_X_MIN_M, BOARD_X_MAX_M = -0.495, 0.595
BOARD_Z_MIN_M, BOARD_Z_MAX_M = 0.265, 1.055
# 분면 인접 관계. 가지치기로 비인접 분면끼리 직행이 생기면 MoveJoint 폴백은 overview 를
# 경유한다 (FK 실측 2026-09-10: nw<->se 관절공간 직선 보드여유 37mm, ne->sw 스윙 234°,
# overview<->어느 분면이든 202mm 이상).
_ADJACENT_QUADRANTS = {
    frozenset(("nw", "ne")), frozenset(("ne", "se")),
    frozenset(("se", "sw")), frozenset(("sw", "nw")),
}

# Single-cell test gate — all 4 cells validated (RUN-20260527-012)
_INITIAL_SINGLE_CELL_CANDIDATES = [
    "root/nw",
    "root/nw_flat",
    "root/ne",
    "root/se",
    "root/sw",
]
# [2026-09-09] 실기 순회 순서로 되돌림.
# 종전 값 ["root/sw", "root/nw_flat", "root/ne", "root/se"] 는 데모 촬영용으로
# SW 부터 돌게 바꾼 것이었다. 실기 기록(민1 STEP 6)의 순서는 아래이고,
# overview 를 경유하지 않는 INTER_CELL_DIRECT 로 검증됐다.
_ALL_CELLS_ZORDER = ["root/nw", "root/ne", "root/se", "root/sw"]

_MAX_SPLINE_PTS = 12
_SPLINE_TIME_SCALE = 0.75
_SPLINE_MIN_TIME = 0.5
_DEFAULT_SCAN_DWELL_SEC = 12.0
_GRIPPER_APPROACH_POS = 600   # 스캔 이동 중 그리퍼 pre-close 개도 (0=완전열림, 700=완전닫힘)
_OVERVIEW_TOLERANCE_DEG = 1.0
# [SPEED 2026-09-09] 60/90 -> 120/180. 실기는 테이블 흔들림 때문에 낮췄지만
# (민1 STEP 7) 시뮬에는 그 제약이 없다. 실기 값으로 돌리려면 파라미터로 지정한다.
_DEFAULT_SCAN_MOVEJ_VEL_DEG_S = 120.0
_DEFAULT_SCAN_MOVEJ_ACC_DEG_S2 = 180.0
_DEFAULT_OVERVIEW_RETURN_VEL_DEG_S = 120.0
_DEFAULT_OVERVIEW_RETURN_ACC_DEG_S2 = 180.0
_DEFAULT_MOVEJ_SERVICE_TIMEOUT_SEC = 30.0
# [T4b 2026-09-11] 적응 분할용 IK 시드 수. 부모 관절이 시드 하나로 들어가고 나머지는 무작위 —
# 돌아온 해 전부 중 부모와 가장 가까운 것을 고른다 (subcell_pose.derive_subcell_joints_deg).
_SUBDIVIDE_IK_SEEDS = 32
# True: _init_motion_gen loads robot spheres + whiteboard cuboid + self-collision
# (validated in RUN-20260527-012). Motion is still gated by use_for_automated_motion
# in the candidates YAML, which the operator sets after physical E-stop verification.
_COLLISION_BACKEND_READY_FOR_MOTION = True

_OP_LIMITS_DEG = [
    (-225.0, 225.0),
    (-95.0,   95.0),
    (-155.0, 155.0),
    (-170.0, 170.0),
    (-130.0, 130.0),
    (-225.0, 225.0),
]

_JOINT_LIMITS_RAD = [
    (-6.273185, 6.273185),
    (-1.648063, 1.648063),
    (-2.6953,   2.6953  ),
    (-6.273185, 6.273185),
    (-2.346194, 2.346194),
    (-6.273185, 6.273185),
]
_OVERVIEW_WRAP_EQUIVALENT_JOINT_IDX = {0, 3, 5}
_MOVE_TARGET_WRAP_EQUIVALENT_JOINT_IDX = {3, 5}  # keep J1 branch explicit in taught YAML


def _as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _wrap_aware_joints_within_tolerance_deg(current_rad, target_deg, tolerance_deg):
    if not current_rad or len(current_rad) != len(target_deg):
        return False
    current_deg = np.rad2deg(current_rad).tolist()
    for idx, (cur, target) in enumerate(zip(current_deg, target_deg)):
        if idx in _OVERVIEW_WRAP_EQUIVALENT_JOINT_IDX:
            diff = min(abs((target + 360.0 * k) - cur) for k in range(-2, 3))
        else:
            diff = abs(target - cur)
        if diff > tolerance_deg:
            return False
    return True


def _mat4_to_pos_quat_wxyz(mat4: np.ndarray) -> Tuple[List[float], List[float]]:
    pos = mat4[:3, 3].tolist()
    q_xyzw = Rotation.from_matrix(mat4[:3, :3]).as_quat()
    q_wxyz = [float(q_xyzw[3]), float(q_xyzw[0]), float(q_xyzw[1]), float(q_xyzw[2])]
    return pos, q_wxyz


class ScanExecutorNode(Node):

    def _declare_and_load_params(self) -> None:
        self.declare_parameter("execute_motion", False)
        self.declare_parameter("target_cell", "")
        self.declare_parameter("manual_validation_mode", False)
        self.declare_parameter("scan_movej_vel_deg_s", _DEFAULT_SCAN_MOVEJ_VEL_DEG_S)
        self.declare_parameter("scan_movej_acc_deg_s2", _DEFAULT_SCAN_MOVEJ_ACC_DEG_S2)
        self.declare_parameter(
            "overview_return_vel_deg_s", _DEFAULT_OVERVIEW_RETURN_VEL_DEG_S
        )
        self.declare_parameter(
            "overview_return_acc_deg_s2", _DEFAULT_OVERVIEW_RETURN_ACC_DEG_S2
        )
        self.declare_parameter(
            "movej_service_timeout_sec", _DEFAULT_MOVEJ_SERVICE_TIMEOUT_SEC
        )
        self.declare_parameter("enable_pick_integration", True)
        self.declare_parameter("max_total_picks", 0)  # 0 = unlimited
        # 셀 간 이동을 cuRobo 로 계획할지. false 면 실기와 동일한 순수 MoveJoint.
        # [2026-09-10 기본값 True -> False] 09-09 에 넣은 뒤 한 번도 실행된 적이 없다:
        # 계획에 필요한 MotionGen(self._mg) 은 enable_runtime_curobo_preview 경로에서만
        # 만들어지므로 조건이 조용히 실패해 늘 MoveJoint 폴백으로 갔다 (02:02·10:13 런 모두
        # `cuRobo 경유` 0건). 필요 자체도 사라졌다 — 분면 자세를 실기 v12 티칭값으로
        # 되돌린 뒤 순차 이동의 관절공간 직선 보드여유는 최소 181mm(보드 810, FK 20쌍 전수)
        # 이고, 가지치기로 생기는 비인접 쌍은 overview 경유가 맡는다. 켜려면 스캔 시작 시
        # _init_motion_gen() 을 불러야 한다 (warmup 수십 초, 세 번째 cuRobo 인스턴스).
        self.declare_parameter("plan_scan_transit", False)
        self.declare_parameter("collect_then_pick", False)
        self.declare_parameter("collect_pick_ready_cell", "")
        self.declare_parameter("pick_complete_settle_sec", 1.0)
        self.declare_parameter("pick_timeout_sec", 60.0)
        self.declare_parameter("attempted_target_blacklist_radius_m", 0.025)
        self.declare_parameter("scan_dwell_sec", _DEFAULT_SCAN_DWELL_SEC)
        self.declare_parameter("return_to_overview_at_end", True)
        # [2026-09-10] 원안 1·2단계 복원 — overview 에서 1차 스캔 후 익은 과실이 있는
        # 분면만 순회한다(가지치기). false 면 실기 최종본과 동일한 4분면 전수 순회.
        # 판정은 pick_pose 가 아니라 scene_positions(세그 중심) 로 한다: 실기 줄기
        # 키포인트는 overview 거리에서 안정화되지 않아 pick_pose 로 자르면 전부 잘린다.
        self.declare_parameter("overview_prescan", False)
        # [T4b 2026-09-11] 적응 분할 — 분면 근거리 스캔의 중복 제거 후보가 이 수 이상이면 그 분면을
        # 2×2 로 쪼개고 **후보가 있는 세부 칸만** 부모 자세에서 유도한 세부 자세로 내려가 다시 스캔·pick.
        # 0 = 끔(기본) = 실기와 동일. 실기는 쪼갤지 여부를 사람이 오프라인에서 정해 YAML 에 세부 자세를
        # 넣었다(NW 4칸) — 런타임 판정이 없다. 세부 자세는 부모 관절 FK → x·z 만 세부 칸 중심으로
        # 평행이동(y·방향 동일) → 부모 시드 IK 로 계산한다(새 좌표 하드코딩 없음). 관절 변화가
        # subdivide_max_joint_delta_deg 를 넘거나 IK 가 실패하면 SUBDIVIDE_REJECTED 로 그 칸은
        # 부모 자세 pick(종전 동작)으로 퇴화한다. 깊이 상한 2 — 세부 칸은 다시 쪼개지 않는다.
        # 이동은 실기와 같은 MoveJoint (오프라인 FK 검사: check_subcell_scan_poses.py).
        self.declare_parameter("subdivide_min_candidates", 0)
        self.declare_parameter("subdivide_max_joint_delta_deg", 60.0)
        self.declare_parameter("enable_runtime_curobo_preview", False)
        self.declare_parameter("runtime_curobo_preview_retries", 2)
        self._execute_motion = bool(self.get_parameter("execute_motion").value)
        self._target_cell = str(self.get_parameter("target_cell").value)
        self._manual_validation_mode = bool(
            self.get_parameter("manual_validation_mode").value
        )
        self._scan_movej_vel = float(self.get_parameter("scan_movej_vel_deg_s").value)
        self._scan_movej_acc = float(self.get_parameter("scan_movej_acc_deg_s2").value)
        self._overview_return_vel = float(
            self.get_parameter("overview_return_vel_deg_s").value
        )
        self._overview_return_acc = float(
            self.get_parameter("overview_return_acc_deg_s2").value
        )
        self._movej_service_timeout_sec = float(
            self.get_parameter("movej_service_timeout_sec").value
        )
        self._enable_pick_integration = _as_bool(
            self.get_parameter("enable_pick_integration").value
        )
        self._max_total_picks = int(self.get_parameter("max_total_picks").value)
        self._plan_scan_transit = _as_bool(
            self.get_parameter("plan_scan_transit").value)
        self._collect_then_pick = _as_bool(
            self.get_parameter("collect_then_pick").value
        )
        self._collect_pick_ready_cell = str(
            self.get_parameter("collect_pick_ready_cell").value
        ).strip()
        self._pick_complete_settle_sec = max(
            0.0, float(self.get_parameter("pick_complete_settle_sec").value)
        )
        self._pick_timeout_sec = max(
            5.0, float(self.get_parameter("pick_timeout_sec").value)
        )
        self._attempted_target_blacklist_radius_m = max(
            0.0,
            float(self.get_parameter("attempted_target_blacklist_radius_m").value),
        )
        self._total_picks_attempted = 0
        self._last_cell_completed_picks = 0
        self._last_cell_skipped_attempted = 0
        self._scan_dwell_sec = max(
            1.0, float(self.get_parameter("scan_dwell_sec").value)
        )
        self._return_to_overview_at_end = bool(
            self.get_parameter("return_to_overview_at_end").value
        )
        self._overview_prescan = _as_bool(
            self.get_parameter("overview_prescan").value)
        self._subdivide_min_candidates = max(
            0, int(self.get_parameter("subdivide_min_candidates").value))
        self._subdivide_max_joint_delta_deg = max(
            1.0, float(self.get_parameter("subdivide_max_joint_delta_deg").value))
        self._runtime_curobo_preview_enabled = bool(
            self.get_parameter("enable_runtime_curobo_preview").value
        )
        self._runtime_curobo_preview_retries = int(
            self.get_parameter("runtime_curobo_preview_retries").value
        )

    def __init__(self) -> None:
        super().__init__("scan_executor_node")

        self._current_joints: Optional[List[float]] = None
        self._started = False
        self._mg: Optional[MotionGen] = None
        self._subdivide_solver: Optional[SubdivideSolver] = None   # [T4b] 적응 분할 IK
        self._detection_count: int = 0
        self._detection_poses: List[PoseStamped] = []
        self._attempted_pick_positions: List[np.ndarray] = []
        self._detection_lock = threading.Lock()
        self._scene_positions: List[np.ndarray] = []     # overview 1차 스캔 입력
        self._prev_scan_cell: Optional[str] = None        # 직전 스캔 분면 (폴백 경유 판단)
        self._pick_complete_event = threading.Event()
        self._last_movej_command_deg: Optional[List[float]] = None
        self._runtime_preview_lock = threading.Lock()
        self._declare_and_load_params()

        pkg = get_package_share_directory("strawberry_motion")
        candidates_path = Path(pkg) / "config" / _CANDIDATES_FNAME
        with candidates_path.open() as fh:
            data = yaml.safe_load(fh)
        candidate_cfg = data["scan_pose_candidates"]
        self._overview_joints_deg = [
            float(v) for v in candidate_cfg.get("curobo_start_joints_deg", [])
        ]
        if len(self._overview_joints_deg) != 6:
            raise RuntimeError(
                "%s missing valid curobo_start_joints_deg" % _CANDIDATES_FNAME
            )
        self._candidate_authorized = bool(
            candidate_cfg.get("use_for_automated_motion", False)
            and candidate_cfg.get("collision_world_validated_for_motion", False)
            and _COLLISION_BACKEND_READY_FOR_MOTION
        )
        self._targets: Dict[str, dict] = {
            t["cell_id"]: t
            for t in candidate_cfg["targets"]
            if t.get("tcp_transform_base") is not None
        }
        self.get_logger().info(
            "Loaded %d scan targets from %s" % (len(self._targets), candidates_path)
        )

        if not self._candidate_authorized:
            self.get_logger().warn(
                "Motion locked: set use_for_automated_motion=true in %s "
                "after physical E-stop verification." % _CANDIDATES_FNAME
            )
        if self._manual_validation_mode:
            self.get_logger().warn(
                "manual_validation_mode=true: single-cell MoveJoint validation is "
                "allowed, but target_cell=all remains blocked unless YAML is authorized."
            )
        self.get_logger().info(
            "MoveJoint speeds: scan vel=%.1f acc=%.1f, overview return vel=%.1f acc=%.1f"
            % (
                self._scan_movej_vel,
                self._scan_movej_acc,
                self._overview_return_vel,
                self._overview_return_acc,
            )
        )
        self.get_logger().info(
            "MoveJoint service dispatch timeout: %.1fs; arrival is verified from /joint_states"
            % self._movej_service_timeout_sec
        )
        if self._collect_then_pick:
            self.get_logger().warn(
                "collect_then_pick=true: scan poses only collect targets; "
                "pick is triggered after moving to collect_pick_ready_cell or parent cell."
            )
        if self._runtime_curobo_preview_enabled:
            self.get_logger().warn(
                "Runtime cuRobo preview enabled: plans are logged only; "
                "execution still uses verified YAML MoveJoint poses."
            )

        if self._subdivide_min_candidates > 0:
            self._init_subdivide_solver()

        cb = rclpy.callback_groups.ReentrantCallbackGroup()
        self.create_subscription(JointState, "/dsr01/joint_states", self._joint_cb, 10)
        # YOLO detection input — publishers use /strawberry/detection/pick_pose
        # scan_executor gates delivery to curobo_planner one pose at a time
        self.create_subscription(
            PoseStamped, "/strawberry/detection/pick_pose", self._pick_cb, 10
        )
        # overview 1차 스캔용 — 익은 과실 중심 좌표 (세그 채널). overview_prescan 일 때만 소비.
        self.create_subscription(
            Float64MultiArray, "/strawberry/detection/scene_positions",
            self._scene_cb, 10
        )
        self.create_subscription(
            Empty, "/dsr01/curobo/pick_complete", self._pick_complete_cb, 10
        )
        self._pick_trigger_pub = self.create_publisher(
            PoseStamped, "/dsr01/curobo/pick_pose", 10
        )
        self._gripper_pos_pub = self.create_publisher(
            Int32, "/dsr01/gripper/position_cmd", 10
        )
        self._state_pub = self.create_publisher(
            String, "/strawberry/exploration/set_cell_state", 10
        )
        self._status_pub = self.create_publisher(String, "/strawberry/scan/status", 10)
        self.create_service(Trigger, "/strawberry/scan/start", self._start_cb, callback_group=cb)
        self._cli_spline = self.create_client(
            MoveSplineJoint, "/dsr01/motion/move_spline_joint", callback_group=cb
        )
        self._cli_movej = self.create_client(
            MoveJoint, "/dsr01/motion/move_joint", callback_group=cb
        )

        self.get_logger().info(
            "scan_executor_node ready; explicit /strawberry/scan/start required"
        )

        try:    # ── HUD 계측 (제거: 이 4줄만 지우면 된다) ──
            import sys, os; sys.path.append(os.path.expanduser(os.environ.get(
                "HARVEST_HUD_DIR", "~/strawberry_grasp_environment/strawberry_harvest/scripts/hud")))
            import harvest_probe; harvest_probe.attach("scan", self)
        except Exception: pass

    def _init_motion_gen(self) -> None:
        if self._mg is not None:
            return
        self.get_logger().info(
            "Initialising cuRobo MotionGen (spheres + whiteboard + self-collision)"
        )
        tensor_args = TensorDeviceType(device=torch.device("cuda:0"))

        with _ROBOT_YML.open() as fh:
            robot_data = deepcopy(yaml.safe_load(fh))
        kine = robot_data["robot_cfg"]["kinematics"]
        kine["urdf_path"] = str(_URDF_PATH)
        kine["collision_spheres"] = str(_SPHERES_PATH)
        robot_cfg = RobotConfig.from_dict(robot_data, tensor_args=tensor_args)

        pkg = get_package_share_directory("strawberry_motion")
        world_yaml = Path(pkg) / "config" / _COLLISION_WORLD_FNAME
        with world_yaml.open() as fh:
            world_meta = yaml.safe_load(fh)["scan_collision_world"]
        cuboids = [
            Cuboid(
                name=o["name"],
                pose=[float(v) for v in o["pose_wxyz"]],
                dims=[float(v) for v in o["dims_m"]],
            )
            for o in world_meta["objects"]
            if o.get("enabled", True) and o.get("type") == "cuboid"
        ]
        world_cfg = WorldConfig(cuboid=cuboids)

        mg_cfg = MotionGenConfig.load_from_robot_config(
            robot_cfg, world_cfg, tensor_args=tensor_args,
            num_trajopt_seeds=16, num_graph_seeds=16,
            collision_cache={"obb": 30, "mesh": 10},
            use_cuda_graph=False,
            self_collision_check=True,
            self_collision_opt=True,
        )
        self._mg = MotionGen(mg_cfg)
        self._mg.warmup(warmup_js_trajopt=False)
        self._mg.detach_object_from_robot()
        self.get_logger().info("cuRobo MotionGen ready")

    def _init_subdivide_solver(self) -> None:
        """[T4b] 적응 분할용 cuRobo IK 솔버 (보드 큐보이드 + 자기충돌). subdivide_min_candidates>0 일 때만.

        MotionGen(_init_motion_gen) 이 아니라 IKSolver 만 만든다 — 계획이 아니라 FK/IK 만 필요하고,
        세부 자세 이동은 실기와 같은 MoveJoint 다. 초기화가 실패하면 분할을 끄고 종전 동작으로 간다.
        """
        pkg = get_package_share_directory("strawberry_motion")
        world_yaml = Path(pkg) / "config" / _COLLISION_WORLD_FNAME
        t0 = time.time()
        try:
            self._subdivide_solver = SubdivideSolver(
                _ROBOT_YML, _URDF_PATH, _SPHERES_PATH, world_yaml,
                num_seeds=_SUBDIVIDE_IK_SEEDS)
        except Exception as exc:
            self._subdivide_solver = None
            self._subdivide_min_candidates = 0
            self.get_logger().error(
                "SUBDIVIDE_DISABLED IK solver init failed: %r — 적응 분할 없이(부모 자세 pick) 진행"
                % (exc,))
            return
        self.get_logger().info(
            "SUBDIVIDE_IK_READY min_candidates=%d max_joint_delta=%.0fdeg seeds=%d init=%.1fs"
            % (self._subdivide_min_candidates, self._subdivide_max_joint_delta_deg,
               _SUBDIVIDE_IK_SEEDS, time.time() - t0))

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _joint_cb(self, msg: JointState) -> None:
        jmap = {n: p for n, p in zip(msg.name, msg.position)}
        joints = [jmap.get(n) for n in _JOINT_NAMES]
        if None not in joints:
            self._current_joints = joints

    def _pick_cb(self, msg: PoseStamped) -> None:
        with self._detection_lock:
            self._detection_count += 1
            self._detection_poses.append(msg)

    def _scene_cb(self, msg: Float64MultiArray) -> None:
        data = list(msg.data)
        pts = [np.array(data[i:i + 3], dtype=float)
               for i in range(0, len(data) - len(data) % 3, 3)]
        with self._detection_lock:
            self._scene_positions = pts

    def _pick_complete_cb(self, _msg: Empty) -> None:
        self._pick_complete_event.set()

    def _start_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        allowed, reason = motion_start_allowed(
            execute_motion=self._execute_motion,
            candidate_authorized=self._candidate_authorized,
            has_joint_state=self._current_joints is not None,
            manual_validation_mode=(
                self._manual_validation_mode and self._target_cell != "all"
            ),
        )
        if self._started:
            allowed, reason = False, "scan already started"
        if allowed:
            if self._target_cell == "all":
                # 4-cell traversal mode: bypasses single-cell gate
                reason = "traversal mode all cells accepted"
            else:
                allowed, reason = single_cell_request_allowed(
                    self._target_cell, _INITIAL_SINGLE_CELL_CANDIDATES
                )
        if allowed and not _wrap_aware_joints_within_tolerance_deg(
            self._current_joints or [], self._overview_joints_deg, _OVERVIEW_TOLERANCE_DEG
        ):
            allowed = False
            reason = "current joints do not match verified overview pose within 1.0 deg"
        response.success = allowed
        response.message = reason
        if not allowed:
            self._pub_status("START_REJECTED " + reason)
            return response
        self._started = True
        self._pub_status("START_ACCEPTED explicit request; initial pose verified")
        threading.Thread(target=self._scan_sequence_run, daemon=True).start()
        return response

    def _scan_sequence_run(self) -> None:
        # [PLANNER-FIX #002] _started는 스캔 중복 시작을 막는 진행중 플래그인데
        # 시퀀스가 끝나도 해제되지 않아 프로세스당 단 1회만 스캔이 가능했다.
        # 실기에서도 수확 1사이클 후 재트리거가 "scan already started"로 거부되어
        # 노드를 재시작해야 하므로 실기 결함이다. 종료 시 해제한다.
        # 시작 게이트 6단계는 매 트리거마다 그대로 평가되므로 완화되는 안전 조건은 없다.
        try:
            self._scan_sequence()
        finally:
            self._started = False
            self._pub_status("READY_FOR_NEXT_START")

    # ── helpers ───────────────────────────────────────────────────────────────

    def _pub_state(self, cell_id: str, state: str) -> None:
        msg = String()
        msg.data = "%s=%s" % (cell_id, state)
        self._state_pub.publish(msg)

    def _pub_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(text)

    def _traj_ok(self, traj: np.ndarray, label: str) -> bool:
        deg = np.rad2deg(traj)
        for i, (lo, hi) in enumerate(_OP_LIMITS_DEG):
            vmin, vmax = float(np.min(deg[:, i])), float(np.max(deg[:, i]))
            if vmin < lo or vmax > hi:
                self.get_logger().warn(
                    "%s J%d %.1f~%.1f° outside op limits %.1f~%.1f°"
                    % (label, i + 1, vmin, vmax, lo, hi)
                )
                return False
        return True

    def _plan(
        self, start_joints: List[float], pos: List[float], quat_wxyz: List[float], label: str,
        max_retries: int = 5,
    ) -> Optional[Tuple[np.ndarray, float]]:
        start = CuroboJointState.from_position(
            position=torch.tensor(
                [start_joints], device="cuda:0", dtype=torch.float32
            ),
            joint_names=_JOINT_NAMES,
        )
        goal = Pose(
            position=torch.tensor([pos], device="cuda:0", dtype=torch.float32),
            quaternion=torch.tensor([quat_wxyz], device="cuda:0", dtype=torch.float32),
        )
        if self._mg is None:
            self.get_logger().error("MotionGen unavailable")
            return None
        for attempt in range(max_retries):
            result = self._mg.plan_single(
                start, goal, MotionGenPlanConfig(enable_graph=True, max_attempts=4)
            )
            if not result.success.item():
                self.get_logger().warn(
                    "%s plan attempt %d/%d failed: %s"
                    % (label, attempt + 1, max_retries, getattr(result, "status", "?"))
                )
                continue
            traj = result.get_interpolated_plan().position.cpu().numpy()
            if not self._traj_ok(traj, label):
                self.get_logger().warn(
                    "%s traj limits violated on attempt %d/%d — retrying"
                    % (label, attempt + 1, max_retries)
                )
                continue
            endpoint_rad = traj[-1].tolist()
            endpoint_deg = [round(float(np.rad2deg(j)), 1) for j in endpoint_rad]
            motion_time = float(result.motion_time.item())
            self.get_logger().info(
                "%s plan endpoint_deg=[%s]  curobo_time=%.2fs  attempt=%d"
                % (label, " ".join("%.1f" % d for d in endpoint_deg), motion_time, attempt + 1)
            )
            return traj, motion_time, endpoint_rad
        self.get_logger().error("%s plan failed after %d attempts" % (label, max_retries))
        return None

    def _exec_spline(
        self, traj_rad: np.ndarray, vel: float = 120.0, min_time: float = 3.0
    ) -> bool:
        if not self._cli_spline.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveSplineJoint service not available")
            return False
        deg = np.rad2deg(traj_rad)
        n = deg.shape[0]
        if n > _MAX_SPLINE_PTS:
            idx = np.linspace(0, n - 1, _MAX_SPLINE_PTS, dtype=int)
            deg = deg[idx]
            n = _MAX_SPLINE_PTS
        # Skip the first waypoint (current/start position).
        # MoveSplineJoint moves from the robot's current position through the
        # given via-points. Including the start as waypoint[0] causes a
        # near-zero first segment that Doosan silently rejects when the robot's
        # actual joints don't perfectly match the planned start.
        deg = deg[1:]
        n = len(deg)
        req = MoveSplineJoint.Request()
        req.pos_cnt = n
        for row in deg:
            pt = Float64MultiArray()
            pt.data = row.tolist()
            req.pos.append(pt)
        req.vel = [float(vel)] * 6
        req.acc = [float(vel) * 1.5] * 6
        # Compute minimum feasible time from actual trajectory arc length.
        # cuRobo plans aggressively (often < 1 s) but Doosan rejects if
        # req.time < max_joint_arc / vel.  Use 1.5x safety margin, 3 s min.
        path_lengths = np.sum(np.abs(np.diff(np.rad2deg(traj_rad), axis=0)), axis=0)
        req.time = float(max(np.max(path_lengths) / vel * 1.5, min_time))
        req.mode = 0
        req.sync_type = 0
        future = self._cli_spline.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 60.0:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().error("Spline future timed out after 60s")
            return False
        resp = future.result()
        if resp is None:
            self.get_logger().error("Spline future result is None")
            return False
        self.get_logger().info(
            "Spline response: success=%s  msg=%r  pos_cnt=%d  req_time=%.2fs"
            % (resp.success, getattr(resp, "msg", "N/A"), n, req.time)
        )
        if not resp.success:
            self.get_logger().error("MoveSplineJoint returned success=False")
        return bool(resp.success)

    def _wait_for_joints(
        self, target_rad: List[float], tolerance_deg: float, timeout_sec: float
    ) -> bool:
        deadline = time.time() + timeout_sec
        target_deg = np.rad2deg(target_rad).tolist()
        while time.time() < deadline:
            if self._current_joints and joints_within_tolerance_deg(
                self._current_joints, target_deg, tolerance_deg
            ):
                return True
            time.sleep(0.1)
        return False

    def _movej(self, joints_deg: List[float], vel: float = 40.0, acc: float = 40.0) -> bool:
        if not self._cli_movej.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveJoint service not available")
            return False
        joints_deg = self._shortest_equivalent_joints(joints_deg)
        self._last_movej_command_deg = list(joints_deg)
        req = MoveJoint.Request()
        req.pos = [float(v) for v in joints_deg]
        req.vel = vel
        req.acc = acc
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0
        future = self._cli_movej.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < self._movej_service_timeout_sec:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().warn(
                "MoveJoint service response not returned within %.1fs; treating command as dispatched and verifying arrival from /joint_states"
                % self._movej_service_timeout_sec
            )
            return True
        ok = future.done() and future.result() and future.result().success
        self.get_logger().info(
            "MoveJoint response: success=%s  target=[%s]  vel=%.1f  acc=%.1f"
            % (
                ok,
                " ".join("%.1f" % v for v in joints_deg),
                vel,
                acc,
            )
        )
        if not ok:
            self.get_logger().error("MoveJoint failed")
        return bool(ok)

    def _shortest_equivalent_joints(
        self,
        target_deg: List[float],
        reference_joints_rad: Optional[List[float]] = None,
        log_rewrite: bool = True,
    ) -> List[float]:
        """Choose the nearest equivalent angle for wrap-capable joints.

        DART may record a valid pose as e.g. J4=-238 deg while the same physical
        wrist pose is J4=+121 deg. Sending the far representation to MoveJoint
        makes the robot take a visually unnecessary long rotation. Keep the
        taught pose, but rewrite J1/J4/J6 to the 360-deg equivalent closest to
        the current measured joint state and inside the hard robot limits.
        """
        if reference_joints_rad is not None:
            current_deg = np.rad2deg(reference_joints_rad).tolist()
        elif self._current_joints is not None:
            current_deg = np.rad2deg(self._current_joints).tolist()
        elif self._last_movej_command_deg is not None:
            current_deg = list(self._last_movej_command_deg)
        else:
            return [float(v) for v in target_deg]

        adjusted = [float(v) for v in target_deg]
        for idx in _MOVE_TARGET_WRAP_EQUIVALENT_JOINT_IDX:
            lo = float(np.rad2deg(_JOINT_LIMITS_RAD[idx][0]))
            hi = float(np.rad2deg(_JOINT_LIMITS_RAD[idx][1]))
            base = adjusted[idx]
            candidates = [base + 360.0 * k for k in range(-2, 3)]
            candidates = [c for c in candidates if lo <= c <= hi]
            if not candidates:
                continue
            best = min(candidates, key=lambda c: abs(c - current_deg[idx]))
            if log_rewrite and abs(best - base) > 1e-6:
                self.get_logger().info(
                    "Joint equivalent rewrite J%d %.1f -> %.1f deg "
                    "(current %.1f, shorter by %.1f deg)"
                    % (
                        idx + 1,
                        base,
                        best,
                        current_deg[idx],
                        abs(base - current_deg[idx]) - abs(best - current_deg[idx]),
                    )
                )
            adjusted[idx] = best
        return adjusted

    @staticmethod
    def _deduplicate_poses(
        poses: List[PoseStamped], min_dist_m: float = 0.030
    ) -> List[PoseStamped]:
        """Remove poses within min_dist_m of an already-kept pose, then sort
        left-to-right / top-to-bottom in wall frame (X asc, Z desc)."""
        kept: List[PoseStamped] = []
        for p in poses:
            pos = np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z])
            if all(
                np.linalg.norm(
                    pos - np.array([k.pose.position.x, k.pose.position.y, k.pose.position.z])
                ) > min_dist_m
                for k in kept
            ):
                kept.append(p)
        # Sort into 4 quadrants, but prefer lower stem-level candidates first.
        # The NW occlusion run often sees high leaf/top candidates before the
        # actual KP1/stem target; with max_total_picks=1 that wastes the run.
        if len(kept) > 1:
            xs = [p.pose.position.x for p in kept]
            zs = [p.pose.position.z for p in kept]
            x_mid = (max(xs) + min(xs)) / 2.0
            z_mid = (max(zs) + min(zs)) / 2.0

            def _quadrant(p):
                x, z = p.pose.position.x, p.pose.position.z
                if z < z_mid and x <= x_mid:
                    return 0  # SW: lower-left, usually stem-level first
                if z < z_mid and x > x_mid:
                    return 1  # SE
                if z >= z_mid and x <= x_mid:
                    return 2  # NW
                return 3      # NE

            kept.sort(key=lambda p: (_quadrant(p), p.pose.position.x, p.pose.position.z))
        return kept

    @staticmethod
    def _pose_position_array(pose: PoseStamped) -> np.ndarray:
        return np.array(
            [
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            ],
            dtype=float,
        )

    def _pose_is_attempted(self, pose: PoseStamped) -> bool:
        if self._attempted_target_blacklist_radius_m <= 0.0:
            return False
        pos = self._pose_position_array(pose)
        return any(
            float(np.linalg.norm(pos - attempted))
            <= self._attempted_target_blacklist_radius_m
            for attempted in self._attempted_pick_positions
        )

    def _mark_pose_attempted(self, pose: PoseStamped) -> None:
        if self._attempted_target_blacklist_radius_m <= 0.0:
            return
        pos = self._pose_position_array(pose)
        self._attempted_pick_positions.append(pos)
        self._pub_status(
            "PICK_ATTEMPT_BLACKLIST_ADD pos=(%.0f,%.0f,%.0f)mm radius=%.0fmm"
            % (
                pos[0] * 1000,
                pos[1] * 1000,
                pos[2] * 1000,
                self._attempted_target_blacklist_radius_m * 1000,
            )
        )

    @staticmethod
    def _quadrant_name(cell_id: Optional[str]) -> Optional[str]:
        parts = (cell_id or "").split("/")
        if len(parts) >= 2 and parts[1][:2] in ("nw", "ne", "se", "sw"):
            return parts[1][:2]
        return None

    @staticmethod
    def _quadrant_bounds(cell_id: Optional[str]):
        """cell_id 의 분면 외곽 (x0, x1, z0, z1). 분면이 아니면 보드 전체."""
        quad = None
        parts = (cell_id or "").split("/")
        if len(parts) >= 2:
            quad = parts[1][:2]
        xm, zm = BOARD_SUBCELL_X_MID_M, BOARD_SUBCELL_Z_MID_M
        if quad == "nw":
            return BOARD_X_MIN_M, xm, zm, BOARD_Z_MAX_M
        if quad == "ne":
            return xm, BOARD_X_MAX_M, zm, BOARD_Z_MAX_M
        if quad == "se":
            return xm, BOARD_X_MAX_M, BOARD_Z_MIN_M, zm
        if quad == "sw":
            return BOARD_X_MIN_M, xm, BOARD_Z_MIN_M, zm
        return BOARD_X_MIN_M, BOARD_X_MAX_M, BOARD_Z_MIN_M, BOARD_Z_MAX_M

    @staticmethod
    def _subcell_of_pose(pose: PoseStamped, parent_cell: Optional[str]) -> str:
        """탐지 좌표가 부모 분면 안에서 어느 세부 칸(nw/ne/se/sw)인지. 중심선은 부모 분면 경계.

        _group_poses_by_subcell 과 [T4b] 세부 자세 재스캔 필터가 같은 함수를 쓴다 — 단일 출처.
        """
        x0, x1, z0, z1 = ScanExecutorNode._quadrant_bounds(parent_cell)
        x_mid = (x0 + x1) / 2.0
        z_mid = (z0 + z1) / 2.0
        x, z = pose.pose.position.x, pose.pose.position.z
        if z >= z_mid and x <= x_mid:
            return "nw"
        if z >= z_mid and x > x_mid:
            return "ne"
        if z < z_mid and x > x_mid:
            return "se"
        return "sw"

    @staticmethod
    def _group_poses_by_subcell(
        poses: List[PoseStamped], parent_cell: Optional[str] = None,
    ) -> List[Tuple[str, List[PoseStamped]]]:
        """Split detections inside the current scan cell into a logical 2x2 order.

        The robot currently has one taught scan pose per root cell.  This helper
        does not move the robot to four new sub-poses; it partitions the
        detections from that single view.  Harvesting proceeds lower-first:
        parent/sw -> parent/se -> parent/nw -> parent/ne.  In practice the
        lower detections are usually closer to the stem/KP1 level, while upper
        detections are more likely to be leaves, calyx, or hard branch cases.

        In collect_then_pick mode this helper is bypassed. The executor first
        scans all physical sub-poses, then moves to the parent pick-ready pose.
        """
        if not poses:
            return [(subcell, []) for subcell in ("sw", "se", "nw", "ne")]

        # [FIX 2026-09-08] 보드 고정 격자. 탐지 결과에 의존하지 않는다.
        # 종전에는 탐지 bbox 중점을 썼고 (그리고 1개일 때는 무조건 sw 로 몰았다),
        # 수확이 진행돼 남은 딸기가 줄면 경계가 움직여 같은 딸기가 패스마다
        # 다른 서브셀로 분류됐다.
        # [FIX 2026-09-10] 중심선은 **부모 분면**의 중심이다. 보드 전체 중심선을 쓰면
        # 분면 안의 탐지가 전부 한 구석으로 몰려 2차 분할이 의미를 잃는다.
        groups: Dict[str, List[PoseStamped]] = {"nw": [], "ne": [], "se": [], "sw": []}
        for pose in poses:
            groups[ScanExecutorNode._subcell_of_pose(pose, parent_cell)].append(pose)

        ordered: List[Tuple[str, List[PoseStamped]]] = []
        for subcell in ("sw", "se", "nw", "ne"):
            subposes = groups[subcell]
            subposes.sort(key=lambda p: (p.pose.position.x, p.pose.position.z))
            ordered.append((subcell, subposes))
        return ordered

    @staticmethod
    def _target_position_from_config(target: dict) -> Optional[np.ndarray]:
        mat_rows = target.get("tcp_transform_base") if target else None
        if mat_rows is None:
            return None
        try:
            mat4 = np.array(mat_rows, dtype=float)
            if mat4.shape == (3, 4):
                mat4 = np.vstack([mat4, [0.0, 0.0, 0.0, 1.0]])
            return mat4[:3, 3].astype(float)
        except Exception:
            return None

    def _rank_poses_for_pick_ready(
        self, poses: List[PoseStamped], ready_target: Optional[dict]
    ) -> List[PoseStamped]:
        """Rank collected candidates from the pose that will actually pick.

        The old lower-left-first ordering was useful to avoid high leaf targets,
        but in collect-then-pick it can pick a far-left strawberry that is hard
        to reach from the central NW pick-ready branch. Prefer candidates whose
        X/Z position is closest to the pick-ready TCP center, then prefer lower
        stem-level Z as a tie-breaker.
        """
        if len(poses) <= 1:
            return poses
        ready_pos = self._target_position_from_config(ready_target)
        if ready_pos is None:
            return poses

        def _score(p: PoseStamped) -> Tuple[float, float, float]:
            dx = p.pose.position.x - float(ready_pos[0])
            dz = p.pose.position.z - float(ready_pos[2])
            xz_dist = float(np.hypot(dx, dz))
            # Keep Y as a weak tie-breaker only. Perception Y can drift and the
            # planner clamps to the wall surface, so do not over-weight it.
            # [FIX 2026-09-09] 보드 y 하드코딩 제거. 672 로 굳어 있어서 보드를
            # 810 으로 옮긴 뒤에도 옛 평면 기준으로 순위를 매겼다.
            y_offset = abs(float(p.pose.position.y) - BOARD_SURFACE_Y_M)
            return (xz_dist, y_offset, p.pose.position.x)

        ranked = sorted(poses, key=_score)
        summary = "  ".join(
            "(%.0f,%.0f,%.0f)mm score=%.0f"
            % (
                p.pose.position.x * 1000,
                p.pose.position.y * 1000,
                p.pose.position.z * 1000,
                _score(p)[0] * 1000,
            )
            for p in ranked[:5]
        )
        self._pub_status(
            "COLLECT_PICK_READY_RANK center=(%.0f,%.0f,%.0f)mm %s"
            % (ready_pos[0] * 1000, ready_pos[1] * 1000, ready_pos[2] * 1000, summary)
        )
        return ranked

    def _wait_for_planner(self, timeout_sec: float = 60.0) -> bool:
        """Block until curobo_planner_node has subscribed to /dsr01/curobo/pick_pose."""
        deadline = time.time() + timeout_sec
        warned = False
        while time.time() < deadline:
            if self._pick_trigger_pub.get_subscription_count() > 0:
                return True
            if not warned:
                self.get_logger().info(
                    "Waiting for curobo_planner to subscribe to pick_pose topic …"
                )
                warned = True
            time.sleep(0.5)
        self.get_logger().error(
            "curobo_planner did not subscribe within %.0fs — picks will be skipped" % timeout_sec
        )
        return False

    def _trigger_picks_for_cell(
        self,
        cell_id: str,
        poses: List[PoseStamped],
        pick_timeout_sec: float = 60.0,
        poses_are_ranked: bool = False,
    ) -> int:
        """Publish poses to curobo_planner one at a time; return attempted count.

        The planner currently reports completion on an Empty topic, so this
        executor cannot know whether the pick actually succeeded. Treat a
        completed/timeout cycle as one consumed attempt and blacklist the pose
        so a rescan can move on to the remaining strawberries instead of
        re-trying the same failed coordinate.
        """
        self._last_cell_skipped_attempted = 0
        unique_all = list(poses) if poses_are_ranked else self._deduplicate_poses(poses)
        unique: List[PoseStamped] = []
        skipped_attempted = 0
        for pose in unique_all:
            if self._pose_is_attempted(pose):
                skipped_attempted += 1
                self._pub_status(
                    "PICK_SKIP_ATTEMPTED %s pos=(%.0f,%.0f,%.0f)mm"
                    % (
                        cell_id,
                        pose.pose.position.x * 1000,
                        pose.pose.position.y * 1000,
                        pose.pose.position.z * 1000,
                    )
                )
                continue
            unique.append(pose)
        self._last_cell_skipped_attempted = skipped_attempted
        self._pub_status(
            "PICK_SEQUENCE_START %s — %d candidate targets (raw=%d skipped_attempted=%d)"
            % (cell_id, len(unique), len(poses), skipped_attempted)
        )
        self._pub_status(
            "PICK_ALL_DETECTED_TARGETS %s timeout=%.0fs max_total_picks=%d"
            % (cell_id, pick_timeout_sec, self._max_total_picks)
        )
        if poses_are_ranked and unique:
            self._pub_status(
                "PICK_SEQUENCE_USING_RANKED_ORDER %s first=(%.0f,%.0f,%.0f)mm"
                % (
                    cell_id,
                    unique[0].pose.position.x * 1000,
                    unique[0].pose.position.y * 1000,
                    unique[0].pose.position.z * 1000,
                )
            )
        if not self._wait_for_planner():
            return 0
        attempted = 0
        for i, pose in enumerate(unique):
            if self._max_total_picks > 0 and self._total_picks_attempted >= self._max_total_picks:
                self._pub_status(
                    "PICK_LIMIT_REACHED max_total_picks=%d — skipping remaining targets"
                    % self._max_total_picks
                )
                break
            self._total_picks_attempted += 1
            attempted += 1
            self._mark_pose_attempted(pose)
            self._pick_complete_event.clear()
            self._pub_status(
                "PICK_TRIGGER %s %d/%d pos=(%.0f,%.0f,%.0f)mm"
                % (
                    cell_id, i + 1, len(unique),
                    pose.pose.position.x * 1000,
                    pose.pose.position.y * 1000,
                    pose.pose.position.z * 1000,
                )
            )
            self._pick_trigger_pub.publish(pose)
            completed = self._pick_complete_event.wait(timeout=pick_timeout_sec)
            if not completed:
                # One retry — guards against the first-publish race during DDS discovery
                self.get_logger().warn(
                    "PICK_TIMEOUT %s %d/%d — retrying once" % (cell_id, i + 1, len(unique))
                )
                self._pick_complete_event.clear()
                self._pick_trigger_pub.publish(pose)
                completed = self._pick_complete_event.wait(timeout=pick_timeout_sec)
            if completed:
                self._pub_status(
                    "PICK_ATTEMPT_DONE %s %d/%d — planner sequence ended "
                    "(success is verified by gripper/manual KPI, not this Empty topic)"
                    % (cell_id, i + 1, len(unique))
                )
                if self._pick_complete_settle_sec > 0.0:
                    self._pub_status(
                        "PICK_ATTEMPT_SETTLE %.1fs before next target"
                        % self._pick_complete_settle_sec
                    )
                    time.sleep(self._pick_complete_settle_sec)
            else:
                self._pub_status(
                    "PICK_TIMEOUT %s %d/%d — %.0fs elapsed; continuing"
                    % (cell_id, i + 1, len(unique), pick_timeout_sec)
                )
        self._pub_status(
            "PICK_SEQUENCE_DONE %s — %d/%d attempted" % (cell_id, attempted, len(unique))
        )
        return attempted

    def _is_at_overview(self) -> bool:
        return self._current_joints is not None and _wrap_aware_joints_within_tolerance_deg(
            self._current_joints, self._overview_joints_deg, _OVERVIEW_TOLERANCE_DEG
        )

    def _wait_at_overview(self, timeout_sec: float = 10.0) -> bool:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self._is_at_overview():
                return True
            time.sleep(0.05)
        return False

    def _start_runtime_curobo_preview(self, cell_id: str, target: dict) -> None:
        """Start a non-blocking runtime cuRobo preview.

        Preview is diagnostics only. Never let GPU planning delay the verified
        YAML MoveJoint scan sequence.
        """
        if self._current_joints is None:
            self._pub_status("CUROBO_PREVIEW_SKIPPED %s no current joint state" % cell_id)
            return
        if not self._runtime_preview_lock.acquire(blocking=False):
            self._pub_status(
                "CUROBO_PREVIEW_SKIPPED %s previous preview still running" % cell_id
            )
            return

        start_joints = list(self._current_joints)
        target_snapshot = deepcopy(target)

        def worker() -> None:
            try:
                self._preview_runtime_curobo_plan(cell_id, target_snapshot, start_joints)
            finally:
                self._runtime_preview_lock.release()

        threading.Thread(
            target=worker,
            name="curobo_preview_%s" % cell_id.replace("/", "_"),
            daemon=True,
        ).start()

    def _preview_runtime_curobo_plan(
        self, cell_id: str, target: dict, start_joints: List[float]
    ) -> None:
        """Log a runtime cuRobo plan from a captured start state without executing it."""
        mat_rows = target.get("tcp_transform_base")
        if mat_rows is None:
            self._pub_status("CUROBO_PREVIEW_SKIPPED %s missing tcp_transform_base" % cell_id)
            return

        try:
            mat4 = np.array(mat_rows, dtype=float)
            if mat4.shape == (3, 4):
                mat4 = np.vstack([mat4, [0.0, 0.0, 0.0, 1.0]])
            pos, quat_wxyz = _mat4_to_pos_quat_wxyz(mat4)
            self._init_motion_gen()
            result = self._plan(
                start_joints,
                pos,
                quat_wxyz,
                "runtime_preview/%s" % cell_id,
                max_retries=max(1, self._runtime_curobo_preview_retries),
            )
        except Exception as exc:
            self._pub_status("CUROBO_PREVIEW_FAIL %s exception=%r" % (cell_id, exc))
            return

        if result is None:
            self._pub_status("CUROBO_PREVIEW_FAIL %s no valid runtime plan" % cell_id)
            return

        traj, motion_time, endpoint_rad = result
        del traj
        current_deg = np.rad2deg(start_joints).tolist()
        endpoint_deg = np.rad2deg(endpoint_rad).tolist()
        endpoint_deg = self._shortest_equivalent_joints(
            endpoint_deg, reference_joints_rad=start_joints, log_rewrite=False
        )
        plan_delta = np.abs(np.asarray(endpoint_deg) - np.asarray(current_deg))

        yaml_endpoint = target.get("endpoint_joints_deg") or []
        yaml_endpoint = self._shortest_equivalent_joints(
            yaml_endpoint, reference_joints_rad=start_joints, log_rewrite=False
        )
        yaml_delta = np.abs(np.asarray(yaml_endpoint) - np.asarray(current_deg))

        self._pub_status(
            "CUROBO_PREVIEW_VALID %s endpoint=[%s] max_delta=%.1f l1=%.1f "
            "wrist=%.1f time=%.2fs | yaml max_delta=%.1f l1=%.1f wrist=%.1f"
            % (
                cell_id,
                " ".join("%.1f" % d for d in endpoint_deg),
                float(np.max(plan_delta)),
                float(np.sum(plan_delta)),
                float(plan_delta[3] + plan_delta[5]),
                motion_time,
                float(np.max(yaml_delta)),
                float(np.sum(yaml_delta)),
                float(yaml_delta[3] + yaml_delta[5]),
            )
        )

    # ── scan sequence (runs in background thread) ─────────────────────────────

    def _overview_prescan_filter(self, scan_order: List[str]) -> List[str]:
        """원안 1·2단계: overview 에서 1차 스캔 후 익은 과실이 있는 분면만 남긴다.

        로봇은 시작 게이트 덕에 이미 overview 에 있다 — 이동 없이 dwell 만 한다.
        'root=SCANNING' 을 먼저 발행해 시뮬 비전 모킹의 분면 필터를 끈다
        (실기 fusion 노드는 이 토픽을 무시한다). 이 발행이 없으면 같은 프로세스의
        2회차 런에서 직전 런 마지막 분면의 필터가 남는다.
        """
        self._pub_state("root", "SCANNING")
        with self._detection_lock:
            self._scene_positions = []
        self._pub_status(
            "OVERVIEW_SCAN_STARTED — scene_positions 로 분면별 익은 과실 판정, 최대 %.1fs"
            % self._scan_dwell_sec)
        deadline = time.time() + self._scan_dwell_sec
        stable = 0
        last_counts = None
        counts = {q: 0 for q in ("nw", "ne", "se", "sw")}
        while time.time() < deadline:
            with self._detection_lock:
                pts = list(self._scene_positions)
            counts = {q: 0 for q in ("nw", "ne", "se", "sw")}
            for p in pts:
                x, z = float(p[0]), float(p[2])
                if z >= BOARD_SUBCELL_Z_MID_M:
                    counts["nw" if x <= BOARD_SUBCELL_X_MID_M else "ne"] += 1
                else:
                    counts["sw" if x <= BOARD_SUBCELL_X_MID_M else "se"] += 1
            # 좌표는 0.5~1초마다 오므로, 같은 집계가 연속 4회(약 2초)면 안정으로 본다.
            if pts and counts == last_counts:
                stable += 1
                if stable >= 4:
                    break
            else:
                stable = 0
            last_counts = dict(counts)
            time.sleep(0.5)
        self._pub_status("OVERVIEW_SCAN %s" % "  ".join(
            "%s:%d" % (q, counts[q]) for q in ("nw", "ne", "se", "sw")))
        kept, skipped = [], []
        for cell_id in scan_order:
            parts = cell_id.split("/")
            quad = parts[1][:2] if len(parts) >= 2 else None
            if quad in counts and counts[quad] == 0:
                skipped.append(cell_id)
                self._pub_state(cell_id, "SCANNED_EMPTY")
            else:
                kept.append(cell_id)
        if skipped:
            self._pub_status("TRAVERSAL_PRUNED skip=%s — overview 1차 스캔에서 익은 과실 0개"
                             % skipped)
        if not kept:
            self._pub_status("TRAVERSAL_PRUNED_ALL — 익은 과실이 보이는 분면이 없다")
        return kept

    def _scan_sequence(self) -> None:
        self._attempted_pick_positions = []
        self._prev_scan_cell = None
        scan_order = self._compute_scan_order()
        if self._overview_prescan and self._target_cell == "all":
            scan_order = self._overview_prescan_filter(scan_order)
        cell_detections: Dict[str, int] = {}
        collect_then_pick_active = (
            self._collect_then_pick
            and self._enable_pick_integration
            and self._target_cell != "all"
        )
        single_cell_reobserve_pick = (
            collect_then_pick_active
            and len(scan_order) == 1
            and self._max_total_picks != 1
        )
        collected_poses: List[PoseStamped] = []
        if collect_then_pick_active:
            ready_cell = self._collect_pick_ready_cell or self._target_cell
            self._pub_status(
                "COLLECT_THEN_PICK_ENABLED scan_cells=%s pick_ready_cell=%s"
                % (scan_order, ready_cell)
            )

        if single_cell_reobserve_pick:
            cell_id = scan_order[0]
            self._pub_status(
                "SINGLE_CELL_REOBSERVE_PICK enabled target=%s max_total_picks=%d"
                % (cell_id, self._max_total_picks)
            )
            while self._max_total_picks <= 0 or self._total_picks_attempted < self._max_total_picks:
                collected_poses = []
                before_attempts = self._total_picks_attempted
                if not self._scan_one_cell(
                    cell_id, scan_order, collect_then_pick_active,
                    collected_poses, cell_detections,
                ):
                    return
                if not self._finish_collect_then_pick(collected_poses, cell_detections):
                    return
                if self._total_picks_attempted <= before_attempts:
                    self._pub_status(
                        "SINGLE_CELL_REOBSERVE_PICK_STOP no new target after rescan"
                    )
                    break
            self._finish_scan_sequence(cell_detections)
            return

        for cell_id in scan_order:
            rescan_pass = 0
            while True:
                if not self._scan_one_cell(
                    cell_id, scan_order, collect_then_pick_active,
                    collected_poses, cell_detections,
                ):
                    return
                single_cell_reobserve_direct = (
                    self._target_cell != "all"
                    and len(scan_order) == 1
                    and not collect_then_pick_active
                    and self._enable_pick_integration
                    and self._max_total_picks != 1
                )
                if (
                    (self._target_cell != "all" and not single_cell_reobserve_direct)
                    or collect_then_pick_active
                    or not self._enable_pick_integration
                    or (
                        self._last_cell_completed_picks <= 0
                        and not (
                            single_cell_reobserve_direct
                            and self._last_cell_skipped_attempted > 0
                        )
                    )
                ):
                    break
                if (
                    self._max_total_picks > 0
                    and self._total_picks_attempted >= self._max_total_picks
                ):
                    break
                rescan_pass += 1
                if rescan_pass >= 5:
                    self._pub_status(
                        "CELL_RESCAN_STOP %s max repeated passes reached" % cell_id
                    )
                    break
                if single_cell_reobserve_direct:
                    self._pub_status(
                        "SINGLE_CELL_RESCAN_AFTER_ATTEMPT %s pass=%d — "
                        "excluding attempted target(s), checking remaining candidates"
                        % (cell_id, rescan_pass)
                    )
                else:
                    self._pub_status(
                        "CELL_RESCAN_AFTER_PICK %s pass=%d — checking remaining targets"
                        % (cell_id, rescan_pass)
                    )

        if collect_then_pick_active:
            if not self._finish_collect_then_pick(collected_poses, cell_detections):
                return

        self._finish_scan_sequence(cell_detections)

    def _compute_scan_order(self) -> List[str]:
        if self._target_cell == "all":
            # [FIX 2026-09-09] 종전: [c for c in _ALL_CELLS_ZORDER if c in self._targets]
            # _ALL_CELLS_ZORDER 의 'root/nw_flat' 이 YAML 에 없어 **NW 가 조용히 빠지고
            # 3분면만** 돌았다 (원안 6단계 "4개 영역 전부"에 위배). 별칭 해석 + 누락 경고.
            scan_order, missing_quadrants = resolve_traversal_order(
                _ALL_CELLS_ZORDER, self._targets)
            if missing_quadrants:
                self._pub_status(
                    "TRAVERSAL_QUADRANT_MISSING %s — 해당 분면의 scan pose 가 YAML 에 없다"
                    % ",".join(missing_quadrants)
                )
            self._pub_status(
                "TRAVERSAL_SCAN_STARTED cells=%s (%d/4 quadrants)"
                % (scan_order, 4 - len(missing_quadrants))
            )
        else:
            # If sub-cells exist in YAML, scan in canonical visual order.
            # Target filtering should happen in candidate selection, not by
            # silently changing the physical scan traversal order.
            sub_cell_order = [
                "%s/%s" % (self._target_cell, s) for s in ("nw", "ne", "se", "sw")
            ]
            available_subs = [c for c in sub_cell_order if c in self._targets]
            if available_subs:
                scan_order = available_subs
                self._pub_status(
                    "SUBCELL_SCAN_STARTED parent=%s cells=%s" % (self._target_cell, scan_order)
                )
            else:
                scan_order = [self._target_cell]
                self._pub_status("SINGLE_CELL_SCAN_STARTED target=%s" % self._target_cell)

        return scan_order

    def _move_to_scan_cell_and_wait(self, cell_id, target) -> bool:
        endpoint_deg = target.get("endpoint_joints_deg")
        if endpoint_deg is None:
            self._pub_status(
                "CONFIG_ERROR %s missing endpoint_joints_deg in YAML — aborting" % cell_id
            )
            self._pub_state(cell_id, "PLANNING_FAIL")
            return False

        self._pub_state(cell_id, "SCANNING")
        if self._runtime_curobo_preview_enabled:
            self._start_runtime_curobo_preview(cell_id, target)
        self._pub_status(
            "MOVING_TO %s  endpoint_deg=[%s]  (direct MoveJoint, YAML pose)"
            % (cell_id, " ".join("%.1f" % d for d in endpoint_deg))
        )

        # 스캔 이동 시작 전 그리퍼 pre-close — 수 초 이동하는 동안 완료됨
        _gmsg = Int32()
        _gmsg.data = _GRIPPER_APPROACH_POS
        self._gripper_pos_pub.publish(_gmsg)

        # [FIX 2026-09-09] 분면마다 scan pose 가 달라지면서 관절공간 직선이
        # 보드를 관통하게 됐다 (sw->nw -3mm, nw->ne -23mm 실측).
        # 보드가 든 MotionGen 으로 먼저 계획하고, 실패하면 종전 MoveJoint 로 간다.
        transit_deg = self._shortest_equivalent_joints(endpoint_deg)
        planned = None
        # plan_scan_transit=false(기본) 면 실기와 동일하게 순수 MoveJoint 로만 간다.
        # v12 티칭 자세에서는 관절공간 직선도 순차 쌍 보드여유 최소 181mm(보드 810) 로
        # 안전하다 (제가 IK 로 만들었던 ee y=400mm 자세에서는 -23mm 로 관통했다).
        # true 여도 self._mg 가 None 이면 이 분기는 타지 않는다 — 파라미터 선언부 주석 참고.
        if (self._plan_scan_transit
                and self._mg is not None and self._current_joints is not None):
            planned = plan_joint_space(
                self._mg, _JOINT_NAMES, list(self._current_joints),
                np.deg2rad(transit_deg).tolist(), self.get_logger())
        moved = False
        if planned is not None and self._exec_spline(
                planned, vel=self._scan_movej_vel, min_time=1.2):
            self._last_movej_command_deg = list(transit_deg)
            self._pub_status("MOVING_TO %s — cuRobo 경유 (보드 회피)" % cell_id)
            moved = True
        else:
            # MoveJoint 폴백. 비인접 분면 직행(가지치기로 생김)은 overview 를 경유한다.
            prev_quad = self._quadrant_name(self._prev_scan_cell)
            this_quad = self._quadrant_name(cell_id)
            via_overview = (
                prev_quad is not None and this_quad is not None
                and prev_quad != this_quad
                and frozenset((prev_quad, this_quad)) not in _ADJACENT_QUADRANTS
            )
            movej_ok = True
            if via_overview:
                self._pub_status(
                    "TRANSIT_VIA_OVERVIEW %s -> %s — 비인접 분면, MoveJoint 폴백은 overview 경유"
                    % (self._prev_scan_cell, cell_id))
                movej_ok = self._movej(
                    self._overview_joints_deg,
                    vel=self._overview_return_vel, acc=self._overview_return_acc,
                ) and self._wait_at_overview(timeout_sec=30.0)
            if movej_ok:
                movej_ok = self._movej(
                    endpoint_deg, vel=self._scan_movej_vel, acc=self._scan_movej_acc)
            moved = movej_ok
        if not moved:
            self._pub_status("EXEC_FAIL %s MoveJoint failed — aborting scan sequence" % cell_id)
            self._pub_state(cell_id, "PLANNING_FAIL")
            self._pub_status("RETURNING_TO_OVERVIEW after failure")
            self._movej(
                self._overview_joints_deg,
                vel=self._overview_return_vel,
                acc=self._overview_return_acc,
            )
            return False

        arrival_target_deg = self._last_movej_command_deg or endpoint_deg
        endpoint_rad = [float(np.deg2rad(d)) for d in arrival_target_deg]
        arrival_timeout = 90.0
        arrived = self._wait_for_joints(endpoint_rad, 3.0, arrival_timeout)
        if not arrived:
            self._pub_status(
                "EXEC_TIMEOUT %s — robot did not arrive at endpoint within %.0fs; aborting"
                % (cell_id, arrival_timeout)
            )
            self._pub_state(cell_id, "PLANNING_FAIL")
            self._movej(
                self._overview_joints_deg,
                vel=self._overview_return_vel,
                acc=self._overview_return_acc,
            )
            self._prev_scan_cell = None
            return False
        self._prev_scan_cell = cell_id
        return True

    def _process_cell_detections(self, cell_id, count, poses_snapshot,
                                  collect_then_pick_active, collected_poses) -> bool:
        """탐지 결과 처리. False = 시퀀스 중단 (세부 자세 이동 실패; 로봇은 이미 overview 로 복귀)."""
        self._last_cell_completed_picks = 0
        if count > 0:
            self._pub_state(cell_id, "TARGET_FOUND")
            self._pub_status(
                "TARGET_FOUND %s %d pick candidate(s) detected" % (cell_id, count)
            )
            if self._enable_pick_integration:
                unique = self._deduplicate_poses(poses_snapshot)
                if collect_then_pick_active:
                    collected_poses.extend(unique)
                    self._pub_status(
                        "COLLECT_TARGETS %s kept=%d total_buffer=%d"
                        % (cell_id, len(unique), len(collected_poses))
                    )
                else:
                    subgroups = self._group_poses_by_subcell(unique, cell_id)
                    subgroup_msg = "  ".join(
                        "%s/%s:%d" % (cell_id, subcell, len(subposes))
                        for subcell, subposes in subgroups
                    )
                    self._pub_status(
                        "SUBCELL_SCAN_ORDER %s %s" % (cell_id, subgroup_msg)
                    )
                    # [T4b 2026-09-11] 적응 분할 — 후보 밀도가 임계 이상이면 세부 자세로 내려간다.
                    if self._should_subdivide(cell_id, len(unique)):
                        return self._subdivide_and_pick(cell_id, subgroups)
                    for subcell, subposes in subgroups:
                        logical_cell = "%s/%s" % (cell_id, subcell)
                        self._pub_state(logical_cell, "SCANNING")
                        if not subposes:
                            self._pub_status(
                                "SUBCELL_EMPTY %s no pick candidate" % logical_cell
                            )
                            self._pub_state(logical_cell, "SCANNED_EMPTY")
                            continue
                        attempted = self._trigger_picks_for_cell(
                            logical_cell, subposes, pick_timeout_sec=self._pick_timeout_sec
                        )
                        self._last_cell_completed_picks += attempted
                        self._pub_state(
                            logical_cell,
                            "PICK_ATTEMPTED" if attempted > 0 else "SCANNED_EMPTY",
                        )
        else:
            self._pub_state(cell_id, "SCANNED_EMPTY")
            self._pub_status("SCANNED_EMPTY %s no detection in dwell window" % cell_id)
        return True

    # ── [T4b 2026-09-11] 적응 분할 ────────────────────────────────────────────

    def _should_subdivide(self, cell_id: str, n_candidates: int) -> bool:
        thr = self._subdivide_min_candidates
        if thr <= 0 or self._subdivide_solver is None:
            return False
        if len(cell_id.split("/")) != 2:
            return False          # 깊이 상한 2: 세부 칸(root/sw/nw)은 다시 쪼개지 않는다
        if n_candidates < thr:
            self._pub_status(
                "SUBDIVIDE_SKIP %s candidates=%d < %d — 잎(leaf), 부모 자세에서 pick"
                % (cell_id, n_candidates, thr))
            return False
        return True

    def _derive_subcell_target(self, parent_cell: str, subcell: str,
                               parent_joints_deg: List[float]) -> Optional[dict]:
        """부모 자세에서 세부 칸 자세를 계산한다. None 이면 SUBDIVIDE_REJECTED (사유는 상태로 발행)."""
        logical_cell = "%s/%s" % (parent_cell, subcell)
        offset = subcell_center_offset_m(self._quadrant_bounds(parent_cell), subcell)
        limits_deg = [(float(np.rad2deg(lo)), float(np.rad2deg(hi)))
                      for lo, hi in _JOINT_LIMITS_RAD]
        try:
            joints, info = derive_subcell_joints_deg(
                parent_joints_deg, offset,
                self._subdivide_solver.fk, self._subdivide_solver.ik,
                limits_deg=limits_deg,
                max_delta_deg=self._subdivide_max_joint_delta_deg,
                wrap_idx=_SUBCELL_WRAP_IDX)
        except Exception as exc:   # cuRobo 예외는 분할 포기로 흡수 — 시퀀스는 계속 간다
            self._pub_status(
                "SUBDIVIDE_REJECTED %s reason=IK_ERROR %r — 부모 자세에서 pick" % (logical_cell, exc))
            return None
        if joints is None:
            self._pub_status(
                "SUBDIVIDE_REJECTED %s reason=%s goal_ee_mm=%s ik_solutions=%d — 부모 자세에서 pick"
                % (logical_cell, info.get("reason"), info.get("goal_ee_mm"),
                   info.get("ik_solutions", 0)))
            return None
        self._pub_status(
            "SUBCELL_POSE %s dJ_max=%.1fdeg dJ=[%s] ee_mm=%s -> %s joints_deg=[%s]"
            " (부모 FK + x·z 평행이동, 부모 시드 IK)"
            % (logical_cell, info["max_delta_deg"],
               " ".join("%.0f" % d for d in info["delta_deg"]),
               info["parent_ee_mm"], info["goal_ee_mm"],
               " ".join("%.1f" % v for v in joints)))
        target = dict(self._targets[parent_cell])
        target.update({"cell_id": logical_cell, "endpoint_joints_deg": joints,
                       "derived_from": parent_cell})
        return target

    def _subdivide_and_pick(self, cell_id: str, subgroups) -> bool:
        """분면을 2×2 로 쪼개 **후보 있는 세부 칸만** 세부 자세로 내려가 재스캔·pick 한다.

        세부 자세 이동 실패는 부모 분면 이동 실패와 같은 의미(로봇은 overview 로 복귀)라 False 를
        돌려 시퀀스를 중단한다. 유도 실패(SUBDIVIDE_REJECTED)는 그 칸만 부모 자세 pick 으로
        퇴화하고 계속 간다. 세부 자세에서의 재스캔은 시뮬 비전이 분면 단위로 주는 좌표 중
        이 칸의 것만 쓴다 (장애물 등록은 분면 단위 그대로 — 보수적).
        """
        parent_target = self._targets[cell_id]
        parent_joints = [float(v) for v in parent_target["endpoint_joints_deg"]]
        n_candidates = sum(len(sp) for _, sp in subgroups)
        nonempty = [sc for sc, sp in subgroups if sp]
        self._pub_status(
            "SUBDIVIDE %s candidates=%d >= %d cells=%s — 후보 있는 세부 칸만 세부 자세로 방문"
            % (cell_id, n_candidates, self._subdivide_min_candidates, nonempty))
        at_parent_pose = True
        for subcell, subposes in subgroups:
            logical_cell = "%s/%s" % (cell_id, subcell)
            if not subposes:
                self._pub_status("SUBCELL_EMPTY %s no pick candidate — 2단 가지치기" % logical_cell)
                self._pub_state(logical_cell, "SCANNED_EMPTY")
                continue
            target = self._derive_subcell_target(cell_id, subcell, parent_joints)
            if target is None:
                if not at_parent_pose:
                    self._pub_status(
                        "SUBDIVIDE_RETURN_TO_PARENT %s — 부모 자세로 복귀한 뒤 pick" % cell_id)
                    if not self._move_to_scan_cell_and_wait(cell_id, parent_target):
                        return False
                    at_parent_pose = True
                self._pub_state(logical_cell, "SCANNING")
                attempted = self._trigger_picks_for_cell(
                    logical_cell, subposes, pick_timeout_sec=self._pick_timeout_sec)
            else:
                if not self._move_to_scan_cell_and_wait(logical_cell, target):
                    return False
                at_parent_pose = False
                count, poses = self._dwell_collect_detections(logical_cell, False)
                in_cell = [p for p in poses if self._subcell_of_pose(p, cell_id) == subcell]
                unique_sub = self._deduplicate_poses(in_cell)
                self._pub_status(
                    "SUBCELL_SCAN %s raw=%d in_cell=%d unique=%d (분면 시야 중 이 칸의 것만)"
                    % (logical_cell, count, len(in_cell), len(unique_sub)))
                if not unique_sub:
                    self._pub_status(
                        "SUBCELL_EMPTY %s no pick candidate at sub-pose" % logical_cell)
                    self._pub_state(logical_cell, "SCANNED_EMPTY")
                    continue
                attempted = self._trigger_picks_for_cell(
                    logical_cell, unique_sub, pick_timeout_sec=self._pick_timeout_sec)
            self._last_cell_completed_picks += attempted
            self._pub_state(
                logical_cell, "PICK_ATTEMPTED" if attempted > 0 else "SCANNED_EMPTY")
        return True

    def _dwell_collect_detections(self, cell_id: str, collect_then_pick_active: bool):
        """스캔 자세에서 dwell 동안 pick_pose 탐지를 모은다 → (count, poses)."""
        with self._detection_lock:
            self._detection_count = 0
            self._detection_poses = []
        joints_now = self._current_joints or []
        joints_deg_str = " ".join("%.1f" % np.rad2deg(j) for j in joints_now)
        self._pub_status(
            "AT_SCAN_POSE %s joints_deg=[%s] — adaptive detection wait up to %.1fs"
            % (cell_id, joints_deg_str, self._scan_dwell_sec)
        )
        detection_deadline = time.time() + self._scan_dwell_sec
        while time.time() < detection_deadline:
            with self._detection_lock:
                if (
                    self._detection_count > 0
                    and not collect_then_pick_active
                    and self._max_total_picks == 1
                ):
                    break
            time.sleep(0.05)
        with self._detection_lock:
            return self._detection_count, list(self._detection_poses)

    def _scan_one_cell(self, cell_id, scan_order, collect_then_pick_active,
                       collected_poses, cell_detections) -> bool:
        if cell_id not in self._targets:
            self.get_logger().warn("%s not in candidates — skipping" % cell_id)
            return True

        target = self._targets[cell_id]
        if not self._move_to_scan_cell_and_wait(cell_id, target):
            return False

        # dwell 동안 탐지를 모은다 (세부 칸 재스캔[T4b]도 같은 helper 를 쓴다).
        count, poses_snapshot = self._dwell_collect_detections(
            cell_id, collect_then_pick_active)
        cell_detections[cell_id] = count

        if not self._process_cell_detections(
                cell_id, count, poses_snapshot, collect_then_pick_active,
                collected_poses):
            return False

        # After picks (or empty cell) go directly to next scan pose from current
        # position. HOME/overview recovery is reserved for explicit recovery
        # policy (e.g. future VLA after repeated failed pick attempts).
        if cell_id != scan_order[-1]:
            self._pub_status("INTER_CELL_DIRECT — no overview reset; continuing to next cell")
        return True

    def _finish_collect_then_pick(self, collected_poses, cell_detections) -> bool:
        unique_all = self._deduplicate_poses(collected_poses)
        if not unique_all:
            self._pub_status("COLLECT_THEN_PICK_EMPTY no candidates after full scan")
        else:
            ready_cell = self._collect_pick_ready_cell or self._target_cell
            ready_target = self._targets.get(ready_cell)
            ready_joints = ready_target.get("endpoint_joints_deg") if ready_target else None
            if ready_joints is None:
                self._pub_status(
                    "COLLECT_THEN_PICK_BLOCKED missing pick-ready pose %s" % ready_cell
                )
                self._pub_state(self._target_cell, "PLANNING_FAIL")
                return False
            unique_all = self._rank_poses_for_pick_ready(unique_all, ready_target)
            self._pub_status(
                "COLLECT_THEN_PICK_READY_MOVE %s candidates=%d best=(%.0f,%.0f,%.0f)mm"
                % (
                    ready_cell,
                    len(unique_all),
                    unique_all[0].pose.position.x * 1000,
                    unique_all[0].pose.position.y * 1000,
                    unique_all[0].pose.position.z * 1000,
                )
            )
            if not self._movej(
                ready_joints, vel=self._scan_movej_vel, acc=self._scan_movej_acc
            ):
                self._pub_status(
                    "COLLECT_THEN_PICK_BLOCKED pick-ready MoveJoint failed"
                )
                self._pub_state(self._target_cell, "PLANNING_FAIL")
                return False
            arrival_target_deg = self._last_movej_command_deg or ready_joints
            ready_rad = [float(np.deg2rad(d)) for d in arrival_target_deg]
            if not self._wait_for_joints(ready_rad, 3.0, 90.0):
                self._pub_status(
                    "COLLECT_THEN_PICK_BLOCKED pick-ready pose not confirmed"
                )
                self._pub_state(self._target_cell, "PLANNING_FAIL")
                return False
            attempted = self._trigger_picks_for_cell(
                "%s/best" % self._target_cell,
                unique_all,
                pick_timeout_sec=self._pick_timeout_sec,
                poses_are_ranked=True,
            )
            self._pub_state(
                self._target_cell,
                "PICK_ATTEMPTED" if attempted > 0 else "SCANNED_EMPTY",
            )
        return True

    def _finish_scan_sequence(self, cell_detections) -> None:
        # Return to overview is optional.  During harvest experiments we often
        # continue from the last cell pose so VLA/recovery logic can decide when
        # HOME is actually needed.
        if not self._return_to_overview_at_end:
            self._pub_status("SCAN_COMPLETE stay_at_last_scan_pose=true")
            return

        self._pub_status("RETURNING_TO_OVERVIEW")
        if not self._movej(
            self._overview_joints_deg,
            vel=self._overview_return_vel,
            acc=self._overview_return_acc,
        ):
            self._pub_status("ABORT overview return failed after scan sequence")
            return
        if not self._wait_at_overview():
            self._pub_status("ABORT overview pose was not confirmed after scan sequence")
            return
        joints_ov = self._current_joints or []
        joints_ov_str = " ".join("%.1f" % np.rad2deg(j) for j in joints_ov)
        self._pub_status("AT_OVERVIEW joints_deg=[%s]" % joints_ov_str)

        # Publish harvest priority order (most detections first).
        if cell_detections:
            harvest_order = sorted(
                cell_detections.items(), key=lambda x: x[1], reverse=True
            )
            order_str = "  ".join(
                "%s:%d" % (cid, cnt) for cid, cnt in harvest_order
            )
            self._pub_status("HARVEST_PRIORITY_ORDER %s" % order_str)

        self._pub_status("SCAN_COMPLETE")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanExecutorNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
