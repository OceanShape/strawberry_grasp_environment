import os
import time
import threading
import math
import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
from dsr_msgs2.srv import MoveSplineJoint, MoveJoint, MoveLine, ChangeOperationSpeed
from dsr_gripper_tcp_interfaces.srv import SetPosition, GetState
from dsr_gripper_tcp_interfaces.action import SafeGrasp

try:
    import torch
    from curobo.types.math import Pose
    from curobo.types.robot import RobotConfig
    from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
    from curobo.geom.types import Cuboid, WorldConfig
    from curobo.types.base import TensorDeviceType
    import yaml
    CUROBO_AVAILABLE = True
except ImportError:
    CUROBO_AVAILABLE = False


class SimExecutorBridgeNode(Node):
    # MoveLine은 45mm급 미세 직선 이동이므로 관절이 이보다 크게 움직이는 해는
    # elbow-flip(반대 브랜치) 해다 — 실행하면 팔 전체가 휘둘리며 바닥을 뚫는다
    MOVELINE_MAX_JOINT_DELTA_DEG = 45.0
    MOVELINE_IK_RETRIES = 5
    IK_NUM_SEEDS = 32
    COLLISION_ACTIVATION_DISTANCE_M = 0.005   # planner_bootstrap 과 동일해야 한다
    SPLINE_STEP_DEG = 2.0             # 스플라인 웨이포인트 사이 보간 간격
    # [FIX 2026-09-09] 구간당 고정 0.10s -> req.time 배분.
    # 고정값이면 12점 스플라인이 이동량과 무관하게 항상 ~1.2s 만에 **명령만** 끝나서,
    # 100도 넘게 도는 복귀 구간에서 팔이 30도 넘게 뒤처진 채로 다음 단계가 시작됐다.
    # [SPEED 2026-09-09] 아티큘레이션 드라이브 게인을 10배로 올렸으므로
    # (robot_assembly.usd: 팔 stiffness 1e5->1e6 / damping 1e4->1e5) 팔이 훨씬
    # 빨리 따라온다. 그에 맞춰 시간 상수도 줄인다. 실행 시간은 sim_speed_scale
    # 파라미터로 한 번에 조절한다 (기본 2.0 = 종전의 2배 속도).
    SPLINE_MIN_TOTAL_SEC = 0.35       # req.time 이 없거나 너무 짧을 때의 하한
    SPLINE_MIN_STEP_SLEEP_SEC = 0.003
    # ★ [FIX 2026-09-10] 관절 속도 상한으로 실행 시간의 **바닥**을 잡는다.
    #
    # 종전에는 total_sec = req.time / sim_speed_scale 이라, 큰 스윙일수록 명령이
    # 빨리 끝나 아티큘레이션이 못 따라왔다. 실측 (스캔 자세 -> pre-approach):
    #     sw 딸기 3개   40.7 / 62.9 / 43.1 deg   -> 잘 따라옴 (수확 정상)
    #     ne 딸기       108.1 deg                -> 못 따라옴
    #     nw 딸기 2개   195.6 / 196.7 deg        -> 못 따라옴
    # 못 따라온 채로 도착 대기가 타임아웃되면, **다음 MoveLine 이 상대 이동**이라
    # 그 잔차가 그대로 남아 조우가 파지점보다 앞에서 닫힌다.
    # 사용자 보고와 일치: sw 3개는 완벽, nw/ne 는 앞쪽을 집음.
    SPLINE_MAX_JOINT_SPEED_DEG_S = 120.0
    ARM_ARRIVAL_TOL_DEG = 1.5         # "도착" 판정 허용 오차
    # 3.0 -> 8.0. 제때 도착하면 비용이 0 이고, 못 하면 잔차가 다음 상대 이동으로
    # 전파되므로 넉넉히 기다리는 편이 항상 낫다.
    ARM_ARRIVAL_TIMEOUT_SEC = 8.0
    # [FIX 2026-09-08] 5mm 스텝은 화면에서 "깔짝깔짝" 끊겨 보였다.
    # 시드 수정으로 스텝당 관절 이동량이 1deg 수준이 되어 촘촘하게 나눠도 안전하다.
    MOVELINE_STEP_MM = 2.0            # 직선 보간 간격
    MOVELINE_MIN_STEP_SLEEP_SEC = 0.003
    MOVELINE_MAX_TOTAL_SEC = 2.0
    # [FIX 2026-09-10] 스텝 수 상한. MoveLine 실소요는 명령 속도가 아니라 **스텝 수 × IK 1회
    # (~150ms)** 가 지배한다 (10:13 런 실측: 30/40/45/120mm 전부 스텝당 144~153ms).
    # 2mm 고정이면 배치 하강 120mm 가 60스텝 = 9.2초로, 스스로 계산한 페이싱 1.5초를
    # 6배 넘겼다. 실기 movel 은 제어기가 직선을 처리하므로 이 비용은 시뮬에만 있는
    # 인공물이다. 24 로 자르면 120mm 만 5mm 스텝이 되고(관절 이동 0.3→0.75도, 가드 45도)
    # 파지 쪽 30/40/45mm(16/20/23스텝) 는 그대로다.
    MOVELINE_MAX_STEPS = 24
    # [DIAG 2026-09-10] 직선 종점의 **측방** 오차 경고선. 계측 전용 — 동작을 막지 않는다.
    MOVELINE_LATERAL_WARN_MM = 5.0

    def __init__(self):
        super().__init__('sim_executor_bridge_node')
        self.get_logger().info("Sim Executor Bridge Node initializing...")

        self.cb_group = ReentrantCallbackGroup()
        self.current_joints = [0.0] * 6
        self.joint_state_received = False   # /dsr01/joint_states 를 한 번이라도 받았는가
        self.last_arm_command_deg = None     # 마지막으로 발행한 팔 6축 명령 (deg)
        self.gripper_position = 600      # 판정용 리드백 (get_state 가 돌려주는 값)
        self.gripper_command = 600       # 명령값 = 시뮬 조우가 실제로 갈 개도

        # ── 파지 판정 (기하 기반) ────────────────────────────────────────
        # 실기 그리퍼는 조우(jaw) 위치로 파지를 판정한다 (pos>=695 빈손 / <=685 접촉).
        # 이전 mock은 set_position 명령값을 그대로 되돌려줘서 close(700) → 항상 빈손이었다.
        # 이제 TCP와 딸기의 실제 거리로 판정한다: 조우 사이에 딸기가 있으면 접촉,
        # 없으면 빈손. 그래야 "접근이 틀리면 파지 실패"가 시뮬에서도 참이 된다.
        #
        # ⚠️ tool_tcp_offset_m 은 planner 의 ee_to_tcp_offset_m 과 **같은 값이어야 한다.**
        # planner 는 grasp 종점을 straw - (grasp_offset + ee_to_tcp)·approach_dir 로 잡으므로,
        # 여기서 계산하는 거리는 |grasp_offset + planner_offset - bridge_offset| 이 된다.
        # 두 값이 같으면 grasp_offset(15~30mm)만 남아 capture radius(35mm) 안에 들어오고,
        # 어긋나면 그 차이가 그대로 더해져 **딸기를 제대로 잡아도 빈손 판정**이 난다.
        # (planner 160mm / bridge 160mm 이던 2026-09-07 16:44 런: d_tcp=14.8mm → CONTACT.
        #  planner 208mm / bridge 160mm 이면 d_tcp=63mm → 전 타겟 EMPTY.)
        # [2026-09-09] grasp_offset 사다리 맨 앞에 0.0 이 붙어 정상 파지의 d_tcp 가
        # sqrt(15^2+35^2)=38mm -> **35mm(z bias 만)** 로 내려간다. 반경 45mm 는 그대로
        # 두어도 정상 35mm 는 통과하고 30mm 빗나감(46mm)은 걸러진다.
        # [2026-09-08] 판정 반경 35 -> 45mm.
        # 플래너가 pick_target_z_bias_m=35mm 로 **줄기(과실중심 +35mm)** 를 겨냥하는데
        # 이 판정은 **과실 중심까지의 거리**를 잰다. 즉 정상 파지에서도 d_tcp 가
        # 설계상 sqrt(15^2+35^2)=38mm 라 35mm 임계값과 겹쳐 동전던지기가 됐다
        # (실측 33.9/34.9/36.0/36.2mm — CONTACT/EMPTY 가 무작위로 갈림).
        # 45mm 면 정상 파지(35~38mm)는 통과하고 30mm 이상 빗나간 경우(48mm)는 걸러진다.
        # [REVERTED 2026-09-10 rev2] 55 -> 45mm 로 되돌린다.
        #
        # 55mm 로 넓힌 것은 "같은 자세인데 딸기마다 갈린다"에 대한 대증요법이었다.
        # 진짜 원인은 판정이 아니라 **도착 지연**이었다 (SPLINE_MAX_JOINT_SPEED_DEG_S
        # 주석 참조). 스플라인이 큰 스윙을 못 따라간 채 타임아웃되면 그 잔차가
        # 다음 상대 MoveLine 에 전파되어 조우가 20mm 넘게 앞에서 닫힌다.
        # 반경을 넓히면 **그 실패를 CONTACT 로 오판정**해 그대로 배치까지 진행한다
        # (사용자 보고: "앞쪽을 잡는데 분리 성공으로 인식하고 배치까지 감").
        #
        # 페이싱을 고친 뒤의 판별력:
        #     정상(오프셋 15mm)      d_tcp 38.1mm  -> 통과 (여유 6.9mm)
        #     실행오차 +5mm          d_tcp 40.3mm  -> 통과
        #     도착지연 +20mm         d_tcp 49.5mm  -> **차단**  <- 잡아야 하는 것
        # 45mm 가 이 둘을 정확히 가른다. 넓히면 안 된다.
        # [FIX 2026-09-10 rev3] 45 -> 38mm.
        # 정상 파지의 d_tcp 는 **32.1mm** 다 — 38.1mm 로 알던 것이 틀렸다.
        # 진입 방향 (0.069, 0.913, 0.402) 의 z 성분 때문에 오프셋과 z bias 가
        # 일부 상쇄된다. 대조표:
        #     실제 오프셋 15mm(정상)  d_tcp 32.1mm  along  +0.9mm
        #     실제 오프셋 25mm        d_tcp 33.9mm  along +10.9mm
        #     실제 오프셋 35mm        d_tcp 38.3mm  along +20.9mm
        #     실제 오프셋 45mm        d_tcp 44.6mm  along +30.9mm
        # 45mm 반경은 30mm 나 짧게 닫힌 것까지 통과시킨다. 실제 런 로그의
        # d_tcp 36~39mm 는 이미 15~20mm 짧았다는 뜻이었다.
        self.declare_parameter("grasp_capture_radius_m", 0.038)
        self.declare_parameter("tool_tcp_offset_m", 0.236)   # planner -p ee_to_tcp_offset_m:=0.236 과 일치
        # 파지 판정 허용치 (툴 축 분해 기준).
        #   depth : 조우가 딸기보다 앞/뒤로 얼마나 벗어나도 되는가. 정상 +0.9mm
        #   lateral: 툴 축에서의 수직 거리 상한. 정상 32.1mm (z bias 성분)
        # 조우 물림 구간 (TCP 기준, m). 실측 파츠 기하에서 나온 값이다.
        self.declare_parameter("jaw_capture_near_m", -0.006)
        self.declare_parameter("jaw_capture_far_m", 0.027)
        self.declare_parameter("grasp_lateral_tolerance_m", 0.020)
        # 플래너의 pick_target_z_bias_m 과 **같은 값이어야 한다** (T3 인자).
        self.declare_parameter("grasp_target_z_bias_m", 0.035)
        self.declare_parameter("grasp_judgement_enabled", True)
        # 실행 속도 배율. 클수록 빠르다. 1.0 = 2026-09-09 이전 속도.
        self.declare_parameter("sim_speed_scale", 2.0)
        # stroke 700(완전 닫힘)이 몇 rad 인가. 1.0 = 실기 스톡 상한(파츠 간격 9.4mm),
        # 1.08 = 커스텀 파츠가 실제로 무는 값(0.7mm). USD 관절 상한도 같이 맞춰야 한다.
        self.declare_parameter("gripper_close_rad", 1.08)
        self.grasp_radius = float(self.get_parameter("grasp_capture_radius_m").value)
        self.tool_offset = float(self.get_parameter("tool_tcp_offset_m").value)
        self.grasp_judgement = bool(self.get_parameter("grasp_judgement_enabled").value)
        self.jaw_near_mm = float(self.get_parameter("jaw_capture_near_m").value) * 1000.0
        self.jaw_far_mm = float(self.get_parameter("jaw_capture_far_m").value) * 1000.0
        self.grasp_lateral_tol = float(self.get_parameter("grasp_lateral_tolerance_m").value)
        self.grasp_z_bias = float(self.get_parameter("grasp_target_z_bias_m").value)
        self.speed_scale = max(0.1, float(self.get_parameter("sim_speed_scale").value))
        self.gripper_close_rad = float(self.get_parameter("gripper_close_rad").value)
        self.strawberries = []          # Isaac Sim이 알려주는 익은 딸기 좌표 (base/world)
        self.last_grasp_reason = "no judgement yet"
        # [T2 2026-09-10] 파지 부착 이벤트. 판정은 여기(브릿지)가 하고, 딸기를 실제로
        # 그리퍼에 붙이고 떼는 것은 Isaac 스크립트가 이 토픽을 구독해서 한다.
        #   "ATTACH x y z" — CONTACT 판정된 과실 중심 (m, base 프레임). Isaac 은 가장 가까운
        #                    딸기 prim 을 찾아 그 순간의 그리퍼 밑동 기준 상대 트랜스폼을 잡는다.
        #   "RELEASE"      — 부착 중 조우가 열림. 그 자리에 정지.
        # 좌표로 지목하는 이유: 브릿지는 딸기를 /isaac_sim/strawberries 의 좌표로만 알고
        # prim 이름을 모른다. 새 판정 로직을 Isaac 쪽에 다시 만들지 않는다 (SUBMISSION_PLAN T2).
        self.grasp_event_pub = self.create_publisher(String, '/sim/grasp_event', 10)
        self.last_grasp_berry = None    # 직전 _judge_grasp 가 재본 과실 중심 (np.array) / None
        self.attached_berry = None      # ATTACH 발행 후 RELEASE 전까지의 과실 중심

        self.berry_sub = self.create_subscription(
            PoseArray, '/isaac_sim/strawberries', self.strawberry_cb, 10,
            callback_group=self.cb_group)

        self.joint_sub = self.create_subscription(
            JointState, '/dsr01/joint_states', self.joint_cb, 10, callback_group=self.cb_group)
        self.joint_pub = self.create_publisher(JointState, '/joint_command', 10)

        # Doosan Motion Services
        self.srv_move_spline = self.create_service(
            MoveSplineJoint, '/dsr01/motion/move_spline_joint', self.move_spline_cb, callback_group=self.cb_group)
        self.srv_move_joint = self.create_service(
            MoveJoint, '/dsr01/motion/move_joint', self.move_joint_cb, callback_group=self.cb_group)
        self.srv_move_line = self.create_service(
            MoveLine, '/dsr01/motion/move_line', self.move_line_cb, callback_group=self.cb_group)
        self.srv_change_speed = self.create_service(
            ChangeOperationSpeed, '/dsr01/motion/change_operation_speed', self.change_speed_cb, callback_group=self.cb_group)

        # Gripper Services / Action
        self.srv_set_position = self.create_service(
            SetPosition, '/gripper_service/set_position', self.set_position_cb, callback_group=self.cb_group)
        self.srv_get_state = self.create_service(
            GetState, '/gripper_service/get_state', self.get_state_cb, callback_group=self.cb_group)
        self.action_safe_grasp = ActionServer(
            self, SafeGrasp, '/gripper_service/safe_grasp', self.safe_grasp_cb, callback_group=self.cb_group)

        # IK Solver Setup
        self.ik_solver = None
        self.init_curobo_ik()

        self.get_logger().warn(
            "GRASP_JUDGE_MODEL: tool_tcp_offset=%.0fmm capture_radius=%.0fmm enabled=%s "
            "sim_speed_scale=%.1f gripper_close=%.3frad "
            "— planner 의 ee_to_tcp_offset_m 과 반드시 같아야 한다 (다르면 전 타겟 GRASP_EMPTY)"
            % (self.tool_offset * 1000, self.grasp_radius * 1000, self.grasp_judgement,
               self.speed_scale, self.gripper_close_rad))
        self.get_logger().info("Sim Executor Bridge Node Ready! (Listening to Doosan & Gripper services)")

        try:    # ── HUD 계측 (제거: 이 4줄만 지우면 된다) ──
            import sys, os; sys.path.append(os.path.expanduser(os.environ.get(
                "HARVEST_HUD_DIR", "~/strawberry_grasp_environment/strawberry_harvest/scripts/hud")))
            import harvest_probe; harvest_probe.attach("controller", self)
        except Exception: pass

    def init_curobo_ik(self):
        if not CUROBO_AVAILABLE:
            self.get_logger().warn("curobo is not installed. MoveLine IK will be skipped.")
            return

        try:
            from ament_index_python.packages import get_package_share_directory
            config_dir = os.path.join(
                get_package_share_directory("e0509_gripper_description"),
                "config", "curobo"
            )
            robot_config_name = "e0509_gripper.yml" # fallback
            config_path = os.path.join(config_dir, robot_config_name)
            if not os.path.exists(config_path):
                self.get_logger().warn(f"Robot config not found at {config_path}")
                return

            with open(config_path, "r", encoding="utf-8") as f:
                robot_cfg_data = yaml.safe_load(f)
            
            robot_kin = robot_cfg_data["robot_cfg"]["kinematics"]
            robot_kin["urdf_path"] = os.path.join(config_dir, "e0509_gripper.urdf")
            robot_kin["collision_spheres"] = os.path.join(config_dir, "e0509_spheres.yml")

            tensor_args = TensorDeviceType(device=torch.device("cuda:0"))
            robot_cfg = RobotConfig.from_dict(robot_cfg_data, tensor_args=tensor_args)

            # [FIX 2026-09-08] IK 솔버에 **충돌 월드를 준다.**
            #
            # 종전에는 world_model 인자를 생략(=None)해서 이 솔버가 장애물을 전혀
            # 몰랐다. 플래너(MotionGen)는 보드를 피해 waypoint 를 내지만, 그 사이를
            # 잇는 **MoveLine 구간은 전부 이 솔버가 푼다** — 최종 직선 진입,
            # 줄기 하강, 분리 당김, retreat, 배치 이송. 그래서 이동 중 그리퍼 끝이
            # 보드를 그대로 통과했다.
            # 플래너와 같은 environment.yaml 을 읽어 동일한 보드를 넣는다.
            world_cfg = None
            try:
                env_yaml = os.path.join(
                    get_package_share_directory("e0509_gripper_description"),
                    "config", "environment.yaml")
                if not os.path.exists(env_yaml):
                    env_yaml = os.path.expanduser(
                        "~/strawberry_grasp_environment/src/"
                        "e0509_gripper_description/config/environment.yaml")
                with open(env_yaml, "r", encoding="utf-8") as ef:
                    env = yaml.safe_load(ef) or {}
                cuboids = [
                    Cuboid(name=str(o["name"]),
                           pose=[float(v) for v in o["pose"]],
                           dims=[float(v) for v in o["dims"]])
                    for o in env.get("objects", [])
                    if o.get("enabled", True) and o.get("type", "cuboid") == "cuboid"
                ]
                if cuboids:
                    world_cfg = WorldConfig(cuboid=cuboids)
                    self.get_logger().warn(
                        "MOVELINE_COLLISION_WORLD: "
                        f"{[c.name for c in cuboids]} loaded from {env_yaml}")
            except Exception as exc:                                  # noqa: BLE001
                self.get_logger().error(
                    f"MOVELINE_COLLISION_WORLD load failed ({exc}); "
                    "MoveLine will NOT avoid obstacles")

            ik_config = IKSolverConfig.load_from_robot_config(
                robot_cfg,
                world_cfg,
                tensor_args=tensor_args,
                num_seeds=self.IK_NUM_SEEDS,
                self_collision_check=False,
                use_cuda_graph=False,
                # [2026-09-10] 플래너(planner_bootstrap.py)와 같은 값. 근거는 그쪽 주석.
                collision_activation_distance=self.COLLISION_ACTIVATION_DISTANCE_M,
            )
            self.ik_solver = IKSolver(ik_config)
            self.get_logger().info("cuRobo IK Solver successfully initialized for MoveLine.")
        except Exception as e:
            self.get_logger().warn(f"Failed to initialize cuRobo IK: {e}")

    # ── 그리퍼 관절 구동 (Stage1, 2026-09-07) ──────────────────────────
    # 종전에는 set_position 이 self.gripper_position 숫자만 바꾸고 끝나서
    # **시뮬 그리퍼가 한 번도 움직이지 않았다.** 그래서 씬의 초기 개도(거의 완전
    # 닫힘)가 런 내내 유지됐고, cuRobo 는 완전열림으로 계획하는데 실제는 닫힘이라
    # 툴 길이가 27.5mm 어긋나 보드를 관통했다.
    #
    # 이제 팔 관절과 함께 그리퍼 관절도 /joint_command 로 발행한다.
    # 이름은 main_scene.usd 의 articulation DOF 이름과 같아야 한다
    # (layout_layer.usd 의 over "joints" 아래 프림 이름).
    # robot.urdf 의 팔 6축 한계(deg) + 여유 5도. _publish_joint_command 의 방어선.
    ARM_JOINT_LIMIT_DEG = [365.0, 100.0, 160.0, 365.0, 140.0, 365.0]

    GRIPPER_JOINT_NAMES = ["rh_p12_rn", "rh_r2", "rh_l1", "rh_l2"]

    # [FIX 2026-09-08 rev2] stroke(0~700) -> 관절각(rad) 배율 = 1.0.
    #
    # RH-P12-RN 은 평행사변형이라 l1/r1 과 l2/r2 가 **같은 각도**로 (축은 반대로)
    # 움직여야 손가락이 회전하지 않고 평행 이동한다.
    # 그런데 robot.urdf(USD 임포트 원본)의 관절 상한이 쌍마다 다르다:
    #     rh_p12_rn(r1), rh_l1 : upper = 1.1 rad (63.03deg)
    #     rh_r2,        rh_l2  : upper = 1.0 rad (57.30deg)
    # 따라서 **평행사변형을 유지한 채 갈 수 있는 최대는 1.0 rad** 이다.
    #
    # 직전 커밋에서 배율을 1.101 로 올린 것이 회귀였다. 네 관절에 같은 값을 보내면
    # l2/r2 만 USD 한계 1.0 에서 잘려 l1 과 각도가 어긋나고, 손가락이 기울면서
    # **몸통은 겹치고 팁은 벌어지는 "가위" 모양**이 된다. 실측(stroke 700, 비율 유지시):
    #     전체 최소 간격 -33.4mm (겹침) / 팁 간격 +21.1mm (벌어짐)  ← 가위
    # 배율 1.0 이면 stroke 700 에서 네 관절 모두 57.30deg = l2/r2 상한에 정확히 닿고,
    # 평행을 유지한 채 파츠 간격 9.4mm 로 닫힌다 (KP1 자석 5mm 를 무는 모습).
    #
    # 별도의 접촉각 클램프는 두지 않는다 — 1.0 rad 자체가 기구 상한이다.
    # ★ [FIX 2026-09-10] 1.0 -> 1.08 rad.
    #
    # 1.0 은 RH-P12-RN 제조사 xacro 의 rh_r2/rh_l2 상한인데, 그건 **스톡 손가락**
    # 기준이다. 15cm 커스텀 파츠를 붙이면 손가락이 길어져 1.0 에서 파츠 끝이
    # 9.4mm 벌어진 채 멈춘다 — 5mm 줄기를 물지 못한다.
    # 실제 메시(gripper_parts.stl)로 잰 두 파츠 최소 간격:
    #     0.926 rad (stroke 600, 접근)  17.5mm
    #     1.000 rad (종전 닫힘)          9.4mm   <- 못 문다
    #     1.080 rad (현재 닫힘)           0.7mm   <- 문다
    #     1.101 rad                      0.1mm   (접촉)
    # 파츠는 완전히 평행하다(평행도 실측 0.0mm) — 간격이 전체 면에 균일하므로
    # 이 값이 곧 "무는 정도"다.
    #
    # ⚠️ 실기 하드웨어는 1.0 에서 멈춘다. **의도한 시뮬 편차**이며
    #    robot_assembly.usd 의 rh_r2/rh_l2 상한 override 와 한 쌍이다.
    #    실기와 맞추려면 둘 다 되돌린다 (gripper_close_rad:=1.0).
    GRIPPER_JOINT_RANGE_RAD = 1.08
    GRIPPER_STROKE_TO_RAD = GRIPPER_JOINT_RANGE_RAD / 700.0

    def _gripper_joint_rad(self) -> float:
        """시뮬 조우가 실제로 갈 각도. **리드백이 아니라 명령값**을 쓴다.

        리드백(gripper_position)은 "조우가 물체에 걸려 멈춘 개도"를 흉내 낸 값이라
        접촉 시 670 이 된다. 그 값으로 관절을 구동하면 화면에서 조우가 덜 닫힌 채
        멈춰 "줄기를 쥐지 못한" 것처럼 보인다. 씬에 줄기 지오메트리가 없어 조우를
        물리적으로 막을 것이 없으므로, 닫으라는 명령이면 실제로 끝까지 닫는 것이 맞다.
        판정은 종전대로 리드백으로 한다.
        """
        limit = getattr(self, "gripper_close_rad", self.GRIPPER_JOINT_RANGE_RAD)
        rad = float(self.gripper_command) * (limit / 700.0)
        return max(0.0, min(rad, limit))

    def _publish_gripper_only(self):
        """팔 명령은 그대로 두고 그리퍼 개도만 갱신해 발행한다.

        ★ [FIX 2026-09-09] **측정값(current_joints)이 아니라 마지막 명령값**을 쓴다.

        종전에는 리드백을 그대로 명령으로 되돌려 실었다. Isaac 아티큘레이션은
        /joint_command 를 **드라이브 목표**로 받으므로, 팔이 아직 목표로 가는
        중일 때 이걸 발행하면 **그 순간의 자세가 새 목표가 되어 모션이 그 자리에서
        멈춘다.** 실측(2026-09-09 런):
          MoveSpline 복귀 종점 [88.0 -94.4 129.9 -184.1 -31.3 93.4]
          -> 29ms 뒤 SetPosition(600) 이 리드백을 덮어써 J6 가 93.4 가 아니라
             124.7 에서 얼어붙음 (31도 어긋남)
        이게 "배치 후 J6 가 갸우뚱한 채로 다음 딸기로 간다" 의 정체다. 그리고 노드는
        다음 pick 시작 자세를 current_joints 로 저장하므로(pick_sequence_executor.py:793)
        그 어긋남이 **다음 복귀 목표로 그대로 이월되어 사이클마다 누적**됐다.
        마지막 명령값을 다시 실으면 진행 중인 모션을 건드리지 않는다.
        """
        if self.last_arm_command_deg is not None:
            arm_deg = list(self.last_arm_command_deg)
        else:
            arm_deg = [math.degrees(j) for j in self.current_joints]
        self._publish_joint_command(arm_deg)

    def joint_cb(self, msg: JointState):
        if len(msg.position) >= 6:
            self.current_joints = list(msg.position)[:6]
            self.joint_state_received = True

    def strawberry_cb(self, msg: PoseArray):
        self.strawberries = [
            np.array([p.position.x, p.position.y, p.position.z]) for p in msg.poses
        ]

    def _tcp_and_ee_position(self):
        """현재 관절값에서 그리퍼 베이스(ee)와 파지점(TCP) 좌표를 구한다.

        cuRobo 설정의 ee_link는 'gripper_rh_p12_rn_base'(그리퍼 밑동)이므로,
        실제 파지점은 툴 +Z 방향으로 tool_tcp_offset_m 만큼 더 나간 지점이다.
        """
        if self.ik_solver is None or self.current_joints is None:
            return None, None
        try:
            st = torch.tensor([self.current_joints], device="cuda:0", dtype=torch.float32)
            fk = self.ik_solver.kinematics.get_state(st)
            ee = fk.ee_position[0].detach().cpu().numpy()
            q = fk.ee_quaternion[0].detach().cpu().numpy()      # wxyz
            rot = ScipyRotation.from_quat([q[1], q[2], q[3], q[0]])
            tcp = ee + rot.apply([0.0, 0.0, self.tool_offset])
            return np.asarray(ee, dtype=float), np.asarray(tcp, dtype=float)
        except Exception as e:                                   # noqa: BLE001
            self.get_logger().warn(f"FK for grasp judgement failed: {e}")
            return None, None

    def _judge_grasp(self):
        """조우 사이에 딸기가 있는가. (True=접촉, False=빈손, None=판정 불가)"""
        if not self.grasp_judgement:
            return None
        ee, tcp = self._tcp_and_ee_position()
        if tcp is None:
            self.last_grasp_reason = "FK unavailable"
            return None
        if not self.strawberries:
            self.last_grasp_reason = "no strawberry positions received yet"
            return None
        d_tcp = min(float(np.linalg.norm(tcp - b)) for b in self.strawberries)
        d_ee = min(float(np.linalg.norm(ee - b)) for b in self.strawberries)
        # [DIAG 2026-09-10] 스칼라 거리로는 "앞에서 닫혔다"를 못 가린다.
        # 툴 +Z(진입 방향)로 분해하면 깊이 오차가 그대로 보인다.
        #   along  = 조우가 딸기보다 얼마나 **앞**에 섰나 (mm). 정상 ~ +1mm
        #   lateral= 툴 축에서의 수직 거리. 정상 ~ 32mm (z bias 성분)
        # 정상 파지의 d_tcp 는 **32.1mm** 다 (38.1 이 아니다) — 진입 방향의
        # z 성분(0.402) 때문에 오프셋 15mm 와 z bias 35mm 가 일부 상쇄된다.
        # ★ [FIX 2026-09-10 rev2] 판정 대상은 과실 중심이 아니라 **줄기(파지 목표점)** 다.
        # 플래너는 과실 중심 +z_bias(35mm) 의 줄기를 겨냥한다(pick_target_z_bias_m).
        # 과실 중심으로 재면 lateral 에 항상 35mm 가 얹혀 판별이 흐려진다.
        # 줄기 기준이면 정상 파지에서 lateral ~ 0, along = 오프셋 그대로가 된다.
        stems = [np.asarray(b, dtype=float) + np.array([0.0, 0.0, self.grasp_z_bias])
                 for b in self.strawberries]
        near_idx = min(range(len(stems)), key=lambda i: float(np.linalg.norm(tcp - stems[i])))
        near = stems[near_idx]
        self.last_grasp_berry = np.asarray(self.strawberries[near_idx], dtype=float)   # [T2] 부착 대상
        along_mm = lateral_mm = float("nan")
        try:
            _st = torch.tensor([self.current_joints], device="cuda:0", dtype=torch.float32)
            _q = self.ik_solver.kinematics.get_state(_st).ee_quaternion[0].cpu().numpy()
            _z = ScipyRotation.from_quat([_q[1], _q[2], _q[3], _q[0]]).apply([0.0, 0.0, 1.0])
            _v = np.asarray(near, dtype=float) - np.asarray(tcp, dtype=float)
            along_mm = float(np.dot(_v, _z)) * 1000.0
            lateral_mm = float(np.linalg.norm(_v - np.dot(_v, _z) * _z)) * 1000.0
        except Exception:                                        # noqa: BLE001
            pass
        # ★ [FIX 2026-09-10] 판정 기준을 d_tcp(스칼라) -> **along/lateral** 로.
        #
        # d_tcp 는 깊이 오차에 둔감하다. 15mm 짧게 닫혀도 35.8mm 라 정상 32.1mm 와
        # 3.7mm 밖에 차이가 안 난다 — 반경을 어디에 두든 "앞에서 닫힘"을 못 거른다.
        # 진입 방향의 z 성분(0.402) 때문에 오프셋과 z bias 가 상쇄되기 때문이다.
        # along(툴 축 성분)은 깊이 오차를 그대로 준다: 정상 +0.9mm, 15mm 짧으면 +15.9mm.
        # 조우가 실제로 무는 구간(실측): ee+230.5 ~ ee+262.5mm.
        # 플래너 TCP 는 ee+236mm 이므로 TCP 기준 **-5.5 ~ +26.5mm** 다.
        # 줄기가 이 안에 들어와야 물린다. 오프셋 15mm -> along +15 (물림),
        # 오프셋 40mm -> along +40 (조우보다 앞, 못 뭄).
        if not (math.isnan(along_mm) or math.isnan(lateral_mm)):
            hit = (self.jaw_near_mm <= along_mm <= self.jaw_far_mm
                   and lateral_mm <= self.grasp_lateral_tol * 1000.0)
        else:
            hit = d_tcp <= self.grasp_radius   # FK 실패 시 종전 방식으로 폴백
        self.last_grasp_reason = (
            "줄기기준 along=%+.1fmm(조우 %.0f~%.0f) lateral=%.1fmm(허용%.0f) "
            "| 과실기준 d_tcp=%.1fmm -> %s"
            % (along_mm, self.jaw_near_mm, self.jaw_far_mm, lateral_mm,
               self.grasp_lateral_tol * 1000, d_tcp * 1000,
               "CONTACT" if hit else "EMPTY")
        )
        # 두 거리를 모두 남긴다: tool_offset이 실제와 다르면 d_ee 쪽이 더 작게 나오므로
        # 로그만 보고 오프셋을 교정할 수 있다.
        self.get_logger().info("GRASP_JUDGE %s" % self.last_grasp_reason)
        return hit

    def _publish_joint_command(self, joint_positions_deg):
        """팔 6축 + 그리퍼 4축을 함께 발행한다.

        Isaac 브릿지 스크립트가 msg.name 으로 DOF 를 찾아 매핑하므로 순서는
        무관하다. 그리퍼를 매번 같이 실어 보내는 이유는, 팔만 보내면 스크립트가
        나머지 DOF 를 현재값으로 채워 그리퍼가 영영 초기 상태에 머물기 때문이다.
        """
        # ⚠️ name/position 은 **각각 한 번만** 대입한다.
        # rclpy 는 대입 즉시 position 을 array.array('d') 로 바꾸므로, 대입한 뒤
        # `msg.position = msg.position + [...]` 로 덧붙이면
        #   TypeError: can only append array (not "list") to array
        # 가 나서 첫 SetPosition/MoveJoint 에서 노드가 죽는다 (2026-09-08 발생).
        # name 은 평범한 list 라 통과해서 position 만 터진다 — 눈에 잘 안 띈다.
        # [GUARD 2026-09-09] 관절 한계를 벗어난 명령은 **발행하지 않는다.**
        # 2026-09-09 의 MoveSpline 단위 버그(deg/rad 혼용, 57.3배)는 수천 도짜리
        # 명령을 조용히 흘려보내 로봇이 판스톱까지 몇 바퀴 도는 동작을 만들었고,
        # 어느 로그에도 흔적이 없었다. 단위 실수는 언제든 재발할 수 있으므로
        # 발행 직전에 한 번 거른다. 값은 robot.urdf 관절 한계 + 여유 5도.
        if any(abs(v) > lim for v, lim in
               zip(joint_positions_deg, self.ARM_JOINT_LIMIT_DEG)):
            self.get_logger().error(
                "JOINT_COMMAND_REJECTED 관절 한계 초과 — [%s] deg (한계 %s). "
                "단위(deg/rad) 오류일 가능성이 높다."
                % (" ".join("%.1f" % v for v in joint_positions_deg),
                   self.ARM_JOINT_LIMIT_DEG))
            return
        self.last_arm_command_deg = [float(v) for v in joint_positions_deg]
        arm_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        arm_rad = [math.radians(j) for j in joint_positions_deg]
        if len(arm_rad) != len(arm_names):
            self.get_logger().warn(
                f"MoveJoint payload has {len(arm_rad)} values, expected "
                f"{len(arm_names)} — publishing arm joints only")
            arm_names = arm_names[:len(arm_rad)]
        grip_rad = self._gripper_joint_rad()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = arm_names + list(self.GRIPPER_JOINT_NAMES)
        msg.position = arm_rad + [grip_rad] * len(self.GRIPPER_JOINT_NAMES)
        self.joint_pub.publish(msg)

    def _wait_for_arm_arrival(self, target_deg, label: str) -> bool:
        """팔이 실제로 목표에 닿을 때까지 기다린다.

        [FIX 2026-09-09] 종전에는 명령을 다 발행하면 곧바로 success 를 되돌렸다.
        실기 Doosan 서비스는 **모션이 끝나야** 응답하므로, 노드는 응답을 곧 도착으로
        믿고 다음 단계를 시작한다. 시뮬에서만 그 약속이 깨져 있었고, 그래서
          - 복귀가 덜 끝난 자세에서 다음 딸기 경로가 계산되고
          - 노드가 pick 시작 자세로 저장하는 current_joints 가 흔들렸다.
        타임아웃돼도 경고만 남기고 True 를 되돌린다 — 시퀀스를 막지 않는다.
        """
        if not self.joint_state_received:
            time.sleep(0.2)
            return True
        deadline = time.time() + self.ARM_ARRIVAL_TIMEOUT_SEC
        worst = float("inf")
        while time.time() < deadline:
            cur = [math.degrees(j) for j in self.current_joints]
            worst = max(abs(c - t) for c, t in zip(cur, target_deg))
            if worst <= self.ARM_ARRIVAL_TOL_DEG:
                return True
            time.sleep(0.02)
        # 잔차를 손끝 오차(mm)로도 환산해 남긴다 — 이 값이 그대로 다음 상대
        # MoveLine 에 전파되어 "파지점보다 앞에서 닫힘" 으로 나타난다.
        tip_err_mm = float("nan")
        try:
            cur = [math.radians(v) for v in
                   [math.degrees(j) for j in self.current_joints]]
            fk_now = self.ik_solver.kinematics.get_state(
                torch.tensor([cur], device="cuda:0", dtype=torch.float32))
            fk_tgt = self.ik_solver.kinematics.get_state(
                torch.tensor([[math.radians(v) for v in target_deg]],
                             device="cuda:0", dtype=torch.float32))
            tip_err_mm = float(np.linalg.norm(
                fk_now.ee_position[0].cpu().numpy()
                - fk_tgt.ee_position[0].cpu().numpy())) * 1000.0
        except Exception:                                        # noqa: BLE001
            pass
        self.get_logger().error(
            "ARM_ARRIVAL_TIMEOUT %s: %.1fs 안에 목표에 못 닿음 "
            "(최대 관절오차 %.1fdeg, 손끝오차 %.1fmm) — 이 잔차는 다음 상대 이동에 "
            "그대로 전파된다"
            % (label, self.ARM_ARRIVAL_TIMEOUT_SEC, worst, tip_err_mm))
        return False

    # --- Motion Services ---

    def move_spline_cb(self, req, res):
        self.get_logger().info(f"MoveSplineJoint called with {req.pos_cnt} points")
        # [FIX 2026-09-08] 웨이포인트 사이를 관절공간에서 보간해 실행한다.
        #
        # 플래너는 cuRobo 궤적을 **12점으로 다운샘플**해 보낸다. 종전에는 그 12점을
        # 0.1s 간격으로 그대로 던지기만 해서, 실제 경로가 계획된 궤적이 아니라
        # 웨이포인트 사이를 아티큘레이션 드라이브가 제멋대로 잇는 모양이 됐다.
        # 배치 이송처럼 보드 근처에서 시작하는 구간에서 코너를 잘라 먹어 보드에
        # 부딪히는 것처럼 보였다. 사이를 촘촘히 채우면 계획 궤적을 훨씬 가깝게 따른다.
        # ⚠️ [FIX 2026-09-09] 단위. req.pos 는 **도(deg)**, current_joints 는 **라디안(rad)** 이다.
        #
        # 직전 보간 코드는 둘을 섞어 보간한 뒤 결과에 math.degrees 를 또 입혔다.
        # _publish_joint_command 는 입력을 deg 로 보고 radians() 로 되돌리므로,
        # 순수하게 **57.2958배**가 그대로 관절 명령이 됐다.
        #   스캔 포즈(87.98,-94.92,129.89,175.94,-31.34,93.42)에서 첫 웨이포인트로 갈 때
        #   f=1 발행값 = 5156.6 / -5156.6 / 6875.5 / 9740.3 / -1718.9 / 5443.1 deg
        # 즉 모든 관절에 수천 도를 명령해 판스톱까지 몇 바퀴씩 돌며 서로 박는 동작이 난다
        # (2026-09-09 사용자 보고: "한 바퀴 돌면서 관절들이 전부 충돌하는 배배 꼬는 동작").
        # sub 계산에도 같은 오류가 있어 한 구간을 4783토막으로 쪼개어 20us 간격으로 마구 발행했다.
        # move_spline_cb 는 항상 success=True 를 되돌려 로그에도 안 남았다.
        # 모든 계산을 **deg 로 통일**한다.
        pts = [list(p.data) for p in req.pos]        # deg
        if not pts:
            res.success = True
            return res
        cur_deg = [math.degrees(j) for j in self.current_joints]
        if self.joint_state_received and len(cur_deg) == len(pts[0]):
            prev = cur_deg
        else:
            # /dsr01/joint_states 미수신(Isaac Play 전)이면 current_joints 는 초깃값 [0]*6 이라
            # 그걸 시작점으로 쓰면 영점 자세를 경유하는 큰 사톱이 생긴다.
            self.get_logger().warn(
                "MoveSpline: joint_states 미수신 — 첫 웨이포인트를 시작점으로 사용한다")
            prev = list(pts[0])
        # [FIX 2026-09-09] 구간 시간을 **req.time 을 이동량 비율로 배분**해 정한다.
        # 종전의 구간당 고정 0.10s 는 12점 스플라인을 이동량과 무관하게 ~1.2s 만에
        # 발행해버려, 100도 넘게 도는 복귀 구간에서 아티큘레이션이 따라오지 못했다.
        # 노드가 보내는 req.time 은 궤적 호길이/속도로 계산한 값이라 그대로 쓰면 된다.
        spans = []
        cursor = prev
        for tgt in pts:
            spans.append(max(abs(a - b) for a, b in zip(tgt, cursor)))
            cursor = tgt
        total_span = sum(spans) or 1.0
        total_sec = max((float(req.time) if req.time > 0 else 0.0) / self.speed_scale,
                        self.SPLINE_MIN_TOTAL_SEC,
                        total_span / self.SPLINE_MAX_JOINT_SPEED_DEG_S)
        for tgt, span in zip(pts, spans):
            sub = max(1, int(math.ceil(span / self.SPLINE_STEP_DEG)))
            step_sleep = max(self.SPLINE_MIN_STEP_SLEEP_SEC,
                             total_sec * (span / total_span) / sub)
            for i in range(1, sub + 1):
                f = i / float(sub)
                self._publish_joint_command(
                    [p + (t - p) * f for p, t in zip(prev, tgt)])
                time.sleep(step_sleep)
            prev = list(tgt)
        # 실기 MoveSplineJoint 는 모션이 끝나야 응답한다. 시뮬도 그렇게 맞춘다.
        self._wait_for_arm_arrival(pts[-1], "MoveSpline")
        self.get_logger().info(
            "MoveSpline ok: %d pts / %.2fs -> end=[%s]deg"
            % (len(pts), total_sec, " ".join("%.1f" % v for v in pts[-1])))
        res.success = True
        return res

    def move_joint_cb(self, req, res):
        self.get_logger().info(f"MoveJoint called to {req.pos}")
        self._publish_joint_command(req.pos)
        if req.time > 0:
            time.sleep(req.time)
        self._wait_for_arm_arrival(list(req.pos), "MoveJoint")
        res.success = True
        return res

    def move_line_cb(self, req, res):
        self.get_logger().info(f"MoveLine called (pos={req.pos}, ref={req.ref}, mode={req.mode})")
        if self.ik_solver is None:
            self.get_logger().warn("IK Solver not available, faking MoveLine success.")
            res.success = True
            return res

        try:
            # mode=1 (REL) 만 지원. mode=0 (절대 좌표 movel)은 시뮬 파이프라인에서 사용하지 않음
            if req.mode != 1:
                self.get_logger().error(
                    f"MoveLine mode={req.mode} is not supported by the sim bridge (relative only)")
                res.success = False
                return res

            # 현재 FK 구하기
            start_state = torch.tensor([self.current_joints], device="cuda:0", dtype=torch.float32)
            fk_result = self.ik_solver.kinematics.get_state(start_state)
            current_pose = Pose(position=fk_result.ee_position.clone(),
                                quaternion=fk_result.ee_quaternion.clone())
            _start_ee = fk_result.ee_position[0].detach().cpu().numpy().copy()

            delta_m = np.array([req.pos[0], req.pos[1], req.pos[2]]) / 1000.0

            if req.ref == 1:
                # DR_TOOL: 델타가 툴 좌표계 기준 → 현재 EE 자세로 회전시켜 베이스 기준으로 변환
                q_wxyz = current_pose.quaternion[0].cpu().numpy()
                delta_m = ScipyRotation.from_quat(
                    [q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]]).apply(delta_m)

            # [FIX 2026-09-08] 직선을 실제로 보간해서 간다.
            #
            # 종전에는 요청 거리 전체(예: 45mm)를 **IK 한 번으로 점프**했다. 그래서
            #   (a) 화면상 직선 이동이 아니라 순간이동이었고,
            #   (b) 종점 IK 해가 현재 자세에서 MOVELINE_MAX_JOINT_DELTA_DEG(45도)를
            #       넘게 떨어지면 통째로 실패했다.
            #       실측: sw 첫 딸기 (-100,645,440)mm 에서 매번
            #       "FINAL_APPROACH_STRAIGHT MoveLine failed" → "ABORT: 직선 진입 실패".
            #       cuRobo 가 도달 불가라고 한 게 아니라 이 가드에 걸린 것이다.
            # 5mm 씩 끊어 직전 해를 seed 로 이어 풀면 스텝당 이동량이 작아 가드에
            # 걸리지 않고, 경로도 실제 직선이 된다. 가드 자체는 elbow-flip 방지에
            # 여전히 필요하므로 값은 그대로 둔다.
            dist_mm = float(np.linalg.norm(delta_m)) * 1000.0
            n_steps = max(1, int(math.ceil(dist_mm / self.MOVELINE_STEP_MM)))
            if n_steps > self.MOVELINE_MAX_STEPS:
                self.get_logger().info(
                    f"MoveLine {dist_mm:.0f}mm: {n_steps} -> {self.MOVELINE_MAX_STEPS} steps "
                    f"({dist_mm / self.MOVELINE_MAX_STEPS:.1f}mm/step, 자유공간 이송)")
                n_steps = self.MOVELINE_MAX_STEPS
            vel_mm_s = float(req.vel[0]) if len(req.vel) > 0 and req.vel[0] > 0 else 60.0
            total_sec = min(dist_mm / vel_mm_s / self.speed_scale, self.MOVELINE_MAX_TOTAL_SEC)
            step_sleep = max(self.MOVELINE_MIN_STEP_SLEEP_SEC, total_sec / n_steps)

            base_pos = current_pose.position.clone()
            cur_state = start_state
            worst_delta = 0.0
            ok = True
            t_loop0 = time.time()       # [DIAG 2026-09-10] 실소요. total_sec 은 계획값일 뿐이다
            for i in range(1, n_steps + 1):
                frac = i / float(n_steps)
                step_pos = base_pos.clone()
                for axis in range(3):
                    step_pos[0, axis] = base_pos[0, axis] + float(delta_m[axis]) * frac
                sol, dd = self._solve_ik_nearest(
                    Pose(position=step_pos, quaternion=current_pose.quaternion),
                    cur_state)
                if sol is None:
                    self.get_logger().warn(
                        f"MoveLine IK Failed at step {i}/{n_steps} "
                        f"({dist_mm*frac:.0f}/{dist_mm:.0f}mm, best delta={dd:.1f}deg)")
                    ok = False
                    break
                worst_delta = max(worst_delta, dd)
                self._publish_joint_command([math.degrees(j) for j in sol])
                cur_state = torch.tensor([sol], device="cuda:0", dtype=torch.float32)
                time.sleep(step_sleep)
            if ok:
                arrived = self._wait_for_arm_arrival(
                    [math.degrees(j) for j in cur_state[0].cpu().numpy()], "MoveLine")
                # [DIAG 2026-09-10] **실제로 간 거리**. 명령은 45mm 인데 드라이브가
                # 못 따라오면 짧게 끝나고 그대로 파지에 들어간다.
                try:
                    _now = self.ik_solver.kinematics.get_state(torch.tensor(
                        [self.current_joints], device="cuda:0", dtype=torch.float32))
                    # ⚠️ cuRobo get_state() 는 내부 버퍼를 재사용한다. fk_result 를
                    # 그대로 들고 있으면 이후 호출이 덮어써서 항상 0mm 가 나온다
                    # (2026-09-10 실측: 전 구간 "실제 0.0mm" 오보).
                    _end_ee = _now.ee_position[0].cpu().numpy().copy()
                    _moved = float(np.linalg.norm(_end_ee - _start_ee)) * 1000.0
                    # [DIAG 2026-09-10] **방향까지** 본다.
                    #
                    # 위 _moved 는 시작점~끝점의 스칼라 거리다. 옆으로 밀려도
                    # 거리만 맞으면 통과한다. 실제로 그런 일이 있었다: 09-10 13:12 런의
                    # 배치 하강(BASE -Z 120mm)은 전부 "실이동 115.8~116.6mm" 로 정상
                    # 판정됐는데, 과실 정지 위치를 역산하면 그리퍼가 계획 자세에서
                    # 최대 21mm 어긋나 있었다 (계란판 배치가 들쭉날쭉해 보인 원인).
                    #
                    # 종점 오차를 명령 축 방향(along)과 그 수직 성분(lateral)으로
                    # 나눠 남긴다. along 은 종전 부족분과 같은 정보이고, lateral 이
                    # 새로 보이는 값이다. **판정·동작은 바꾸지 않는다 — 계측만 한다.**
                    # 파지 조우(TCP)보다 과실이 툴 축으로 ~250mm 앞에 매달리므로
                    # 이 lateral 은 과실 위치에서 그대로 또는 더 크게 나타난다.
                    _err = _end_ee - (_start_ee + delta_m)
                    _axis = delta_m / max(float(np.linalg.norm(delta_m)), 1e-9)
                    _along_mm = float(np.dot(_err, _axis)) * 1000.0
                    _lat_mm = float(np.linalg.norm(
                        _err - np.dot(_err, _axis) * _axis)) * 1000.0
                    _endlog = (self.get_logger().warn
                               if _lat_mm > self.MOVELINE_LATERAL_WARN_MM
                               else self.get_logger().info)
                    _endlog("MOVELINE_END_ERR: along=%+.1fmm lateral=%.1fmm |err|=%.1fmm "
                            "(명령 %.0fmm, %s)"
                            % (_along_mm, _lat_mm,
                               float(np.linalg.norm(_err)) * 1000.0, dist_mm,
                               "TOOL" if req.ref == 1 else "BASE"))
                    # [FIX 2026-09-10] 임계값은 스텝 길이를 따른다. 3mm 는 2mm 스텝 기준값이라
                    # 스텝 상한에 걸린 5mm 스텝 이동(배치 하강 120mm)에서 매번 거짓 경고가 났다
                    # (11:05 런: 부족 3.5~4.1mm ×6, 다음 동작이 관절공간 절대 목표라 잔차 무의미).
                    _short_tol = max(3.0, dist_mm / n_steps)
                    if abs(_moved - dist_mm) > _short_tol or not arrived:
                        # [FIX 2026-09-10] 결과 문구를 호출 구간에 맞춘다.
                        #
                        # 종전에는 어떤 MoveLine 이든 "이만큼 파지점 앞에서 닫힌다"
                        # 로 끝났다. 2026-09-10 02:02 런에서 이 경고 5건은 **전부
                        # 후퇴**(TOOL -Z 45mm) 구간이었고, 후퇴에는 닫는 동작이
                        # 아예 없다 — 로그만 보고 파지 문제로 오독하게 된다.
                        # (09-09 17:23 의 손끝오차 오보와 같은 종류의 함정이다.)
                        #
                        # 브릿지는 플래너의 단계 이름을 모르므로 **방향으로만**
                        # 구분한다. 파지점을 향해 들어가는 구간은 TOOL +z 하나뿐이다:
                        #   TOOL +z : 진입 -> 부족분이 그대로 "덜 들어간 채 닫힘"
                        #   그 외   : 하강·분리·후퇴·배치 -> 잔차가 다음 상대 이동에 전파
                        # 축에 안 붙은 델타까지는 구분하지 않는다. 지금 시퀀스의
                        # MoveLine 은 전부 한 축이다.
                        _frame = "TOOL" if req.ref == 1 else "BASE"
                        _dz_mm = float(req.pos[2]) if len(req.pos) > 2 else 0.0
                        _tail = ("이만큼 파지점 앞에서 닫힌다"
                                 if (req.ref == 1 and _dz_mm > 0.0)
                                 else "이 잔차는 다음 상대 이동에 그대로 전파된다")
                        self.get_logger().error(
                            "MOVELINE_SHORT: 명령 %.0fmm 인데 실제 %.1fmm "
                            "(부족 %.1fmm, 도착판정 %s, %s dz=%+.1fmm) — %s"
                            % (dist_mm, _moved, dist_mm - _moved, arrived,
                               _frame, _dz_mm, _tail))
                    else:
                        self.get_logger().info(
                            "MoveLine 실이동 %.1fmm / 명령 %.0fmm" % (_moved, dist_mm))
                except Exception:                                # noqa: BLE001
                    pass
                self.get_logger().info(
                    f"MoveLine ok: {dist_mm:.0f}mm / {n_steps} steps / 계획 {total_sec:.2f}s "
                    f"실소요 {time.time() - t_loop0:.2f}s "
                    f"(worst step joint delta {worst_delta:.1f}deg)")
            res.success = ok

        except Exception as e:
            self.get_logger().error(f"MoveLine Exception: {e}")
            res.success = False

        return res

    def _solve_ik_nearest(self, goal_pose, start_state):
        """현재 관절에서 가장 가까운 IK 해를 고르고, 먼 브랜치 해는 걸러낸다.

        cuRobo IK는 seed가 무작위라 같은 목표에도 ~20% 확률로 elbow-flip 해를
        반환하고, 드물게는 8개 seed 전부가 먼 브랜치로 수렴한다 (실측 2026-07-16).
        따라서 최근접 선택 + 관절 이동량 상한 가드 + 재시도가 모두 필요하다.
        반환: (해 리스트 or None, 최선 해의 최대 관절 이동량 deg)
        """
        start_np = start_state[0].cpu().numpy()
        best_sol, best_delta = None, float("inf")
        for _ in range(self.MOVELINE_IK_RETRIES):
            # [FIX 2026-09-08] ★ seed_config 로 **현재 관절에서 시작**하게 한다.
            #
            # 종전에는 start_state 를 2번째 위치인자로 넘겼는데, 그 자리는
            #   solve_single(goal_pose, retract_config, seed_config, ...)
            # 의 **retract_config** (널스페이스 정규화 목표)다. 시드가 아니다.
            # 그래서 cuRobo 는 매번 무작위 seed 에서 최적화를 시작했고, 어느 IK
            # 브랜치로 수렴할지 보장이 없었다 — docs/concepts.md 가 "같은 목표
            # 30번 중 6번이 팔꿈치 반전" 이라고 기록한 그 현상의 원인이다.
            # 최근접 선택(return_seeds)만으로는 8개 후보가 전부 먼 브랜치면 못 구한다.
            #
            # 실측(45mm 직선을 9스텝 보간, 익은 딸기 6개):
            #   retract_config 만  -> 6/6 실패, 스텝당 이동량 179~181deg (팔꿈치 반전)
            #   seed_config 사용   -> 6/6 통과, 스텝당 이동량 **1deg**
            seed = start_state.view(1, 1, -1).repeat(1, self.IK_NUM_SEEDS, 1)
            result = self.ik_solver.solve_single(
                goal_pose, start_state, seed_config=seed, return_seeds=8)
            success = result.success.view(-1).cpu().numpy()
            solutions = result.solution.view(len(success), -1).cpu().numpy()
            for idx in np.where(success)[0]:
                delta_deg = math.degrees(float(np.abs(solutions[idx] - start_np).max()))
                if delta_deg < best_delta:
                    best_sol, best_delta = solutions[idx], delta_deg
            if best_delta <= self.MOVELINE_MAX_JOINT_DELTA_DEG:
                return best_sol.tolist(), best_delta
        return None, best_delta

    def change_speed_cb(self, req, res):
        res.success = True
        return res

    # --- Gripper Services / Action ---

    # 완전히 닫으라는 명령으로 간주하는 개도. planner의 close는 set_position(700).
    GRIPPER_CLOSE_COMMAND_MIN = 690
    GRIPPER_POS_ON_CONTACT = 670    # 딸기에 걸려 멈춘 개도 (<=685 → 접촉 판정)
    GRIPPER_POS_ON_EMPTY = 700      # 끝까지 닫힘   (>=695 → 빈손 판정)

    # ── [T2 2026-09-10] 부착/해제 이벤트 ─────────────────────────────────
    def _on_gripper_close(self, hit):
        """CONTACT 판정이면 그 과실을 ATTACH 로 지목한다. 이미 부착 중이면 중복 발행 안 함."""
        if not hit or self.last_grasp_berry is None or self.attached_berry is not None:
            return
        b = self.last_grasp_berry
        msg = String()
        msg.data = "ATTACH %.4f %.4f %.4f" % (float(b[0]), float(b[1]), float(b[2]))
        self.grasp_event_pub.publish(msg)
        self.attached_berry = b
        self.get_logger().info("GRASP_ATTACH 과실 (%.0f, %.0f, %.0f)mm -> /sim/grasp_event"
                               % (b[0] * 1000, b[1] * 1000, b[2] * 1000))

    def _on_gripper_open(self):
        """부착 중에 조우가 열리면 RELEASE. (스캔 전 pre-close 600 은 부착 중이 아니라 무시된다.)"""
        if self.attached_berry is None:
            return
        msg = String()
        msg.data = "RELEASE"
        self.grasp_event_pub.publish(msg)
        self.get_logger().info("GRASP_RELEASE -> /sim/grasp_event")
        self.attached_berry = None

    def set_position_cb(self, req, res):
        self.get_logger().info(f"SetPosition called to {req.position}")
        if req.position >= self.GRIPPER_CLOSE_COMMAND_MIN:
            # 닫기 명령: 명령값을 그대로 되돌리면 항상 빈손이 된다.
            # 조우 사이에 딸기가 있으면 거기서 멈추므로 접촉 개도를 리드백한다.
            hit = self._judge_grasp()
            if hit is None:
                self.gripper_position = req.position     # 판정 불가 → 종전 동작
            else:
                self.gripper_position = (self.GRIPPER_POS_ON_CONTACT if hit
                                         else self.GRIPPER_POS_ON_EMPTY)
                self._on_gripper_close(hit)                      # [T2]
        else:
            self.gripper_position = req.position
            self._on_gripper_open()                              # [T2]
        # [Stage1] 숫자만 바꾸지 말고 실제로 시뮬 조우를 움직인다.
        # 시각 개도는 **명령값** 기준 (판정 리드백과 분리 — _gripper_joint_rad 주석 참조).
        self.gripper_command = int(req.position)
        self._publish_gripper_only()
        res.success = True
        res.message = "Simulated Gripper Moved"
        return res

    def get_state_cb(self, req, res):
        res.success = True
        res.state.present_position = self.gripper_position
        res.state.present_current = 200 # 모의 전류값
        return res

    def safe_grasp_cb(self, goal_handle):
        self.get_logger().info(f"SafeGrasp action called to {goal_handle.request.target_position}")

        time.sleep(0.25 / self.speed_scale * 2.0)
        # 이전에는 670을 하드코딩해 무조건 성공이었다. 이제 기하로 판정한다.
        hit = self._judge_grasp()
        if hit is None:
            hit = True   # 판정 불가(딸기 좌표 미수신 등) → 종전처럼 성공 처리
        self.gripper_position = (self.GRIPPER_POS_ON_CONTACT if hit
                                 else self.GRIPPER_POS_ON_EMPTY)
        self._on_gripper_close(hit)                                  # [T2]
        self.gripper_command = int(goal_handle.request.target_position)
        self._publish_gripper_only()   # [Stage1] 실제 조우 구동

        goal_handle.succeed()

        result = SafeGrasp.Result()
        result.success = bool(hit)
        result.final_position = self.gripper_position
        result.final_current = 250 if hit else 5
        result.reason = ("GRASP_CONTACT_DETECTED" if hit else "GRASP_EMPTY")
        return result

def main(args=None):
    rclpy.init(args=args)
    node = SimExecutorBridgeNode()
    
    # Use MultiThreadedExecutor for action servers and multiple services
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
