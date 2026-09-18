#!/usr/bin/env python3
"""결과 바 '분리 실패' 규칙 오프라인 재현 검사 — 09-18 재정의. 런타임 노드가 아니다.

실행기 **원본 클래스** PickSequenceExecutor(src/strawberry_motion/scripts/pick_sequence_executor.py)를
가짜 주입물(로거·모션 함수·그리퍼·트레이 실행기)로 세워 `run()` 을 실제 제어 흐름 그대로 돌리고,
그 위에 HUD 프로브를 실제로 붙여(harvest_probe.attach("planner", 가짜 노드)) 결과 칸이 규칙대로
붙는지 본다. 판정 규칙 자체(result_bar.outcome_of_pick_end)는 프로브가 부르는 대로 쓰고
여기서 다시 구현하지 않는다 — 규칙을 베껴 쓰면 규칙이 틀려도 검사가 통과한다.

확인하는 것 (시나리오 = run() 한 번):
  * 직선 진입 실패 / 열린 조우 하강 실패 / 그리퍼 닫기 실패(+후퇴까지 실패) / 빈손 게이트 차단 /
    후퇴 실패 / 진입 뒤 run() 예외 → 결과 칸 정확히 +1 'detach_failed', 로그 사유(straight_entry,
    open_stem_descent, grasp=…, retreat_failed, run() raised after …)
  * 배치 성공 → +1 'placed', 배치 실패 → +1 'place_failed' (run 종료 판정이 칸을 더 붙이지 않는다)
  * 당김 명령만 실패 → 결과는 배치 쪽에서 정해지고 '당김 명령 실패' 한 줄만 남는다
  * 회색(+0): 파지 후보 전부 IK 실패, 프리어프로치 스플라인 실패, x 가드 스킵
  * 배치 기능이 꺼진 구성 → +0
  * 두 픽 연속 → 픽당 상태가 리셋돼 결과가 하나씩만
  * 단계 진행 바 상태 순서 ENTER → GRASP → DETACH → RETREAT → PLACE → RETURN
  * labels/manifest.json 의 DETACH 라벨 '당김', 분리 실패 범례 '분리 실패' 폭 89.0
  * 래핑 지점 이름이 하나라도 사라지면 프로브가 남기는 '계측 지점 없음' 경고를 실패로 본다

각 시나리오는 status_bus.reset() 뒤에 돌려서 result.outcomes 목록 전체가 그 픽들의 결과가 되게 한다
(증가분 = 목록 자체). succeeded·failed·dropped 도 같은 기준으로 본다.

rclpy / omni 는 쓰지 않는다 (실행기 모듈이 numpy 와 같은 패키지의 순수 정책 모듈만 부른다).
status_bus 스냅샷·로그는 임시 디렉터리로 돌려서 돌고 있는 런의 /tmp/harvest_hud_*.json 을 건드리지 않는다.

실행 (ROS 환경 불필요, 리포 루트에서):
  python3 strawberry_harvest/scripts/hud/check_result_bar_probe.py [-v] [--keep]
전부 통과하면 종료 코드 0, 하나라도 어긋나면 1 이다.
"""
import argparse
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
MOTION_SCRIPTS = os.path.join(REPO, "src", "strawberry_motion", "scripts")

# ★ 미러 파일·로그를 임시 디렉터리로. bus_sink 는 import 시점에 DEFAULT_DIR 을 읽으므로
#   hud 모듈을 import 하기 **전에** 정해야 한다.
_TMP = tempfile.mkdtemp(prefix="check_result_bar_")
os.environ["HARVEST_HUD_DIR_RUNTIME"] = _TMP
os.environ["HARVEST_HUD_LOG"] = os.path.join(_TMP, "planner_bus.log")
os.environ["HARVEST_HUD_ROLE"] = "planner"

sys.path.insert(0, HERE)
sys.path.insert(0, MOTION_SCRIPTS)

import numpy as np                                                   # noqa: E402

import bus_sink                                                      # noqa: E402
import harvest_probe                                                 # noqa: E402
import result_bar                                                    # noqa: E402
import status_bus                                                    # noqa: E402

from grasp_candidate_policy import legacy_grasp_endpoint             # noqa: E402
from harvest_motion_params import PRE_APPROACH_OFFSET                # noqa: E402
from pick_sequence_executor import PickSequenceExecutor              # noqa: E402

BUS_LOG = os.environ["HARVEST_HUD_LOG"]

#: 09-18 런(run_logs/20260918_094517)에서 4번째 시도가 막힌 타겟 root/ne/sw (280,783,700)mm.
TARGET_M = (0.280, 0.783, 0.700)

#: 스캔 프로세스 스냅샷. 프로브의 _sync_run 이 런 경계를 여기서 읽는다.
SCAN_STARTED_AT = 1758000000.0

START_JOINTS_DEG = [0.0, -20.0, 90.0, 0.0, 50.0, 0.0]
TRAJ = np.deg2rad(np.array([START_JOINTS_DEG,
                            [2.0, -20.0, 90.0, 0.0, 50.0, 0.0]], dtype=float))

#: run_nodes.sh 의 시뮬 실행 구성 (legacy_160mm, 열린 조우 하강·역순 후퇴 켜짐).
BASE_PARAMS = dict(
    measured_tcp_model=False,
    measured_tcp_plan_only=False,
    flat_grasp_only=False,
    ee_to_tcp_offset_m=0.236,
    pick_target_x_bias_m=0.0,
    pick_target_z_bias_m=0.035,
    nw_high_target_z_threshold_m=0.90,
    nw_high_target_crane_z_offset_m=0.005,
    nw_high_target_descent_extra_below_kp1_m=0.0,
    nw_high_target_base_y_nudge_m=0.0,
    leftmost_extra_advance_request_m=0.0,
    leftmost_wall_safety_margin_m=-0.030,
    leftmost_allow_wall_model_override=False,
    allow_unverified_grasp_place=False,
    execute_marker_place_release=True,
    use_taught_slot0_place_reference=True,
    hold_after_taught_slot0_place=False,
)

#: 한 픽의 대본. 시나리오는 이 값만 바꾼다.
DEFAULT_STEP = dict(
    target_m=TARGET_M,         # 09-18 런의 타겟. x 가드 밖 값을 주면 프리어프로치 전에 스킵된다
    plan_ok=True,              # cuRobo plan (파지 후보 IK)
    spline_ok=True,            # 프리어프로치 스플라인
    final_approach_ok=True,    # 직선 진입
    final_approach_raises=False,   # 직선 진입에서 예외 — run() 이 예외로 끝나는 픽(진입 뒤라 분리 실패)
    descent_ok=True,           # 열린 조우 하강 (BASE -Z)
    grasp_result="GRASP_CONTACT_DETECTED",
    pull_ok=True,              # 당김 (BASE -Z 40mm)
    retreat_ok=True,           # 후퇴 (진입 역순)
    place_status="success",
    enable_marker_place=True,
    hold_on_place_failure=False,
    open_stem_descent_enabled=True,
    straight_reverse_retreat_enabled=True,
)


# ---- 가짜 주입물 ---------------------------------------------------------

class _Logger:
    def __init__(self, sink):
        self._sink = sink

    def info(self, message):
        self._sink.append(("info", str(message)))

    def warn(self, message):
        self._sink.append(("warn", str(message)))

    def error(self, message):
        self._sink.append(("error", str(message)))

    def debug(self, message):
        self._sink.append(("debug", str(message)))


class FakeNode:
    """프로브가 보는 노드. get_logger + 실행기 두 개 + 타이머만 있으면 된다."""

    def __init__(self):
        self.lines = []
        self._logger = _Logger(self.lines)
        self.calls = []
        self.pick_start_joints = None
        self.pick_sequence_executor = None
        self.tray_place_executor = None

    def get_logger(self):
        return self._logger

    def create_timer(self, period_sec, callback):
        handle = {"period": period_sec, "callback": callback}
        self.calls.append(("create_timer", period_sec))
        return handle

    def destroy_timer(self, handle):
        self.calls.append(("destroy_timer", None))

    def _return_to_pick_start_scan_pose(self, reason):
        self.calls.append(("return_to_pick_start_scan_pose", reason))
        return True


class FakeRuntimeLog:
    def __init__(self):
        self.events = []

    def log(self, event, **fields):
        self.events.append((event, fields))


class FakeGripperClient:
    def __init__(self, sink):
        self._sink = sink

    def open_for_stem_descent(self):
        self._sink.append(("gripper", "open_for_stem_descent"))


class FakeMotionGen:
    def __init__(self, sink):
        self._sink = sink

    def detach_object_from_robot(self):
        self._sink.append(("motion_gen", "detach_object_from_robot"))


class FakeTrayPlaceExecutor:
    """트레이 배치 실행기. 프로브가 이 메서드를 감싸 배치 성공·배치 실패 칸을 붙인다."""

    def __init__(self, step):
        self._step = step
        self.calls = []

    def execute_marker_place_after_retreat(self, retreat_joints):
        self.calls.append(list(retreat_joints) if retreat_joints is not None else None)
        return self._step["place_status"], list(retreat_joints or [])


class FakeGraspSearchExecutor:
    """legacy 오프셋 탐색 — 원본 grasp_search_executor.try_legacy_grasp_offsets 와 같은 모양.

    끝점 계산은 원본과 같은 함수(legacy_grasp_endpoint)를 쓰고, 계획 호출만 가짜 plan 으로 간다.
    """

    def __init__(self, plan_fn, ee_to_tcp_offset_m):
        self._plan = plan_fn
        self._ee_to_tcp_offset_m = ee_to_tcp_offset_m

    def try_legacy_grasp_offsets(self, grasp_retry_offsets, straw, approach_dir,
                                 q_retry, pre_joints, r_pre_for_variant, variant,
                                 ee_pre, grasp_search, crane_z_offset_m=0.0):
        for grasp_offset in grasp_retry_offsets:
            grasp_search.attempt_count += 1
            if grasp_offset >= PRE_APPROACH_OFFSET:
                continue
            ee_g_try = legacy_grasp_endpoint(
                straw, grasp_offset, self._ee_to_tcp_offset_m,
                approach_dir, crane_z_offset_m)
            r_grasp = self._plan(pre_joints, ee_g_try.tolist(), q_retry, num_ik_seeds=32)
            if r_grasp is None:
                continue
            grasp_search.select_legacy_grasp(
                r_pre_for_variant, r_grasp, grasp_offset, variant,
                approach_dir, q_retry, ee_pre, ee_g_try)
            return


class _Vec:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class _Quat:
    def __init__(self, w, x, y, z):
        self.w, self.x, self.y, self.z = w, x, y, z


class _Pose:
    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation


class _Header:
    def __init__(self, frame_id):
        self.frame_id = frame_id


class FakeTargetMsg:
    """pick_pose 콜백이 받는 PoseStamped 모양."""

    def __init__(self, xyz_m):
        self.header = _Header("base")
        self.pose = _Pose(_Vec(*xyz_m), _Quat(0.5, 0.5, -0.5, 0.5))


def build_executor(step):
    """가짜 주입물로 원본 실행기 한 대를 세운다. 반환: (node, executor, tray, trace)"""
    node = FakeNode()
    trace = node.lines
    runtime_log = FakeRuntimeLog()
    tray = FakeTrayPlaceExecutor(step)

    def plan_fn(start_joints, target_pos, target_quat_wxyz, num_ik_seeds=32,
                max_attempts=None, timeout_sec=None, max_joint_delta_deg=None):
        if not step["plan_ok"]:
            return None
        return (TRAJ, 1.0)

    def execute_spline_fn(traj_rad, motion_time):
        trace.append(("spline", float(motion_time)))
        return bool(step["spline_ok"])

    def execute_base_z_relative_fn(distance_m, label, vel_mm_s=None):
        trace.append(("base_z", label))
        return bool(step["descent_ok"])

    def execute_base_relative_line_fn(delta_m, label, vel_mm_s=None, acc_mm_s2=None):
        trace.append(("base_line", label))
        return True

    def execute_tool_z_line_fn(distance_m, motion_label=None, vel_mm_s=None,
                               acc_mm_s2=None, min_distance_m=0.02):
        trace.append(("tool_z", motion_label))
        return True

    def execute_pitch_detach_fn():
        trace.append(("pitch_detach", None))
        return bool(step["pull_ok"])

    def execute_retreat_steps_fn(steps, vel_mm_s=None, acc_mm_s2=None):
        trace.append(("retreat_steps", [s["label"] for s in steps]))
        return bool(step["retreat_ok"])

    def close_and_verify_grasp_fn():
        trace.append(("close_grasp", step["grasp_result"]))
        return step["grasp_result"], 700, 120, "offline_harness"

    def execute_final_approach_fn(final_state, final_approach_distance, ret_grasp,
                                  measured_best_depth_m, used_pre_ee_pos,
                                  used_grasp_quat, used_grasp_variant, used_approach_dir):
        trace.append(("final_approach", float(final_approach_distance)))
        if step["final_approach_raises"]:
            raise RuntimeError("offline harness: 직선 진입 중 예외")
        return bool(step["final_approach_ok"])

    executor = PickSequenceExecutor(
        node=node,
        runtime_log=runtime_log,
        gripper_client=FakeGripperClient(trace),
        motion_gen=FakeMotionGen(trace),
        grasp_search_executor=FakeGraspSearchExecutor(
            plan_fn, BASE_PARAMS["ee_to_tcp_offset_m"]),
        tray_place_executor=tray,
        plan_fn=plan_fn,
        execute_spline_fn=execute_spline_fn,
        execute_base_z_relative_fn=execute_base_z_relative_fn,
        execute_base_relative_line_fn=execute_base_relative_line_fn,
        execute_tool_z_line_fn=execute_tool_z_line_fn,
        execute_pitch_detach_fn=execute_pitch_detach_fn,
        execute_retreat_steps_fn=execute_retreat_steps_fn,
        plan_to_fixed_joints_pose_fn=lambda *a, **k: (True, TRAJ),
        nearest_equivalent_joints_fn=lambda joints_deg: list(joints_deg),
        grasp_candidates_for_target_fn=lambda straw: [0.015, 0.020, 0.025],
        published_roll_grasp_variant_fn=lambda quat_wxyz: None,
        close_and_verify_grasp_fn=close_and_verify_grasp_fn,
        compute_final_approach_distance_fn=lambda *a, **k: 0.045,
        execute_final_approach_fn=execute_final_approach_fn,
        register_neighbor_obstacles_fn=lambda target: trace.append(("neighbors", "register")),
        clear_neighbor_obstacles_fn=lambda: trace.append(("neighbors", "clear")),
        reset_gripper_fn=lambda: trace.append(("gripper", "reset")),
        abort_pick_with_complete_fn=lambda: trace.append(("abort", "pick_complete")),
        publish_pick_complete_fn=lambda: trace.append(("publish", "pick_complete")),
        hold_pick_sequence_fn=lambda reason: trace.append(("hold", reason)),
        current_joints_getter=lambda: list(np.deg2rad(START_JOINTS_DEG)),
        marker_place_slot_idx_getter=lambda: 1,
        increment_marker_place_slot_idx_fn=lambda: trace.append(("slot", "increment")),
        enable_marker_place=step["enable_marker_place"],
        hold_on_place_failure=step["hold_on_place_failure"],
        open_stem_descent_enabled=step["open_stem_descent_enabled"],
        straight_reverse_retreat_enabled=step["straight_reverse_retreat_enabled"],
        **BASE_PARAMS
    )
    node.pick_sequence_executor = executor
    node.tray_place_executor = tray
    return node, executor, tray, trace


# ---- 검사 실행 -----------------------------------------------------------

def write_scan_snapshot():
    """프로브가 런 경계를 읽는 스캔 프로세스 스냅샷을 임시 디렉터리에 만든다."""
    payload = {"role": "scan", "written_at": SCAN_STARTED_AT, "pid": 0,
               "state": {"run": {"started_at": SCAN_STARTED_AT}}}
    with open(bus_sink.path_for("scan"), "w", encoding="utf-8") as f:
        json.dump(payload, f)


def bus_log_size():
    try:
        return os.path.getsize(BUS_LOG)
    except OSError:
        return 0


def sequence_states_since(offset):
    states = []
    try:
        with open(BUS_LOG, encoding="utf-8") as f:
            f.seek(offset)
            for line in f:
                parts = line.rstrip("\n").split("|", 2)
                if len(parts) == 3 and parts[1] == "sequence":
                    fields = json.loads(parts[2])
                    if "state" in fields:
                        states.append(fields["state"])
    except OSError:
        pass
    return states


def in_order(states, wanted):
    it = iter(states)
    return all(any(seen == want for seen in it) for want in wanted)


def cell_notes(stderr_text):
    """프로브가 칸을 붙일 때마다 남기는 '결과 바 n번째 = …' 줄만 뽑는다."""
    return [line for line in stderr_text.splitlines() if "결과 바 " in line]


def run_case(case):
    steps = [dict(DEFAULT_STEP, **override) for override in case["steps"]]
    step = dict(steps[0])                      # 실행기 생성 파라미터는 첫 픽 기준
    node, executor, tray, trace = build_executor(step)

    status_bus.reset()                         # 시나리오마다 빈 결과 바에서 시작
    offset = bus_log_size()
    captured = io.StringIO()
    error = None
    with contextlib.redirect_stderr(captured):
        harvest_probe.attach("planner", node, directory=_TMP)
        try:
            for one in steps:
                step.clear()
                step.update(one)               # 같은 대본 객체를 갈아끼운다(래퍼 유지)
                try:
                    executor.run(FakeTargetMsg(step["target_m"]))
                except Exception as exc:       # noqa: BLE001 — 원본 예외가 그대로 나오는지 본다
                    error = type(exc).__name__
        finally:
            harvest_probe.detach()

    stderr_text = captured.getvalue()
    result = status_bus.snapshot()["result"]
    observed = {
        "outcomes": list(result["outcomes"]),
        "succeeded": result["succeeded"],
        "failed": result["failed"],
        "dropped": result["dropped"],
        "place_calls": len(tray.calls),
        "why": [line.split(", ")[-1].rstrip(")") for line in cell_notes(stderr_text)],
        "states": sequence_states_since(offset),
        "error": error,
        "stderr": stderr_text,
    }

    checks = []
    expect = case["expect"]
    checks.append(("결과 칸", expect["outcomes"], observed["outcomes"]))
    for key in ("succeeded", "failed", "dropped", "place_calls", "error"):
        if key in expect:
            checks.append((key, expect[key], observed[key]))
    if "why" in expect:
        checks.append(("사유", expect["why"], observed["why"]))
    if "states" in expect:
        # 순서만 본다 — 앞뒤에 PLAN·APPROACH 같은 다른 단계가 더 있어도 된다.
        checks.append(("단계 순서 포함", True, in_order(observed["states"], expect["states"])))
    if "stderr_has" in expect:
        for needle in expect["stderr_has"]:
            checks.append(("stderr '%s'" % needle, True, needle in stderr_text))
    if "stderr_lacks" in expect:
        for needle in expect["stderr_lacks"]:
            checks.append(("stderr 없음 '%s'" % needle, True, needle not in stderr_text))

    passed = all(exp == obs for _, exp, obs in checks)
    return passed, checks, observed


CASES = [
    dict(key="1", name="직선 진입 실패",
         steps=[dict(final_approach_ok=False)],
         expect=dict(outcomes=["detach_failed"], why=["straight_entry"],
                     succeeded=0, failed=0, dropped=0, place_calls=0)),
    dict(key="2", name="열린 조우 하강 실패",
         steps=[dict(descent_ok=False)],
         expect=dict(outcomes=["detach_failed"], why=["open_stem_descent"],
                     succeeded=0, failed=0, dropped=0, place_calls=0)),
    dict(key="3a", name="그리퍼 닫기 실패 (후퇴는 성공)",
         steps=[dict(grasp_result="GRIPPER_CLOSE_FAILED")],
         expect=dict(outcomes=["detach_failed"], why=["grasp=GRIPPER_CLOSE_FAILED"],
                     succeeded=0, failed=1, dropped=0, place_calls=0)),
    dict(key="3b", name="그리퍼 닫기 실패 + 후퇴도 실패(hold)",
         steps=[dict(grasp_result="GRIPPER_CLOSE_FAILED", retreat_ok=False)],
         expect=dict(outcomes=["detach_failed"], why=["grasp=GRIPPER_CLOSE_FAILED"],
                     succeeded=0, failed=1, dropped=0, place_calls=0)),
    dict(key="4", name="빈손 -> 배치 게이트 차단",
         steps=[dict(grasp_result="GRASP_EMPTY", hold_on_place_failure=False)],
         expect=dict(outcomes=["detach_failed"], why=["grasp=GRASP_EMPTY"],
                     succeeded=0, failed=1, dropped=0, place_calls=0)),
    dict(key="5", name="후퇴 실패",
         steps=[dict(retreat_ok=False)],
         expect=dict(outcomes=["detach_failed"], why=["retreat_failed"],
                     succeeded=0, failed=0, dropped=0, place_calls=0)),
    dict(key="6", name="배치 성공",
         steps=[dict()],
         expect=dict(outcomes=["placed"], why=["place_status=success"],
                     succeeded=1, failed=0, dropped=0, place_calls=1)),
    dict(key="7", name="배치 실패",
         steps=[dict(place_status="failed", hold_on_place_failure=False)],
         expect=dict(outcomes=["place_failed"], why=["place_status=failed"],
                     succeeded=0, failed=0, dropped=1, place_calls=1)),
    dict(key="8", name="당김 명령만 실패",
         steps=[dict(pull_ok=False)],
         expect=dict(outcomes=["placed"], succeeded=1, dropped=0, place_calls=1,
                     stderr_has=["당김 명령 실패"])),
    dict(key="9a", name="파지 후보 전부 거부 (회색)",
         steps=[dict(plan_ok=False)],
         expect=dict(outcomes=[], succeeded=0, failed=0, dropped=0, place_calls=0,
                     stderr_lacks=["결과 바 "])),
    dict(key="9b", name="프리어프로치 스플라인 실패 (회색)",
         steps=[dict(spline_ok=False)],
         expect=dict(outcomes=[], succeeded=0, failed=0, dropped=0, place_calls=0,
                     stderr_lacks=["결과 바 "])),
    dict(key="9c", name="x 가드로 프리어프로치 전 스킵 (회색)",
         steps=[dict(target_m=(0.60, 0.783, 0.700))],
         expect=dict(outcomes=[], succeeded=0, failed=0, dropped=0, place_calls=0,
                     stderr_lacks=["결과 바 "])),
    # 진입 뒤 run() 이 예외로 끝나도 분리 실패 칸이 붙는다(프로브 _wrap 의 always). 예외는 그대로 밖으로 나온다.
    dict(key="9d", name="진입 뒤 run() 이 예외로 끝난 픽 (분리 실패, 예외는 통과)",
         steps=[dict(final_approach_raises=True)],
         expect=dict(outcomes=["detach_failed"], why=["run() raised after straight_entry"],
                     error="RuntimeError", place_calls=0)),
    dict(key="10", name="배치 기능 꺼짐",
         steps=[dict(enable_marker_place=False)],
         expect=dict(outcomes=[], succeeded=0, failed=0, dropped=0, place_calls=0)),
    dict(key="11", name="두 픽 연속 (진입 실패 -> 배치 성공)",
         steps=[dict(final_approach_ok=False), dict()],
         expect=dict(outcomes=["detach_failed", "placed"],
                     why=["straight_entry", "place_status=success"],
                     succeeded=1, failed=0, dropped=0, place_calls=1)),
    dict(key="12", name="정상 픽의 단계 순서",
         steps=[dict()],
         expect=dict(outcomes=["placed"],
                     states=["ENTER", "GRASP", "DETACH", "RETREAT", "PLACE", "RETURN"])),
]


def check_labels():
    """단계 진행 바 DETACH 라벨과 결과 바 범례 라벨 (make_labels.py 가 구운 manifest)."""
    with open(os.path.join(HERE, "labels", "manifest.json"), encoding="utf-8") as f:
        manifest = json.load(f)
    checks = [
        ("state_DETACH.text", "당김", manifest["state_DETACH"]["text"]),
        ("final_detach_failed.text", "분리 실패", manifest["final_detach_failed"]["text"]),
        ("final_detach_failed.w", 89.0, manifest["final_detach_failed"]["w"]),
        ("result_bar.LABEL_KO[detach_failed]", "분리 실패",
         result_bar.LABEL_KO["detach_failed"]),
        ("status_bus.SEQUENCE_STATES 에 DETACH", True,
         "DETACH" in status_bus.SEQUENCE_STATES),
    ]
    return all(exp == obs for _, exp, obs in checks), checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="시나리오마다 프로브 stderr 를 그대로 찍는다")
    parser.add_argument("--keep", action="store_true",
                        help="임시 HUD 디렉터리를 지우지 않는다 (버스 로그를 직접 보려면)")
    args = parser.parse_args()

    write_scan_snapshot()
    print("임시 HUD 디렉터리: %s" % _TMP)
    print("실행기: %s" % os.path.join(MOTION_SCRIPTS, "pick_sequence_executor.py"))
    print()

    failures = 0
    all_stderr = []
    for case in CASES:
        passed, checks, observed = run_case(case)
        all_stderr.append(observed["stderr"])
        mark = "OK  " if passed else "FAIL"
        print("[%s] %-3s %s" % (mark, case["key"], case["name"]))
        for label, expected, got in checks:
            same = "=" if expected == got else "!"
            if not passed or args.verbose:
                print("        %s %-16s 기대 %r / 관측 %r" % (same, label, expected, got))
        if "states" in case["expect"] and (not passed or args.verbose):
            print("        · 관측 단계 %r" % (observed["states"],))
        if args.verbose:
            for line in observed["stderr"].splitlines():
                print("        | %s" % line)
        if not passed:
            failures += 1

    print()
    passed, checks = check_labels()
    print("[%s] 라벨  단계 '당김' · 범례 '분리 실패'" % ("OK  " if passed else "FAIL"))
    for label, expected, got in checks:
        if not passed or args.verbose:
            print("        %s %-34s 기대 %r / 관측 %r"
                  % ("=" if expected == got else "!", label, expected, got))
    if not passed:
        failures += 1

    # 계측 지점 이름이 바뀌면 프로브는 조용히 빠지고 경고만 남긴다 — 그 경고 자체를 실패로 본다.
    joined = "\n".join(all_stderr)
    drifted = [line for line in joined.splitlines()
               if "계측 지점 없음" in line or "attach 실패" in line
               or "가 아직 없다" in line or "가 없다" in line]
    print("[%s] 계측  래핑 지점 전부 존재 (%d)"
          % ("OK  " if not drifted else "FAIL", len(drifted)))
    for line in drifted:
        print("        | %s" % line)
    if drifted:
        failures += 1

    print()
    print("시나리오 %d개 · 실패 %d개" % (len(CASES) + 2, failures))
    if args.keep:
        print("임시 HUD 디렉터리를 남긴다: %s" % _TMP)
    else:
        shutil.rmtree(_TMP, ignore_errors=True)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
