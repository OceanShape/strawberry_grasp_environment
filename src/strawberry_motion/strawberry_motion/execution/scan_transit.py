"""세부영역 간 스캔 이동 경로 계획 — scan_executor 보조 모듈.

[신설 2026-09-09]

스캔 이동은 종전까지 `_movej` (관절공간 직선)였다. 네 분면의 scan pose 가 **전부
같은 값**이라 이동 자체가 없었으므로 문제가 드러나지 않았다.

분면별 scan pose 를 실제로 부여하자마자 드러난 실측값 (ee y=400mm 평면,
compute_subcell_scan_poses.py):

    overview -> root/sw   스윙 102.0deg   보드여유   24mm
    root/sw  -> root/nw   스윙 135.2deg   보드여유   -3mm   <-- 관통
    root/nw  -> root/ne   스윙 174.9deg   보드여유  -23mm   <-- 관통
    root/ne  -> root/se   스윙 132.4deg   보드여유   14mm
    root/se  -> overview  스윙 105.2deg   보드여유   24mm

관절공간 직선은 경로 충돌을 아무도 보지 않는다. 자세를 다르게 골라도 마찬가지였다
— "보드여유 50mm" 를 제약으로 넣고 체인 DP 를 돌리면 팔꿈치를 뒤로 접은 자세
(J3=-120deg)에 구간 스윙 250deg 가 나온다. 자세 선택의 문제가 아니라 **이동
프리미티브의 문제**다.

scan_executor 는 이미 보드가 든 MotionGen 을 들고 있으므로(`_init_motion_gen`),
그것으로 관절공간 목표를 계획한다. 계획이 실패하면 호출자가 종전 MoveJoint 로
되돌아가므로 기존 동작은 보존된다.
"""
from typing import List, Optional

import numpy as np


def plan_joint_space(motion_gen, joint_names: List[str],
                     start_rad, goal_rad, logger=None,
                     max_attempts: int = 4) -> Optional[np.ndarray]:
    """start_rad -> goal_rad 관절공간 계획. 실패하면 None.

    반환: (N, 6) 관절 궤적 (rad). 호출자가 _exec_spline 으로 실행한다.
    """
    try:
        import torch
        from curobo.types.robot import JointState as CuroboJointState
        from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
    except Exception as exc:                                    # noqa: BLE001
        if logger:
            logger.warn("SCAN_TRANSIT import 실패: %s" % exc)
        return None
    if motion_gen is None:
        return None
    try:
        start = CuroboJointState.from_position(
            position=torch.tensor([list(start_rad)], device="cuda:0",
                                  dtype=torch.float32),
            joint_names=joint_names)
        goal = CuroboJointState.from_position(
            position=torch.tensor([list(goal_rad)], device="cuda:0",
                                  dtype=torch.float32),
            joint_names=joint_names)
        result = motion_gen.plan_single_js(
            start, goal, MotionGenPlanConfig(max_attempts=max_attempts))
        if not bool(result.success.item()):
            if logger:
                logger.warn("SCAN_TRANSIT 계획 실패 (%s) — MoveJoint 로 되돌아간다"
                            % getattr(result, "status", "no status"))
            return None
        traj = result.get_interpolated_plan().position.cpu().numpy()
        if logger:
            logger.info("SCAN_TRANSIT ok: %d pts / %.2fs"
                        % (traj.shape[0], float(result.motion_time)))
        return traj
    except Exception as exc:                                    # noqa: BLE001
        if logger:
            logger.warn("SCAN_TRANSIT 예외: %s — MoveJoint 로 되돌아간다" % exc)
        return None
