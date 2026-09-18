"""세부 칸(깊이 2) 스캔 자세 유도 — 쿼드트리 적응 분할의 계산부.

[신설 2026-09-11, T4b]

실기에는 "언제 쪼갤지" 를 정하는 런타임 규칙이 없었다. 쪼갤지 여부는 사람이 오프라인에서
정해 YAML 에 세부 자세를 넣는 방식이었고(NW 4칸만 티칭, `compute_nw_pick_ready_pose.py`
`SUBCELLS_DEG`), 그 자세들은 ee y=433mm 한 평면 위에서 x·z 만 달랐다.

여기서는 세부 자세를 **부모 분면 자세에서 계산**한다 — 새 좌표를 하드코딩하지 않는다:

  1. 부모 관절 → FK → ee 자세 (위치 + 방향)
  2. ee 위치를 x·z 로 세부 칸 중심 쪽(분면 폭·높이의 1/4)으로 평행이동. 방향은 부모와 동일.
     y 는 [2026-09-14] 사다리(`derive_subcell_joints_tiered`)로 정한다: ① lab_plane — 실기 깊이 2 티칭
     평면 `LAB_SUBCELL_EE_Y_M` 으로 옮김, ② parent_y — 부모 y 유지(09-11 동작). ①이 실패할 때만 ②를 시도한다.
  3. 부모 관절을 시드로 IK. 돌아온 해 중 부모와 가장 가까운 것을 고른다 (J1/J4/J6 은 360° 등가 중 최근접)
  4. IK 실패(IK_FAIL)·관절 한계(LIMIT)·부모 대비 관절 변화 최대값이 한도(기본 60°) 초과(JOINT_DELTA)면 그 단계는
     실패. 두 단계 모두 실패하면 None — 호출자는 부모 자세에서 pick 하는 종전 동작으로 퇴화한다 (SUBDIVIDE_REJECTED)

이 모듈은 rclpy 를 import 하지 않는다. scan_executor_node 와 오프라인 검사
(`scripts/check_subcell_scan_poses.py`) 가 같은 함수를 쓴다 — 단일 출처.
cuRobo 는 `build_subdivide_solver` 안에서만 지연 import 한다.
"""
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

#: 세부 칸 방문 순서. scan_executor_node._group_poses_by_subcell 의 순서와 같아야 한다
#: (아래쪽 먼저 — 아래 탐지가 줄기 높이일 가능성이 높다는 실기 주석을 따른다).
SUBCELL_ORDER: Tuple[str, ...] = ("sw", "se", "nw", "ne")

#: 360° 등가 표현이 있는 관절 (J1, J4, J6). scan_executor_node 의 두 집합을 합친 것 —
#: 관절 변화량을 잴 때는 J1 도 최근접 등가로 본다 (IK 는 J1 을 -210 으로 돌려줄 수 있고
#: 그것은 150 과 같은 자세다). 명령으로 보낼 때는 부모 표현에 맞춘 값을 보낸다.
WRAP_EQUIVALENT_JOINT_IDX = (0, 3, 5)

#: [2026-09-14] 실기 깊이 2 티칭 자세의 ee y (m). `_baseline/A_strawberry_motion/scripts/compute_nw_pick_ready_pose.py`
#: SUBCELLS_DEG 4자세를 FK 하면 전부 이 평면(보드 810 에서 377mm)에 있고, v12 분면 자세(ee y 317~339, 보드에서 470~490mm)
#: 보다 약 100mm 가깝다. 세부 자세는 x·z 만이 아니라 y 도 이 평면으로 옮긴다 — "깊이가 깊을수록 보드에 가깝다"의 1차 출처.
#: 0 이면 종전(부모 y 유지). 검증: check_subcell_scan_poses.py --lab-fk
LAB_SUBCELL_EE_Y_M = 0.433

#: [2026-09-14] D455 카메라 원점의 ee(그리퍼 베이스) 프레임 오프셋 (m). main_scene.usd 에서 읽음:
#: /World/robot_assembly/rh_p12_rn_base/rsd455 을 rh_p12_rn_base 프레임으로 (-87.1, 7.3, 63.4) mm.
#: 툴 축(+z) 앞으로 63mm, 옆으로 87mm. 카메라-보드 거리 = 보드 y - (ee + R·offset).y
CAMERA_OFFSET_IN_EE_M = (-0.0871, 0.0073, 0.0634)


def camera_board_distance_mm(ee_pos_m, ee_quat_wxyz, board_y_m: float) -> float:
    """스캔 자세에서 카메라 원점과 보드 앞면의 y 거리 (mm)."""
    w, x, y, z = [float(v) for v in ee_quat_wxyz]
    ox, oy, oz = CAMERA_OFFSET_IN_EE_M
    # 회전 행렬 두 번째 행 (y 성분만 필요)
    r10 = 2.0 * (x * y + w * z)
    r11 = 1.0 - 2.0 * (x * x + z * z)
    r12 = 2.0 * (y * z - w * x)
    cam_y = float(ee_pos_m[1]) + r10 * ox + r11 * oy + r12 * oz
    return (float(board_y_m) - cam_y) * 1000.0


def subcell_center_offset_m(bounds: Sequence[float], subcell: str) -> Tuple[float, float]:
    """분면 외곽 (x0, x1, z0, z1) 에서 세부 칸 중심까지의 (dx, dz) — 분면 중심 기준.

    분면을 2×2 로 나누면 세부 칸 중심은 분면 중심에서 폭의 1/4 만큼 떨어진다.
    """
    x0, x1, z0, z1 = [float(v) for v in bounds]
    qx, qz = (x1 - x0) / 4.0, (z1 - z0) / 4.0
    dx = -qx if subcell in ("nw", "sw") else qx
    dz = qz if subcell in ("nw", "ne") else -qz
    return dx, dz


def nearest_equivalent_deg(target_deg: Sequence[float], ref_deg: Sequence[float],
                           wrap_idx: Sequence[int], limits_deg: Sequence[Tuple[float, float]]
                           ) -> List[float]:
    """wrap 관절을 ref 에 가장 가까운 360° 등가로 바꾼다 (한계 안의 후보만)."""
    out = [float(v) for v in target_deg]
    for j in wrap_idx:
        lo, hi = limits_deg[j]
        cands = [out[j] + 360.0 * k for k in range(-2, 3)]
        valid = [c for c in cands if lo <= c <= hi] or cands
        out[j] = min(valid, key=lambda c: abs(c - ref_deg[j]))
    return out


def derive_subcell_joints_deg(
    parent_joints_deg: Sequence[float],
    offset_xz_m: Tuple[float, float],
    fk_fn: Callable[[List[float]], Tuple[List[float], List[float]]],
    ik_fn: Callable[[List[float], List[float], List[float]], List[List[float]]],
    *,
    limits_deg: Sequence[Tuple[float, float]],
    max_delta_deg: float,
    wrap_idx: Sequence[int] = WRAP_EQUIVALENT_JOINT_IDX,
    goal_y_m: Optional[float] = None) -> Tuple[Optional[List[float]], Dict]:
    """부모 자세에서 세부 칸 자세(관절, deg)를 유도한다.

    fk_fn(joints_rad) -> (pos_m[3], quat_wxyz[4])
    ik_fn(pos_m, quat_wxyz, seed_rad) -> 성공한 해들의 목록 (각각 joints_rad[6])

    반환 (joints_deg | None, info). info 에는 판정 근거가 들어간다 —
    parent_ee_mm, goal_ee_mm, ik_solutions, max_delta_deg, delta_deg, reason.
    """
    parent_deg = [float(v) for v in parent_joints_deg]
    parent_rad = np.deg2rad(parent_deg).tolist()
    pos, quat = fk_fn(parent_rad)
    goal = [float(pos[0]) + float(offset_xz_m[0]),
            float(goal_y_m) if goal_y_m else float(pos[1]),
            float(pos[2]) + float(offset_xz_m[1])]
    info: Dict = {
        "parent_ee_mm": [round(float(v) * 1000.0, 1) for v in pos],
        "goal_ee_mm": [round(v * 1000.0, 1) for v in goal],
        "ik_solutions": 0,
    }
    sols = ik_fn(goal, [float(v) for v in quat], parent_rad)
    info["ik_solutions"] = len(sols)
    if not sols:
        info["reason"] = "IK_FAIL"
        return None, info

    best = None
    for s in sols:
        s_deg = nearest_equivalent_deg(np.rad2deg(s).tolist(), parent_deg, wrap_idx, limits_deg)
        delta = [abs(a - b) for a, b in zip(s_deg, parent_deg)]
        mx = max(delta)
        if best is None or mx < best[0]:
            best = (mx, s_deg, delta)
    mx, s_deg, delta = best
    info["max_delta_deg"] = round(float(mx), 1)
    info["delta_deg"] = [round(float(d), 1) for d in delta]
    for j, (lo, hi) in enumerate(limits_deg):
        if not (lo <= s_deg[j] <= hi):
            info["reason"] = "LIMIT J%d %.1f" % (j + 1, s_deg[j])
            return None, info
    if mx > max_delta_deg:
        info["reason"] = "JOINT_DELTA %.1f > %.1f" % (mx, max_delta_deg)
        return None, info
    return [round(float(v), 2) for v in s_deg], info


def derive_subcell_joints_tiered(
    parent_joints_deg, offset_xz_m, fk_fn, ik_fn, *, limits_deg, max_delta_deg, wrap_idx,
    lab_plane_y_m: Optional[float]):
    """[2026-09-14] 세부 자세 유도 사다리 — (joints, info, tier).

    tier "lab_plane": ee y 를 실기 깊이 2 티칭 평면(LAB_SUBCELL_EE_Y_M)으로 옮긴 목표. 깊이 2 가 깊이 1 보다
                      보드에 가깝다 — 이때만 시야를 세부 칸으로 좁힌다.
    tier "parent_y" : 부모 y 유지, x·z 만 평행이동 (09-11 동작). lab_plane 이 IK 밖이거나 관절 변화 한도를
                      넘을 때의 대안. 시야는 부모 분면 그대로(거리가 같으므로).
    둘 다 실패하면 (None, info, None) — 호출자가 부모 자세 pick 으로 퇴화한다.
    lab_plane_y_m 이 None/0 이면 parent_y 만 시도한다.
    """
    tiers = []
    if lab_plane_y_m:
        tiers.append(("lab_plane", float(lab_plane_y_m)))
    tiers.append(("parent_y", None))
    last_info = None
    for tier, y in tiers:
        joints, info = derive_subcell_joints_deg(
            parent_joints_deg, offset_xz_m, fk_fn, ik_fn, limits_deg=limits_deg,
            max_delta_deg=max_delta_deg, wrap_idx=wrap_idx, goal_y_m=y)
        info = dict(info); info["tier"] = tier
        if joints is not None:
            return joints, info, tier
        if last_info is None:
            last_info = info
        else:
            last_info = dict(info, first_tier_reason=last_info.get("reason"),
                             first_tier=last_info.get("tier"))
    return None, last_info, None


def joint_line_min_board_clearance_mm(
    a_deg: Sequence[float], b_deg: Sequence[float],
    spheres_fn: Callable[[np.ndarray], np.ndarray],
    board_y_m: float, n_samples: int = 60,
) -> Tuple[float, float]:
    """관절공간 직선(MoveJoint) a→b 를 따라 로봇 충돌구가 보드 앞면에 가장 가까운 여유 (mm).

    spheres_fn(q_rad[N,6]) -> [N, n_sph, 4] (x, y, z, r; cuRobo 는 미사용 구를 r=-10 으로 채운다)
    반환 (최소 여유 mm, 그 위치의 경로 비율 0~1). 양수 = 보드 앞.
    """
    a = np.deg2rad(np.asarray(a_deg, dtype=float))
    b = np.deg2rad(np.asarray(b_deg, dtype=float))
    ts = np.linspace(0.0, 1.0, int(n_samples))
    q = np.array([a + t * (b - a) for t in ts])
    sph = np.asarray(spheres_fn(q), dtype=float)
    clear = float(board_y_m) - (sph[..., 1] + sph[..., 3])
    clear = np.where(sph[..., 3] > 0.0, clear, np.inf)
    idx = np.unravel_index(int(np.argmin(clear)), clear.shape)
    return float(clear[idx] * 1000.0), float(ts[idx[0]])


class SubdivideSolver:
    """cuRobo IKSolver 를 감싼 FK/IK/충돌구 콜러블 묶음. GPU 필요."""

    def __init__(self, robot_yml: Path, urdf_path: Path, spheres_path: Path,
                 world_yaml: Path, num_seeds: int = 32, device: str = "cuda:0"):
        import torch
        import yaml
        from copy import deepcopy
        from curobo.geom.types import Cuboid, WorldConfig
        from curobo.types.base import TensorDeviceType
        from curobo.types.math import Pose
        from curobo.types.robot import RobotConfig
        from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

        self._torch = torch
        self._Pose = Pose
        self.device = device
        self.num_seeds = int(num_seeds)
        tensor_args = TensorDeviceType(device=torch.device(device))
        with Path(robot_yml).open() as fh:
            robot_data = deepcopy(yaml.safe_load(fh))
        kine = robot_data["robot_cfg"]["kinematics"]
        kine["urdf_path"] = str(urdf_path)
        kine["collision_spheres"] = str(spheres_path)
        robot_cfg = RobotConfig.from_dict(robot_data, tensor_args=tensor_args)
        with Path(world_yaml).open() as fh:
            world_meta = yaml.safe_load(fh)["scan_collision_world"]
        cuboids = [
            Cuboid(name=o["name"], pose=[float(v) for v in o["pose_wxyz"]],
                   dims=[float(v) for v in o["dims_m"]])
            for o in world_meta["objects"]
            if o.get("enabled", True) and o.get("type") == "cuboid"
        ]
        ik_cfg = IKSolverConfig.load_from_robot_config(
            robot_cfg, WorldConfig(cuboid=cuboids), tensor_args=tensor_args,
            num_seeds=self.num_seeds, position_threshold=0.005, rotation_threshold=0.05,
            self_collision_check=True, self_collision_opt=True,
            use_cuda_graph=False, collision_cache={"obb": 30, "mesh": 10},
        )
        self.solver = IKSolver(ik_cfg)

    def fk(self, joints_rad: List[float]) -> Tuple[List[float], List[float]]:
        q = self._torch.tensor([joints_rad], device=self.device, dtype=self._torch.float32)
        st = self.solver.fk(q)
        return (st.ee_position[0].detach().cpu().numpy().astype(float).tolist(),
                st.ee_quaternion[0].detach().cpu().numpy().astype(float).tolist())

    def ik(self, pos_m: List[float], quat_wxyz: List[float], seed_rad: List[float]
           ) -> List[List[float]]:
        t = self._torch
        goal = self._Pose(
            position=t.tensor([pos_m], device=self.device, dtype=t.float32),
            quaternion=t.tensor([quat_wxyz], device=self.device, dtype=t.float32))
        seed = t.tensor([[seed_rad]], device=self.device, dtype=t.float32)      # (n=1, 1, dof)
        retract = t.tensor([seed_rad], device=self.device, dtype=t.float32)     # (1, dof)
        res = self.solver.solve_single(goal, retract_config=retract, seed_config=seed,
                                       return_seeds=self.num_seeds)
        succ = res.success.reshape(-1).detach().cpu().numpy()
        # res.solution 은 (1, K, dof). js_solution 은 그리퍼 관절(gripper_rh_r1)이 붙은 7열이라 쓰지 않는다.
        js = res.solution.reshape(-1, len(seed_rad)).detach().cpu().numpy()
        return [js[i].astype(float).tolist() for i in range(len(succ)) if bool(succ[i])]

    def spheres(self, q_rad: np.ndarray) -> np.ndarray:
        q = self._torch.tensor(np.asarray(q_rad, dtype=float), device=self.device,
                               dtype=self._torch.float32)
        st = self.solver.kinematics.get_state(q)
        return st.link_spheres_tensor.detach().cpu().numpy().astype(float)
