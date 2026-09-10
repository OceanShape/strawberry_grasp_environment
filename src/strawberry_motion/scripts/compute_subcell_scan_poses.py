#!/usr/bin/env python3
"""Offline IK helper - NOT a runtime node.  [NEW 2026-09-09]

SUPERSEDED 2026-09-09: the YAML now carries the real robot's v12 taught poses.
This script's output (ee on a y=400mm plane) left the fingertip only 56mm from
the board - the gripper touched the fruit at the scan pose and drove into the
board during the pick.  The real teaching puts the gripper base at y=280..339mm,
fingertip 97..156mm clear.  Kept as a tool for re-deriving poses if the board or
the tool changes; do not run it expecting to replace the taught values.


Derives one scan pose per quadtree quadrant (root/nw, ne, se, sw) for the
simulation board, keeping as much of the real-robot teaching as possible.

sim2real policy (what is kept vs re-derived)
--------------------------------------------
KEPT from the real taught poses (compute_nw_pick_ready_pose.py SUBCELLS_DEG,
the only surviving record of real sub-cell scan poses):
  * tool orientation - identical quaternion, tool +Z = (0.069, 0.913, 0.402)
  * the "constant-y stand-off plane" structure: all four real poses put the
    gripper base (ee) on ONE plane y = 433.0 mm and vary only (x, z)
  * that y value itself, 433.0 mm

RE-DERIVED here:
  * the (x, z) of each pose.  The real record only covers NW's four
    sub-SUB-cells (root/nw/nw ... root/nw/se); there is no surviving
    quadrant-level pose.  Quadrant centres come from the measured board
    (environment.yaml) split at the runtime grid mid-lines
    scan_executor_node.BOARD_SUBCELL_{X,Z}_MID_M.

Fingertip check: the gripper's forward-most collision sphere reaches
236.3 mm from ee, i.e. the planner TCP (236.0 mm) IS the fingertip.  At
y_plane = 433 mm the tip sits 23.3 mm in front of the board (672.0 mm).

Run:
  cd src/strawberry_motion/scripts && python3 compute_subcell_scan_poses.py
Then paste the printed YAML block into
config/scan_pose_candidates_refit_candidate.yaml.
"""
import os
import sys

import numpy as np
import torch
import yaml

from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.types.math import Pose
from curobo.geom.types import WorldConfig, Cuboid
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
CFG = os.path.join(REPO, "src", "e0509_gripper_description", "config")
CUROBO_CFG = os.path.join(CFG, "curobo")

OVERVIEW_DEG = [87.98, -94.91, 129.9, 175.94, -31.34, 93.42]

# Real taught stand-off plane (see module docstring).  Fallbacks are tried in
# order if cuRobo rejects the real one for collision.
Y_PLANE_CANDIDATES_M = [0.433, 0.400, 0.370, 0.340, 0.310]

# Runtime quadrant split (scan_executor_node.BOARD_SUBCELL_*_MID_M)
X_MID_M, Z_MID_M = 0.050, 0.660


def board_from_env():
    with open(os.path.join(CFG, "environment.yaml")) as f:
        env = yaml.safe_load(f)
    obj = next(o for o in env["objects"] if o["name"] == "whiteboard")
    return obj, [Cuboid(name=str(o["name"]), pose=[float(v) for v in o["pose"]],
                        dims=[float(v) for v in o["dims"]])
                 for o in env["objects"] if o.get("enabled", True)]


def quadrant_centres(obj):
    cx, _, cz = obj["pose"][0], obj["pose"][1], obj["pose"][2]
    dx, _, dz = obj["dims"]
    x0, x1 = cx - dx / 2.0, cx + dx / 2.0
    z0, z1 = cz - dz / 2.0, cz + dz / 2.0
    return {
        "root/nw": ((x0 + X_MID_M) / 2.0, (Z_MID_M + z1) / 2.0),
        "root/ne": ((X_MID_M + x1) / 2.0, (Z_MID_M + z1) / 2.0),
        "root/sw": ((x0 + X_MID_M) / 2.0, (z0 + Z_MID_M) / 2.0),
        "root/se": ((X_MID_M + x1) / 2.0, (z0 + Z_MID_M) / 2.0),
    }


def main():
    obj, cuboids = board_from_env()
    board_face_y = obj["pose"][1] - obj["dims"][1] / 2.0
    centres = quadrant_centres(obj)

    tensor_args = TensorDeviceType(device=torch.device("cuda:0"))
    with open(os.path.join(CUROBO_CFG, "e0509_gripper.yml")) as f:
        cfg = yaml.safe_load(f)
    kin = cfg["robot_cfg"]["kinematics"]
    kin["urdf_path"] = os.path.join(CUROBO_CFG, "e0509_gripper.urdf")
    kin["collision_spheres"] = os.path.join(CUROBO_CFG, "e0509_spheres.yml")
    robot_cfg = RobotConfig.from_dict(cfg, tensor_args=tensor_args)

    ik = IKSolver(IKSolverConfig.load_from_robot_config(
        robot_cfg, WorldConfig(cuboid=cuboids), tensor_args=tensor_args,
        num_seeds=200, self_collision_check=True, use_cuda_graph=False))

    seed_state = torch.tensor([np.deg2rad(OVERVIEW_DEG)],
                              device="cuda:0", dtype=torch.float32)
    fk = ik.kinematics.get_state(seed_state)
    quat = fk.ee_quaternion.clone()          # keep the real taught orientation
    zax = Pose(position=fk.ee_position, quaternion=quat).get_rotation()  # noqa
    print("board front face y = %.1f mm" % (board_face_y * 1000))
    print("overview ee = %s mm\n" % np.round(
        fk.ee_position[0].cpu().numpy() * 1000, 1))

    # ── 후보 수집 ────────────────────────────────────────────────────
    # 분면마다 "overview 에서 가장 가까운 해" 를 따로 고르면, 이웃 분면끼리
    # 손목 브랜치가 제각각이 되어 순회 중 100도 넘는 스윙이 생긴다.
    # 그래서 후보를 전부 모아두고, 순회 순서를 따라 **연쇄로** 고른다.
    ORDER = ["root/sw", "root/nw", "root/ne", "root/se"]   # _ALL_CELLS_ZORDER

    def wrap_norm(sol_deg, ref_deg):
        """J4/J6 를 ref 에 가장 가까운 360도 등가로 정규화."""
        out = list(sol_deg)
        for i in (3, 5):
            best = min((out[i] + 360.0 * k for k in range(-2, 3)),
                       key=lambda v: abs(v - ref_deg[i]))
            out[i] = best
        return out

    def gather(y_plane):
        cand = {}
        for cell in ORDER:
            cx, cz = centres[cell]
            goal = Pose(
                position=torch.tensor([[cx, y_plane, cz]], device="cuda:0",
                                      dtype=torch.float32),
                quaternion=quat)
            # ⚠️ seed_config 를 주지 않는다 — 여기서는 **브랜치 다양성**이 목적이다.
            # overview 로 전부 시딩하면 200 seed 가 한 해로 수렴해 후보가 1개가 되고,
            # 분면끼리 손목 브랜치가 어긋난 채로 굳어 순회 스윙이 175도까지 커진다.
            # (런타임 MoveLine 의 최근접 해 선택과는 목적이 정반대다.)
            uniq = []
            for _ in range(6):
                out = ik.solve_single(goal, return_seeds=60)
                ok = out.success.view(-1).cpu().numpy()
                sols = out.solution.view(len(ok), -1).cpu().numpy()
                for i in np.where(ok)[0]:
                    d = wrap_norm(np.rad2deg(sols[i]), OVERVIEW_DEG)
                    if all(max(abs(a - b) for a, b in zip(d, u)) > 3.0
                           for u in uniq):
                        uniq.append(d)
            if not uniq:
                return None
            uniq.sort(key=lambda d: max(abs(a - b) for a, b in zip(d, OVERVIEW_DEG)))
            cand[cell] = uniq[:12]
        return cand

    def chain_cost(pick):
        """overview -> sw -> nw -> ne -> se -> overview 의 최대 구간 관절이동."""
        seq = [OVERVIEW_DEG] + [pick[c] for c in ORDER] + [OVERVIEW_DEG]
        return max(max(abs(a - b) for a, b in zip(seq[i], seq[i + 1]))
                   for i in range(len(seq) - 1))

    # ── 순회 구간 충돌 검사 ──────────────────────────────────────────
    # 스캔 이동은 cuRobo 가 아니라 **직접 MoveJoint** 다 (_move_to_scan_cell_and_wait).
    # 즉 관절공간 직선이라 경로 충돌을 아무도 안 본다. 네 분면이 모두 같은 자세였을
    # 때는 이동 자체가 없어 문제가 없었지만, 이제 분면마다 자세가 달라지므로
    # 구간마다 보드를 스치지 않는지 여기서 확인해야 한다.
    bmin = np.array([obj["pose"][0] - obj["dims"][0] / 2.0,
                     obj["pose"][1] - obj["dims"][1] / 2.0,
                     obj["pose"][2] - obj["dims"][2] / 2.0])
    bmax = np.array([obj["pose"][0] + obj["dims"][0] / 2.0,
                     obj["pose"][1] + obj["dims"][1] / 2.0,
                     obj["pose"][2] + obj["dims"][2] / 2.0])

    def sweep_clearance(qa_deg, qb_deg, n=80):
        qs = np.linspace(np.asarray(qa_deg, float), np.asarray(qb_deg, float), n)
        st = ik.kinematics.get_state(
            torch.tensor(np.deg2rad(qs), device="cuda:0", dtype=torch.float32))
        sph = st.link_spheres_tensor.detach().cpu().numpy()
        c, r = sph[..., :3], sph[..., 3]
        d = np.maximum(bmin - c, 0.0) + np.maximum(c - bmax, 0.0)
        dist = np.linalg.norm(d, axis=-1) - r
        dist = np.where(r > 0.0, dist, np.inf)
        return float(dist.min())

    # ── 자세 선택 ────────────────────────────────────────────────────
    # 목적함수: **overview 형상에서의 이탈 최소화.**
    #
    # 처음에는 "관절공간 직선(MoveJoint)으로 분면 사이를 오갈 때 보드 여유 >= 50mm"
    # 를 제약으로 넣고 체인 DP 를 돌렸다. 결과는 팔꿈치를 뒤로 접은 자세(J3 = -120도)
    # 에 구간 스윙 250도 — 여유는 확보되지만 쓸 수 없는 궤적이었다.
    # 원인은 자세 선택이 아니라 **이동 프리미티브**다. 스캔 이동은 지금
    # _move_to_scan_cell_and_wait 가 직접 MoveJoint 로 보내므로 경로 충돌을
    # 아무도 보지 않는다. 네 분면이 전부 같은 자세였을 때는 이동이 없어 드러나지
    # 않았을 뿐이다. 자세를 아무리 잘 골라도 이 구조에서는 안전과 매끄러움을
    # 동시에 만족시킬 수 없다 -> 스캔 이동을 cuRobo 계획으로 돌려야 한다.
    #
    # 따라서 여기서는 자세만 정직하게 고르고, 관절공간 직선의 여유는 **참고로
    # 출력**해 판단 근거를 남긴다.
    MIN_CLEARANCE_M = 0.050

    best = None
    for y_plane in Y_PLANE_CANDIDATES_M:
        cand = gather(y_plane)
        if cand is None:
            print("  y=%.0fmm  IK 실패 (충돌 또는 도달 불가)" % (y_plane * 1000))
            continue
        pick = {}
        for c in ORDER:
            pick[c] = min(cand[c], key=lambda d: max(
                abs(a - b) for a, b in zip(d, OVERVIEW_DEG)))
        worst = max(max(abs(a - b) for a, b in zip(pick[c], OVERVIEW_DEG))
                    for c in ORDER)
        print("  y=%.0fmm  후보 %-16s overview 대비 최대이탈 %.1fdeg"
              % (y_plane * 1000, str([len(cand[c]) for c in ORDER]), worst))
        best = (y_plane, pick, worst)
        break                      # sim2real: 실기 433mm 에 가장 가까운 가용 평면

    if best is None:
        print("모든 평면 실패"); return 1
    y_plane, pick, worst = best
    print("\n선택: ee y = %.0f mm (실기 티칭 433mm 대비 %+.0fmm)"
          % (y_plane * 1000, y_plane * 1000 - 433.0))
    seq_names = ["overview"] + ORDER + ["overview"]
    seq = [OVERVIEW_DEG] + [pick[k] for k in ORDER] + [OVERVIEW_DEG]
    print("\n  관절공간 직선(MoveJoint) 기준 참고치 — 이 값이 작아서 cuRobo 경유가 필요하다")
    for i in range(len(seq) - 1):
        clr = sweep_clearance(seq[i], seq[i + 1]) * 1000
        print("      %-9s -> %-9s  스윙 %6.1fdeg  보드여유 %5.0fmm %s"
              % (seq_names[i], seq_names[i + 1],
                 max(abs(a - b) for a, b in zip(seq[i], seq[i + 1])),
                 clr, "" if clr >= MIN_CLEARANCE_M * 1000 else "  <-- 위험"))
    results = {}
    for cell in ORDER:
        st = torch.tensor([np.deg2rad(pick[cell])], device="cuda:0",
                          dtype=torch.float32)
        ee = ik.kinematics.get_state(st).ee_position[0].cpu().numpy()
        cx, cz = centres[cell]
        err = np.linalg.norm(ee - np.array([cx, y_plane, cz])) * 1000
        results[cell] = (np.array(pick[cell]), y_plane, ee, err, 0.0)

    print("\n" + "=" * 78)
    print("YAML block  (scan_pose_candidates.targets 아래에 붙여넣기)")
    print("=" * 78)
    for cell in ("root/nw", "root/ne", "root/se", "root/sw"):
        if cell not in results:
            print("    # %s : IK 실패 — 수동 확인 필요" % cell)
            continue
        deg, y_plane, ee, err, d = results[cell]
        tip_y = ee[1] + 0.2363 * 0.913
        print('    - cell_id: "%s"' % cell)
        print("      tcp_transform_base: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]")
        print("      endpoint_joints_deg: [%s]"
              % ", ".join("%.2f" % v for v in deg))
        print("      # ee=(%.0f, %.0f, %.0f)mm  tip y=%.0fmm  board gap=%.0fmm"
              % (ee[0] * 1000, ee[1] * 1000, ee[2] * 1000, tip_y * 1000,
                 (board_face_y - tip_y) * 1000))


if __name__ == "__main__":
    sys.exit(main())
