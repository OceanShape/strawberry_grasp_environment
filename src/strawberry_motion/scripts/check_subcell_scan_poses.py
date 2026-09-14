#!/usr/bin/env python3
"""세부 칸 스캔 자세 오프라인 검사 — T4b 적응 분할 (2026-09-11). 런타임 노드가 아니다.

scan_executor_node 가 런타임에 하는 것과 **같은 함수**(execution/subcell_pose.py)로
네 분면 × 네 세부 칸의 자세를 유도해 표로 찍는다:
  * 부모 대비 관절 변화(최대·관절별), 수락/거부 사유
  * 유도 자세의 ee 가 목표(부모 ee + x·z 평행이동)와 얼마나 맞는지 (mm)
  * MoveJoint(관절공간 직선) 이동 쌍의 보드 앞면 여유 — 부모↔세부, 세부↔세부 전부

실행 (ROS 환경 source 후, 리포 루트에서):
  python3 src/strawberry_motion/scripts/check_subcell_scan_poses.py [--max-delta 60] [--seeds 32]
"""
import argparse
import itertools
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_SRC = os.path.dirname(HERE)                      # src/strawberry_motion
sys.path.insert(0, PKG_SRC)                          # 소스 트리를 install 보다 먼저

from strawberry_motion.execution import scan_executor_node as sen   # noqa: E402
from strawberry_motion.execution.subcell_pose import (                # noqa: E402
    LAB_SUBCELL_EE_Y_M, SUBCELL_ORDER, SubdivideSolver, WRAP_EQUIVALENT_JOINT_IDX,
    camera_board_distance_mm, derive_subcell_joints_tiered,
    derive_subcell_joints_deg, joint_line_min_board_clearance_mm,
    subcell_center_offset_m,
)

import yaml  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-delta", type=float, default=60.0)
    ap.add_argument("--seeds", type=int, default=32)
    ap.add_argument("--yaml", default=os.path.join(PKG_SRC, "config", sen._CANDIDATES_FNAME))
    ap.add_argument("--ee-y", type=float, default=LAB_SUBCELL_EE_Y_M,
                    help="세부 자세 ee y (m). 기본 실기 티칭 평면 0.433, 0 = 부모 y 유지(09-11 동작)")
    ap.add_argument("--lab-fk", action="store_true",
                    help="_baseline 의 실기 NW 세부 자세 SUBCELLS_DEG 를 FK 해 ee 를 찍는다 (0.433 의 출처 확인)")
    args = ap.parse_args()

    with open(args.yaml, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)["scan_pose_candidates"]
    targets = {t["cell_id"]: t for t in cfg["targets"] if t.get("endpoint_joints_deg")}
    limits_deg = [(float(np.rad2deg(lo)), float(np.rad2deg(hi))) for lo, hi in sen._JOINT_LIMITS_RAD]
    board_y = sen.BOARD_SURFACE_Y_M

    print("robot yml : %s" % sen._ROBOT_YML)
    print("world     : %s (board front y=%.3f m)" % (sen._COLLISION_WORLD_FNAME, board_y))
    print("max delta : %.1f deg   seeds: %d   subcell ee y: %s" % (
        args.max_delta, args.seeds, ("%.3f m (board -%.0f mm)" % (args.ee_y, (board_y - args.ee_y) * 1000)) if args.ee_y else "parent"))
    solver = SubdivideSolver(sen._ROBOT_YML, sen._URDF_PATH, sen._SPHERES_PATH,
                             os.path.join(PKG_SRC, "config", sen._COLLISION_WORLD_FNAME),
                             num_seeds=args.seeds)
    if args.lab_fk:
        import re as _re
        src = open(os.path.join(os.path.dirname(os.path.dirname(PKG_SRC)), "_baseline", "A_strawberry_motion",
                                "scripts", "compute_nw_pick_ready_pose.py"), encoding="utf-8").read()
        print("\nlab NW depth-2 taught poses (FK, ee = gripper base):")
        for name, vals in _re.findall(r'"(root/nw/\w+)":\s*\[([^\]]+)\]', src):
            q = [float(v) for v in vals.split(",")]
            pos, _ = solver.fk(np.deg2rad(q).tolist())
            print("  %-11s ee=(%7.1f, %7.1f, %7.1f) mm  board -%.0f mm" % (name, pos[0]*1000, pos[1]*1000, pos[2]*1000, (board_y - pos[1]) * 1000))

    for cell_id in ("root/nw", "root/ne", "root/se", "root/sw"):
        if cell_id not in targets:
            print("\n%s: YAML 에 없음" % cell_id)
            continue
        parent = [float(v) for v in targets[cell_id]["endpoint_joints_deg"]]
        bounds = sen.ScanExecutorNode._quadrant_bounds(cell_id)
        ppos, pquat = solver.fk(np.deg2rad(parent).tolist())
        print("\n=== %s  parent=[%s]  bounds x[%.3f,%.3f] z[%.3f,%.3f]  parent ee y=%.0f mm (board -%.0f, camera-board %.0f mm)" % (
            cell_id, " ".join("%.1f" % v for v in parent), *bounds, ppos[1] * 1000, (board_y - ppos[1]) * 1000,
            camera_board_distance_mm(ppos, pquat, board_y)))
        poses = {}
        print("  %-4s %-22s %-36s %4s %8s  %-32s %s" % (
            "sub", "goal ee (mm)", "reached ee (mm)", "sols", "maxdJ", "dJ per joint", "verdict"))
        for sub in SUBCELL_ORDER:
            off = subcell_center_offset_m(bounds, sub)
            joints, info, tier = derive_subcell_joints_tiered(
                parent, off, solver.fk, solver.ik, limits_deg=limits_deg,
                max_delta_deg=args.max_delta, wrap_idx=WRAP_EQUIVALENT_JOINT_IDX,
                lab_plane_y_m=args.ee_y or None)
            reached = "-"
            if joints is not None:
                pos, quat = solver.fk(np.deg2rad(joints).tolist())
                err = np.linalg.norm(np.array(pos) - np.array(info["goal_ee_mm"]) / 1000.0) * 1000
                reached = "(%s) err %.1f cam-board %.0f" % (" ".join("%.0f" % (p * 1000) for p in pos), err,
                                                            camera_board_distance_mm(pos, quat, board_y))
                poses[sub] = joints
            print("  %-4s %-22s %-36s %4d %8s  %-32s %s" % (
                sub, "(%s)" % " ".join("%.0f" % v for v in info["goal_ee_mm"]), reached,
                info["ik_solutions"], info.get("max_delta_deg", "-"),
                " ".join("%.0f" % d for d in info.get("delta_deg", [])) or "-",
                "ACCEPT[%s] joints=[%s]" % (tier, " ".join("%.1f" % v for v in joints)) if joints
                else "REJECT %s (lab_plane: %s)" % (info.get("reason"), info.get("first_tier_reason", "-"))))
        # 이동 쌍 보드 여유 — MoveJoint 는 관절공간 직선. 세부 자세는 부모 표현에 맞춰져 있다.
        pairs = [("parent", sub) for sub in poses] + list(itertools.combinations(poses, 2))
        print("  board clearance along joint-space line (min over 60 samples, mm):")
        for a, b in pairs:
            ja = parent if a == "parent" else poses[a]
            jb = poses[b]
            clr, at = joint_line_min_board_clearance_mm(ja, jb, solver.spheres, board_y)
            flag = "" if clr >= 50.0 else "   <-- < 50 mm"
            print("    %-6s -> %-3s : %7.1f  (at %.2f)%s" % (a, b, clr, at, flag))
    print("\nnote: 부모 자세 자체의 여유도 parent->sub 구간의 t=0 에 포함된다.")


if __name__ == "__main__":
    main()
