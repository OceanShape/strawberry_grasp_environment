#!/usr/bin/env python3
"""Offline tray-slot reachability check - NOT a runtime node.  [NEW 2026-09-10, T4-1]

Answers, before an Isaac run: which taught-grid tray slots can the planner
actually place into?  It rebuilds the planner's own MotionGen (same robot
config, same collision world, same plan config as
tray_place_executor._compute_taught_slot_above_target / execute_*), generates
the slot ABOVE / RELEASE goals exactly the way the planner does (Slot0 taught
joints -> FK -> grid offset from the SLOT0/1/3 taught references -> +120 mm),
and plans to every slot from realistic start states.

Start states: the six real RETREAT end poses of a completed run
(runtime JSONL event `retreat_step_complete`), plus OVERVIEW.  The place
plan in the real sequence starts exactly there.

Run (flat imports -> must run from this directory):
  cd src/strawberry_motion/scripts && python3 check_tray_slot_reachability.py \
      ../../../log/m3/20260910T120206-06df4423/curobo_planner_node_*.jsonl

Why this exists: 2026-09-10 03:11 run - slot 1 Plan OK, slot 2 IK_FAIL x2.
Slot 2 is the x~400 column nearest the robot base (is_row2 = slot % 3 == 2)
and gets a 15 deg pitch tilt (row2_place_pitch_tilt_deg).  Slots 3/4/6/7
had never been tried.  T4-1 wants taught_slot_sequence 0,1,3,4,6,7 - this
script says whether that sequence is plannable at all.
"""
import glob
import json
import os
import sys

import numpy as np
import torch
import yaml
from scipy.spatial.transform import Rotation as SciR

from curobo.geom.types import Cuboid
from curobo.types.math import Pose
from curobo.types.robot import JointState as CuroboJointState
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig

from harvest_motion_params import (
    MAX_TAUGHT_PLACE_TRANSFER_JOINT_DELTA_DEG,
    OPERATIONAL_JOINT_LIMITS_DEG,
    OVERVIEW_JOINTS_DEG,
    TAUGHT_SLOT0_ABOVE_CLEARANCE_M,
    TAUGHT_SLOT0_PLACE_REFERENCE_JOINTS_DEG,
    TAUGHT_SLOT0_PLACE_REFERENCE_POSX_MM_DEG,
    TAUGHT_SLOT1_PLACE_REFERENCE_POSX_MM_DEG,
    TAUGHT_SLOT3_PLACE_REFERENCE_POSX_MM_DEG,
    TAUGHT_TRAY_SLOT_COUNT,
    WRAP_EQUIVALENT_JOINT_IDX,
)
from planner_bootstrap import build_curobo_motion_gen

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
ENV_YAML = os.path.join(REPO, "src", "e0509_gripper_description", "config", "environment.yaml")
ROW2_TILT_DEG = 15.0          # planner_bootstrap default row2_place_pitch_tilt_deg
PLAN_CFG = dict(num_ik_seeds=64, max_attempts=3, timeout=2.0, enable_graph_attempt=None)
DEV = "cuda:0"
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


def load_cuboids():
    with open(ENV_YAML, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return [Cuboid(name=str(o["name"]), pose=[float(v) for v in o["pose"]],
                   dims=[float(v) for v in o["dims"]])
            for o in data.get("objects", []) if o.get("enabled", True)]


def load_retreat_starts(jsonl_paths):
    starts = []
    for p in jsonl_paths:
        for line in open(p, encoding="utf-8"):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if r.get("event") == "retreat_step_complete" and r["data"].get("ok"):
                starts.append(("retreat#%d" % (len(starts) + 1),
                               [float(v) for v in r["data"]["current_joints_rad"]]))
    return starts


def slot_offset_m(slot_index):
    s0 = np.array(TAUGHT_SLOT0_PLACE_REFERENCE_POSX_MM_DEG[:3], float)
    s1 = np.array(TAUGHT_SLOT1_PLACE_REFERENCE_POSX_MM_DEG[:3], float)
    s3 = np.array(TAUGHT_SLOT3_PLACE_REFERENCE_POSX_MM_DEG[:3], float)
    h, v = divmod(slot_index, 3)
    return (h * (s3 - s0) + v * (s1 - s0)) / 1000.0


def nearest_equiv(vals_deg, ref_deg):
    out = list(vals_deg)
    for j in WRAP_EQUIVALENT_JOINT_IDX:
        lo, hi = OPERATIONAL_JOINT_LIMITS_DEG[j]
        cands = [out[j] + 360.0 * k for k in range(-2, 3)]
        valid = [c for c in cands if lo <= c <= hi] or cands
        out[j] = min(valid, key=lambda c: abs(c - ref_deg[j]))
    return out


def guard(traj_rad, start_rad):
    """Same three guards the planner applies after plan_single (trajectory_guards)."""
    traj = np.rad2deg(traj_rad).astype(float)
    start = np.rad2deg(start_rad)
    for i in range(len(traj)):
        traj[i] = nearest_equiv(traj[i], start if i == 0 else traj[i - 1])
    for j, (lo, hi) in enumerate(OPERATIONAL_JOINT_LIMITS_DEG):
        if traj[:, j].min() < lo or traj[:, j].max() > hi:
            return "limit J%d" % (j + 1)
    for j in WRAP_EQUIVALENT_JOINT_IDX:
        if len(traj) > 1 and np.abs(np.diff(traj[:, j])).max() > 270.0:
            return "spline_jump J%d" % (j + 1)
    for j, mx in enumerate(MAX_TAUGHT_PLACE_TRANSFER_JOINT_DELTA_DEG):
        ref = traj[0, j] if j in WRAP_EQUIVALENT_JOINT_IDX else start[j]
        if np.abs(traj[:, j] - ref).max() > mx:
            return "swing J%d" % (j + 1)
    return None


def main():
    jsonl = [p for a in sys.argv[1:] for p in glob.glob(a)]
    starts = load_retreat_starts(jsonl)
    starts.append(("overview", np.deg2rad(OVERVIEW_JOINTS_DEG).tolist()))
    print("start states: %d (%s)" % (len(starts), ", ".join(n for n, _ in starts)))

    mg = build_curobo_motion_gen(measured_tcp_model=False, static_cuboids=load_cuboids())
    ik = mg.ik_solver

    def fk(rad):
        st = mg.kinematics.get_state(torch.tensor([rad], device=DEV, dtype=torch.float32))
        return (st.ee_position[0].cpu().numpy().astype(float),
                st.ee_quaternion[0].cpu().numpy().astype(float))

    ref_rad = np.deg2rad(TAUGHT_SLOT0_PLACE_REFERENCE_JOINTS_DEG).tolist()
    ref_pos, ref_quat = fk(ref_rad)
    print("slot0 taught FK = %s mm  (POSX const %s)" % (
        np.round(ref_pos * 1000, 1), TAUGHT_SLOT0_PLACE_REFERENCE_POSX_MM_DEG[:3]))

    def tilted(q_wxyz):
        w, x, y, z = q_wxyz
        r = SciR.from_euler("y", ROW2_TILT_DEG, degrees=True) * SciR.from_quat([x, y, z, w])
        q = r.as_quat()
        return np.array([q[3], q[0], q[1], q[2]])

    def pose(pos, q):
        return Pose(position=torch.tensor([pos], device=DEV, dtype=torch.float32),
                    quaternion=torch.tensor([q], device=DEV, dtype=torch.float32))

    rows = []
    for slot in range(TAUGHT_TRAY_SLOT_COUNT):
        is_row2 = slot % 3 == 2
        release = ref_pos + slot_offset_m(slot)
        above = release.copy()
        above[2] += TAUGHT_SLOT0_ABOVE_CLEARANCE_M
        variants = [("plain", ref_quat)]
        if is_row2:
            variants.insert(0, ("tilt15", tilted(ref_quat)))   # what the planner actually sends
        for vname, q in variants:
            ok_starts, fails, end_js = [], [], []
            for sname, s_rad in starts:
                res = mg.plan_single(
                    CuroboJointState.from_position(
                        position=torch.tensor([s_rad], device=DEV, dtype=torch.float32),
                        joint_names=JOINT_NAMES),
                    pose(above.tolist(), q.tolist()), MotionGenPlanConfig(**PLAN_CFG))
                if not res.success.item():
                    fails.append("%s:%s" % (sname, str(res.status).split(".")[-1]))
                    continue
                traj = res.get_interpolated_plan().position.cpu().numpy()
                g = guard(traj, s_rad)
                if g:
                    fails.append("%s:%s" % (sname, g))
                    continue
                ok_starts.append(sname)
                end_js.append(np.round(np.rad2deg(traj[-1]), 1))
            # release pose: IK feasibility (non-row2 descends with a bridge MoveLine)
            out = ik.solve_single(pose(release.tolist(), q.tolist()))
            rel_ok = bool(out.success.view(-1).any().item())
            rows.append((slot, vname, above, release, len(ok_starts), len(starts), fails, rel_ok, end_js))

    print("\n%-4s %-6s %-24s %-24s %-8s %-7s %s" % (
        "slot", "quat", "above goal (mm)", "release goal (mm)", "plan OK", "rel IK", "failures"))
    for slot, vname, above, release, nok, n, fails, rel_ok, end_js in rows:
        print("%-4d %-6s %-24s %-24s %2d/%-5d %-7s %s" % (
            slot, vname, np.round(above * 1000, 1), np.round(release * 1000, 1),
            nok, n, "OK" if rel_ok else "FAIL", "; ".join(fails) if fails else "-"))
    print("\nend joints (deg) of successful ABOVE plans:")
    for slot, vname, *_rest in rows:
        end_js = _rest[-1]
        if end_js:
            print("  slot %2d %-6s  %s" % (slot, vname, " | ".join(str(e) for e in end_js[:3])))


if __name__ == "__main__":
    main()
