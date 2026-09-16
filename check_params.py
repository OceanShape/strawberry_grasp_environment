#!/usr/bin/env python3
"""문서(docs/parameters.md)와 실제 코드/에셋의 수치가 일치하는지 검사한다.

[신설 2026-09-09]

이 프로젝트는 같은 숫자가 여러 파일에 중복돼 있다 — 보드 y 하나가 5곳,
툴 오프셋이 2곳, 분할 격자가 2곳. 실제로 2026-09-09 에 보드를 옮기면서
cell_markers.usd 와 scan_executor_node.py 두 곳이 빠져 로봇이 딸기 앞을 집었다.
(cell_markers.usd 는 2026-09-10 에 제거 — 분면 표시는 whiteboard.usd 안의
highlight 오버레이가 맡고, 보드 prim 의 자식이라 보드와 같이 움직인다. 검사 대상 아님.)
코드로는 전혀 안 보이는 실패라 실행 전에 기계적으로 확인한다.

사용:  python3 check_params.py        (종료코드 0 = 정합)
"""
import os
import re
import sys

import yaml

ROOT = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(ROOT, "strawberry_harvest")
MOTION = os.path.join(ROOT, "src", "strawberry_motion")
SIMCORE = os.path.join(ROOT, "src", "strawberry_sim_core", "strawberry_sim_core")
GRIPCFG = os.path.join(ROOT, "src", "e0509_gripper_description", "config")


def grab(path, pattern):
    with open(path, encoding="utf-8") as f:
        m = re.search(pattern, f.read())
    return m.group(1) if m else None


def main():
    layout = os.path.join(SCENE, "scenes", "layers", "layout_layer.usd")
    physics = os.path.join(SCENE, "scenes", "layers", "physics_layer.usd")
    params = os.path.join(MOTION, "scripts", "harvest_motion_params.py")
    scan = os.path.join(MOTION, "strawberry_motion", "execution", "scan_executor_node.py")
    bridge = os.path.join(SIMCORE, "sim_executor_bridge_node.py")
    fake = os.path.join(SIMCORE, "fake_vision_node.py")
    quad = os.path.join(SIMCORE, "quadrant_filter.py")

    board_y = float(grab(layout, r'whiteboard"\n *\{\n[^}]*translate = \(0\.05, ([\d.]+),'))
    guard_y = float(grab(layout, r'board_guard[^}]*translate = \(0\.05, ([\d.]+),'))
    berry_y = float(grab(layout, r'strawberry_ripe_01"[^}]*translate = \([-\d.]+, ([\d.]+),'))
    stem_y = float(grab(physics, r'localPos0 = \([-\d.]+, ([\d.]+),'))
    wall = float(grab(params, r'\nWALL_SURFACE_Y_M = ([\d.]+)'))
    board_const = float(grab(scan, r'BOARD_SURFACE_Y_M = ([\d.]+)'))
    fake_board = float(grab(fake, r'"board_surface_y_m", ([\d.]+)'))

    envs = [os.path.join(GRIPCFG, "environment.yaml"),
            os.path.join(MOTION, "config", "environment.yaml")]
    scanworld = os.path.join(MOTION, "config", "scan_collision_world.yaml")

    fails = []

    def chk(label, ok, detail=""):
        print("  %s  %-42s %s" % ("OK " if ok else "!! ", label, detail))
        if not ok:
            fails.append(label)

    print("보드 기하 (5곳이 한 세트)")
    chk("layout_layer  보드 앞면", True, "%.4f" % board_y)
    chk("layout_layer  board_guard", abs(guard_y - (board_y + 0.10)) < 1e-6,
        "%.4f (기대 %.4f = 보드 + 0.10)" % (guard_y, board_y + 0.10))
    chk("layout_layer  딸기 y", abs(berry_y - (board_y - 0.0272)) < 1e-6,
        "%.4f (기대 %.4f = 보드 - 0.0272)" % (berry_y, board_y - 0.0272))
    chk("physics_layer localPos0", abs(stem_y - berry_y) < 1e-9,
        "%.4f (딸기와 동일해야 함)" % stem_y)
    chk("harvest_params WALL_SURFACE_Y_M", abs(wall - board_y) < 1e-6, "%.4f" % wall)
    chk("scan_executor  BOARD_SURFACE_Y_M", abs(board_const - board_y) < 1e-6, "%.4f" % board_const)
    chk("fake_vision    board_surface_y_m", abs(fake_board - board_y) < 1e-6, "%.4f" % fake_board)

    for p in envs:
        o = next(x for x in yaml.safe_load(open(p))["objects"] if x["name"] == "whiteboard")
        front = o["pose"][1] - o["dims"][1] / 2.0
        chk("cuRobo 상자 앞면  %s" % os.path.relpath(p, ROOT),
            abs(front - board_y) < 1e-6, "%.4f" % front)
    o = yaml.safe_load(open(scanworld))["scan_collision_world"]["objects"][0] \
        if "scan_collision_world" in yaml.safe_load(open(scanworld)) else None
    if o is None:
        d = yaml.safe_load(open(scanworld))
        o = next(x for x in (d.get("objects") or d[list(d)[0]]["objects"])
                 if x["name"] == "whiteboard")
    front = o["pose_wxyz"][1] - o["dims_m"][1] / 2.0
    chk("cuRobo 상자 앞면  scan_collision_world", abs(front - board_y) < 1e-6, "%.4f" % front)

    print("\n분할 격자 (2곳이 같아야 함)")
    for name, path in (("scan_executor", scan), ("quadrant_filter", quad)):
        x = float(grab(path, r'BOARD_SUBCELL_X_MID_M = ([\d.]+)'))
        z = float(grab(path, r'BOARD_SUBCELL_Z_MID_M = ([\d.]+)'))
        chk("%-16s x_mid / z_mid" % name, (x, z) == (0.050, 0.660), "%.3f / %.3f" % (x, z))

    print("\n툴 · 파지")
    tool = float(grab(bridge, r'"tool_tcp_offset_m", ([\d.]+)'))
    chk("bridge tool_tcp_offset_m", abs(tool - 0.236) < 1e-9,
        "%.3f (T3 의 ee_to_tcp_offset_m 과 같아야 함)" % tool)
    # [2026-09-10] 파지 판정은 **줄기 기준 along/lateral**. 조우가 실제로 무는
    # 구간(TCP 기준 -5.5~+26.5mm)에 줄기가 들어와야 CONTACT 다.
    near = float(grab(bridge, r'"jaw_capture_near_m", (-?[\d.]+)'))
    far = float(grab(bridge, r'"jaw_capture_far_m", ([\d.]+)'))
    zb = float(grab(bridge, r'"grasp_target_z_bias_m", ([\d.]+)'))
    offs = grab(os.path.join(MOTION, "scripts", "harvest_motion_params.py"),
                r'GRASP_RETRY_OFFSETS = \[([^\]]+)\]')
    offs = [float(v) for v in offs.split(",")]
    chk("파지 오프셋 사다리 <= 조우 물림 한계", max(offs) <= far + 1e-9,
        "최대 %.0fmm / 물림 한계 %.0fmm  (넘으면 조우가 줄기 앞에서 닫힌다)"
        % (max(offs) * 1000, far * 1000))
    chk("첫 오프셋이 물림 구간 안", near <= offs[0] <= far,
        "%.0fmm  (구간 %.0f~%.0fmm)" % (offs[0] * 1000, near * 1000, far * 1000))
    chk("bridge z bias == planner pick_target_z_bias_m", abs(zb - 0.035) < 1e-9,
        "%.3f  (T3 인자와 같아야 함)" % zb)
    act_p = float(grab(os.path.join(MOTION, "scripts", "harvest_motion_params.py"),
                       r'COLLISION_ACTIVATION_DISTANCE_M = ([\d.]+)'))
    act_b = float(grab(bridge, r'COLLISION_ACTIVATION_DISTANCE_M = ([\d.]+)'))
    chk("충돌 활성거리 플래너 == 브릿지", abs(act_p - act_b) < 1e-9,
        "%.3f / %.3f  (실기는 벽 없음=0, 시뮬은 5mm 로 통일)" % (act_p, act_b))
    # 그리퍼 닫힘 각도와 USD 관절 상한은 **한 쌍**이다.
    # 닫힘값이 상한보다 크면 l2/r2 만 잘려 손가락이 기울고 가위 모양이 된다.
    close = float(grab(bridge, r'"gripper_close_rad", ([\d.]+)'))
    usd_upper_deg = None
    try:
        from pxr import Usd, UsdPhysics
        stage = Usd.Stage.Open(os.path.join(SCENE, "scenes", "main_scene.usd"))
        for prim in stage.Traverse():
            if prim.GetName() in ("rh_r2", "rh_l2"):
                val = UsdPhysics.RevoluteJoint(prim).GetUpperLimitAttr().Get()
                usd_upper_deg = val if usd_upper_deg is None else min(usd_upper_deg, val)
    except Exception as exc:            # pxr 없이도 나머지 검사는 돌아가야 한다
        print("     (USD 상한 검사 생략: %s)" % exc)
    if usd_upper_deg is not None:
        upper_rad = usd_upper_deg * 3.141592653589793 / 180.0
        chk("gripper_close_rad <= USD rh_r2/rh_l2 상한", close <= upper_rad + 1e-6,
            "닫힘 %.3f rad / 상한 %.3f rad (%.3f deg)" % (close, upper_rad, usd_upper_deg))
    chk("gripper_close_rad (파츠 간격)", abs(close - 1.08) < 1e-9,
        "%.3f rad -> 파츠 간격 0.3mm. 1.0 이면 9.4mm 로 줄기를 못 문다" % close)

    print("\n순회")
    first = grab(scan, r'_ALL_CELLS_CLOCKWISE_ORDER = \["([^"]+)"')
    chk("_ALL_CELLS_CLOCKWISE_ORDER 첫 셀", first == "root/nw", "%s (실기 순서 nw->ne->se->sw)" % first)

    print()
    if fails:
        print("불일치 %d건 — docs/parameters.md 와 대조할 것" % len(fails))
        return 1
    print("전부 정합 (docs/parameters.md 기준)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
