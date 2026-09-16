#!/usr/bin/env python3
"""녹화용 뷰포트 카메라 구도 검산 — 보드·트레이·로봇이 화면 어디에 오는지, HUD·손목 카메라 창과 겹치는지 (S7, 2026-09-16).

    python3 strawberry_harvest/scripts/scene_tools/check_camera_framing.py                      # extension.toml 의 현재 핀
    python3 strawberry_harvest/scripts/scene_tools/check_camera_framing.py --az 35 --el 30 --dist 3.0 --focal 24
    python3 strawberry_harvest/scripts/scene_tools/check_camera_framing.py --pos -1.42,-1.74,2.36 --rot 55.24,0,-30.24 --focal 24
    python3 strawberry_harvest/scripts/scene_tools/check_camera_framing.py --search               # 격자 탐색, 조건 만족 중 보드 폭 상위

numpy 만 쓴다 (Isaac·pxr 없이). 핀홀 카메라 근사 — Kit 퍼스펙티브 카메라(조리개 20.955, 16:9)와 같은 화각 규칙이다.
씬 지점은 layout_layer / lab_environment 의 값을 상수로 옮겨 적었다(아래) — 보드·계란판을 옮기면 여기도 고친다.

Kit 카메라 규칙: 로컬 -Z 를 본다. rotateXYZ (rx, 0, rz) 에서 시선 = (-sin rz·sin rx, cos rz·sin rx, -cos rx).
    -> 목표점 T 를 보게 하려면 rx = acos(-d_z), rz = atan2(-d_x, d_y) (d = 단위 시선). 이 스크립트가 그걸 계산해 준다.
"""
import argparse
import itertools
import math
import os
import re

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
TOML = os.path.join(REPO, "strawberry_harvest/kit_ext/strawberry.sim.setup/config/extension.toml")

APERTURE = 20.955          # Kit 퍼스펙티브 카메라 horizontalAperture (확장이 같은 값으로 고정한다)
ASPECT = 16.0 / 9.0        # 1080p 녹화

#: 씬 지점 (world, m). 보드 = lab_environment whiteboard (0.05, 0.81, 0.66) 스케일 1.09×0.79.
BOARD_C = np.array([0.05, 0.81, 0.66])
BOARD = [(-0.495, 0.81, 0.265), (0.595, 0.81, 0.265), (-0.495, 0.81, 1.055), (0.595, 0.81, 1.055)]
#: 계란판 = layout_layer egg_carton translate (0.7710, 0.1366, 0), 3×5 컵 68mm 피치가 -x·-y 로 → 대략 x 0.60~0.80, y -0.02~0.17, 높이 ~80mm
TRAY = [(0.80, 0.17, 0.0), (0.60, 0.17, 0.0), (0.80, -0.02, 0.0), (0.60, -0.02, 0.0), (0.80, 0.17, 0.08), (0.60, -0.02, 0.08)]
#: 로봇: 베이스 원점, 올라간 팔꿈치(대략), 보드 앞 손목
ROBOT = [(0.0, 0.0, 0.0), (0.0, 0.3, 1.15), (0.0, 0.55, 0.95)]
#: 화면 위 UI 영역 (비율). HUD 패널 좌상단(완료 줄까지), 손목 카메라 창 좌하단 (isaac_sim_viewport_display.py 의 크기 기준)
UI = {"HUD": (0.0, 0.24, 0.0, 0.43), "PiP": (0.0, 0.27, 0.60, 1.0)}


def rx_mat(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rz_mat(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def look_at(P, T):
    d = (T - P) / np.linalg.norm(T - P)
    return math.degrees(math.acos(-d[2])), math.degrees(math.atan2(-d[0], d[1]))


def project(P, rx, rz, focal, pts):
    """월드 점 -> 화면 비율 (0..1, 왼쪽 위 원점). 프레임 밖이면 0..1 을 벗어난다."""
    R = rz_mat(math.radians(rz)) @ rx_mat(math.radians(rx))
    th = APERTURE / (2.0 * focal)
    tv = th / ASPECT
    out = []
    for w in pts:
        c = R.T @ (np.asarray(w, float) - P)
        out.append((0.5 + c[0] / (-c[2]) / (2 * th), 0.5 - c[1] / (-c[2]) / (2 * tv)))
    return out


def box(pp):
    return min(p[0] for p in pp), max(p[0] for p in pp), min(p[1] for p in pp), max(p[1] for p in pp)


def overlaps(b, ui):
    return not (b[1] < ui[0] or b[0] > ui[1] or b[3] < ui[2] or b[2] > ui[3])


def pose_from_angles(az, el, dist):
    v = np.array([-math.sin(math.radians(az)) * math.cos(math.radians(el)),
                  -math.cos(math.radians(az)) * math.cos(math.radians(el)),
                  math.sin(math.radians(el))])
    return BOARD_C + dist * v


def evaluate(P, rx, rz, focal):
    b, t, r = box(project(P, rx, rz, focal, BOARD)), box(project(P, rx, rz, focal, TRAY)), box(project(P, rx, rz, focal, ROBOT))
    return {"board": b, "tray": t, "robot": r, "board_w": b[1] - b[0],
            "in_frame": all(0 <= v <= 1 for bb in (b, t, r) for v in bb),
            "hits": [n for n, ui in UI.items() for bb in (b, t) if overlaps(bb, ui)]}


def hfov(focal):
    return 2 * math.degrees(math.atan(APERTURE / (2 * focal)))


def report(P, rx, rz, focal, tag=""):
    e = evaluate(P, rx, rz, focal)
    print("%spos (%.3f, %.3f, %.3f)  rot (%.2f, 0, %.2f)  focal %.1f  (hfov %.0f deg, %.0f mm-eq)"
          % (tag, P[0], P[1], P[2], rx, rz, focal, hfov(focal), 36.0 / (APERTURE / focal)))
    for k in ("board", "tray", "robot"):
        x0, x1, y0, y1 = e[k]
        print("  %-6s x %5.1f%% .. %5.1f%%   y %5.1f%% .. %5.1f%%%s" % (k, x0 * 100, x1 * 100, y0 * 100, y1 * 100,
              ("   (board width %.0f%%)" % (e["board_w"] * 100)) if k == "board" else ""))
    print("  frame: %s   UI overlap: %s" % ("all in" if e["in_frame"] else "CLIPPED", ", ".join(sorted(set(e["hits"]))) or "none"))
    return e


def read_toml():
    s = open(TOML, encoding="utf-8").read()
    def vec(name):
        m = re.search(r'%s\s*=\s*\[([^\]]+)\]' % re.escape(name), s)
        return tuple(float(v) for v in m.group(1).split(","))
    m = re.search(r'persp_focal_length\s*=\s*([0-9.]+)', s)
    return np.array(vec("persp_translate")), vec("persp_rotate_xyz"), float(m.group(1)) if m else 18.147


def search():
    rows = []
    for az, el, d, F in itertools.product((25, 30, 35, 40), (25, 30, 35), (2.6, 2.8, 3.0, 3.2, 3.4, 3.6), (20.4, 22.0, 24.0, 26.0, 29.1)):
        P = pose_from_angles(az, el, d)
        for T in itertools.product((0.0, 0.1, 0.2, 0.3), (0.4, 0.55, 0.7), (0.4, 0.5, 0.6)):
            rx, rz = look_at(P, np.array(T))
            e = evaluate(P, rx, rz, F)
            b, t = e["board"], e["tray"]
            # 피사체가 x>=28% (UI 회피), 보드 위 4% 여유, 트레이·로봇이 프레임 안
            if e["in_frame"] and b[0] >= 0.28 and b[2] >= 0.04 and t[1] <= 0.96 and t[3] <= 0.95 and not e["hits"]:
                rows.append((e["board_w"], az, el, d, F, T, P, rx, rz))
    rows.sort(key=lambda r: -r[0])
    print("조건 만족 %d 개 — 보드 폭 상위 10" % len(rows))
    for bw, az, el, d, F, T, P, rx, rz in rows[:10]:
        print("board %.0f%% | az %d el %d dist %.1f focal %.1f target %s -> pos (%.3f, %.3f, %.3f) rot (%.2f, 0, %.2f)"
              % (bw * 100, az, el, d, F, T, P[0], P[1], P[2], rx, rz))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pos", help="x,y,z (m)")
    ap.add_argument("--rot", help="rx,ry,rz (deg, XYZ)")
    ap.add_argument("--az", type=float, help="보드 정면 기준 왼쪽 각도 (deg)")
    ap.add_argument("--el", type=float, help="위에서 내려다보는 각도 (deg)")
    ap.add_argument("--dist", type=float, help="보드 중심에서 거리 (m)")
    ap.add_argument("--target", default="0,0.7,0.4", help="시선이 향하는 점 x,y,z (az/el/dist 모드)")
    ap.add_argument("--focal", type=float)
    ap.add_argument("--search", action="store_true")
    a = ap.parse_args()
    if a.search:
        search()
        return
    if a.az is not None:
        P = pose_from_angles(a.az, a.el, a.dist)
        rx, rz = look_at(P, np.array([float(v) for v in a.target.split(",")]))
        report(P, rx, rz, a.focal or 24.0, tag="[az/el/dist] ")
        return
    if a.pos:
        P = np.array([float(v) for v in a.pos.split(",")])
        rx, _, rz = (float(v) for v in a.rot.split(","))
        report(P, rx, rz, a.focal or 18.147, tag="[pos/rot] ")
        return
    P, (rx, _, rz), F = read_toml()
    report(P, rx, rz, a.focal or F, tag="[extension.toml] ")
    print("(참고) 09-10 핀:")
    report(np.array([-1.86373, -1.15613, 2.08606]), 58.1076, -46.70705, 18.147, tag="[09-10] ")


if __name__ == "__main__":
    main()
