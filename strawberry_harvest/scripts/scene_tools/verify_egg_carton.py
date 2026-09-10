"""T4-3 검증 (2차) — 조립된 씬에서 계란판의 규칙성과 런 7 과실의 착지를 대조한다.

    python3 strawberry_harvest/scripts/scene_tools/verify_egg_carton.py   (리포 루트에서)

게이트: (1) 물리 스키마 0건·브릿지 발행 필터 무관 (2) 배치값 = fit 값 (3) 컨테이너 규칙성 — 수평, 직교,
컵 15개 동일 (4) 과실 6개가 각자 **자기 셀** 안에 있다. 컵 중심 치우침·림/벽 겹침 깊이는 **정보로만** 출력한다 —
그것이 티칭 오차의 눈에 보이는 크기이고, 컨테이너를 거기에 맞추지 않기로 했다.
"""
import os
import sys

import numpy as np
from pxr import Usd, UsdGeom
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import egg_carton_geom as G  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
st = Usd.Stage.Open(os.path.join(REPO, "strawberry_harvest/scenes/main_scene.usd"))
ok = True


def chk(c, m):
    global ok
    print(("  ✅ " if c else "  ❌ ") + m)
    ok = ok and bool(c)


print("[1] 플래너 입출력 불변")
pub = [p.GetName() for p in st.Traverse() if "strawberry" in p.GetName().lower()
       and "robot" not in p.GetName().lower() and "unripe" not in p.GetName().lower()]
chk(len(pub) == 6, "브릿지 발행 대상 6개 (계란판 무관)")
carton = st.GetPrimAtPath("/World/egg_carton")
chk(carton and carton.IsValid(), "/World/egg_carton 존재")
phys = [a for q in Usd.PrimRange(carton) for a in q.GetAppliedSchemas() if "Physics" in a]
chk(not phys, "물리 스키마 0건")

print("[2] 배치 = fit")
M = np.array(UsdGeom.Xformable(carton).ComputeLocalToWorldTransform(0))
t_scene = M[3, :3]; yaw_scene = float(np.arctan2(M[0, 1], M[0, 0]))
t_fit, yaw_fit, res = G.fit_pose()
chk(np.allclose(t_scene, t_fit, atol=5e-4) and abs(yaw_scene - yaw_fit) < 1e-3,
    "translate %s mm, yaw %+.2f° (fit 과 일치)" % (np.round(t_scene * 1000, 1).tolist(), np.degrees(yaw_scene)))

print("[3] 컨테이너 규칙성 (애셋 프레임)")
shell = st.GetPrimAtPath("/World/egg_carton/shell")
P = np.array(UsdGeom.Mesh(shell).GetPointsAttr().Get())
rim_z = P[np.isclose(P[:, 2], G.PLATE_TOP_Z_M)][:, 2]
chk(len(rim_z) > 0 and np.ptp(rim_z) < 1e-6, "판 윗면 수평 (z = %.1fmm 단일값)" % (G.PLATE_TOP_Z_M * 1000))
floor_z = P[np.isclose(P[:, 2], G.CUP_FLOOR_Z_M)][:, 2]
chk(len(floor_z) == 15 * G.CUP_SIDES * 2, "컵 바닥 15개 같은 높이 z=%.1fmm (테이블 상판 위)" % (G.CUP_FLOOR_Z_M * 1000))
centers = np.array([G.cup_center_asset_m(s)[:2] for s in range(15)])
dcol = centers[1] - centers[0]; drow = centers[3] - centers[0]
chk(abs(dcol @ drow) < 1e-12 and abs(np.linalg.norm(dcol) - G.PITCH_COL_M) < 1e-9 and abs(np.linalg.norm(drow) - G.PITCH_ROW_M) < 1e-9,
    "직교 격자, 피치 %.1f × %.1f mm" % (G.PITCH_COL_M * 1000, G.PITCH_ROW_M * 1000))
# 컵 동일성: 각 컵의 판 아래 점(벽 링·바닥)을 컵 중심 기준으로 모아 기대 점집합과 행 단위로 비교
def _ring(rx, ry, z, n=G.CUP_SIDES):
    a = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    return np.stack([rx * np.cos(a), ry * np.sin(a), np.full(n, z)], 1)
_r = [_ring(rx, ry, z) for z, rx, ry in G.CUP_RINGS_M[1:]]            # 판 아래 링 3개
expected = np.vstack([_r[0], _r[0], _r[1], _r[1], _r[2], _r[2]])        # 띠 2장 + 바닥에 각 2회 등장
def _canon(a):
    a = np.round(a, 6) + 0.0
    return a[np.lexsort((a[:, 2], a[:, 1], a[:, 0]))]
exp_c = _canon(expected)
same, worst = True, 0.0
for s in range(15):
    c = G.cup_center_asset_m(s)
    pts = P[(np.abs(P[:, 0] - c[0]) < G.PITCH_COL_M / 2) & (np.abs(P[:, 1] - c[1]) < G.PITCH_ROW_M / 2)
            & (P[:, 2] < G.PLATE_TOP_Z_M - 1e-6) & (P[:, 2] > G.SKIRT_BOTTOM_Z_M + 1e-6)] - c
    if pts.shape != expected.shape:
        same = False; print("     slot %2d 점 수 %d ≠ 기대 %d" % (s, len(pts), len(expected))); continue
    dmax = float(np.abs(_canon(pts) - exp_c).max()); worst = max(worst, dmax)
    same &= dmax < 1e-5
chk(same, "컵 15개 형상 동일 (컵당 %d점, 기대 점집합과 최대 차 %.2e m)" % (len(expected), worst))
chk(P[:, 2].min() >= G.SKIRT_BOTTOM_Z_M - 1e-9, "쉘 최저점 = 스커트 %.0fmm (컵이 그보다 내려가지 않음)" % (G.SKIRT_BOTTOM_Z_M * 1000))

print("[4] 과실 6개 **예상** 착지 = 직교화 격자 ee + 런 %s 실측 매달림 변위 (실제 착지는 다음 런에서 확인)" % G.RUN_ID)
fl = np.array(UsdGeom.Mesh(st.GetPrimAtPath("/World/strawberry_ripe_01/geo/fruit/mesh")).GetPointsAttr().Get()) * 0.005
fp_world = R.from_euler("z", -81.5, degrees=True).apply(fl)          # 파지→배치 회전 (world)
hx, hy = G.PITCH_COL_M / 2, G.PITCH_ROW_M / 2
in_cell = True; worst_clip = 0.0
for slot in sorted(G.FRUIT_REST_M):
    f = G.predicted_fruit_rest_m(slot)
    fa = G.world_to_asset(f, t_scene, yaw_scene)
    c = G.cup_center_asset_m(slot)
    off = (fa - c)[:2] * 1000
    inside = abs(off[0]) < hx * 1000 and abs(off[1]) < hy * 1000
    in_cell &= inside
    # 표면점을 애셋 프레임으로: 회전 부분만 (평행이동은 fa 로)
    Ra = R.from_euler("z", -yaw_scene).as_matrix()
    surf = (fp_world @ Ra.T) + fa
    below = surf[surf[:, 2] < G.PLATE_TOP_Z_M]
    clip = 0.0
    for q in below:
        rr = G.cup_radii_at_z(q[2])
        if rr is None:
            continue
        sdist = np.hypot((q[0] - c[0]) / rr[0], (q[1] - c[1]) / rr[1])
        if sdist > 1.0:
            clip = max(clip, (sdist - 1.0) * min(rr) * 1000)
    worst_clip = max(worst_clip, clip)
    print("     slot %2d  컵 중심 치우침 (%+5.1f, %+5.1f) mm  %s  림/벽 겹침 %.1fmm  밑끝 z %+.1fmm"
          % (slot, off[0], off[1], "셀 안" if inside else "셀 밖!", clip, (f[2] - G.FRUIT_BOTTOM_BELOW_CENTER_M) * 1000))
chk(in_cell, "예상 착지 6개 전부 자기 셀 안 (반폭 %.1f × %.1f mm)" % (hx * 1000, hy * 1000))
chk(worst_clip <= 1.0, "예상 착지에서 림/벽 겹침 ≤1mm (최대 %.1fmm — 격자가 합동이라 잔차 = 매달린 깊이 차뿐; 1mm 이하는 화면에서 안 보인다)" % worst_clip)
print("     (컨테이너 yaw 0 · 배치 직교화. 치우침 = 분면별 매달린 깊이 차(±4mm) + 런 간 팔 산포(~2mm))")
print("\n" + ("T4-3 검증 통과" if ok else "T4-3 검증 실패"))
sys.exit(0 if ok else 1)
