"""T4-3 검증 — 조립된 씬에서 계란판과 런 7 과실 정지 위치를 대조한다.

    python3 strawberry_harvest/scripts/scene_tools/verify_egg_carton.py   (리포 루트에서)

(1) 물리 스키마 0건·브릿지 발행 필터 무관 (2) 과실 6개가 각자 컵 안에 앉는가 —
림 아래 과실 표면점이 전부 컵 벽 안쪽인지(타원 그릇 정확 판정), 밑끝이 컵 바닥 위인지
(3) 컵 중심 대비 치우침. 배치 슬롯이나 과실 위치가 바뀌면 다시 돌린다.
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


def world_pts(path):
    p = st.GetPrimAtPath(path)
    M = np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0))
    P = np.array(UsdGeom.Mesh(p).GetPointsAttr().Get())
    return (np.hstack([P, np.ones((len(P), 1))]) @ M)[:, :3]


print("[1] 플래너 입출력 불변")
pub = [p.GetName() for p in st.Traverse() if "strawberry" in p.GetName().lower()
       and "robot" not in p.GetName().lower() and "unripe" not in p.GetName().lower()]
chk(len(pub) == 6 and "egg" not in "".join(pub), "브릿지 발행 대상 6개 (계란판 무관)")
carton = st.GetPrimAtPath("/World/egg_carton")
chk(carton and carton.IsValid(), "/World/egg_carton 존재")
phys = [(q.GetPath().pathString, a) for q in Usd.PrimRange(carton) for a in q.GetAppliedSchemas() if "Physics" in a]
chk(not phys, "물리 스키마 0건")

print("[2] 원점·격자")
F0 = np.array(UsdGeom.Xformable(carton).ComputeLocalToWorldTransform(0))[3, :3]
chk(np.allclose(F0, G.fit_origin_m(), atol=5e-4), "translate = fit 원점 %s mm" % np.round(F0 * 1000, 1).tolist())

print("[3] 과실 6개가 컵 안에 앉는가 (런 %s)" % G.RUN_ID)
fruit_local = np.array(UsdGeom.Mesh(st.GetPrimAtPath("/World/strawberry_ripe_01/geo/fruit/mesh")).GetPointsAttr().Get()) * 0.005
Rf = R.from_euler("z", -81.5, degrees=True)      # 파지 자세 → 배치 자세 (툴 +Z: +y → +x 방향)
fp = Rf.apply(fruit_local)
def cup_radii_at(rings, z_local):
    """컵 링 목록에서 높이 z_local 의 (rx, ry) — 링 사이 선형보간. 바닥 아래는 None."""
    if z_local >= rings[0][0]:
        return rings[0][1], rings[0][2]
    for (z1, x1, y1), (z2, x2, y2) in zip(rings, rings[1:]):
        if z2 <= z_local <= z1:
            t = (z1 - z_local) / (z1 - z2)
            return x1 + t * (x2 - x1), y1 + t * (y2 - y1)
    return None


worst_off, worst_pen, worst_sink = 0.0, 0.0, 0.0
for slot, f in sorted(G.FRUIT_REST_M.items()):
    c = F0 + G.cup_used_center_local_m(slot, F0)
    rings = G.cup_rings_local_m(slot, F0)
    floor_world = rings[-1][0] + F0[2]
    off = f - c                                   # 점유 컵은 실측에 맞췄으므로 xy 는 0 이어야 한다
    worst_off = max(worst_off, np.hypot(*off[:2]) * 1000)
    P = fp + f
    below = P[P[:, 2] < rings[0][0] + F0[2]]
    pen, sink = 0.0, 0.0
    for q in below:
        rr = cup_radii_at(rings, q[2] - F0[2])
        if rr is None:                      # 컵 바닥보다 아래 = 바닥판·테이블 안, 보이지 않는다
            sink = max(sink, (floor_world - q[2]) * 1000)
            continue
        s = np.hypot((q[0] - c[0]) / rr[0], (q[1] - c[1]) / rr[1])   # <1 이면 벽 안쪽
        if s > 1.0:
            pen = max(pen, (s - 1.0) * min(rr) * 1000)
    worst_pen = max(worst_pen, pen); worst_sink = max(worst_sink, sink)
    print("     slot %2d  치우침 (%+.1f, %+.1f) mm  림 아래 표면점 %3d개 벽 관통 %.1fmm  바닥(world %+.1fmm) 아래로 잠김 %.1fmm"
          % (slot, off[0] * 1000, off[1] * 1000, len(below), pen, floor_world * 1000, sink))
chk(worst_pen == 0.0, "과실 표면이 컵 **벽**을 뚫지 않는다 (최대 관통 %.1fmm)" % worst_pen)
chk(worst_off < 0.1, "점유 컵 6개는 실측 과실 위치에 정렬 (xy 치우침 최대 %.2fmm)" % worst_off)
shifts = [np.hypot(*G.cup_shift_local_m(s, F0)[:2]) * 1000 for s in G.FRUIT_REST_M]
print("     격자 대비 컵 이동량 %.1f~%.1fmm (분면별 매달린 깊이 차)" % (min(shifts), max(shifts)))
floors = [G.cup_rings_local_m(s, F0)[-1][0] + F0[2] for s in range(15)]
chk(min(floors) >= G.CUP_FLOOR_MIN_WORLD_Z_M - 1e-6,
    "컵 바닥 15개 전부 테이블 상판 위 (최저 %+.1fmm) — 빈 컵에 테이블이 비치지 않는다" % (min(floors) * 1000))
print("     과실 밑끝이 바닥 아래로 잠기는 최대 %.1fmm — 바닥판·테이블 안이라 보이지 않음 (과실 위치는 런 실측 그대로)" % worst_sink)

print("[4] 컵 15개 · 스커트 · 테이블")
shell = world_pts("/World/egg_carton/shell")
skirt_bottom = shell[-4:, 2]            # 생성기가 마지막에 붙이는 스커트 아랫변 4점
chk(np.allclose(skirt_bottom, G.SKIRT_BOTTOM_WORLD_Z_M, atol=1e-4),
    "스커트 밑면 z=%.1fmm = 설정값 (테이블 상판 0 아래로 앉힘)" % (skirt_bottom.mean() * 1000))
chk(abs(shell[:, 2].min() - G.SKIRT_BOTTOM_WORLD_Z_M) < 1e-4, "쉘 최저점 = 스커트 밑면 (%.1fmm) — 컵이 그보다 아래로 내려가지 않는다" % (shell[:, 2].min() * 1000))
tops = [(F0 + G.cup_center_local_m(s))[2] + G.CUP_RIM_DZ_M for s in range(15)]
print("     판 윗면 z 범위 %.1f ~ %.1f mm (격자 z 기울기 반영)" % (min(tops) * 1000, max(tops) * 1000))
print("     바운딩 x %.0f~%.0f  y %.0f~%.0f mm" % (shell[:, 0].min() * 1000, shell[:, 0].max() * 1000, shell[:, 1].min() * 1000, shell[:, 1].max() * 1000))
print("\n" + ("T4-3 검증 통과" if ok else "T4-3 검증 실패"))
sys.exit(0 if ok else 1)
