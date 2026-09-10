"""타겟별 하강 통로와 이웃 과실 사이 간격을 잰다 — 측정 전용, 씬을 바꾸지 않는다.

    python3 strawberry_harvest/scripts/scene_tools/measure_corridor_clearance.py

리포 루트에서 실행한다. Isaac 없이 usd-core 로 main_scene.usd 를 조립한다.

무엇을 재나: 익은 과실(타겟) 6개마다 **덩굴 수직 구간 = 과실중심 +25~+65mm** 선분을 잡고,
그 선분에서 다른 과실 11개의 메시 표면까지 최소거리를 구한다. 열린 조우는 이 선분을 따라
+65mm(하강 시작)에서 +35mm(파지 목표)까지 내려오므로, 이 거리가 조우 폭보다 작으면
그리퍼 파츠가 이웃 과실과 화면상 겹친다. 이웃 과실에는 콜라이더가 없어 물리 영향은 없다.

그리퍼 링크 치수는 스테이지에 저작된 자세에서의 **월드 축 정렬 AABB** 다.
파지 자세의 조우 폭이 아니라 규모 비교용이다.

근거 문서: portfolio/H_scope_decisions.md §4
"""
import numpy as np
from pxr import Usd, UsdGeom

st = Usd.Stage.Open('strawberry_harvest/scenes/main_scene.usd')
Z_LO, Z_HI = 0.025, 0.065   # 덩굴 수직 구간 (gen_vine_asset.py 의 Z_TIP, Z_BEND)


def world(prim, pts):
    m = np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0))
    return (np.hstack([pts, np.ones((len(pts), 1))]) @ m)[:, :3]


def mesh_pts(path):
    p = st.GetPrimAtPath(path)
    return world(p, np.array(UsdGeom.Mesh(p).GetPointsAttr().Get()))


def seg_min_dist(a, b, pts):
    d = b - a
    t = np.clip(((pts - a) @ d) / (d @ d), 0.0, 1.0)
    return np.linalg.norm(a + t[:, None] * d - pts, axis=1).min()


fruits = {p.GetName(): mesh_pts(p.GetPath().AppendPath("geo/fruit/mesh"))
          for p in st.Traverse() if p.GetName().startswith("strawberry_")}
center = {n: (v.min(0) + v.max(0)) / 2 for n, v in fruits.items()}

print("[1] 과실 중심 (world, mm)")
for n in sorted(fruits):
    c = center[n] * 1000
    print("  %-22s (%7.1f, %6.1f, %6.1f)  %s" % (n, c[0], c[1], c[2],
          "비타겟(발행 제외)" if "unripe" in n else "타겟"))

print("\n[2] 타겟 하강 통로(+%d~+%dmm) <-> 다른 과실 표면 최소거리" % (Z_LO * 1000, Z_HI * 1000))
rows = []
for n in sorted(fruits):
    if "unripe" in n:
        continue
    c = center[n]
    a, b = c + np.array([0, 0, Z_LO]), c + np.array([0, 0, Z_HI])
    best = min((seg_min_dist(a, b, pts), m) for m, pts in fruits.items() if m != n)
    rows.append((n, best[1], best[0] * 1000))
    print("  %-22s -> %-22s %+7.1fmm  (%s)" % (n, best[1], best[0] * 1000,
          "비타겟" if "unripe" in best[1] else "타겟"))
d = [r[2] for r in rows]
print("  최소 %+.1fmm / 최대 %+.1fmm / 20mm 이내 %d/%d" %
      (min(d), max(d), sum(x < 20 for x in d), len(d)))

print("\n[3] 그리퍼 링크 AABB (저작 자세, world 축, mm)")
seen = set()
for p in st.Traverse():
    n = p.GetName()
    if not n.startswith("rh_p12_rn") or n in seen:
        continue
    seen.add(n)
    bb = UsdGeom.Imageable(p).ComputeWorldBound(0, "default").ComputeAlignedBox()
    if bb.IsEmpty():
        continue
    s = (np.array(bb.GetMax()) - np.array(bb.GetMin())) * 1000
    print("  %-18s %6.1f x %6.1f x %6.1f" % (n, s[0], s[1], s[2]))
