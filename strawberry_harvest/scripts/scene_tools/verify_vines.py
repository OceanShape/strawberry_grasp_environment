"""T3 검증 — 조립된 씬에서 덩굴 12개를 실제 월드 좌표로 확인한다.

    python3 strawberry_harvest/scripts/scene_tools/verify_vines.py

리포 루트에서 실행한다. Isaac 없이 usd-core 로 main_scene.usd 를 조립해
(1) 브릿지 좌표 발행 필터에 덩굴이 안 걸리는지 (2) 물리 스키마가 0건인지
(3) 보드/줄기에 실제로 닿는지 (4) 플래너가 겨냥하는 두 점이 덩굴 위인지
(5) 이웃 과실과 겹치지 않는지를 본다. 딸기 배치를 바꾸면 반드시 다시 돌린다.
"""
import sys, numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

st = Usd.Stage.Open('strawberry_harvest/scenes/main_scene.usd')
ok = True
def chk(cond, msg):
    global ok
    print(("  ✅ " if cond else "  ❌ ") + msg)
    ok = ok and bool(cond)

def world(prim, pts):
    m = np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0))
    return (np.hstack([pts, np.ones((len(pts), 1))]) @ m)[:, :3]

def mesh_pts(path):
    p = st.GetPrimAtPath(path)
    return world(p, np.array(UsdGeom.Mesh(p).GetPointsAttr().Get()))

print("[1] 플래너 입출력 불변 — 브릿지 좌표 발행 필터")
pub = [p.GetName() for p in st.Traverse()
       if "strawberry" in p.GetName().lower() and "robot" not in p.GetName().lower()
       and "unripe" not in p.GetName().lower()]
RIPE_COUNT = 8   # 2026-09-11 익은 8/4 전환 (unripe_01→ripe_07, unripe_03→ripe_08). 그 전 런 1~9 는 6
chk(len(pub) == RIPE_COUNT and not any("vine" in n for n in pub), "발행 대상 %d개 (덩굴 0개): %s" % (RIPE_COUNT, pub))

print("[2] 물리 없음")
vines = [p for p in st.Traverse() if p.GetName().startswith("vine_")]
chk(len(vines) == 12, "덩굴 프림 12개")
phys = [(q.GetPath().pathString, a) for v in vines for q in Usd.PrimRange(v)
        for a in q.GetAppliedSchemas() if "Physics" in a]
chk(not phys, "물리 스키마 0건")
joints = [j.GetPath().pathString for j in st.Traverse() if j.IsA(UsdPhysics.Joint)]
chk(not any("vine" in j for j in joints), "덩굴 조인트 0건 (전체 조인트 %d)" % len(joints))

print("[3] 보드 부착 / 줄기 연결")
board_y = mesh_pts('/World/lab_environment/whiteboard/board')[:, 1].mean()
tips, anchors = [], []
for v in sorted(vines, key=lambda x: x.GetName()):
    w = mesh_pts(v.GetPath().AppendChild("stem"))
    anchors.append(w[:, 1].max())
    berry = st.GetPrimAtPath("/World/strawberry_" + v.GetName()[5:])
    bw = mesh_pts(berry.GetPath().AppendPath("geo/fruit/mesh"))
    stem_tip = bw[bw[:, 2].argmax()]
    axis = np.array([bw[:, 0].mean(), bw[:, 1].mean()])
    # 덩굴 수직 구간의 중심축 (덩굴 끝 근처 링의 중심)
    low = w[w[:, 2] < w[:, 2].min() + 0.001]
    tips.append((v.GetName(), np.hypot(*(low[:, :2].mean(0) - stem_tip[:2])) * 1000,
                 (stem_tip[2] - low[:, 2].mean()) * 1000))
chk(all(a > board_y for a in anchors),
    "부착점 12개 모두 보드면(%.1fmm) 뒤: %.1f~%.1fmm" % (board_y*1000, min(anchors)*1000, max(anchors)*1000))
chk(max(t[1] for t in tips) < 3.6,
    "덩굴 끝 <-> 메시 줄기끝 측면거리 최대 %.1fmm (겹침 = 1.6+2.0 - 이 값)" % max(t[1] for t in tips))
chk(min(t[2] for t in tips) > 3.0,
    "덩굴 끝이 메시 줄기끝보다 %.1fmm 아래 (파묻힘)" % min(t[2] for t in tips))

print("[4] 플래너가 겨냥하는 점이 덩굴 위에 있는가")
SIDES = 10

def centerline_of(v):
    """튜브 정점을 링 단위로 평균내 중심선을 되살린다 (링 = 연속 10점)."""
    w = mesh_pts(v.GetPath().AppendChild("stem"))
    assert len(w) % SIDES == 0, len(w)
    return w.reshape(-1, SIDES, 3).mean(1)

def dist_to_polyline(g, cl):
    d = np.inf
    for a, b in zip(cl[:-1], cl[1:]):
        ab = b - a
        t = np.clip(np.dot(g - a, ab) / np.dot(ab, ab), 0.0, 1.0)
        d = min(d, float(np.linalg.norm(g - (a + t * ab))))
    return d

for name, off, lab in (("파지 목표", 0.035, "pick_target_z_bias_m"),
                       ("하강 시작", 0.065, "+ CRANE_Z_OFFSET_M")):
    dists = []
    for v in vines:
        c = np.array(UsdGeom.Xformable(v).ComputeLocalToWorldTransform(0))[3, :3]
        dists.append(dist_to_polyline(c + np.array([0.0, 0.0, off]), centerline_of(v)))
    worst = max(dists) * 1000
    chk(len(dists) == 12 and np.isfinite(worst) and worst < 0.2,
        "%s (과실중심 +%.0fmm, %s) 12개 최대 이탈 %.3fmm" % (name, off*1000, lab, worst))

print("[5] 이웃 과실·덩굴과의 간격")
allv = {v.GetName(): mesh_pts(v.GetPath().AppendChild("stem")) for v in vines}
allb = {p.GetName(): mesh_pts(p.GetPath().AppendPath("geo/fruit/mesh"))
        for p in st.Traverse() if p.GetName().startswith("strawberry_")}
worst = (1e9, "", "")
for vn, vw in allv.items():
    for bn, bw in allb.items():
        if bn[11:] == vn[5:]:
            continue
        if (vw.min(0) > bw.max(0) + 0.03).any() or (vw.max(0) < bw.min(0) - 0.03).any():
            continue
        d = np.linalg.norm(vw[:, None, :] - bw[None, :, :], axis=2).min()
        if d < worst[0]:
            worst = (d, vn, bn)
chk(worst[0] > 0, "덩굴 <-> 다른 과실 최소 %+.1fmm (%s vs %s)" % (worst[0]*1000, worst[1], worst[2]))

print("\n" + ("T3 검증 통과" if ok else "T3 검증 실패"))
sys.exit(0 if ok else 1)
