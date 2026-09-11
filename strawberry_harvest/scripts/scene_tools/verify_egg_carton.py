"""T4-3 검증 (4차) — 조립된 씬에서 계란판의 규칙성·정렬과 런 8 실측으로 예측한 착지(4차 인접 시퀀스)를 대조한다.

    python3 strawberry_harvest/scripts/scene_tools/verify_egg_carton.py   (리포 루트에서)

게이트: (1) 물리 스키마 0건·브릿지 발행 필터 무관 (2) 배치값 = fit 값 (3) 컨테이너 규칙성 — 수평, 직교, 컵 15개 동일,
림 사이 능선 ≥ 2mm, 밑변 = 테이블 상판(z 0) (4) 과실 6개(수확 순서로 슬롯 0,1,3,4,6,7)가 각자 **자기 셀** 안, 과실 표면이
컵 벽·바닥을 **어디서도 뚫지 않고**(4차 요구 ① — 옆의 빈 컵에서 보였던 것), 림 여유 ≥1mm, 밑끝이 컵 바닥 위(딱 맞음: 최저 과실은
바닥에 닿음), 꼭지가 림 위로 나온다 (5) 정렬·파라미터 — 컵 격자 중점 y = 테이블 중심축, run_nodes.sh 의 pitch_override / shift /
slot_sequence 가 geom 과 일치, shift 상수가 현재 실측 ideal 과 1mm 안.
"""
import os
import re
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
chk(len(pub) == len(G.PLACE_SLOT_SEQUENCE), "브릿지 발행 대상(익은 과실) %d개 = 배치 슬롯 수 %d (계란판 무관; 09-11 익은 8/4)" % (len(pub), len(G.PLACE_SLOT_SEQUENCE)))
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
rim_rx, rim_ry = G.CUP_RINGS_M[0][1], G.CUP_RINGS_M[0][2]
chk(G.PITCH_COL_M - 2 * rim_rx >= G.CUP_WALL_MIN_M - 1e-9 and G.PITCH_ROW_M - 2 * rim_ry >= G.CUP_WALL_MIN_M - 1e-9,
    "림 rx%.1f/ry%.1f (구멍 %.1f × %.1f mm ≥ 과실 최대 지름 %.1f × %.1f) — 능선 x %.1f / y %.1f mm ≥ %.0fmm"
    % (rim_rx * 1000, rim_ry * 1000, 2 * rim_rx * 1000, 2 * rim_ry * 1000,
       2 * G._PROFILE_HX_M.max() * 1000, 2 * G._PROFILE_HY_M.max() * 1000,
       (G.PITCH_COL_M - 2 * rim_rx) * 1000, (G.PITCH_ROW_M - 2 * rim_ry) * 1000, G.CUP_WALL_MIN_M * 1000))
chk(all(a[1] >= b[1] - 1e-9 and a[2] >= b[2] - 1e-9 for a, b in zip(G.CUP_RINGS_M, G.CUP_RINGS_M[1:])),
    "컵 벽이 위로 갈수록 좁아지지 않는다 (오버행 없음, 링 %d단)" % len(G.CUP_RINGS_M))
centers = np.array([G.cup_center_asset_m(s)[:2] for s in range(15)])
dcol = centers[1] - centers[0]; drow = centers[3] - centers[0]
chk(abs(dcol @ drow) < 1e-12 and abs(np.linalg.norm(dcol) - G.PITCH_COL_M) < 1e-9 and abs(np.linalg.norm(drow) - G.PITCH_ROW_M) < 1e-9,
    "직교 격자, 피치 %.1f × %.1f mm" % (G.PITCH_COL_M * 1000, G.PITCH_ROW_M * 1000))
# 컵 동일성: 각 컵의 판 아래 점(벽 링·바닥)을 컵 중심 기준으로 모아 기대 점집합과 행 단위로 비교
def _ring(rx, ry, z, n=G.CUP_SIDES):
    a = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    return np.stack([rx * np.cos(a), ry * np.sin(a), np.full(n, z)], 1)
_r = [_ring(rx, ry, z) for z, rx, ry in G.CUP_RINGS_M[1:]]            # 판 아래 링들 (림 제외)
expected = np.vstack([r for r in _r for _ in range(2)])                  # 각 링은 위 띠·아래 띠(또는 바닥)에 2회 등장
from scipy.spatial import cKDTree
same, worst = True, 0.0
for s in range(15):
    c = G.cup_center_asset_m(s)
    pts = P[(np.abs(P[:, 0] - c[0]) < G.PITCH_COL_M / 2) & (np.abs(P[:, 1] - c[1]) < G.PITCH_ROW_M / 2)
            & (P[:, 2] < G.PLATE_TOP_Z_M - 1e-6) & (P[:, 2] > G.SKIRT_BOTTOM_Z_M + 1e-6)] - c
    if pts.shape != expected.shape:
        same = False; print("     slot %2d 점 수 %d ≠ 기대 %d" % (s, len(pts), len(expected))); continue
    # 점집합 일치: 기대 점 → 컵 점 최근접 거리 (정렬 기반 비교는 %.5f 반올림으로 순서가 뒤집혀 오판한다)
    dmax = float(cKDTree(pts).query(expected)[0].max()); worst = max(worst, dmax)
    same &= dmax < 1e-5
chk(same, "컵 15개 형상 동일 (컵당 %d점, 기대 점집합과 최대 차 %.2e m)" % (len(expected), worst))
chk(abs(P[:, 2].min() - G.SKIRT_BOTTOM_Z_M) < 1e-9 and abs(G.SKIRT_BOTTOM_Z_M) < 1e-9,
    "쉘 최저점 z = %.1fmm = 테이블 상판 (계란판이 테이블 위에 놓인다; 컵 바닥 %.1fmm 은 그 위)" % (P[:, 2].min() * 1000, G.CUP_FLOOR_Z_M * 1000))
table_top = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]).ComputeWorldBound(
    st.GetPrimAtPath("/World/lab_environment/table")).ComputeAlignedRange().GetMax()[2]
chk(abs(t_scene[2] + G.SKIRT_BOTTOM_Z_M - table_top) < 1e-6, "계란판 밑변 world z %.1fmm = 테이블 상판 %.1fmm" % ((t_scene[2] + G.SKIRT_BOTTOM_Z_M) * 1000, table_top * 1000))

print("[4] 착지 후보 = 배치 슬롯 %d칸 × 런 %s 실측 편차 %d개 (정사각 %.0fmm + y %+.1fmm; 어느 과실이 어느 칸에 가든 이 봉투 안이라고 본다)"
      % (len(G.PLACE_SLOT_SEQUENCE), G.RUN_ID, len(G.RUN_SLOT_SEQUENCE), G.PITCH_M * 1000, G.GRID_SHIFT_Y_M * 1000))
fl = np.array(UsdGeom.Mesh(st.GetPrimAtPath("/World/strawberry_ripe_01/geo/fruit/mesh")).GetPointsAttr().Get()) * 0.005
fp_world = R.from_euler("z", -81.5, degrees=True).apply(fl)          # 파지→배치 회전 (world)
hx, hy = G.PITCH_COL_M / 2, G.PITCH_ROW_M / 2
Ra = R.from_euler("z", -yaw_scene).as_matrix()

def _assess(slot, f):
    """한 후보의 (셀 안, 벽 여유, 림 여유, 밑끝−바닥, 림 위 노출, 치우침 xy)."""
    fa = G.world_to_asset(f, t_scene, yaw_scene)
    c = G.cup_center_asset_m(slot)
    off = (fa - c)[:2] * 1000
    inside = abs(off[0]) < hx * 1000 and abs(off[1]) < hy * 1000
    surf = (fp_world @ Ra.T) + fa
    band = surf[np.abs(surf[:, 2] - G.PLATE_TOP_Z_M) < 0.001]
    if len(band) == 0:
        band = surf[np.argsort(np.abs(surf[:, 2] - G.PLATE_TOP_Z_M))[:8]]
    sd = np.hypot((band[:, 0] - c[0]) / rim_rx, (band[:, 1] - c[1]) / rim_ry)
    dist = np.hypot(band[:, 0] - c[0], band[:, 1] - c[1])
    rim_gap = float(((1.0 / np.maximum(sd, 1e-9) - 1.0) * dist * 1000).min())
    below = surf[surf[:, 2] < G.PLATE_TOP_Z_M - 0.001]
    wall = np.inf
    for q in below:
        w = G.cup_radii_at_z(q[2])
        if w is None:
            continue
        sdist = np.hypot((q[0] - c[0]) / w[0], (q[1] - c[1]) / w[1])
        wall = min(wall, (1.0 / max(sdist, 1e-9) - 1.0) * np.hypot(q[0] - c[0], q[1] - c[1]) * 1000)
    bottom = (f[2] - G.FRUIT_BOTTOM_BELOW_CENTER_M - G.CUP_FLOOR_Z_M) * 1000
    visible = (f[2] + G.FRUIT_TOP_ABOVE_CENTER_M - G.PLATE_TOP_Z_M) * 1000
    return inside, wall, rim_gap, bottom, visible, off

in_cell = True; min_wall = np.inf; min_rim_gap = np.inf; min_visible = np.inf; min_bottom = np.inf; max_bottom = -np.inf
by_slot = {}
for slot, f in G.landing_candidates():
    r = _assess(slot, f)
    by_slot.setdefault(slot, []).append(r)
    in_cell &= r[0]; min_wall = min(min_wall, r[1]); min_rim_gap = min(min_rim_gap, r[2])
    min_bottom = min(min_bottom, r[3]); max_bottom = max(max_bottom, r[3]); min_visible = min(min_visible, r[4])
for slot in G.PLACE_SLOT_SEQUENCE:
    rs = by_slot[slot]
    print("     slot %2d  후보 %d  치우침 x %+.1f~%+.1f y %+.1f~%+.1f  벽 여유 최소 %+.1fmm  림 여유 최소 %+.1f  밑끝−바닥 %+.1f~%+.1f  림 위로 ≥ %.1fmm"
          % (slot, len(rs), min(r[5][0] for r in rs), max(r[5][0] for r in rs), min(r[5][1] for r in rs), max(r[5][1] for r in rs),
             min(r[1] for r in rs), min(r[2] for r in rs), min(r[3] for r in rs), max(r[3] for r in rs), min(r[4] for r in rs)))
chk(in_cell, "후보 전부 자기 셀 안 (반폭 %.1f × %.1f mm)" % (hx * 1000, hy * 1000))
chk(min_wall >= 0.0, "과실 표면이 컵 벽을 어디서도 뚫지 않는다 (후보 최소 여유 %+.1fmm; 설계 여유 %.1fmm − 프로파일 보간 오차)" % (min_wall, G.CUP_CLEARANCE_M * 1000))
chk(min_rim_gap >= 1.0, "림 높이 z%.0f 에서 과실 표면–림 여유 ≥1mm (최소 %+.1fmm)" % (G.PLATE_TOP_Z_M * 1000, min_rim_gap))
chk(min_bottom >= -0.1 and min_bottom <= 0.2, "컵 바닥이 가장 낮은 후보 밑끝에 딱 맞는다 (밑끝−바닥 최소 %+.1f, 최대 %+.1fmm)" % (min_bottom, max_bottom))
chk(min_visible > 0.0, "과실 꼭지가 림 위로 나온다 (최소 %.1fmm)" % min_visible)
# 실측 착지 (런 격자 = 지금 격자일 때만 의미)
if G.RUN_GRID["pitch_col"] == G.PITCH_COL_M and abs(G.RUN_GRID["shift_y"] - G.GRID_SHIFT_Y_M) < 1e-9:
    worst = min(_assess(s, f)[1] for s, f in G.measured_landings())
    chk(worst >= 0.0, "런 %s 실측 착지 %d개도 지금 컵 안 (벽 여유 최소 %+.1fmm)" % (G.RUN_ID, len(G.RUN_SLOT_SEQUENCE), worst))
print("     (편차 = 분면별 매달린 깊이 차 + 런 간 팔 산포. 다음 런에서 편차가 커지면 여유가 줄 수 있다 — 그때 FRUIT_REST_M 갱신·재생성)")

print("[5] 정렬 — 계란판 중점 = 테이블 중심축, 평행이동 파라미터 일치")
center_w = G.carton_center_world_m(t_scene, yaw_scene)
table = st.GetPrimAtPath("/World/lab_environment/table")
tb = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]).ComputeWorldBound(table).ComputeAlignedRange()
table_axis_y = 0.5 * (tb.GetMin()[1] + tb.GetMax()[1])
chk(abs(table_axis_y - G.TABLE_AXIS_Y_M) < 1e-4, "테이블 중심축 y = %.1fmm (geom TABLE_AXIS_Y_M 과 일치)" % (table_axis_y * 1000))
chk(abs(center_w[1] - table_axis_y) <= 1.0e-3,
    "컵 격자 수평 중점 world (%.1f, %+.2f) mm — y 가 테이블 중심축 위 (|편차| ≤1mm)" % (center_w[0] * 1000, center_w[1] * 1000))
ideal = G.ideal_grid_shift_y_m()
chk(abs(ideal - G.GRID_SHIFT_Y_M) <= 1.0e-3,
    "geom GRID_SHIFT_Y_M %.4f vs 현재 실측(런 %s) 기준 ideal %.4f — 차 %.2fmm ≤ 1mm" % (G.GRID_SHIFT_Y_M, G.RUN_ID, ideal, abs(ideal - G.GRID_SHIFT_Y_M) * 1000))
rn = open(os.path.join(REPO, "scripts", "run_nodes.sh"), encoding="utf-8").read()
def _param(name, ref, fmt_log):
    m = re.search(r"-p %s:=([0-9.+-]+)" % name, rn)
    m2 = re.search(r'need planner\.log "%s=([0-9.+-]+)"' % name, rn)
    chk(m is not None and abs(float(m.group(1)) - ref) < 1e-6, "run_nodes.sh 플래너 인자 %s:=%s = geom %s" % (name, m.group(1) if m else "없음", fmt_log % ref))
    chk(m2 is not None and abs(float(m2.group(1)) - ref) < 1e-6, "run_nodes.sh 기동 로그 대조 %s=%s = geom" % (name, m2.group(1) if m2 else "없음"))
_param("taught_grid_pitch_override_m", G.PITCH_M, "%.4f")
_param("taught_grid_shift_y_m", G.GRID_SHIFT_Y_M, "%.4f")
seq_arg = re.search(r"-p taught_slot_sequence:=([0-9,]+)", rn)
seq_need = re.search(r'need planner\.log "slot_sequence=\\\[([0-9, ]+)\\\]"', rn)
want = ",".join(map(str, G.PLACE_SLOT_SEQUENCE))
chk(seq_arg is not None and seq_arg.group(1) == want, "run_nodes.sh taught_slot_sequence:=%s = geom PLACE_SLOT_SEQUENCE" % (seq_arg.group(1) if seq_arg else "없음"))
chk(seq_need is not None and seq_need.group(1).replace(" ", "") == want, "run_nodes.sh 기동 로그 대조 slot_sequence=[%s] = geom" % (seq_need.group(1) if seq_need else "없음"))
print("\n" + ("T4-3 검증 통과" if ok else "T4-3 검증 실패"))
sys.exit(0 if ok else 1)
