"""assets/props/egg_carton.usd 생성기 (T4-3 계란판 시각 메쉬, 4차).

    python3 strawberry_harvest/scripts/scene_tools/gen_egg_carton_asset.py \
        [strawberry_harvest/assets/props/egg_carton.usd]

일반 python3 + numpy (Isaac 불필요). 형상·좌표·fit 은 전부 egg_carton_geom.py.
물리·콜라이더 없음 — 시각 전용 (SUBMISSION_PLAN §2 게이트: 플래너 입출력 불변).
씬 배치값(translate, orient)과 플래너 파라미터(taught_grid_pitch_override_m, taught_grid_shift_y_m, taught_slot_sequence)를 같이 출력한다 → layout_layer.usd / run_nodes.sh 에 옮긴다.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import egg_carton_geom as G  # noqa: E402


def ellipse_ring(cx, rx, ry, z, n):
    a = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    return np.stack([cx[0] + rx * np.cos(a), cx[1] + ry * np.sin(a), np.full(n, z)], 1)


def project_to_rect(center, dirs, hx, hy):
    """center 에서 dirs 방향 반직선이 반폭 (hx, hy) 직사각형 경계와 만나는 점."""
    out = []
    for d in dirs:
        ts = [hx / abs(d[0]) if abs(d[0]) > 1e-12 else np.inf, hy / abs(d[1]) if abs(d[1]) > 1e-12 else np.inf]
        t = min(ts)
        out.append(center + t * np.array([d[0], d[1], 0.0]))
    return np.array(out)


def build():
    verts, counts, idx = [], [], []

    def add_poly(vs, reverse=False):
        base = len(verts); verts.extend(vs)
        order = list(range(len(vs)))[::-1] if reverse else list(range(len(vs)))
        counts.append(len(vs)); idx.extend(base + o for o in order)

    def add_strip(ring_a, ring_b, flip=False):
        n = len(ring_a); base = len(verts)
        verts.extend(ring_a); verts.extend(ring_b)
        for i in range(n):
            j = (i + 1) % n
            quad = [base + i, base + j, base + n + j, base + n + i]
            counts.append(4); idx.extend(quad[::-1] if flip else quad)

    hx, hy = G.PITCH_COL_M / 2.0, G.PITCH_ROW_M / 2.0
    n = G.CUP_SIDES
    for slot in range(G.ROWS * G.COLS):
        c = G.cup_center_asset_m(slot)
        rings = [ellipse_ring(c, rx, ry, z, n) for z, rx, ry in G.CUP_RINGS_M]
        for a, b in zip(rings, rings[1:]):
            add_strip(a, b, flip=True)               # 컵 벽 (안쪽을 향함)
        add_poly(list(rings[-1]))                    # 바닥
        rim = rings[0]
        outer = project_to_rect(c, rim - c, hx, hy)
        outer[:, 2] = G.PLATE_TOP_Z_M
        add_strip(outer, rim)                        # 판 고리 (셀 직사각형 ↔ 림)

    # 겉 테두리(마진) + 스커트 — 직사각형
    m = G.PLATE_MARGIN_M
    x0, x1 = -(G.COLS - 1) * G.PITCH_COL_M - hx, hx
    y0, y1 = -(G.ROWS - 1) * G.PITCH_ROW_M - hy, hy
    inner = np.array([[x0, y0, G.PLATE_TOP_Z_M], [x1, y0, G.PLATE_TOP_Z_M], [x1, y1, G.PLATE_TOP_Z_M], [x0, y1, G.PLATE_TOP_Z_M]])
    outer = np.array([[x0 - m, y0 - m, G.PLATE_TOP_Z_M], [x1 + m, y0 - m, G.PLATE_TOP_Z_M],
                      [x1 + m, y1 + m, G.PLATE_TOP_Z_M], [x0 - m, y1 + m, G.PLATE_TOP_Z_M]])
    for k in range(4):
        add_poly([inner[k], inner[(k + 1) % 4], outer[(k + 1) % 4], outer[k]])
    bottom = outer.copy(); bottom[:, 2] = G.SKIRT_BOTTOM_Z_M
    add_strip(outer, bottom)                         # 스커트 — 밑변 z=0 = 테이블 상판 (4차). 밑면은 상판과 겹치므로 만들지 않는다.
    return np.array(verts), np.array(counts), np.array(idx)


def vec3(a):
    return "(%.5f, %.5f, %.5f)" % tuple(a)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..", "assets", "props", "egg_carton.usd")
    V, counts, idx = build()
    t, yaw, res = G.fit_pose()
    lo, hi = V.min(0), V.max(0)
    q = (float(np.cos(yaw / 2)), 0.0, 0.0, float(np.sin(yaw / 2)))
    header = f'''#usda 1.0
(
    "계란판 시각 메쉬 — 수평·정사각 격자 68mm·컵 15개 동일·과실 형상을 따르는 깊은 컵(4차). 자세는 씬 배치(layout_layer)에서. 물리 없음"
    defaultPrim = "egg_carton"
    metersPerUnit = 1
    upAxis = "Z"
)

# [T4-3 2026-09-11, 4차] 계란판 — **시각 전용**. 생성기: scripts/scene_tools/gen_egg_carton_asset.py, 형상 단일 출처 egg_carton_geom.py
#
# 1차(폐기): 티칭 격자 왜곡을 판에 그대로 구움 → "균일한 컨테이너" 라는 존재 이유가 사라짐(사용자 지적).
# 2차: 수평·직사각·컵 동일, 얕은 컵(21mm). 3차: 실기 15구 계란판처럼 깊은 컵(56mm) — 그러나 과실(y 전폭 53.8)이
# 실기 행 피치(51.2)보다 넓어 적도가 벽을 뚫었고, 런 8 에서 옆의 빈 컵으로 그것이 보였다.
# 4차: ① 컵이 과실의 수평 최대 지름을 담는다 — 피치를 시뮬 전용 정사각 {G.PITCH_M*1000:.0f}mm 로(플래너 taught_grid_pitch_override_m,
#       실기 컵에는 실기 모형 과실이 인접 3칸에 들어갔으므로 이것은 과실 애셋 치수의 불일치다) ② 컵 안쪽은 과실 프로파일 +
#       착지 치우침 + 여유 {G.CUP_CLEARANCE_M*1000:.1f}mm 를 따르고 바닥은 가장 낮게 놓인 과실 밑끝(z {G.CUP_FLOOR_Z_M*1000:.1f}mm)에 맞춘다 ③ 밑변 z0 = 테이블 상판
#       ④ 인접 칸 배치(taught_slot_sequence {",".join(map(str, G.PLACE_SLOT_SEQUENCE))}).
#
# 이 애셋 = 계란판 자체 프레임. 원점 = slot 0 컵 중심 아래 테이블 상판(z=0). 열은 -x, 행은 -y.
#   피치 {G.PITCH_COL_M*1000:.1f} × {G.PITCH_ROW_M*1000:.1f} mm (시뮬 전용; 실기 티칭 피치는 {G.REAL_PITCH_COL_M*1000:.1f} × {G.REAL_PITCH_ROW_M*1000:.1f})
#   컵 15개 동일: 림 rx{G.CUP_RINGS_M[0][1]*1000:.1f}/ry{G.CUP_RINGS_M[0][2]*1000:.1f} @ z{G.PLATE_TOP_Z_M*1000:.0f} → 바닥 rx{G.CUP_RINGS_M[-1][1]*1000:.1f}/ry{G.CUP_RINGS_M[-1][2]*1000:.1f} @ z{G.CUP_FLOOR_Z_M*1000:.1f} mm, 깊이 {(G.PLATE_TOP_Z_M-G.CUP_FLOOR_Z_M)*1000:.1f}mm, 링 {len(G.CUP_RINGS_M)}단
#   능선 x {(G.PITCH_COL_M-2*G.CUP_RINGS_M[0][1])*1000:.1f} / y {(G.PITCH_ROW_M-2*G.CUP_RINGS_M[0][2])*1000:.1f} mm. 림 위로는 과실 어깨·줄기만 보인다. 스커트 밑변 z{G.SKIRT_BOTTOM_Z_M*1000:.0f} (밑면 없음).
#
# 씬 배치 (layout_layer.usd) — 플래너 겨냥 ee 격자 15점(정사각 {G.PITCH_M*1000:.0f}mm + y {G.GRID_SHIFT_Y_M*1000:+.1f}mm) + 평균 ee→과실 변위(런 {G.RUN_ID}) 에 강체 최소자승(Kabsch):
#   translate = {np.round(t, 4).tolist()} m,  yaw = {np.degrees(yaw):+.2f}° (orient wxyz = {tuple(round(v, 6) for v in q)})
#   컵 격자 수평 중점 (world) = {np.round(G.carton_center_world_m(t, yaw) * 1000, 1).tolist()} mm — y 가 테이블 중심축 0 위에 있다.
#   플래너 파라미터와 쌍: taught_grid_pitch_override_m={G.PITCH_M:.4f}, taught_grid_shift_y_m={G.GRID_SHIFT_Y_M:.4f}, taught_slot_sequence={",".join(map(str, G.PLACE_SLOT_SEQUENCE))} (run_nodes.sh)
#   잔차(겨냥 목표 − 컵 중심, mm) — 두 격자가 합동이라 0:
{chr(10).join("#     slot %2d  %s" % (s, np.round(r, 1).tolist()) for s, r in sorted(res.items()))}
#     과실 정지 위치(런 실측)는 건드리지 않는다. 컵에서 벗어난 과실이 림·능선과 겹치는 것은 콜라이더가 없어 물리 영향이 없다.

def Xform "egg_carton"
{{
    def Mesh "shell" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    )
    {{
        float3[] extent = [{vec3(lo)}, {vec3(hi)}]
        rel material:binding = </egg_carton/Looks/Carton_Mat>
        color3f[] primvars:displayColor = [(0.43, 0.34, 0.23)]
        uniform token subdivisionScheme = "none"
        uniform bool doubleSided = 1
'''
    body = [
        "        int[] faceVertexCounts = [" + ", ".join(str(c) for c in counts) + "]",
        "        int[] faceVertexIndices = [" + ", ".join(str(i) for i in idx) + "]",
        "        point3f[] points = [" + ", ".join(vec3(p) for p in V) + "]",
    ]
    footer = '''    }

    def Scope "Looks"
    {
        def Material "Carton_Mat"
        {
            token outputs:mdl:displacement.connect = </egg_carton/Looks/Carton_Mat/Shader.outputs:out>
            token outputs:mdl:surface.connect = </egg_carton/Looks/Carton_Mat/Shader.outputs:out>
            token outputs:mdl:volume.connect = </egg_carton/Looks/Carton_Mat/Shader.outputs:out>

            def Shader "Shader"
            {
                uniform token info:implementationSource = "sourceAsset"
                uniform asset info:mdl:sourceAsset = @OmniPBR.mdl@
                uniform token info:mdl:sourceAsset:subIdentifier = "OmniPBR"
                color3f inputs:diffuse_color_constant = (0.43, 0.34, 0.23)
                float inputs:reflection_roughness_constant = 0.9
                float inputs:specular_level = 0.1
                token outputs:out (
                    renderType = "material"
                )
            }
        }
    }
}
'''
    with open(out, "w", encoding="utf-8") as f:
        f.write(header + "\n".join(body) + "\n" + footer)
    print("wrote %s  (%d points, %d faces)" % (os.path.normpath(out), len(V), len(counts)))
    print("layout_layer: translate = %s  yaw = %+.2f deg  orient(wxyz) = %s" % (np.round(t, 4).tolist(), np.degrees(yaw), tuple(round(v, 6) for v in q)))
    print("run_nodes.sh: -p taught_grid_pitch_override_m:=%.4f -p taught_grid_shift_y_m:=%.4f -p taught_slot_sequence:=%s   (ideal shift %.4f, 컵 격자 중점 world %s mm)"
          % (G.PITCH_M, G.GRID_SHIFT_Y_M, ",".join(map(str, G.PLACE_SLOT_SEQUENCE)), G.ideal_grid_shift_y_m(),
             np.round(G.carton_center_world_m(t, yaw) * 1000, 2).tolist()))
    print("cup: floor z %.1fmm, rim rx%.1f/ry%.1f, rings %d" % (G.CUP_FLOOR_Z_M * 1000, G.CUP_RINGS_M[0][1] * 1000, G.CUP_RINGS_M[0][2] * 1000, len(G.CUP_RINGS_M)))
    for s in sorted(res):
        print("  slot %2d 겨냥−컵 잔차 %s mm%s" % (s, np.round(res[s], 1).tolist(), "  ← 점유" if s in G.PLACE_SLOT_SEQUENCE else ""))


if __name__ == "__main__":
    main()
