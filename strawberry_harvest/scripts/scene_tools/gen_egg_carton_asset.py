"""assets/props/egg_carton.usd 생성기 (T4-3 계란판 시각 메쉬).

    python3 strawberry_harvest/scripts/scene_tools/gen_egg_carton_asset.py \
        [strawberry_harvest/assets/props/egg_carton.usd]

일반 python3 + numpy 로 돈다 (Isaac 불필요). 형상·좌표는 전부 egg_carton_geom.py.
물리·콜라이더 없음 — 시각 전용 (SUBMISSION_PLAN §2 게이트: 플래너 입출력 불변).
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import egg_carton_geom as G  # noqa: E402


def ellipse_ring(cx, rx, ry, z, n):
    a = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    return np.stack([cx[0] + rx * np.cos(a), cx[1] + ry * np.sin(a), np.full(n, z)], 1)


def project_to_parallelogram(origin, dirs, cell_center, hv, hh):
    """origin 에서 dirs 방향 반직선이 cell_center 중심 평행사변형(±hv, ±hh) 경계와 만나는 점."""
    M = np.stack([hv[:2], hh[:2]], 1)
    o = np.linalg.solve(M, (origin - cell_center)[:2])     # origin 의 (a,b) 좌표
    out = []
    for d in dirs:
        dd = np.linalg.solve(M, d[:2])
        ts = []
        for k in range(2):
            if abs(dd[k]) > 1e-12:
                for lim in (-1.0, 1.0):
                    tt = (lim - o[k]) / dd[k]
                    if tt > 0:
                        ts.append(tt)
        t = min(ts)
        ab = o + t * dd
        out.append(cell_center + ab[0] * hv + ab[1] * hh)
    return np.array(out)


def build():
    F0 = G.fit_origin_m()
    verts, counts, idx = [], [], []

    def add_poly(vs, reverse=False):
        base = len(verts)
        verts.extend(vs)
        order = list(range(len(vs)))
        if reverse:
            order = order[::-1]
        counts.append(len(vs))
        idx.extend(base + o for o in order)

    def add_strip(ring_a, ring_b, flip=False):
        """두 링(같은 점 수) 사이 사각형 띠."""
        n = len(ring_a)
        base = len(verts)
        verts.extend(ring_a)
        verts.extend(ring_b)
        for i in range(n):
            j = (i + 1) % n
            quad = [base + i, base + j, base + n + j, base + n + i]
            if flip:
                quad = quad[::-1]
            counts.append(4)
            idx.extend(quad)

    hv, hh = G.V_COL / 2.0, G.H_ROW / 2.0
    n = G.CUP_SIDES
    for slot in range(G.ROWS * G.COLS):
        cg = G.cup_center_local_m(slot)                # 셀(격자) 중심
        c = G.cup_used_center_local_m(slot, F0)         # 컵 중심 (점유 컵은 실측 과실 위치)
        rings = [ellipse_ring(c, rx, ry, z, n) for z, rx, ry in G.cup_rings_local_m(slot, F0)]
        rim, bot = rings[0], rings[-1]
        # 컵 벽: 안쪽에서 보이므로 법선이 안쪽(위)을 향하게 — 밖에서 볼 때는 판이 가린다
        for a, b in zip(rings, rings[1:]):
            add_strip(a, b, flip=True)
        add_poly(list(bot), reverse=False)          # 바닥 (위를 향함)
        # 판: 셀 경계(평행사변형) ↔ 림 사이 고리. 평면은 컵마다 z 가 다르므로 셀 z 로.
        dirs = rim - c
        outer = project_to_parallelogram(c, dirs, cg, hv, hh)
        outer[:, 2] = cg[2] + G.CUP_RIM_DZ_M
        add_strip(outer, rim, flip=False)           # 위를 향하는 고리

    # 겉 테두리 + 스커트: 전체 평행사변형 (마진 포함), 위 = 판 평면, 아래 = 테이블 밑
    m = G.PLATE_MARGIN_M
    ev = G.V_COL / np.linalg.norm(G.V_COL); eh = G.H_ROW / np.linalg.norm(G.H_ROW)
    corners_rc = [(-0.5, -0.5), (-0.5, G.COLS - 0.5), (G.ROWS - 0.5, G.COLS - 0.5), (G.ROWS - 0.5, -0.5)]
    top = []
    for r, cc in corners_rc:
        p = r * G.H_ROW + cc * G.V_COL
        p = p + m * (np.sign(cc + 0.5 - G.COLS / 2.0 + 1e-9) * ev + np.sign(r + 0.5 - G.ROWS / 2.0 + 1e-9) * eh)
        p[2] = (r * G.H_ROW + cc * G.V_COL)[2] + G.CUP_RIM_DZ_M
        top.append(p)
    top = np.array(top)
    # 마진 띠 (셀 바깥 ~ 테두리) — 셀 경계선을 따라 얇은 고리 4장
    inner = np.array([(r * G.H_ROW + cc * G.V_COL) + [0, 0, G.CUP_RIM_DZ_M] for r, cc in corners_rc])
    for k in range(4):
        a, b = inner[k], inner[(k + 1) % 4]
        A, B = top[k], top[(k + 1) % 4]
        add_poly([a, b, B, A])
    bottom = top.copy()
    bottom[:, 2] = G.SKIRT_BOTTOM_WORLD_Z_M - F0[2]
    add_strip(top, bottom, flip=False)               # 스커트 (바깥을 향하도록)

    V = np.array(verts)
    return F0, V, np.array(counts), np.array(idx)


def vec3(a):
    return "(%.5f, %.5f, %.5f)" % tuple(a)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..", "assets", "props", "egg_carton.usd")
    F0, V, counts, idx = build()
    res = G.fit_residuals_mm()
    lo, hi = V.min(0), V.max(0)
    ang = np.degrees(np.arccos(G.V_COL[:2] @ G.H_ROW[:2] / np.linalg.norm(G.V_COL[:2]) / np.linalg.norm(G.H_ROW[:2])))
    header = f'''#usda 1.0
(
    "계란판 시각 메쉬 — 5x3 컵. 격자는 플래너 티칭 상수, 원점은 런 7 과실 정지 위치 fit. 물리 없음"
    defaultPrim = "egg_carton"
    metersPerUnit = 1
    upAxis = "Z"
)

# [T4-3 2026-09-11] 계란판 — **시각 전용**. 생성기: scripts/scene_tools/gen_egg_carton_asset.py
#
# 왜: T2 부착 이후 과실이 실제로 트레이 자리에 놓이는데, 놓일 계란판이 없어 허공에 정지해 있었다.
#
# SUBMISSION_PLAN §2 게이트 — 플래너 입출력 불변:
#   - 콜라이더·강체·조인트 없음. 프림 이름에 "strawberry" 없음 (브릿지 발행 필터 무관).
#   - 좌표를 새로 만들지 않았다. 격자 벡터는 플래너 상수 TAUGHT_SLOT{{0,1,3}}_PLACE_REFERENCE 그대로:
#       열 축 v = {np.round(G.V_COL*1000,2).tolist()} mm,  행 축 h = {np.round(G.H_ROW*1000,2).tolist()} mm
#     두 축 사이각 {ang:.2f}° — 실기 3점 수동 티칭 오차의 재현. 컵도 그 격자로 놓아야 과실과 맞는다.
#   - 격자 상수는 그리퍼 밑동(ee) 위치이고 과실은 툴 축 ~250mm 앞에 놓이므로, 컵 원점은
#     **런 7({G.RUN_ID}) 과실 정지 위치 6개의 최소자승**이다:  F0 = {np.round(F0*1000,1).tolist()} mm (world)
#     → 이 값이 layout_layer.usd 의 translate. 격자 대비 잔차(mm) — **점유 컵 6개는 이만큼 옮겨 실측 과실에 맞췄다**:
{chr(10).join("#         slot %2d  %s" % (s, np.round(r,1).tolist()) for s, r in sorted(res.items()))}
#     잔차 x 성분은 분면별 접근 기울기(sw 10.1° vs nw/ne 0.2°) 로 매달린 깊이가 8.7mm 다른 것이 원인 (log/m3 §런 7).
#
# 형상: 컵 = 타원 그릇 {G.CUP_RINGS}링. 림은 피치가 허용하는 최대(rx{G.CUP_RIM_RX_M*1000:.0f}/ry{G.CUP_RIM_RY_M*1000:.0f}mm), 그 아래 벽은 **과실 반경 프로파일 + {G.CUP_WALL_MARGIN_M*1000:.0f}mm**\n#   (바닥 최소 rx{G.CUP_BOT_MIN_RX_M*1000:.0f}/ry{G.CUP_BOT_MIN_RY_M*1000:.0f}mm). 판 윗면은 과실 중심 {-G.CUP_RIM_DZ_M*1000:.0f}mm 아래,
#   컵 바닥 목표는 {-G.CUP_BOTTOM_DZ_M*1000:.0f}mm 아래(과실 밑끝 -33.1mm 보다 3mm 여유)지만 **테이블 상판 위 +{G.CUP_FLOOR_MIN_WORLD_Z_M*1000:.0f}mm 를 하한**으로 둔다 —
#   격자 z 기울기(행당 {G.H_ROW[2]*1000:+.1f}mm) 탓에 그대로 두면 바닥이 전부 상판 아래로 가 빈 컵에 테이블 면이 비친다. 낮은 행 과실은
#   밑끝이 바닥보다 아래로 잠기지만(최대 ~10mm) 그 부분은 바닥판·테이블 안이라 보이지 않는다. 판은 셀마다 격자 z 를 따라가므로
#   격자 z 기울기가 그대로 판 기울기다. 스커트는 상판 5mm 아래까지 내려 앉힌다.
#   ⚠️ 하강 중 과실 최대 반경(y 27mm) 이 림(ry 23mm) 을 지날 때 ~4mm 겹쳐 보인다. 콜라이더가 없어 물리 영향은 없다.
#   행 피치 51.2mm 가 과실 y 전폭 53.8mm 보다 작아 림을 더 키울 수 없다 — (A) 접촉 문제와 같은 뿌리.
#
# 색: 펄프 계란판. sRGB (176,158,132) → linear (0.43, 0.34, 0.23), roughness 0.9.

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
    print("F0 (layout_layer translate) = %s m" % np.round(F0, 4).tolist())
    for s, r in sorted(res.items()):
        print("  slot %2d 잔차 %s mm" % (s, np.round(r, 1).tolist()))


if __name__ == "__main__":
    main()
