"""assets/props/vine.usd 생성기 (T3 정적 덩굴).

    python3 strawberry_harvest/scripts/scene_tools/gen_vine_asset.py \
        strawberry_harvest/assets/props/vine.usd

Isaac Script Editor 가 아니라 **일반 python3** 에서 돈다 (usd-core + numpy 면 충분).
형상 파라미터는 전부 vine_geom.py 에 있다 — 굵기·높이·곡률을 바꾸려면 거기를 고치고
이 스크립트를 다시 돌린 뒤 verify_vines.py 로 확인한다.
"""
import os, sys, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vine_geom import (centerline, radii, tube, A, B, TIP,
                       Y_BOARD, Z_BOARD, Z_BEND, Z_TIP, R_BOARD, R_TIP)

pts = centerline()
rad = radii(pts)
v, counts, idx = tube(pts, rad, sides=10)

# 정점 법선 — 축에서 바깥으로. 캡은 면 법선이 따로 필요하지만 튜브 끝은 보드 뒤라 무시.
sides = 10
nrm = np.zeros_like(v)
for i in range(len(pts)):
    ring = v[i * sides:(i + 1) * sides] - pts[i]
    nrm[i * sides:(i + 1) * sides] = ring / np.linalg.norm(ring, axis=1)[:, None]

def vec3(a):
    return "(%.5f, %.5f, %.5f)" % tuple(a)

lo, hi = v.min(0), v.max(0)
header = f'''#usda 1.0
(
    "덩굴 프로토타입 — 보드에서 딸기 줄기 끝까지 이어지는 얇은 곡선 1개. 씬에서 12번 참조하고 배치만 layout_layer.usd 에서 준다"
    defaultPrim = "vine"
    metersPerUnit = 1
    upAxis = "Z"
)

# [T3 2026-09-10] 정적 덩굴 메쉬 — **시각 전용**.
#
# 왜 만드나: 딸기 12개가 보드 앞 27.2mm 허공에 떠 있었다. T2 로 과실이 그리퍼를
#   따라 이동하게 되면서, 붙잡고 있던 것이 화면에 없다는 것이 그대로 드러난다.
#   덩굴이 있으면 과실이 덩굴 끝에서 떨어져 나가는 것으로 읽힌다.
#
# SUBMISSION_PLAN §2 게이트: 플래너 2노드의 입출력을 전혀 바꾸지 않는다.
#   - 콜라이더·강체·조인트 없음 (physics_layer.usd 는 이 프림을 건드리지 않는다).
#   - 프림 이름에 "strawberry" 가 없다 -> 브릿지의 좌표 발행 필터
#     (isaac_sim_script_editor_bridge.py 의 name 검사)에 걸리지 않는다.
#     highlight 오버레이와 같은 규칙이다.
#   - 물리 덩굴·자석 이탈력은 착수 금지 항목(§5). 여기서 하는 것은 메쉬뿐이다.
#
# 좌표계: 원점 = **과실 중심**. 축은 world 와 같다(딸기 orient 는 항등).
#   그래서 layout_layer.usd 의 덩굴 translate 가 같은 딸기의 translate 와 **같은 값**이다.
#   딸기를 옮기면 덩굴 translate 도 같은 값으로 따라 옮기면 된다.
#
# 형상 (로컬, mm):
#   보드 부착   (0, {Y_BOARD*1000:+.1f}, {Z_BOARD*1000:+.1f})  <- world y=812.0. 보드면(810.0) 뒤 2mm 로 끝단 캡을 가린다.
#   수직 시작   (0, {B[1]*1000:+.1f}, {Z_BEND*1000:+.1f})   여기부터 아래는 (0,0) 수직
#   덩굴 끝     (0, 0, {Z_TIP*1000:+.1f})   <- 애셋 메시 줄기(+26.4~+29.5mm)를 지나 꽃받침까지 파묻는다
#   지름 {R_BOARD*2000:.1f} -> {R_TIP*2000:.1f}mm (애셋 메시 줄기 지름 ~4mm 와 맞춘 굵기)
#
#   ★ 끝을 (0,0) 수직으로 두는 이유: 애셋 두 변종의 메시 줄기가 **서로 반대로 기울어 있다**
#     (ripe 끝 (-2.1,+0.5), unripe 끝 (+0.9,-1.3)mm). 한쪽에 맞추면 다른 쪽에서 어긋난다.
#     과실 중심축이 두 변종의 중간이라, 여기서 겹침이 ripe 1.4mm / unripe 2.0mm 로 둘 다 물린다.
#
# ★ 수직 구간을 z +{Z_TIP*1000:.0f} ~ +{Z_BEND*1000:.0f}mm 로 잡은 이유 — 플래너가 겨냥하는 두 점을 다 담기 위해서다.
#     파지 목표 = 과실 중심 +35mm (pick_target_z_bias_m). 조우가 여기서 닫힌다.
#     하강 시작 = 파지 목표 +30mm = +65mm (CRANE_Z_OFFSET_M, open-stem descent).
#   두 점 모두 덩굴 **중심선 위**에 정확히 놓인다(실측 0.00mm). 열린 조우가 덩굴을
#   따라 30mm 내려와 덩굴을 물고, 그 뒤 과실이 딸려 나간다 — 이것이 A-1 클로즈업이다.
#   조우 닫힘 간격은 0.7mm 라 덩굴(파지점 지름 3.2mm)과 면당 1.3mm 겹친다. 콜라이더가 없으니
#   물리 영향은 없고, 화면에서는 "무는" 것으로 보인다.
#
# 색: 애셋 텍스처(frut333.png)의 **줄기 구간 UV 평균**을 그대로 썼다.
#   sRGB (102, 119, 31) -> linear ({0.1338}, {0.1841}, {0.0136}). 메시 줄기와 이어 붙어도 색이 튀지 않는다.
#
# ⚠️ 딸기 배치를 바꾸면 덩굴-이웃과실 간격을 다시 본다. 현재 최소는
#    vine_ripe_06 vs ripe_04 의 **+1.4mm** 다 (관통은 아니지만 여유가 얇다).
#    검증 스크립트: 이 파일을 만든 생성기와 같은 기하식을 쓴다.

def Xform "vine"
{{
    def Mesh "stem" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    )
    {{
        float3[] extent = [{vec3(lo)}, {vec3(hi)}]
        rel material:binding = </vine/Looks/Vine_Mat> (
            bindMaterialAs = "weakerThanDescendants"
        )
        color3f[] primvars:displayColor = [(0.1338, 0.1841, 0.0136)]
        uniform token subdivisionScheme = "none"
'''

body = []
body.append("        int[] faceVertexCounts = [" + ", ".join(str(c) for c in counts) + "]")
body.append("        int[] faceVertexIndices = [" + ", ".join(str(i) for i in idx) + "]")
body.append("        point3f[] points = [" + ", ".join(vec3(p) for p in v) + "]")
body.append('        normal3f[] normals = [' + ", ".join(vec3(n) for n in nrm) + '] (\n'
            '            interpolation = "vertex"\n'
            '        )')

footer = '''    }

    def Scope "Looks"
    {
        def Material "Vine_Mat"
        {
            token outputs:mdl:displacement.connect = </vine/Looks/Vine_Mat/Shader.outputs:out>
            token outputs:mdl:surface.connect = </vine/Looks/Vine_Mat/Shader.outputs:out>
            token outputs:mdl:volume.connect = </vine/Looks/Vine_Mat/Shader.outputs:out>

            def Shader "Shader"
            {
                uniform token info:implementationSource = "sourceAsset"
                uniform asset info:mdl:sourceAsset = @OmniPBR.mdl@
                uniform token info:mdl:sourceAsset:subIdentifier = "OmniPBR"
                color3f inputs:diffuse_color_constant = (0.1338, 0.1841, 0.0136)
                float inputs:reflection_roughness_constant = 0.72
                float inputs:specular_level = 0.25
                token outputs:out (
                    renderType = "material"
                )
            }
        }
    }
}
'''
out = sys.argv[1] if len(sys.argv) > 1 else "strawberry_harvest/assets/props/vine.usd"
open(out, "w").write(header + "\n".join(body) + "\n" + footer)
print("wrote %s  (%d points, %d faces)" % (out, len(v), len(counts)))
