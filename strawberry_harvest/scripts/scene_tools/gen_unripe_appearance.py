#!/usr/bin/env python3
"""scenes/layers/appearance_layer.usd 생성기 — 안 익은(비대상) 과실 외형 오버라이드 (2026-09-17).

    python3 strawberry_harvest/scripts/scene_tools/gen_unripe_appearance.py

시스템 python3 + pxr 에서 돈다 (Isaac 불필요). PIL 이 있으면 익은 과실 텍스처 평균과의 색차도 로그에 찍는다(없으면 생략).

왜 필요한가: 안 익은 과실 텍스처(애셋 그대로)는 익은 과실과 같은 난색 계열이다 — 파일 전체 sRGB 평균 익은 frut333 (166,96,47),
안 익은 frut_2333 (146,122,48). 슬라이드 축소 크기에서도 둘이 갈리게 하려는 것이 사용자 요구다(2026-09-17).
안 익은 과실의 diffuseColor 를 연한 녹백색 상수로 덮는다 — 익은 빨강과는 멀고, 흰색 계열(병든 딸기 색으로 읽힌다)과도 겹치지 않는 색.

어디에 쓰나 (전부 표시 계층 — 플래너·상태 머신·ROS 인터페이스와 무관):
  * 애셋(assets/strawberry/** — strawberry_*.usd crate, textures)은 건드리지 않는다. 덮는 opinion 은 이 스크립트가 통째로 만드는
    scenes/layers/appearance_layer.usd 에만 있다. main_scene.usd 는 그 레이어를 subLayers 맨 앞에 한 줄로 붙이기만 한다
    (이 스크립트는 main_scene 을 고치지도 저장하지도 않는다 — 없으면 안내 후 종료).
  * layout_layer·physics_layer 에 넣지 않은 이유: gen_random_layout.py 가 그 두 파일을 텍스트로 다시 쓴다(정규식 치환).
  * 로컬 레이어 스택의 opinion 은 어느 레이어든 애셋 reference 보다 강하다 — 맨 앞 자리는 다른 로컬 레이어와의 순서만 정한다.

무엇을 덮나:
  * 과실 = /World 직계 자식, Xformable, 이름(소문자)에 'strawberry' 포함·'robot' 미포함 — Isaac 브릿지
    (isaac_sim_script_editor_bridge.py 의 _ripe_berry_roots)와 같은 규칙. 그중 이름에 'unripe' 가 있으면 비대상(덮는다),
    없으면 타겟(브릿지가 발행하는 익은 과실, 그대로 둔다). 개수·이름·머티리얼·셰이더 이름은 코드에 없다 — 매번 씬에서 찾는다.
  * 과실 하위 메시(와 materialBind 서브셋)에 바인딩된 머티리얼의 surface 셰이더가 UsdPreviewSurface 이면 그 inputs:diffuseColor.
  * 값만 Set 하면 애셋의 텍스처 연결(Image_Texture.outputs:rgb)이 이긴다. 무인자 DisconnectSource() 가 명시적 빈 연결
    (`inputs:diffuseColor.connect = None`)을 써야 상수 값이 이긴다 (ClearSources() 로는 약한 레이어의 연결이 안 막힌다).
  * 노말·러프니스 등 다른 입력은 건드리지 않는다 — 요철·광택은 애셋 그대로.

실패 규칙: 비대상 과실 하나라도 대상 입력을 못 찾으면 과실마다 경고 한 줄 뒤 종료 코드 1 (부분 적용 금지, 레이어를 저장하지 않는다).
저장 뒤 씬을 새로 열어 검증하고, 어긋나면 종료 코드 1.
생성물은 씬 재로드 뒤 보인다. 색을 바꾸려면 UNRIPE_SRGB 만 고치고 이 스크립트를 다시 돌린다.
"""
import math
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
SCENE = os.path.join(REPO, "strawberry_harvest/scenes/main_scene.usd")
LAYER = os.path.join(REPO, "strawberry_harvest/scenes/layers/appearance_layer.usd")
WORLD_PATH = "/World"

# ---- 색 ---------------------------------------------------------------------------------
# [2026-09-17] 연한 녹백색 sRGB #B4D69A. UsdPreviewSurface diffuseColor 는 linear 라 sRGB 디코드한 값을 쓴다
#   (→ linear 0.4564, 0.6724, 0.3231 — 아래 srgb_to_linear 가 계산한다. 상수는 hex 하나만).
# 선택 근거 (CIELAB, D65, ΔE76 — 실행 로그에 같은 수치를 다시 계산해 찍는다):
#   * C* 34 — 흰색 계열로 보이지 않는 채도 (L* 82 로 밝지만 녹색이 분명하다).
#   * 순백 #FFFFFF 과 ΔE 39, 회백색 (228,228,222) 과 ΔE 32 — 흰색 계열(병든 딸기 색)과 겹치지 않는다.
#   * 익은 과실 텍스처(sRGB 전체 평균)와 ΔE 59 — 슬라이드 축소 크기에서도 익은 과실과 갈린다.
# 더 연하게/진하게 = 이 hex 를 바꾸고 다시 돌린다 (위 수치는 로그에서 다시 확인).
UNRIPE_SRGB = 0xB4D69A

# 색차 로그용 비교 기준 (덮는 값이 아니다)
REF_WHITE_SRGB = (255, 255, 255)
REF_GRAY_WHITE_SRGB = (228, 228, 222)

LAYER_DOC = ("외형 오버라이드 레이어 — 안 익은(비대상) 과실의 diffuseColor 를 연한 녹백색 상수로 덮어 익은 과실과 구분한다. "
             "애셋(assets/strawberry/**)은 불변, 텍스처 연결은 이 레이어의 빈 연결로만 막는다 (시각 전용, 플래너 입출력 불변). "
             "생성물 — 손으로 고치지 말고 scripts/scene_tools/gen_unripe_appearance.py 를 다시 돌린다")

TAG = "[appearance]"
EPS = 1e-6


# ---- 색 계산 ------------------------------------------------------------------------------
def hex_to_srgb8(hexv):
    return tuple((hexv >> shift) & 255 for shift in (16, 8, 0))


def srgb_to_linear(rgb8):
    out = []
    for v in rgb8:
        c = v / 255.0
        out.append(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4)
    return tuple(out)


def srgb8_to_lab(rgb8):
    """sRGB 8bit → CIELAB (D65)."""
    r, g, b = srgb_to_linear(rgb8)
    x = 0.4124564 * r + 0.3575761 * g + 0.1804375 * b
    y = 0.2126729 * r + 0.7151522 * g + 0.0721750 * b
    z = 0.0193339 * r + 0.1191920 * g + 0.9503041 * b

    def f(t):
        return t ** (1.0 / 3.0) if t > (6.0 / 29.0) ** 3 else t / (3.0 * (6.0 / 29.0) ** 2) + 4.0 / 29.0

    fx, fy, fz = f(x / 0.95047), f(y / 1.0), f(z / 1.08883)
    return 116.0 * fy - 16.0, 500.0 * (fx - fy), 200.0 * (fy - fz)


def delta_e76(lab1, lab2):
    return math.sqrt(sum((p - q) ** 2 for p, q in zip(lab1, lab2)))


def texture_mean_srgb8(path):
    """텍스처 파일 sRGB 전체 평균. PIL 이 없거나 못 읽으면 None."""
    try:
        from PIL import Image, ImageStat
        with Image.open(path) as im:
            return tuple(ImageStat.Stat(im.convert("RGB")).mean)
    except Exception:                                              # noqa: BLE001 — 로그용 부가 수치
        return None


# ---- 씬 탐색 ------------------------------------------------------------------------------
def classify_fruits(stage):
    """(타겟, 비대상) prim 목록. 규칙은 브릿지 _ripe_berry_roots 와 같다 (/World 직계 자식만)."""
    world = stage.GetPrimAtPath(WORLD_PATH)
    if not world or not world.IsValid():
        return None, None
    targets, non_targets = [], []
    for prim in world.GetChildren():
        name = prim.GetName().lower()
        if "strawberry" not in name or "robot" in name or not prim.IsA(UsdGeom.Xformable):
            continue
        (non_targets if "unripe" in name else targets).append(prim)
    return targets, non_targets


def check_ripeness_variant(prim, unripe):
    """variantSet ripeness 선택이 이름과 어긋나면 경고 한 줄 (판정은 이름 기준 그대로)."""
    vsets = prim.GetVariantSets()
    if not vsets.HasVariantSet("ripeness"):
        return
    sel = vsets.GetVariantSet("ripeness").GetVariantSelection()
    if sel and (sel == "unripe") != unripe:
        print("%s 경고: %s 이름은 %s 인데 variant ripeness = %r" % (TAG, prim.GetPath(), "unripe" if unripe else "ripe", sel))


def surface_shaders(fruit):
    """과실 하위 메시에 바인딩된 surface 셰이더 {경로: UsdShade.Shader}. 문제가 있으면 (None, 이유).

    RTX 가 보는 바인딩(full → allPurpose)과 기본(allPurpose) 둘 다 본다. MDL surface 출력이 따로 있으면
    RTX 가 UsdPreviewSurface 를 안 쓰므로 여기서 덮어도 화면에 안 나온다 — 실패로 본다.
    """
    shaders = {}
    n_mesh = 0
    for prim in Usd.PrimRange(fruit):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        n_mesh += 1
        bind_prims = [prim] + [s.GetPrim() for s in UsdShade.MaterialBindingAPI(prim).GetMaterialBindSubsets()]
        materials = {}
        for bp in bind_prims:
            for purpose in (UsdShade.Tokens.allPurpose, UsdShade.Tokens.full):
                material, _rel = UsdShade.MaterialBindingAPI(bp).ComputeBoundMaterial(purpose)
                if material:
                    materials[material.GetPath()] = material
        if not materials:
            return None, "메시 %s 에 바인딩된 머티리얼이 없다" % prim.GetPath()
        for mpath, material in materials.items():
            shader = material.ComputeSurfaceSource()[0]
            if not shader:
                return None, "머티리얼 %s surface 출력에 연결된 셰이더가 없다" % mpath
            if shader.GetShaderId() != "UsdPreviewSurface":
                return None, "머티리얼 %s surface 셰이더 %s 가 UsdPreviewSurface 가 아니다 (id %r)" % (
                    mpath, shader.GetPath(), shader.GetShaderId())
            mdl = material.ComputeSurfaceSource("mdl")[0]
            if mdl and mdl.GetPath() != shader.GetPath():
                return None, "머티리얼 %s 에 MDL surface(%s)가 따로 있다 — RTX 에서 오버라이드가 안 보인다" % (mpath, mdl.GetPath())
            if not shader.GetInput("diffuseColor"):
                return None, "셰이더 %s 에 inputs:diffuseColor 가 없다" % shader.GetPath()
            shaders[shader.GetPath()] = shader
    if n_mesh == 0:
        return None, "하위 메시가 없다"
    return shaders, None


def input_state(shader, skip=("diffuseColor",)):
    """셰이더 입력 스냅샷 {이름: (연결 원천 목록, 값)} — 오버라이드가 다른 입력을 안 건드렸는지 비교용."""
    state = {}
    for inp in shader.GetInputs():
        name = inp.GetBaseName()
        if name in skip:
            continue
        srcs = tuple(sorted((str(s.source.GetPath()), str(s.sourceName)) for s in inp.GetConnectedSources()[0]))
        state[name] = (srcs, inp.Get())
    return state


def texture_output_of(inp):
    """입력 값을 실제로 내는 것이 UsdUVTexture 출력이면 그 셰이더, 아니면 None."""
    attrs = inp.GetValueProducingAttributes()
    if len(attrs) != 1 or not UsdShade.Output.IsOutput(attrs[0]):
        return None
    src = UsdShade.Shader(attrs[0].GetPrim())
    return src if src and src.GetShaderId() == "UsdUVTexture" else None


def texture_file_of(inp):
    tex = texture_output_of(inp)
    if not tex:
        return None
    file_inp = tex.GetInput("file")
    val = file_inp.Get() if file_inp else None
    return (val.resolvedPath or None) if val is not None else None


# ---- 본문 ---------------------------------------------------------------------------------
def ensure_layer_file():
    if os.path.exists(LAYER):
        return
    layer = Sdf.Layer.CreateNew(LAYER, args={"format": "usda"})
    layer.documentation = LAYER_DOC
    if not layer.Save():
        sys.exit("%s 빈 레이어 생성 실패: %s" % (TAG, LAYER))
    print("%s 빈 레이어 생성: %s" % (TAG, os.path.relpath(LAYER, REPO)))


def open_stage_with_layer():
    stage = Usd.Stage.Open(SCENE)
    if not stage:
        sys.exit("%s 씬을 못 열었다: %s" % (TAG, SCENE))
    layer = Sdf.Layer.FindOrOpen(LAYER)
    if not layer:
        sys.exit("%s 오류: %s 를 USD 레이어로 못 열었다 — 파일을 지우고 다시 돌리면 빈 레이어부터 만든다" % (TAG, LAYER))
    stack = stage.GetLayerStack(includeSessionLayers=False)
    real = os.path.realpath(LAYER)
    if not any(l.realPath and os.path.realpath(l.realPath) == real for l in stack):
        print("%s 오류: main_scene.usd 의 subLayers 에 ./layers/appearance_layer.usd 가 없다." % TAG, file=sys.stderr)
        print("%s   main_scene.usd 의 subLayers 맨 앞(가장 강한 자리)에 @./layers/appearance_layer.usd@ 한 줄을 추가한 뒤 다시 돌린다."
              " (이 스크립트는 main_scene.usd 를 고치지 않는다)" % TAG, file=sys.stderr)
        sys.exit(1)
    root = stage.GetRootLayer()
    first = root.subLayerPaths[0] if len(root.subLayerPaths) else None
    first_abs = Sdf.ComputeAssetPathRelativeToLayer(root, first) if first else None
    if not first_abs or os.path.realpath(first_abs) != real:
        print("%s 경고: appearance_layer 가 subLayers 맨 앞이 아니다 (맨 앞 = %s) — 다른 로컬 레이어의 같은 입력 opinion 이 이길 수 있다"
              % (TAG, first))
    return stage, layer


def collect(stage, targets, non_targets):
    """비대상 셰이더 목록과 타겟 텍스처 파일. 비대상 하나라도 못 찾으면 과실마다 경고 한 줄 뒤 종료 코드 1."""
    jobs, failed = [], 0
    for fruit in non_targets:
        shaders, why = surface_shaders(fruit)
        if shaders is None:
            print("%s 경고: 비대상 %s — %s" % (TAG, fruit.GetPath(), why), file=sys.stderr)
            failed += 1
            continue
        for spath in sorted(shaders):
            jobs.append((fruit, shaders[spath]))
    if failed:
        print("%s 오류: 비대상 과실 %d개에서 대상 입력을 못 찾았다 — 부분 적용 금지, 레이어를 저장하지 않는다" % (TAG, failed),
              file=sys.stderr)
        sys.exit(1)
    ripe_files = set()
    for fruit in targets:
        shaders, _why = surface_shaders(fruit)
        for shader in (shaders or {}).values():
            path = texture_file_of(shader.GetInput("diffuseColor"))
            if path:
                ripe_files.add(path)
    return jobs, sorted(ripe_files)


def log_color_metrics(rgb8, ripe_files):
    lab = srgb8_to_lab(rgb8)
    chroma = math.hypot(lab[1], lab[2])
    parts = ["L* %.1f C* %.1f" % (lab[0], chroma),
             "ΔE76 순백 %.1f" % delta_e76(lab, srgb8_to_lab(REF_WHITE_SRGB)),
             "회백색%s %.1f" % (str(REF_GRAY_WHITE_SRGB).replace(" ", ""), delta_e76(lab, srgb8_to_lab(REF_GRAY_WHITE_SRGB)))]
    for path in ripe_files:
        mean = texture_mean_srgb8(path)
        if mean is None:
            parts.append("익은 텍스처 %s 평균 생략(PIL 없음/읽기 실패)" % os.path.basename(path))
        else:
            parts.append("익은 텍스처 %s sRGB 평균 %.1f" % (os.path.basename(path), delta_e76(lab, srgb8_to_lab(mean))))
    if not ripe_files:
        parts.append("익은 텍스처 없음(비교 생략)")
    print("%s 색 #%06X: %s" % (TAG, UNRIPE_SRGB, ", ".join(parts)))


def author(stage, layer, jobs, lin):
    """레이어를 비우고(문서 재설정) 비대상 diffuseColor 마다 빈 연결 + 상수."""
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    up = UsdGeom.GetStageUpAxis(stage)
    layer.Clear()
    layer.documentation = LAYER_DOC
    layer.pseudoRoot.SetInfo(UsdGeom.Tokens.metersPerUnit, mpu)
    layer.pseudoRoot.SetInfo(UsdGeom.Tokens.upAxis, up)
    stage.SetEditTarget(Usd.EditTarget(layer))
    value = Gf.Vec3f(*lin)
    for _fruit, shader in jobs:
        inp = shader.GetInput("diffuseColor")
        if not inp.DisconnectSource() or not inp.Set(value):
            sys.exit("%s 오류: %s diffuseColor 쓰기 실패 — 레이어를 저장하지 않는다" % (TAG, shader.GetPath()))
        print("%s %s diffuseColor <- #%06X linear(%.4f, %.4f, %.4f) (texture connection blocked)"
              % (TAG, shader.GetPath(), UNRIPE_SRGB, lin[0], lin[1], lin[2]))


def verify(expect_targets, expect_non_targets, jobs_before, lin):
    """저장된 파일로 새로 열어 확인한다. 실패 줄 목록을 돌려준다."""
    errors = []
    # (1) 레이어 파일 자체 — usda 텍스트, over 만, 속성 스펙은 diffuseColor 뿐, 수 = 덮은 셰이더 수
    with open(LAYER, "rb") as f:
        if not f.read(9).startswith(b"#usda 1.0"):
            errors.append("레이어 파일이 usda 텍스트가 아니다")
    disk = Sdf.Layer.OpenAsAnonymous(LAYER)
    n_attr, stray = 0, []

    def walk(path):
        nonlocal n_attr
        if path == Sdf.Path.absoluteRootPath:
            return
        spec = disk.GetObjectAtPath(path)
        if isinstance(spec, Sdf.PrimSpec):
            if spec.specifier != Sdf.SpecifierOver:
                stray.append("%s specifier %s" % (path, spec.specifier))
        elif isinstance(spec, Sdf.AttributeSpec):
            if spec.name == "inputs:diffuseColor":
                n_attr += 1
            else:
                stray.append(str(path))
        elif isinstance(spec, Sdf.RelationshipSpec):
            stray.append(str(path))

    disk.Traverse(Sdf.Path.absoluteRootPath, walk)
    if stray:
        errors.append("레이어에 diffuseColor 밖의 스펙: %s" % ", ".join(stray[:5]))
    if n_attr != len(jobs_before):
        errors.append("레이어의 diffuseColor 스펙 %d개 != 덮은 셰이더 %d개" % (n_attr, len(jobs_before)))
    if not disk.documentation.startswith(LAYER_DOC[:10]):
        errors.append("레이어 doc 이 없다")

    # (2) 합성 결과 — 새 스테이지 (이전 핸들을 놓았어도 레지스트리에 남아 있으면 디스크에서 다시 읽는다)
    cached = Sdf.Layer.Find(LAYER)
    if cached:
        cached.Reload(force=True)
    stage = Usd.Stage.Open(SCENE)
    targets, non_targets = classify_fruits(stage)
    if [p.GetPath() for p in targets] != expect_targets or [p.GetPath() for p in non_targets] != expect_non_targets:
        errors.append("재오픈 후 과실 분류가 달라졌다")
    value = Gf.Vec3f(*lin)
    n_normal = 0
    for shader_path, before in jobs_before:
        shader = UsdShade.Shader(stage.GetPrimAtPath(shader_path))
        inp = shader.GetInput("diffuseColor") if shader else None
        if not inp:
            errors.append("%s diffuseColor 를 못 찾았다" % shader_path)
            continue
        attrs = inp.GetValueProducingAttributes()
        if len(attrs) != 1 or attrs[0].GetPath() != inp.GetAttr().GetPath():
            errors.append("%s diffuseColor 값 생산자가 입력 자신이 아니다: %s" % (shader_path, [str(a.GetPath()) for a in attrs]))
        got = inp.Get()
        if got is None or any(abs(got[i] - value[i]) > EPS for i in range(3)):
            errors.append("%s diffuseColor 값 %s != %s" % (shader_path, got, tuple(value)))
        after = input_state(shader)
        if after != before:
            changed = sorted(k for k in set(before) | set(after) if before.get(k) != after.get(k))
            errors.append("%s 다른 입력이 바뀌었다: %s" % (shader_path, changed))
        normal = shader.GetInput("normal")
        if before.get("normal", ((), None))[0]:
            if normal and texture_output_of(normal):
                n_normal += 1
            else:
                errors.append("%s normal 텍스처 연결이 끊겼다" % shader_path)
    n_target_shaders = 0
    for fruit in targets:
        shaders, why = surface_shaders(fruit)
        if shaders is None:
            errors.append("타겟 %s — %s" % (fruit.GetPath(), why))
            continue
        for spath, shader in sorted(shaders.items()):
            n_target_shaders += 1
            if not texture_output_of(shader.GetInput("diffuseColor")):
                errors.append("타겟 %s diffuseColor 가 텍스처 연결이 아니다" % spath)
    return errors, n_normal, n_target_shaders


def main():
    try:
        sys.stdout.reconfigure(line_buffering=True)   # 파이프로 받아도 경고(stderr)와 로그 줄 순서가 섞이지 않게
    except (AttributeError, ValueError):
        pass
    lin = srgb_to_linear(hex_to_srgb8(UNRIPE_SRGB))
    ensure_layer_file()
    stage, layer = open_stage_with_layer()
    # 이전 생성물을 먼저 비운다 — 탐색·스냅샷이 애셋 원래 합성(텍스처 연결)을 보게. 실패하면 저장하지 않으니 디스크는 그대로다.
    layer.Clear()
    targets, non_targets = classify_fruits(stage)
    if targets is None:
        sys.exit("%s 오류: %s 가 없다" % (TAG, WORLD_PATH))
    if not targets and not non_targets:
        sys.exit("%s 오류: /World 직계 자식에서 과실을 못 찾았다 — 씬 또는 판정 규칙 확인" % TAG)
    for prim in targets:
        check_ripeness_variant(prim, unripe=False)
    for prim in non_targets:
        check_ripeness_variant(prim, unripe=True)
    print("%s 과실: 타겟 %d개 (텍스처 유지), 비대상 %d개 (덮음: %s)" % (
        TAG, len(targets), len(non_targets), ", ".join(p.GetName() for p in non_targets) or "없음"))
    if not non_targets:
        print("%s 경고: 비대상 과실이 없다 — 레이어는 비운 채로 저장한다" % TAG)

    jobs, ripe_files = collect(stage, targets, non_targets)
    log_color_metrics(hex_to_srgb8(UNRIPE_SRGB), ripe_files)
    jobs_before = [(shader.GetPath(), input_state(shader)) for _fruit, shader in jobs]

    author(stage, layer, jobs, lin)
    if not layer.Save():
        sys.exit("%s 오류: %s 저장 실패" % (TAG, LAYER))
    expect_targets = [p.GetPath() for p in targets]
    expect_non_targets = [p.GetPath() for p in non_targets]
    del jobs, targets, non_targets, layer, stage      # 저장본만 남기고 핸들을 놓는다 (검증은 새로 연다)

    errors, n_normal, n_target_shaders = verify(expect_targets, expect_non_targets, jobs_before, lin)
    if errors:
        for e in errors:
            print("%s 검증 실패: %s" % (TAG, e), file=sys.stderr)
        sys.exit(1)
    print("%s 검증 OK (저장본 재오픈): 비대상 셰이더 %d개 diffuseColor = 상수(값 생산 = 입력 자신), 노말 텍스처 연결 %d개 유지, "
          "다른 입력 불변 / 타겟 셰이더 %d개 diffuseColor 텍스처 연결 유지 / 레이어 스펙 = over + diffuseColor %d개"
          % (TAG, len(jobs_before), n_normal, n_target_shaders, len(jobs_before)))
    print("%s 요약: 비대상 과실 %d개 · 셰이더 %d개 diffuseColor <- #%06X, 타겟 과실 %d개 불변 -> %s (씬 재로드 뒤 보인다)"
          % (TAG, len(expect_non_targets), len(jobs_before), UNRIPE_SRGB, len(expect_targets), os.path.relpath(LAYER, REPO)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
