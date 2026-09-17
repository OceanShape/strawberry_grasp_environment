"""
scene_fruit.py — 씬에 실제로 있는 딸기를 세어 타겟/비대상으로 나눈다. pxr 만 쓴다 (omni·rclpy import 금지).

HUD 의 '타겟 N / 비대상 M' 과 결과 바 칸 수의 출처다 (2026-09-17). 개수는 어디에도 적지 않고 매번 씬에서 센다.
Kit 밖(시스템 python3 + pxr)에서도 main_scene.usd 로 그대로 돌려 볼 수 있다.

판정 규칙은 Isaac 브릿지(isaac_sim_script_editor_bridge.py 의 _ripe_berry_roots)와 **같다** — 파이프라인이
실제로 타겟으로 발행하는 과실과 화면의 N 이 같아야 하기 때문이다.

  과실      /World 직계 자식, Xformable, 이름(소문자)에 'strawberry' 포함, 'robot' 미포함
  타겟      그중 이름에 'unripe' 가 없는 것 (브릿지가 /isaac_sim/strawberries 로 발행하는 익은 과실)
  비대상    그중 이름에 'unripe' 가 있는 것 (브릿지가 발행하지 않는다 — 시각 장애물)

이름 판정이 파이프라인 기준이고, variantSet 'ripeness' 선택은 교차 확인에만 쓴다(어긋나면 mismatch 로 돌려준다).
수확돼 트레이로 옮겨졌거나 떨어진 과실도 prim 은 씬에 남으므로 런 중에도 N 은 변하지 않는다.
"""
from __future__ import annotations

from typing import Dict, List

from pxr import Usd, UsdGeom, UsdShade

WORLD_PATH = "/World"


def _is_fruit_name(name: str) -> bool:
    n = name.lower()
    return "strawberry" in n and "robot" not in n


def classify(stage) -> Dict[str, List[str]]:
    """{"targets": [경로], "non_targets": [경로], "mismatch": [경로]} — 경로는 이름 순."""
    out = {"targets": [], "non_targets": [], "mismatch": []}
    if stage is None:
        return out
    world = stage.GetPrimAtPath(WORLD_PATH)
    if not world or not world.IsValid():
        return out
    for prim in world.GetChildren():
        if not _is_fruit_name(prim.GetName()) or not prim.IsA(UsdGeom.Xformable):
            continue
        path = prim.GetPath().pathString
        unripe = "unripe" in prim.GetName().lower()
        out["non_targets" if unripe else "targets"].append(path)
        vset = prim.GetVariantSets()
        if vset.HasVariantSet("ripeness"):
            sel = vset.GetVariantSet("ripeness").GetVariantSelection()
            if sel and (sel == "unripe") != unripe:
                out["mismatch"].append(path)
    for key in out:
        out[key].sort()
    return out


def diffuse_is_constant(stage, fruit_path: str) -> bool:
    """과실 메시에 바인딩된 표면 셰이더의 diffuseColor 가 텍스처 연결 없이 상수로 정해지면 True.

    안 익은 과실 색 오버라이드(layers/appearance_layer.usd)가 합성에 들어왔는지를 HUD 가 로그 한 줄로
    확인하는 데만 쓴다 — 씬을 다시 열지 않으면 오버라이드가 안 보이는 일이 잦아서.
    """
    try:
        root = stage.GetPrimAtPath(fruit_path)
        for prim in Usd.PrimRange(root):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            material, _rel = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
            if not material:
                continue
            shader = material.ComputeSurfaceSource()[0]
            if not shader:
                continue
            inp = shader.GetInput("diffuseColor")
            if not inp:
                continue
            attrs = inp.GetValueProducingAttributes()
            return bool(attrs) and attrs[0].GetPath() == inp.GetAttr().GetPath()
    except Exception:                                              # noqa: BLE001
        return False
    return False
