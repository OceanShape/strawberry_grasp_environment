# Strawberry Harvest — Isaac Sim USD 프로젝트

구조 원칙, 각 파일의 역할, 작업 규칙의 자세한 내용은 [`../docs/usd_structure.md`](../docs/usd_structure.md)를 참고하세요.

## 시작하기

Isaac Sim에서 **`scenes/main_scene.usd`** 를 여세요.

## 빠른 참조 — 무엇을 바꾸려면 어느 파일?

| 하고 싶은 것 | 수정할 파일 |
|---|---|
| 딸기 위치 변경 / 랜덤화 | `scenes/layers/layout_layer.usd` |
| 줄기 분리 강도 (breakForce) | `scenes/layers/physics_layer.usd` |
| 딸기 물리 (질량, 마찰) | `assets/strawberry/strawberry_physics.usd` |
| 조명 변경 | `scenes/layers/lighting_layer.usd` |
| 로봇 커스터마이징 | `assets/robot/robot_assembly.usd` |
| 물리 없이 비주얼만 확인 | Layer 창에서 `physics_layer.usd` mute |
