# Isaac Sim USD 프로젝트 구조 및 작업 규칙

`strawberry_harvest/` 폴더의 구성 원칙, 현재 디렉토리 구조, 각 파일의 역할, 작업할 때 어느 파일을 수정해야 하는지를 설명합니다.

---

## 1. 핵심 원칙

1. **애셋과 씬을 분리한다.** 재사용 가능한 지오메트리(로봇, 딸기, 소품)는 개별 USD 애셋 파일로 만들고, 씬 파일은 이들을 `reference` 또는 `payload`로 조립만 한다. 씬 파일에 지오메트리를 직접 쓰지 않는다.
2. **물리는 별도 레이어(sublayer)로 분리한다.** 비주얼과 물리 속성을 분리하면 물리 파라미터 튜닝 시 diff 관리가 쉽고, 물리 없이 비주얼만 확인하는 것도 가능하다.
3. **원본 애셋은 직접 수정하지 않는다.** URDF 임포트 결과물(`doosan_e0509_rh_p12_rn/`)은 수정 금지. 커스터마이징은 조립본(`robot_assembly.usd`)의 오버라이드로 처리한다.
4. **자주 안 바뀌는 것은 payload, 항상 필요한 것은 reference.** 정적 환경(`lab_environment.usd`)처럼 무거운 지오메트리는 payload로 필요할 때만 로드하여 반복 테스트 속도를 확보한다.

---

## 2. 디렉토리 구조

```
strawberry_harvest/
├── assets/
│   ├── robot/
│   │   ├── doosan_e0509_rh_p12_rn/        # URDF 임포트 원본 결과물 — 수정 금지
│   │   │   ├── robot.usd                  #   variantSet: Physics / Sensor / Robot
│   │   │   └── configuration/             #   base(지오메트리) · physics(joint/drive) · robot(Isaac API) 레이어
│   │   └── robot_assembly.usd             # 로봇 + D455 카메라 조립본 — 씬에서는 이것만 참조
│   ├── strawberry/
│   │   ├── strawberry.usd                 # 애셋 인터페이스 — variantSet "ripeness" (ripe/unripe)
│   │   ├── strawberry_physics.usd         # 물리 레이어: 질량 0.02 kg, convexHull 콜라이더 (튜닝은 여기서)
│   │   ├── strawberry_ripe.usd            # 지오메트리 + 머티리얼 (prim 구조: /strawberry/geo/fruit/mesh)
│   │   ├── strawberry_unripe.usd          # 〃 (내장 DomeLight 제거됨 — 조명은 씬 레이어로 이동)
│   │   └── textures/
│   ├── props/
│   │   ├── table.usd                      # 테이블 (defaultPrim=table)
│   │   ├── whiteboard.usd                 # 보드(격자 텍스처) + highlight(분면 오버레이 4장) + attach_points
│   │   └── textures/
│   └── omniverse_imports/                 # Nucleus/클라우드 애셋 로컬 사본 — 직접 네트워크 참조 금지
│       └── Isaac/
│           ├── Sensors/Intel/RealSense/rsd455.usd
│           └── Materials/Base/...         # rsd455가 상대경로로 참조하는 MDL + 텍스처 미러
├── scenes/
│   ├── main_scene.usd                     # ★ 최상위 씬 — 조립 전용, 직접 수정 최소화
│   ├── lab_environment.usd                # 정적 환경 조립본 (테이블+보드) — payload로 로드됨
│   ├── layers/
│   │   ├── layout_layer.usd               # 배치: 딸기 위치, 로봇 베이스, 초기 관절 포즈 (domain randomization 대상)
│   │   ├── physics_layer.usd              # PhysicsScene, 정적 콜라이더, 딸기 줄기(stem) fixed joint
│   │   └── lighting_layer.usd             # 무텍스처 DomeLight(채움) + 상단 RectLight 1개(주광)
│   └── textures/studio.hdr
├── configs/
│   ├── calibration_eye_in_hand.npz        # eye-in-hand 캘리브레이션 원본 데이터
│   └── camera_calibration_to_isaac.py     # OpenCV → Isaac Sim 좌표 변환 스크립트
└── scripts/                               # Isaac Sim Script Editor에서 실행하는 스크립트
    ├── isaac_sim_script_editor_bridge.py  # 딸기 좌표 발행 + /joint_command 수신 → 로봇 구동
    │                                       #   ⚠️ ASCII 전용 (Kit 이 한글을 '?' 로 찍는다)
    ├── isaac_sim_viewport_display.py       # 뷰포트 표시: 상태 HUD + 보드 분면 하이라이트 + 손목 카메라 창 (ASCII 전용)
    └── self_collision_logger_script.py    # 자기 충돌 감지 → log/collision_*.log 기록
```

---

## 3. 각 구성 요소 상세

### 로봇 (`assets/robot/`)

- `doosan_e0509_rh_p12_rn/robot.usd`는 URDF Importer가 생성한 원본입니다. **절대 직접 수정하지 마세요.** URDF를 재빌드하면 이 폴더 전체가 교체됩니다.
- 로봇 커스터마이징(카메라 부착 위치, 오버라이드 등)은 `robot_assembly.usd`에서만 합니다.
- 그리퍼 종류를 바꿔가며 실험할 가능성이 있다면 `robot_assembly.usd`에 `variantSet(gripper_type)`을 추가하는 것을 고려하세요.

**`robot_assembly.usd`에 현재 적용된 오버라이드 (2026-07-10 기준):**

| 오버라이드 | 대상 prim | 이유 |
|---|---|---|
| `physics:rigidBodyEnabled=False` | `rh_p12_rn_base/rsd455/RSD455` | NVIDIA 순정 D455 애셋은 독립 강체로 설계되어, 그리퍼 링크 밑에 조립하면 "강체 안의 강체"가 되어 articulation 초기화가 실패함 (2026-07-09 수정) |
| joint drive `stiffness=1e5` / `damping=1e4` | `joints/joint_1`~`joint_6` | URDF 임포트 기본 게인(stiffness 54~2648, damping ≈0)이 너무 물러 팔이 출렁이고 덜덜거림 → 산업용 위치 제어 수준으로 상향 (2026-07-10 수정) |
| joint drive `stiffness=1e4` / `damping=1e3` | `joints/rh_*` (그리퍼 4관절) | 같은 이유. 파지력 과다 방지를 위해 팔보다 한 단계 낮게 |
| D455 재질 `enable_ORM_texture=0`, `metallic 1.0`, `roughness 0.6`, `diffuse_tint 0.7` (Aluminum_Anodized·Aluminum_Cast), 렌즈 모듈 `roughness 0.45` (OmniPBR) | `rh_p12_rn_base/rsd455/RSD455/Looks/*/Shader` | 순정 ORM 텍스처가 금속 1.0 / 거칠기 0.14(크롬 거울)라, 09-16 조명 개편 뒤 무텍스처 흰 돔라이트가 그대로 비쳐 센서가 반투명 유리처럼 보였음. 무광 알루미늄으로 오버라이드. 원본 rsd455.usd·MDL 무수정 (2026-09-16 수정) |
| 앞면 렌즈 커버 `Visual/Glass` 를 새 재질 `Looks/lens_cover`(OmniPBR, 검정 광택, 블렌드 불투명도 0.35)로 재바인딩 | `rh_p12_rn_base/rsd455/RSD455/Visual/Glass` | 순정 OmniGlass 는 RTX 실시간 렌더에서 불투명 케이스 뒤에 있어도 깊이 판정 없이 합성돼, 뒤에서 봐도 알약 모양 앞면이 케이스를 뚫고 보였다(Glass 숨기면 정상, thin_walled·doubleSided 는 무효 — 헤드리스 재현). OmniPBR 블렌드는 깊이 판정이 되고 앞에서 렌즈도 비친다 (2026-09-16 수정) |
| D455 부착 오프셋 `translate (-87.1, 7.3, 63.4) mm` · `orient` — **캘리브레이션 값을 Isaac 에서 손으로 보정한 것, 첫 커밋 이후 무변경** | `rh_p12_rn_base/rsd455` | 실기는 그리퍼 정중앙·정면 부착이지만 eye-in-hand 캘리브레이션 결과(`configs/camera_calibration_to_isaac.py`)는 툴축에서 수십 mm 벗어나 있고, 시뮬에서 실기 화면을 재현하려면 손 보정이 더 필요했다. **어떤 노드도 이 프림을 참조하지 않는다** — `fake_vision` 은 씬의 실제 과실 자세를 보드 사각형으로 자를 뿐이고, 좌하단 손목 카메라 창은 렌더 전용이다. 화면이 보드 중앙보다 살짝 왼쪽에서 보는 것은 실기 프레임과 같은 방향이며, 검산은 `src/strawberry_motion/scripts/check_wrist_camera_projection.py`(Isaac·GPU 없이 실행, 결과 `log/m3/offline_checks/wrist_camera_projection_20260916.txt`) (2026-09-16 기록) |

⚠️ URDF를 재빌드해서 `doosan_e0509_rh_p12_rn/`이 교체되어도 이 오버라이드들은 assembly 레이어에 남아 있으므로 유지됩니다. 단, 링크/조인트 이름이 바뀌면 오버라이드가 붕 뜨므로(dangling over) 재확인이 필요합니다.

### 딸기 (`assets/strawberry/`)

- `strawberry.usd`에서 `ripeness` variantSet으로 ripe/unripe를 전환합니다.
- 씬에는 **딸기 12개**(익은 6 / 안 익은 6)가 보드 4등분 서브셀에 배치되어 있습니다
  (nw 2/1, ne 1/2, sw 3/0, se 0/3 — 익은 것/안 익은 것). 배치 의도와 제약은
  [`PORTFOLIO_SPRINT.md`](../PORTFOLIO_SPRINT.md) "씬 구성 메모" 참고.
- 물리 파라미터(질량, 콜라이더 근사, 마찰)는 `strawberry_physics.usd`에서만 수정합니다.
- 줄기 분리(picking) 동작은 `scenes/layers/physics_layer.usd`에 정의된 FixedJoint로 모델링되어 있습니다.
  초기값 `breakForce=2 / breakTorque=1`은 과실 자중의 10.2배뿐이라 로봇이 스치기만 해도 끊겨 날아갔습니다.
  **2026-09-07 `breakForce=20 / breakTorque=5`로 상향**(자중의 101.9배).

### 환경/소품 (`assets/props/`, `scenes/lab_environment.usd`)

- 테이블, 화이트보드는 `lab_environment.usd`에 조립되어 있으며, `main_scene.usd`에서 payload로 불러옵니다.
- `whiteboard.usd`에는 딸기를 매다는 `attach_points` prim이 정의되어 있습니다.
- `whiteboard.usd`의 `highlight/` 는 **활성 칸 하이라이트 오버레이**입니다 — 분면 4장 + 세부 칸 16장(09-12).
  2026-09-16 부터 **시안 발광 테두리**(그 전엔 격자 텍스처에 살구 tint 를 곱한 면 채우기): 분면 띠는 격자선과 같은 20mm 로
  선 위에 앉고, 세부 칸은 안쪽 14mm 띠. `scripts/scene_tools/gen_board_highlight.py` 가 이 스코프를 통째로 생성합니다(손으로 고치지 말 것).
  기본은 전부 `invisible` 이고 `scripts/isaac_sim_viewport_display.py` 가 HUD 의 영역 값에 맞춰
  세션 레이어에서 켭니다 (홈 = 4장 전부, 분면 = 그 한 장, 세부 자세 = 세부 칸 한 장). 시각 전용 — 콜라이더 없음.
  종전의 분면 꼭지점 봉(`cell_markers.usd`)은 시야를 가려 같은 날 제거했습니다.

### Omniverse 임포트 (`assets/omniverse_imports/`)

- Nucleus 경로를 씬에서 직접 참조하지 않고, 로컬 사본을 만들어 관리합니다 (오프라인·협업 안정성 확보).
- `rsd455.usd`가 상대경로로 참조하는 MDL 파일과 텍스처도 함께 미러되어 있습니다.

### 씬 레이어 (`scenes/layers/`)

| 레이어 | 역할 |
|---|---|
| `physics_layer.usd` | PhysicsScene, gravity, solver, 전역 충돌 그룹, 딸기 줄기 joint |
| `lighting_layer.usd` | 무텍스처 DomeLight(채움 300) + RectLight `key_top`(로봇·보드 사이 위 1.9 m, 보드 쪽 20° 기울임). 뷰포트 Lights 는 **Stage Lights** 로 둘 것 — Default Light Rig 를 고르면 이 레이어의 조명이 세션 레이어에서 invisible 처리되고 리그의 DistantLight(햇빛)가 대신 켜진다 |
| `layout_layer.usd` | 딸기 위치, 로봇 베이스 위치, 초기 관절 포즈 — 반복 실험 대상 |

sublayer 순서(strength ordering)에 유의하세요. USD의 `subLayers`는 **strongest-first** —
목록에서 **앞에 있는 레이어가 더 강한 opinion**을 가집니다.
`main_scene.usd`의 순서는 `layout_layer` → `physics_layer` → `lighting_layer`이므로
같은 속성을 두 레이어가 건드리면 `layout_layer`가 이깁니다.
(현재는 세 레이어가 서로 다른 속성만 다루므로 실제 충돌은 없습니다.)

---

## 4. 작업 규칙

| 하고 싶은 것 | 수정할 파일 |
|---|---|
| 딸기 위치 변경 / domain randomization | `scenes/layers/layout_layer.usd` **와** `scenes/layers/physics_layer.usd`(줄기 joint `localPos0`) — ⚠️ 좌표가 두 파일에 중복 보유되므로 **반드시 함께** 고친다. 한쪽만 바꾸면 fixed joint가 딸기를 원래 자리로 끌어당긴다 |
| 딸기 물리 파라미터 튜닝 (질량, 마찰) | `assets/strawberry/strawberry_physics.usd` |
| 줄기 분리 강도 튜닝 (breakForce) | `scenes/layers/physics_layer.usd` |
| 조명 변경 | `scenes/layers/lighting_layer.usd` |
| 로봇 커스터마이징 (카메라 위치 등) | `assets/robot/robot_assembly.usd`의 오버라이드 |
| 로봇 관절 강성/감쇠 튜닝 (출렁임·떨림) | `assets/robot/robot_assembly.usd`의 joint drive 오버라이드 (§3 로봇 참고) |
| 물리 없이 비주얼만 확인 | Layer 창에서 `physics_layer.usd`를 mute |
| 환경 소품 변경 (테이블, 보드) | `assets/props/`, `scenes/lab_environment.usd` |

---

## 5. 기존 `robot/` 폴더 대비 변경점

| 항목 | 기존 (`robot/`) | 현재 (`strawberry_harvest/`) |
|---|---|---|
| 씬 파일 | `strawberry_grasp_robot.usd` 하나에 지오메트리·조명·물리·시뮬 상태 혼재 | 씬은 조립만, 각 관심사는 sublayer로 분리 |
| 로봇 로드 경로 | 씬 → `robot_recent.usd` → `robot.usd` 중첩 | 씬 → `robot_assembly.usd` → `robot.usd` 단일 체인 |
| D455 카메라 | 씬 안에서 S3 클라우드 URL 직접 payload | `omniverse_imports/` 로컬 사본 참조 (오프라인 동작) |
| 딸기 | ripe/unripe 별개 파일, DomeLight 내장, 물리 없음 | `ripeness` variantSet, 물리 레이어 분리, RigidBody + 콜라이더 + 줄기 joint |
| 보드 | 씬 파일에 Mesh 직접 정의 | `props/whiteboard.usd` 애셋화 + attach point 정의 |

새로 추가된 물리 (기존 씬에 없었음):
- 딸기: RigidBody + 0.02 kg + convexHull 콜라이더
- 딸기 줄기: FixedJoint + `breakForce=20 / breakTorque=5` (당기면 분리. 초기 2/1은 너무 약해 2026-09-07 상향)
- 테이블/보드: 정적 콜라이더 (triangle mesh)

---

## 6. 실무 체크리스트

- [ ] 모든 애셋 USD 파일에 `defaultPrim` 지정
- [ ] 애셋의 prim 이름은 `/World`가 아니라 애셋 이름 사용 (예: `/strawberry`, `/robot_assembly`)
- [ ] 전체 프로젝트 단위/축 통일: meters, Z-up
- [ ] 외부(CAD, Nucleus) 파츠는 로컬 사본 + 단위/축 변환 wrapper 적용 후 사용
- [ ] 비주얼 메시와 콜라이더 메시 분리 (특히 그리퍼 핑거처럼 접촉이 중요한 부분)
- [ ] 그리퍼 핑거 등 접촉 파트는 convex decomposition 또는 SDF 콜라이더 적용 검토
- [ ] 환경(`lab_environment`)은 payload, 로봇/딸기는 reference로 조립되었는지 확인

---

## 7. 알려진 참고 사항

- `OmniPBR.mdl` / `OmniGlass.mdl` 미해석 경고는 정상입니다. Isaac Sim 내장 MDL 검색 경로로 해석됩니다.
- `robot_base.usd` 내부의 `visuals/world` 미해석 참조 경고는 URDF Importer가 남긴 기존 이슈로, 무해합니다.
- 조명은 09-16 에 촬영용으로 다시 짰다(주광 1 + 채움 1). 원래 딸기 애셋에 딸려 있던 studio.hdr 은 `textures/` 에 남아 있지만 참조하지 않는다.
