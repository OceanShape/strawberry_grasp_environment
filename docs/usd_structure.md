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
│   │   ├── whiteboard.usd                 # 보드 + attach_points(딸기 부착 지점 마커) 포함
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
│   │   └── lighting_layer.usd             # DomeLight(studio HDRI) + RectLight 2개
│   └── textures/studio.hdr
└── configs/
    ├── calibration_eye_in_hand.npz        # eye-in-hand 캘리브레이션 원본 데이터
    └── camera_calibration_to_isaac.py     # OpenCV → Isaac Sim 좌표 변환 스크립트
```

---

## 3. 각 구성 요소 상세

### 로봇 (`assets/robot/`)

- `doosan_e0509_rh_p12_rn/robot.usd`는 URDF Importer가 생성한 원본입니다. **절대 직접 수정하지 마세요.** URDF를 재빌드하면 이 폴더 전체가 교체됩니다.
- 로봇 커스터마이징(카메라 부착 위치, 오버라이드 등)은 `robot_assembly.usd`에서만 합니다.
- 그리퍼 종류를 바꿔가며 실험할 가능성이 있다면 `robot_assembly.usd`에 `variantSet(gripper_type)`을 추가하는 것을 고려하세요.

### 딸기 (`assets/strawberry/`)

- `strawberry.usd`에서 `ripeness` variantSet으로 ripe/unripe를 전환합니다.
- 물리 파라미터(질량, 콜라이더 근사, 마찰)는 `strawberry_physics.usd`에서만 수정합니다.
- 줄기 분리(picking) 동작은 `scenes/layers/physics_layer.usd`에 정의된 FixedJoint의 `breakForce=2 / breakTorque=1`로 모델링되어 있습니다. 이 값은 임시이므로 수확 테스트하며 튜닝하세요.

### 환경/소품 (`assets/props/`, `scenes/lab_environment.usd`)

- 테이블, 화이트보드는 `lab_environment.usd`에 조립되어 있으며, `main_scene.usd`에서 payload로 불러옵니다.
- `whiteboard.usd`에는 딸기를 매다는 `attach_points` prim이 정의되어 있습니다.

### Omniverse 임포트 (`assets/omniverse_imports/`)

- Nucleus 경로를 씬에서 직접 참조하지 않고, 로컬 사본을 만들어 관리합니다 (오프라인·협업 안정성 확보).
- `rsd455.usd`가 상대경로로 참조하는 MDL 파일과 텍스처도 함께 미러되어 있습니다.

### 씬 레이어 (`scenes/layers/`)

| 레이어 | 역할 |
|---|---|
| `physics_layer.usd` | PhysicsScene, gravity, solver, 전역 충돌 그룹, 딸기 줄기 joint |
| `lighting_layer.usd` | DomeLight(studio.hdr) + RectLight 2개 |
| `layout_layer.usd` | 딸기 위치, 로봇 베이스 위치, 초기 관절 포즈 — 반복 실험 대상 |

sublayer 순서(strength ordering)에 유의하세요. 나중에 추가된 레이어가 opinion을 override합니다.

---

## 4. 작업 규칙

| 하고 싶은 것 | 수정할 파일 |
|---|---|
| 딸기 위치 변경 / domain randomization | `scenes/layers/layout_layer.usd` (줄기 joint의 `localPos0`도 함께 이동) |
| 딸기 물리 파라미터 튜닝 (질량, 마찰) | `assets/strawberry/strawberry_physics.usd` |
| 줄기 분리 강도 튜닝 (breakForce) | `scenes/layers/physics_layer.usd` |
| 조명 변경 | `scenes/layers/lighting_layer.usd` |
| 로봇 커스터마이징 (카메라 위치 등) | `assets/robot/robot_assembly.usd`의 오버라이드 |
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
- 딸기 줄기: FixedJoint + `breakForce=2 / breakTorque=1` (당기면 분리)
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
- 조명은 기존 딸기 애셋에 딸려 있던 studio.hdr DomeLight + RectLight 2개를 씬 레이어로 옮긴 것이라 렌더 결과가 미세하게 다를 수 있습니다.
