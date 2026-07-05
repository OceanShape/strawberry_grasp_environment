# Strawberry Harvest — USD 프로젝트 구조

`USD_PROJECT_GUIDE.md`의 원칙(애셋/씬 분리, 물리 레이어 분리, 원본 무수정, payload/reference 구분)에 따라
기존 `robot/` 폴더의 USD 구성을 재구성한 결과물입니다. **기존 `robot/` 폴더는 백업으로 그대로 남아 있으며,
이 폴더는 그것과 독립적으로 동작합니다.**

## 시작하기

Isaac Sim에서 **`scenes/main_scene.usd`** 를 여세요. 환경(payload) + 로봇(reference) + 딸기(reference)가
조립된 최상위 씬입니다.

## 디렉토리 구조

```
strawberry_harvest/
├── assets/                                # 재사용 가능한 애셋 (씬에 직접 지오메트리 없음)
│   ├── robot/
│   │   ├── doosan_e0509_rh_p12_rn/        # URDF 임포트 원본 로봇 패키지 (수정 금지, robot/의 사본)
│   │   │   ├── robot.usd                  #   variantSet: Physics / Sensor / Robot
│   │   │   └── configuration/             #   base(지오메트리)·physics(joint/drive)·robot(Isaac API) 레이어
│   │   └── robot_assembly.usd             # 로봇 + D455 카메라 조립본 (씬에서는 이것만 참조)
│   ├── strawberry/
│   │   ├── strawberry.usd                 # 애셋 인터페이스 — variantSet "ripeness" (ripe/unripe)
│   │   ├── strawberry_physics.usd         # 물리 레이어: 질량 0.02kg, convexHull 콜라이더 (튜닝은 여기서)
│   │   ├── strawberry_ripe.usd            # 지오메트리+머티리얼 (prim 구조: /strawberry/geo/fruit/mesh)
│   │   ├── strawberry_unripe.usd          #   〃 (내장 DomeLight 제거됨 — 조명은 씬 레이어로 이동)
│   │   └── textures/
│   ├── props/
│   │   ├── table.usd                      # 테이블 (구 robot/table.usd 정리본, defaultPrim=table)
│   │   ├── whiteboard.usd                 # 보드 + attach_points(딸기 부착 지점 마커) 포함
│   │   └── textures/
│   └── omniverse_imports/                 # Nucleus/클라우드 애셋 로컬 사본 (직접 네트워크 참조 금지)
│       └── Isaac/
│           ├── Sensors/Intel/RealSense/rsd455.usd
│           └── Materials/Base/...         # rsd455가 상대경로로 참조하는 MDL + 텍스처 미러
├── scenes/
│   ├── main_scene.usd                     # ★ 최상위 씬 — 조립 전용, 직접 수정 최소화
│   ├── lab_environment.usd                # 정적 환경 조립본 (테이블+보드) — payload로 로드됨
│   ├── layers/
│   │   ├── layout_layer.usd               # 배치: 딸기 위치, 로봇 베이스, 초기 관절 포즈 (randomization 대상)
│   │   ├── physics_layer.usd              # PhysicsScene, 정적 콜라이더, 딸기 줄기(stem) fixed joint
│   │   └── lighting_layer.usd             # DomeLight(studio HDRI) + RectLight 2개
│   └── textures/studio.hdr
├── configs/
│   ├── calibration_eye_in_hand.npz        # eye-in-hand 캘리브레이션 원본 데이터
│   └── camera_calibration_to_isaac.py     # OpenCV→Isaac 좌표 변환 스크립트 (구 configuration/script.py)
└── README.md
```

## 기존 구성 대비 바뀐 점

| 항목 | 기존 (robot/) | 재구성 후 |
|---|---|---|
| 씬 파일 | `strawberry_grasp_robot.usd` 하나에 보드 지오메트리·조명·물리·시뮬레이션 잔여 상태가 혼재 | 씬은 조립만, 각 관심사는 sublayer로 분리 |
| 로봇 로드 경로 | 씬 → `robot_recent.usd`(또 다른 씬 파일) → `robot.usd` 중첩 | 씬 → `robot_assembly.usd` → `robot.usd` 단일 체인 |
| D455 카메라 | 씬 안에서 S3 클라우드 URL 직접 payload | `omniverse_imports/` 로컬 사본 참조 (오프라인 동작) |
| 딸기 | ripe/unripe 별개 파일, DomeLight 내장, 물리 없음 | `strawberry.usd` + ripeness variant, 물리 레이어 분리, RigidBody+콜라이더+줄기 joint 적용 |
| 보드 | 씬 파일에 Mesh 직접 정의 | `props/whiteboard.usd` 애셋화 + attach point 정의 |
| 시뮬 잔여물 | 링크 velocity, 뷰포트 Render 설정 등이 파일에 저장됨 | 제거 (의도된 초기 포즈만 layout_layer에 보존) |

새로 **추가된 물리** (기존 씬에는 없었음):
- 딸기: RigidBody + 질량 0.02 kg + convexHull 콜라이더 (`assets/strawberry/strawberry_physics.usd`)
- 딸기 줄기: 월드 고정 FixedJoint + `breakForce=2 / breakTorque=1` — 그리퍼로 당기면 분리됩니다.
  값이 임시이므로 수확 테스트하며 `scenes/layers/physics_layer.usd`에서 튜닝하세요.
- 테이블/보드: 정적 콜라이더 (triangle mesh)

## 작업 규칙

1. 딸기 위치를 바꾸거나 랜덤화 → `scenes/layers/layout_layer.usd` (줄기 joint의 `localPos0`도 함께 이동)
2. 물리 파라미터 튜닝 → 씬 레벨은 `physics_layer.usd`, 딸기 고유 값은 `strawberry_physics.usd`
3. 조명 변경 → `lighting_layer.usd`
4. 로봇 커스터마이징 → `robot_assembly.usd`의 오버라이드로. `doosan_e0509_rh_p12_rn/`는 절대 직접 수정 금지
   (URDF 재빌드 시 통째로 교체되는 폴더입니다)
5. 물리 없이 비주얼만 확인 → Layer 창에서 `physics_layer.usd`를 mute

## 알려진 참고 사항

- `OmniPBR.mdl` / `OmniGlass.mdl` 미해석 경고는 정상입니다 (Isaac Sim 내장 MDL 검색 경로로 해석됨).
- `robot_base.usd` 내부의 `visuals/world` 미해석 참조 경고는 URDF 임포터가 남긴 기존 이슈로, 무해합니다.
- 조명 구성은 기존 씬의 실효 조명(딸기 애셋에 딸려 있던 studio.hdr DomeLight + RectLight 2개)을
  씬 레이어로 옮긴 것이라 렌더 결과가 미세하게 다를 수 있습니다.
