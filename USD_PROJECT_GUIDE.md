# 딸기 수확 로봇 프로젝트 — USD 구조 가이드

이 문서는 Isaac Sim 5.1.0 기반 딸기 수확 로봇 프로젝트의 USD 파일 구조를 리팩토링하기 위한
컨텍스트 문서입니다. 기존에 만들어진 USD 프로젝트를 이 가이드에 맞춰 재구성해 주세요.

## 프로젝트 개요

- **목표**: 6축 로봇 + 그리퍼(커스텀 파츠 포함)로 딸기를 수확하는 동작을 Isaac Sim에서 물리 시뮬레이션 테스트
- **구성 요소**: 딸기, 6축 로봇, 그리퍼, 그리퍼용 커스텀 파츠, Omniverse/Nucleus에서 가져온 외부 파츠,
  주변 환경(테이블, 딸기를 매다는 화이트보드, 광원), 물리 설정(PhysicsScene, RigidBody, Joint 등)
- **환경**: Isaac Sim 5.1.0, units = meters, up-axis = Z

## 핵심 원칙

1. **애셋과 씬을 분리한다.** 재사용 가능한 지오메트리(로봇, 그리퍼, 딸기, 소품)는 개별 USD 애셋 파일로 만들고,
   씬 파일은 이들을 `reference` 또는 `payload`로 조립만 한다. 씬 파일에 지오메트리를 직접 만들지 않는다.
2. **물리는 별도 레이어(sublayer)로 분리한다.** 비주얼/지오메트리와 물리 속성을 분리해야 물리 파라미터 튜닝 시
   diff 관리가 쉽고, 물리 없이 비주얼만 확인하는 것도 가능해진다.
3. **원본 애셋은 직접 수정하지 않는다.** Isaac Sim 제공 로봇 등은 그대로 참조하고, 커스터마이징은 조립본(assembly)
   파일에서 오버라이드나 추가 prim으로 처리한다.
4. **자주 안 바뀌는 것은 payload, 항상 필요한 것은 reference.** 정적 환경(lab_environment)처럼 무거운 지오메트리는
   payload로 필요할 때만 로드해서 반복 테스트 로딩 속도를 확보한다.

## 권장 디렉토리 / 파일 구조

```
strawberry_harvest/
├── assets/
│   ├── robot/
│   │   ├── ur10e.usd              # 원본 6축 로봇 참조 wrapper (Isaac 제공 애셋 그대로 참조, 수정 금지)
│   │   ├── gripper/
│   │   │   ├── gripper.usd        # 그리퍼 본체
│   │   │   └── custom_finger.usd  # 그리퍼에 붙는 커스텀 파츠
│   │   └── robot_assembly.usd     # 로봇+그리퍼+커스텀파츠 조립 (fixed joint로 연결)
│   ├── strawberry/
│   │   ├── strawberry.usd         # 지오메트리 + 머티리얼
│   │   └── strawberry_physics.usd # 딸기 개별 물리 속성 레이어 (질량, 콜라이더, 줄기 joint 등)
│   ├── props/
│   │   ├── table.usd
│   │   └── whiteboard.usd         # 딸기 매다는 보드, 부착 지점(attach point) 포함
│   └── omniverse_imports/         # Nucleus에서 가져온 파츠의 로컬 사본 (네트워크 경로 직접 참조 금지)
├── scenes/
│   ├── lab_environment.usd        # 정적 환경 조립본 (테이블, 보드, 조명) — payload로 불러올 대상
│   ├── main_scene.usd             # 최상위 씬. 환경(payload)+로봇(reference)+딸기(reference) 조립
│   └── layers/
│       ├── physics_layer.usd      # PhysicsScene, gravity, solver 설정, 충돌 그룹, 물리 오버라이드
│       ├── lighting_layer.usd     # 광원 설정
│       └── layout_layer.usd       # 배치(transform) 오버라이드 — 딸기 위치 등 반복 실험 대상
└── configs/                       # 태스크/컨트롤러 설정 (Python 스크립트 쪽)
```

## 각 구성 요소별 상세 지침

### 로봇 + 그리퍼 (assets/robot/)
- 6축 로봇 원본은 Isaac Sim 제공 애셋(`omniverse://.../Isaac/Robots/...`)을 그대로 참조.
- 그리퍼 및 커스텀 파츠는 `robot_assembly.usd`에서 fixed joint로 로봇 flange에 연결.
- 그리퍼 종류를 바꿔가며 실험할 가능성이 있다면 `robot_assembly.usd`에 `variantSet`(예: `gripper_type`)을 걸어둘 것.

### 딸기 (assets/strawberry/)
- 지오메트리/머티리얼과 물리 속성(strawberry_physics.usd)을 분리.
- 줄기 분리(picking) 동작이 필요하면 줄기를 joint(예: `PhysicsJoint` + break force)로 모델링하는 것을 고려.
- 물리 파라미터(질량, 콜라이더 근사, 마찰, break force)는 이 레이어에서만 반복 수정하도록 구조화.

### 환경/소품 (assets/props/, scenes/lab_environment.usd)
- 테이블, 화이트보드, 조명 등은 정적 환경으로 `lab_environment.usd`에 조립.
- `main_scene.usd`에서는 이를 payload로 불러와 로딩 속도를 확보.
- 화이트보드에는 딸기를 매다는 attach point(prim 또는 anchor)를 명시적으로 정의.

### Omniverse/Nucleus 외부 파츠 (assets/omniverse_imports/)
- Nucleus 경로를 씬에서 직접 참조하지 말고, 로컬 사본을 만들어 관리 (오프라인/협업 안정성).
- 단위(cm→m)나 축(Y-up→Z-up)이 다른 경우가 많으므로, 가져올 때 변환 wrapper USD를 씌워서 정규화.

### 물리 레이어 (scenes/layers/physics_layer.usd)
- PhysicsScene, gravity, solver 설정, 전역 충돌 그룹 등 씬 레벨 물리는 여기서 관리.
- 개별 애셋 고유 물리(딸기 질량 등)는 각 애셋의 physics 레이어에 둔다 — 씬 레벨과 애셋 레벨 물리를 혼동하지 말 것.

### 레이아웃 레이어 (scenes/layers/layout_layer.usd)
- 딸기 위치 등 반복적으로 바뀌는 transform 오버라이드를 이 레이어에 모음.
- 추후 Python으로 딸기 위치를 랜덤화(domain randomization)할 때도 이 구조를 그대로 활용 가능.

## 실무 체크리스트

- [ ] 모든 애셋 USD 파일에 `defaultPrim` 지정
- [ ] 애셋의 prim 이름은 `/World`가 아니라 애셋 이름 사용 (예: `/strawberry`, `/robot_assembly`)
- [ ] 전체 프로젝트 단위/축 통일: meters, Z-up
- [ ] 외부(CAD, Nucleus) 파츠는 로컬 사본 + 단위/축 변환 wrapper 적용 후 사용
- [ ] 비주얼 메시와 콜라이더 메시 분리 (특히 그리퍼 핑거처럼 접촉이 중요한 부분)
- [ ] 그리퍼 핑거 등 접촉 파트는 convex decomposition 또는 SDF 콜라이더 적용 검토
- [ ] 물리 레이어(physics_layer.usd)와 지오메트리 레이어 분리 여부 확인
- [ ] 환경(lab_environment)은 payload, 로봇/딸기는 reference로 조립되었는지 확인
- [ ] variantSet(gripper_type 등) 적용 여부 검토

## 리팩토링 시 주의사항

- 기존 프로젝트에서 지오메트리와 물리 속성이 한 파일에 섞여 있다면, 이를 분리하면서 참조 관계(reference/payload)가
  깨지지 않는지 각 단계마다 Isaac Sim에서 열어 확인할 것.
- sublayer 순서(strength ordering)에 유의: 나중에 추가된 sublayer가 opinion을 override 하므로,
  layout_layer나 physics_layer를 씬 최상단에 배치할지 신중히 결정.
- 기존 절대경로(Nucleus 경로 등)로 참조된 부분을 로컬 상대경로로 옮길 때 전체 참조 체인을 점검할 것.
