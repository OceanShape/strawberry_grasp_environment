# Strawberry Grasp Environment

Isaac Sim 5.1.0 기반 딸기 수확 강화학습 환경. 두산 e0509 로봇 팔 + RH-P12-RN 그리퍼 + Realsense D455 카메라로 딸기를 수확하는 물리 시뮬레이션을 구축합니다.

## 빠른 시작

Isaac Sim에서 `strawberry_harvest/scenes/main_scene.usd`를 여세요.

## 디렉토리 구조

```
strawberry_grasp_environment/
├── src/                          # ROS 2 패키지 (URDF 빌드용)
│   ├── doosan_robot2/            # 두산 공식 패키지 (.gitignore — 별도 클론 필요)
│   ├── RH-P12-RN/                # 로보티즈 공식 패키지 (.gitignore — 별도 클론 필요)
│   └── strawberry_grasp_environment_description/  # 통합 xacro 및 커스텀 파츠 메시
├── strawberry_harvest/           # Isaac Sim USD 프로젝트 (메인 작업 폴더)
├── robot/                        # 이전 USD 구성 백업 (참조용으로만 유지)
├── robot.urdf                    # URDF 빌드 결과물 (Isaac Sim URDF Importer용)
├── build_urdf.sh                 # URDF 빌드 + Isaac Sim 절대경로 치환 스크립트
└── docs/                         # 프로젝트 문서
```

## 문서

| 문서 | 내용 |
|---|---|
| [`docs/urdf_setup.md`](docs/urdf_setup.md) | URDF 빌드 방법, 로봇 모델 구성, 트러블슈팅 |
| [`docs/usd_structure.md`](docs/usd_structure.md) | Isaac Sim USD 구조 원칙, 디렉토리 구조, 작업 규칙 |
