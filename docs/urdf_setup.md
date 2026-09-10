# URDF 빌드 및 로봇 모델 구성 가이드

로봇 팔과 그리퍼의 형상·관절 정보를 담은 공식 ROS 2 패키지들을 묶어 하나의 `robot.urdf`로 추출하고, Isaac Sim에서 임포트하는 방법을 안내합니다.

---

## 1. 통합 로봇 모델 구성

`src/strawberry_grasp_environment_description/urdf/robot.urdf.xacro`가 다음 구성 요소를 하나의 Kinematic Chain으로 연결합니다.

| 구성 요소 | 연결 방식 | 비고 |
|---|---|---|
| 두산 e0509 로봇 팔 | 베이스 | `link_6`이 엔드 이펙터 |
| RH-P12-RN 그리퍼 | `link_6` ← fixed joint | `rh_p12_rn_base` 기준 |
| 커스텀 파츠 (3D 프린팅) | 그리퍼 손가락 ← fixed joint | `gripper_parts.stl` 직접 참조 |
| Realsense D455 (더미) | `link_6` ← fixed joint | 실제 카메라는 Isaac Sim 에셋으로 교체 예정 |

### 커스텀 파츠 위치 조정

`robot.urdf.xacro` 내 `<joint name="l2_to_left_custom_part">` 등의 `<origin>` 수치를 수정합니다.

```xml
<origin xyz="0.0 0.0 0.05" rpy="0 0 0"/>
<!-- xyz: 미터(m) 단위 X/Y/Z 이동, rpy: 라디안 단위 Roll/Pitch/Yaw -->
```

오른쪽 파츠를 왼쪽의 Y축 대칭으로 적용하려면 mesh scale을 조정합니다.

```xml
<mesh filename="package://strawberry_grasp_environment_description/meshes/gripper_parts.stl" scale="1 -1 1"/>
```

---

## 2. URDF 빌드 방법

### 사전 준비

`src/doosan_robot2`와 `src/RH-P12-RN`은 용량 문제로 `.gitignore`에 등록되어 있습니다. 저장소를 클론한 경우 해당 공식 패키지를 `src/` 안에 별도로 배치해야 합니다.

### 빌드 실행

```bash
cd /home/oceanshape/strawberry_grasp_environment
./build_urdf.sh
```

스크립트가 수행하는 작업:
1. xacro 렌더링 → `robot.urdf` 생성
2. `package://` 경로를 Isaac Sim용 절대경로로 치환
3. 그리퍼 색상 커스터마이징 (grey → black)

정상 실행 시 `robot.urdf`(Isaac Sim용 절대경로 치환본) 하나가 갱신됩니다.

> **참고**: 스크립트 상단의 `PROJECT` 변수가 이 프로젝트의 절대경로로 설정되어 있습니다. 다른 PC로 옮기면 이 변수만 수정하면 됩니다. 또한 스크립트가 `/opt/ros/humble/setup.bash`를 소싱하는데, ROS 2 미설치 환경에서도 pip으로 설치된 xacro(`~/.local/bin/xacro`)가 있으면 빌드는 동작합니다.

---

## 3. Isaac Sim URDF 임포트

1. Isaac Sim 상단 메뉴: `Isaac Utils → Workflows → URDF Importer`
2. Import 경로를 `robot.urdf`로 지정하여 불러옵니다.
3. 임포트 후 **물리 튜닝 필수**: 각 Revolute Joint의 Drive 속성(Stiffness, Damping)을 설정하지 않으면 로봇이 중력에 의해 늘어집니다.
4. 그리퍼·커스텀 파츠의 Collision이 정상인지 확인합니다.
5. 튜닝 완료 후 로봇 루트를 우클릭 → `Save As...` → `strawberry_harvest/assets/robot/doosan_e0509_rh_p12_rn/` 경로에 저장합니다.

> URDF 임포트 결과물은 이미 `strawberry_harvest/assets/robot/doosan_e0509_rh_p12_rn/`에 존재합니다. xacro를 수정하지 않는 한 이 단계를 다시 수행할 필요는 없습니다.

---

## 4. 트러블슈팅

**Q. 특정 파츠(커스텀 파츠, 카메라 더미 등)가 씬 트리에 표시되지 않습니다.**

Isaac Sim은 질량·관성(inertial) 정보가 없는 링크를 파싱 단계에서 부모에 병합하거나 무시합니다. `robot.urdf.xacro` 내 해당 링크에 임시 `<inertial>` 블록(현재 0.1 kg)이 추가되어 있으므로 정상적으로 로드됩니다. 새로 추가하는 링크에도 반드시 `<inertial>`을 포함하세요.

**Q. 메쉬가 보이지 않고 박스/원통만 표시됩니다.**

`package://` 경로가 포함된 URDF를 Isaac Sim이 해석하지 못하는 경우입니다. `build_urdf.sh`로 절대경로 치환을 완료한 `robot.urdf`를 사용하세요.

**Q. 이전에 만들어둔 `gripper_parts.usd`는 어떻게 하나요?**

현재 워크플로우에서는 `robot.urdf.xacro`가 `gripper_parts.stl`을 직접 참조하므로, 별도의 `gripper_parts.usd`는 필요하지 않습니다. Isaac Sim이 URDF 임포트 시 STL을 읽어 최종 로봇 에셋 내부에 자동으로 내장(Bake)합니다.
