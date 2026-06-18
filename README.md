# Strawberry Grasp URDF Environment Setup Guide

본 문서는 아이작 심(Isaac Sim) 기반의 딸기 수확 강화학습 환경을 구축하기 위해, 두산 로봇 팔(e0509)과 로보티즈 그리퍼(RH-P12-RN), 리얼센스 카메라(D455) 및 자체 제작 커스텀 파츠를 단일 URDF로 통합한 작업 내역과 사용 방법을 안내하는 설명서입니다.

---

## 1. 개요 및 디렉토리 구조

로봇 팔과 그리퍼의 물리적 형상 및 구동축(Joint) 정보를 담고 있는 공식 패키지들을 다운로드하고, 이를 엮어주는 메타 패키지를 생성하여 하나의 로봇 모델(`robot.urdf`)로 추출하는 환경입니다.

**주요 디렉토리 구조:**
```text
/home/sun/strawberry_grasp_environment/
 ├── build_urdf.sh                      # URDF 렌더링 및 경로 자동 치환 스크립트
 ├── robot.urdf                         # Isaac Sim으로 임포트할 최종 결과물 파일
 └── src/
      ├── doosan_robot2/                # 두산 로봇 공식 ROS 2 패키지 (e0509 포함)
      ├── RH-P12-RN/                    # 로보티즈 그리퍼 공식 ROS 패키지
      └── strawberry_grasp_environment_description/ # 통합 쉘(Shell) 및 커스텀 파츠 패키지 (직접 제작)
           ├── meshes/
           │    └── gripper_parts.stl   # 3D 프린터로 출력한 커스텀 파츠 이진 STL 파일
           └── urdf/
                └── robot.urdf.xacro    # 모든 모델을 하나로 조립하는 메인 xacro 파일
```

> **참고 (Git 리포지토리 구성 관련):**
> `src/doosan_robot2/` 및 `src/RH-P12-RN/` 디렉토리는 각 제조사의 공식 리포지토리이며 파일 용량이 큽니다. 따라서 깃허브 업로드 시 제외되도록 `.gitignore`에 등록되어 있습니다. 깃허브에서 이 프로젝트를 클론(Clone)하여 사용할 경우, 해당 공식 패키지들은 별도로 다운로드하여 `src/` 폴더 내에 배치해야 합니다.

---

## 2. 통합 로봇 모델(`robot.urdf.xacro`)의 구성

`src/strawberry_grasp_environment_description/urdf/robot.urdf.xacro` 파일은 다음의 구성 요소들을 하나로 묶어주는 뼈대 역할을 합니다.

1. **로봇 팔 - 그리퍼 결합**: 두산 e0509의 엔드 이펙터(`link_6`)에 로보티즈 그리퍼의 베이스(`rh_p12_rn_base`)를 고정(fixed) 조인트로 결합합니다.
2. **커스텀 파츠 결합 (Shell)**: 
   - 그리퍼의 양쪽 손가락 마지막 링크(`rh_p12_rn_l2`, `rh_p12_rn_r2`)에 각각 `left_custom_part`, `right_custom_part` 링크를 부착해두었습니다.
   - `gripper_parts.stl` 파일을 시각(visual) 모델로 불러오며, 충돌(collision) 연산 최적화를 위해 5cm 크기의 박스 형태(`box`) 플레이스홀더를 임시로 설정해두었습니다.
3. **카메라 결합 (Shell)**: 로봇 팔 끝단(`link_6`)에 Realsense D455를 상징하는 더미 박스(`d455_link`)를 부착해두었습니다. 실제 카메라는 Isaac Sim 환경 내부에서 제공하는 에셋으로 교체될 예정입니다.

---

## 3. 커스텀 파츠 오프셋 및 물리 설정 수정 방법

향후 커스텀 파츠의 실제 부착 위치를 미세 조정하거나 물리 충돌 영역을 구체화하려면 `robot.urdf.xacro` 파일을 수정해야 합니다.

### 부착 위치 및 각도 조절
파일 내부의 `<joint name="l2_to_left_custom_part">` 등에서 `<origin xyz="..." rpy="..." />` 수치를 변경하여 그리퍼 손가락과 커스텀 파츠 간의 부착 오프셋을 맞출 수 있습니다.
- `xyz`: 미터(m) 단위의 X, Y, Z 위치 이동
- `rpy`: 라디안(radian) 단위의 Roll, Pitch, Yaw 회전

### 양쪽 파츠 대칭(Mirror) 적용
오른쪽 파츠 부착 시, `geometry` 안의 메쉬 스케일 값을 조절하여 왼쪽 모델을 대칭시킬 수 있습니다.
```xml
<!-- 예: Y축 기준으로 대칭 -->
<mesh filename="package://strawberry_grasp_environment_description/meshes/gripper_parts.stl" scale="1 -1 1"/>
```

---

## 4. URDF 빌드 및 Isaac Sim 임포트

xacro 파일을 수정한 뒤에는 반드시 이를 하나의 `robot.urdf`로 렌더링해야 합니다.
Isaac Sim은 기본적으로 ROS 패키지의 절대 경로를 스스로 찾지 못하므로, `package://`로 시작하는 경로를 인식하지 못하고 메쉬가 투명하게 나오는 현상이 발생합니다.

이를 방지하기 위해 렌더링 및 **"Isaac Sim 호환을 위한 절대 경로 치환 작업"**을 한 번에 수행하는 스크립트를 제공합니다.

**빌드 실행 명령어:**
```bash
cd /home/sun/strawberry_grasp_environment
./build_urdf.sh
```

**실행 결과:**
스크립트가 정상적으로 실행되면 `robot.urdf`가 갱신됩니다. 갱신된 파일을 Isaac Sim의 **URDF Importer**를 통해 가져오시면, 모든 부품의 메쉬가 정상적으로 로드된 최종 로봇 형태를 확인할 수 있습니다.

---

## 5. 트러블슈팅 (Troubleshooting)

### Q. 특정 파츠(커스텀 파츠, 카메라 더미 등)가 화면이나 트리에 나타나지 않아요!
Isaac Sim과 같은 물리 엔진 기반 시뮬레이터들은 최적화를 위해 **질량(mass)과 관성 모멘트(inertia) 정보가 없는 링크(Dummy Link)를 부모 링크에 강제로 병합하거나 파싱 단계에서 무시**해버리는 특성이 있습니다.
- **해결책**: `robot.urdf.xacro` 파일 내에서 시각적 형태만 가지는 링크(`d455_link`, `left_custom_part` 등)에도 반드시 임시 `<inertial>` 블록(질량과 관성값)을 넣어주어야 합니다. (현재 제공된 코드에는 0.1kg의 임시 질량이 모두 추가되어 있으므로 안전하게 로드됩니다.)

### Q. 메쉬는 안 보이고 네모난 박스나 원통만 보여요!
URDF 내에 `package://` 로 시작하는 ROS 패키지 경로가 포함되어 있으면, Isaac Sim이 해당 경로를 해석하지 못해 외부 메쉬(`.stl`, `.dae`)를 불러오지 못합니다.
- **해결책**: 제공해 드린 `./build_urdf.sh` 스크립트를 실행하시면 `package://` 경로가 모두 컴퓨터의 절대 경로로 치환되므로 이 문제가 완벽히 해결됩니다.

### Q. 예전에 만들어둔 `gripper_parts.usd` 파일은 어떻게 하나요?
- **답변**: 해당 파일은 과거에 커스텀 파츠를 수동으로 조립하기 위해 만들어진 에셋입니다. 현재는 `robot.urdf.xacro` 내에서 `gripper_parts.stl` 원본 파일을 직접 참조하여 하나의 로봇 모델로 빌드하는 워크플로우를 구축하셨으므로, **별도의 `gripper_parts.usd` 파일은 더 이상 시뮬레이션 구성에 필요하지 않습니다.** Isaac Sim이 URDF를 임포트할 때 STL 메쉬를 자체적으로 읽어들여 최종 로봇 에셋(`strawberry_robot.usd`) 내부에 자동으로 내장(Bake)합니다.

---

## 6. 향후 진행 워크플로우 가이드 (Action Plan)

제안해주신 기획도와 워크플로우는 Isaac Sim의 특성과 기존 ROS 기반 로봇 생태계의 장점을 모두 살린 **가장 이상적이고 정석적인 구조**입니다. 로봇 본체의 기구학적 연결(URDF)과 시뮬레이션 환경 및 물리 상호작용(USD)을 분리하는 것은 향후 유지보수와 확장에 매우 큰 도움이 됩니다.

현재 작업하신 디렉토리(`. /home/sun/strawberry_grasp_environment/`)의 상태를 확인해 보았습니다. 현재 상황과 앞으로 어떻게 진행해야 할지 단계별로 명확히 정리해 드릴게요.

### 🔍 현재 상황 진단 (기획도 vs 현재 상태)

*   **잘 되어있는 점 (완료됨):** 기획도의 **"1단계: 로봇 에셋화(URDF)"** 부분은 이미 코드로 완벽하게 준비되어 있습니다. `robot.urdf.xacro` 파일에 두산 e0509 로봇 팔, RH-P12-RN 그리퍼, 커스텀 파츠, 그리고 D455 카메라 더미까지 하나의 완벽한 Kinematic Chain으로 고정(fixed joint)되어 연결되어 있습니다. 또한, Isaac Sim에서 경로 오류 없이 불러올 수 있도록 절대 경로로 치환해 주는 `build_urdf.sh` 스크립트와 최종 결과물인 `robot.urdf`도 생성되어 있습니다.
*   **다른 점 및 남은 작업:** 현재는 텍스트 기반의 **URDF 파일만 존재**할 뿐, 아직 Isaac Sim 내부로 가져가 물리 속성을 부여하고 USD 형태의 로봇 에셋(`strawberry_robot.usd`)으로 변환하는 작업과 **"2단계: 환경 씬(main_simulation.usd) 구축"** 작업은 시작되지 않은 상태입니다.

---

### 🚀 앞으로의 진행 가이드 (Action Plan)

이제 코딩(URDF 수정)은 잠시 멈추고, **Isaac Sim 프로그램을 실행하여 GUI 환경에서 다음 순서대로 진행**하시면 됩니다.

#### **Step 1: 로봇 에셋을 USD로 변환 및 물리 튜닝**
1. Isaac Sim 상단 메뉴 바에서 `Isaac Utils` -> `Workflows` -> `URDF Importer`를 실행합니다.
2. Import 경로를 `/home/sun/strawberry_grasp_environment/robot.urdf`로 지정하고 불러옵니다.
3. **가장 중요한 물리 튜닝:** 임포트된 로봇 트리를 열어 각 관절(Revolute Joint)의 `Drive` 속성(Stiffness, Damping)을 설정해 주어야 합니다. 이 값을 제대로 주지 않으면 로봇이 중력에 의해 축 늘어지거나 발작을 일으킬 수 있습니다.
4. 그리퍼와 커스텀 파츠의 Collision(충돌체)이 정상적으로 잡혀 있는지 확인합니다.
5. 튜닝이 끝난 최상단 로봇 루트(Root)를 우클릭하여 `Save As...`를 누르고, **`strawberry_robot.usd`**라는 이름으로 독립된 파일로 저장합니다. (이제 URDF는 수정할 일이 생기기 전까지 잊으셔도 됩니다!)

#### **Step 2: 시뮬레이션 메인 씬 구축**
1. Isaac Sim에서 File -> `New`를 눌러 완전히 빈 새로운 스테이지를 엽니다. 이 파일을 **`main_simulation.usd`**로 저장합니다.
2. 하단의 `Content` 브라우저를 통해 랩실 배경, 책상, 화이트보드 등의 에셋을 찾아 화면에 끌어다 놓습니다. (Omniverse Nucleus 서버의 `NVIDIA/Assets/Scenes/` 경로 등에 유용한 에셋이 많습니다.)
3. 로봇이나 물건이 바닥/책상을 뚫고 떨어지지 않도록, 책상과 바닥 에셋을 선택하고 우클릭 -> `Add` -> `Physics` -> **`Colliders Preset` (정적 충돌체)**을 적용합니다.

#### **Step 3: 대상 객체(딸기) 배치 및 세밀한 물리 속성 부여**
1. `main_simulation.usd` 화면의 테이블 위에 준비하신 딸기 3D 모델을 배치합니다.
2. 수확 시뮬레이션의 핵심은 그리퍼와 딸기의 상호작용이므로, 딸기를 선택하고 우클릭 -> `Add` -> `Physics` -> **`Rigid Body with Colliders Preset`**을 적용합니다.
3. 딸기의 Property 창에서 질량(Mass)을 실제 딸기 무게(예: 0.02kg)로 맞추고, 표면의 마찰 계수(Friction) 등을 조정합니다.

#### **Step 4: 에셋 조립 (최종 통합)**
1. 현재 열려있는 `main_simulation.usd`의 스테이지에 Step 1에서 만든 **`strawberry_robot.usd`**를 끌어다 놓습니다. (이때 파일 자체를 합치지 않고, `Add as Reference` 또는 `Payload` 방식으로 불러오는 것을 강력히 추천합니다.)
2. 딸기가 놓인 테이블 앞으로 로봇의 베이스 위치를 적절히 이동시켜 시나리오의 초기 상태를 완성합니다.

---

현재 URDF 파일 구조가 매우 깔끔하게 잘 짜여 있어서, 곧바로 Isaac Sim을 켜고 **Step 1(URDF Importer)**부터 실행해 보시면 되겠습니다. 

임포트 후 로봇 관절의 Stiffness/Damping 값을 어느 정도로 주어야 할지 막막하시거나, 딸기의 마찰력 세팅이 뜻대로 되지 않을 때 언제든 다시 말씀해 주세요! 훌륭한 방향으로 가고 계십니다.