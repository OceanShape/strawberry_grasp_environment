# Strawberry Grasp Environment

Isaac Sim 5.1.0 기반 딸기 수확 전체 시뮬레이션 스택. 두산 e0509 로봇 팔 + RH-P12-RN 그리퍼 + Realsense D455 카메라로 딸기를 수확하는 물리 시뮬레이션 환경 구축부터, cuRobo 기반 모션 플래닝 노드 통합까지 담당합니다.

> **포트폴리오 문서**: 전체 파이프라인 설계 및 구성 이유는 [`docs/pipeline_overview.md`](docs/pipeline_overview.md)를 참고하세요.

**현재 상태 (2026-09-10)** — 스캔 → 탐지 → 계획 → 접근 → 파지 → **배치(release)** → 복귀의 수확 핵심 경로가
익은 딸기 4개 연속으로 완주했습니다 (2026-09-07, `log/m2_5/`). 그 뒤 노드 시퀀스와 시뮬을 정합한 09-08~09 수정본은
END-TO-END 재완주 검증 대기 중이며, 남은 작업은 **재완주 → 딸기 부착 → 녹화 → 수치 갱신·푸시** 입니다.
제출 범위·작업 순서는 [`SUBMISSION_PLAN.md`](SUBMISSION_PLAN.md), 착수 금지 목록과 그 근거는 [`portfolio/H_scope_decisions.md`](portfolio/H_scope_decisions.md) 가 기준입니다.

---

## 빠른 시작

전체 파이프라인 실행은 **[`docs/run_guide.md`](docs/run_guide.md)** 를 따르세요 —
**터미널 3개**면 됩니다.

| | 터미널 | 명령 |
|---|---|---|
| 1 | Isaac Sim (브릿지 + 뷰포트 HUD) | `isaacsim` (venv) |
| 2 | 노드 4개 (fake_vision · 가상 제어기 · planner · scan_executor) | `bash scripts/run_nodes.sh` |
| 3 | 트리거 | `ros2 service call /strawberry/scan/start std_srvs/srv/Trigger` |

[`scripts/run_nodes.sh`](scripts/run_nodes.sh) 가 남은 노드 점검 → `check_params.py` →
노드 4개 기동 → **기동 로그 자동 대조**까지 하고, 노드별 로그를 `run_logs/latest/*.log` 로
따로 남깁니다. `Ctrl+C` 한 번이면 네 노드가 모두 정리됩니다.

씬만 보려면 Isaac Sim에서 `strawberry_harvest/scenes/main_scene.usd`를 열면 됩니다
(Isaac Sim 실행 명령은 아래 "Isaac Sim 실행" 섹션 참고 — 환경변수 없이 실행하면 ROS2 Bridge가 켜지지 않습니다).

---

## 디렉토리 구조

```
strawberry_grasp_environment/
├── src/
│   ├── doosan_robot2/                        # 두산 공식 패키지 (.gitignore — 별도 클론 필요)
│   ├── RH-P12-RN/                            # 로보티즈 공식 패키지 (.gitignore — 별도 클론 필요)
│   ├── strawberry_grasp_environment_description/  # 통합 xacro + 커스텀 파츠 메시
│   ├── strawberry_motion/                    # 수확 모션 플래닝 ROS 2 패키지
│   │   ├── strawberry_motion/
│   │   │   └── execution/                    # scan_executor_node, scan_safety
│   │   ├── scripts/                          # curobo_planner_node + 지원 모듈 전체
│   │   └── config/                           # 스캔 포즈·충돌 월드·cuRobo 설정
│   ├── strawberry_sim_core/                  # 가상 브릿지 (fake_vision, sim_executor_bridge, status_monitor)
│   ├── dsr_gripper_tcp_interfaces/           # 그리퍼 액션·서비스 인터페이스 정의
│   └── e0509_gripper_description/            # cuRobo 설정 데이터 전용 패키지
├── strawberry_harvest/                       # Isaac Sim USD 프로젝트 (씬·애셋)
│   └── scripts/                              # Script Editor 브릿지, 충돌 로거
├── robot/                                    # 이전 USD 구성 백업 (참조용)
├── robot.urdf                                # URDF 빌드 결과물 (Isaac Sim URDF Importer용)
├── build_urdf.sh                             # URDF 빌드 + Isaac Sim 절대경로 치환 스크립트
├── portfolio/                                # 제출용 산출물 (다이어그램·Before/After·정량 지표)
├── log/m1/, log/m2_5/                        # 통합 테스트 실행 로그 (근거 보존). 재완주·녹화 런은 log/m3/ 에
├── SUBMISSION_PLAN.md                        # 제출 범위·작업 순서 (최우선 문서)
├── scripts/check_planner.sh                  # 이식 변경 추적 — 원본 스냅샷 대비 diff 재생성
└── docs/                                     # 프로젝트 문서
```

---

## 주요 노드 구성

| 노드 | 위치 | 역할 |
|---|---|---|
| `strawberry_fusion_node` | `src/strawberry_motion/scripts/` | Realsense RGB-D + YOLO → 딸기 3D 타겟 발행 (실기 전용, 시뮬은 fake_vision이 대체) |
| `scan_executor_node` | `src/strawberry_motion/strawberry_motion/execution/` | 스캔 포즈 이동 → 타겟 큐 관리 → planner에 전달 |
| `curobo_planner_node` | `src/strawberry_motion/scripts/` | cuRobo 기반 pre-approach 계획 + pick 시퀀스 실행 |
| `fake_vision_node` | `src/strawberry_sim_core/` | Isaac Sim 딸기 좌표를 실기 비전과 동일 형식으로 발행 |
| `sim_executor_bridge_node` | `src/strawberry_sim_core/` | 두산 motion 서비스 모방 → `/joint_command` 변환 (가상 제어기) |
| `isaac_sim_hud.py` | `strawberry_harvest/scripts/` | 뷰포트 위 상태 HUD — 노드 램프·타겟·단계·완주 결과 (읽기 전용, 파이프라인에 개입하지 않음) |
| Script Editor 브릿지 | `strawberry_harvest/scripts/` | Isaac Sim 내부: 딸기 좌표 발행 + 관절 명령 주입 |

> 노드 실행 방법 및 ROS 2 인터페이스 상세: [`docs/pipeline_overview.md`](docs/pipeline_overview.md)

---

## 문서

문서가 열여섯 개라 겹치는 부분이 있다. **각 주제마다 기준 문서를 하나씩만 정했다** — 아래 표의
"기준" 열이 그것이고, 다른 문서에 같은 내용이 나오면 그쪽이 낡은 것으로 본다.

### 먼저 볼 것

| 문서 | 기준 | 내용 |
|---|---|---|
| **[`SUBMISSION_PLAN.md`](SUBMISSION_PLAN.md)** | **제출 범위·작업 순서 (최우선)** | 산출물 A(영상)까지로 범위 확정, sim2real 게이트, T1~T7 순서. 착수 금지 목록은 [`portfolio/H_scope_decisions.md`](portfolio/H_scope_decisions.md) §2 로 옮겼다. 다른 문서와 충돌하면 이 문서가 이긴다 |
| **[`docs/parameters.md`](docs/parameters.md)** + `check_params.py` | **모든 수치** | 보드 위치·툴 오프셋·그리퍼 개도·속도·관절 가드. **숫자가 헷갈리면 여기만 본다.** 보드를 옮길 때 같이 고칠 6개 파일 목록 포함. `python3 check_params.py` 로 정합 자동 검사 |
| **[`docs/run_guide.md`](docs/run_guide.md)** | **실행 절차** | 터미널 3개 실행 절차, 노드별 파라미터 참고사항, 기동 시 대조할 로그, 증상별 대처표 |
| **[`PROGRESS_REPORT.md`](PROGRESS_REPORT.md)** | **수치·근거 1차 출처** | 09-09 재계측 지표, 해결한 문제 목록, 표현 가이드 |
| **[`PORTFOLIO_SPRINT.md`](PORTFOLIO_SPRINT.md)** | 마일스톤 이력 | M1~M4 판정 기준·결과, 씬 구성 메모, 진행 기록 |
| **[`PLANNER_POLICY_v2.md`](PLANNER_POLICY_v2.md)** | **플래너 수정 원칙·표현 규칙** | **§0-1 (09-14 최우선): 실기 노드 수정은 설계 불일치일 때만, 가드 완화 금지** · 금지 표현 + 09-10 델타(§5.4) |

### 설계·이력

| 문서 | 기준 | 내용 |
|---|---|---|
| [`PROJECT_GOAL.md`](PROJECT_GOAL.md) | **원안 시퀀스** | 최종 목표, 작업 단계, §1-2 에 원래 설계했던 수확 6단계 |
| [`PLANNER_CHANGES.md`](PLANNER_CHANGES.md) | **변경 이력** | 플래너·제어기·씬 수정 로그. 왜 그 값이 됐는지의 근거 |
| [`docs/pipeline_overview.md`](docs/pipeline_overview.md) | **노드/토픽 구조** | 데이터 흐름, 노드 역할, ROS 인터페이스 |
| [`docs/config_reference.md`](docs/config_reference.md) | **설정 파일** | 각 YAML 이 무엇이고 어느 노드가 읽는가 (값 자체는 `parameters.md`) |
| [`PROJECT_SUMMARY.md`](PROJECT_SUMMARY.md) | 이력서용 요약 | 스냅샷. 상세는 위 문서들이 기준 |
| [`docs/next_steps.md`](docs/next_steps.md) | 과거 작업 명세 | 2026-07~08 세션 기록. **현재 범위·순서는 `SUBMISSION_PLAN.md` 가 기준** |

### 환경 구축

| 문서 | 기준 | 내용 |
|---|---|---|
| 이 README (아래) | **설치 절차** | ROS 2 / CUDA / cuRobo / Isaac Sim 설치 |
| [`docs/urdf_setup.md`](docs/urdf_setup.md) | URDF | 빌드 방법, 로봇 모델 구성 |
| [`docs/usd_structure.md`](docs/usd_structure.md) | USD | 레이어 구조 원칙, 작업 규칙 |
| [`docs/concepts.md`](docs/concepts.md) | 개념 노트 | ROS 2·파이썬 환경·Isaac Sim 확장 Q&A |
| [`portfolio/README.md`](portfolio/README.md) | 제출물 | 다이어그램, Before/After, 정량 지표, **영상 클립 4종 체크리스트·카메라 2대 녹화 절차** |

---

## 실행 환경 구조 — 두 개의 파이썬 세계

이 프로젝트는 **서로 다른 파이썬 버전을 쓰는 두 프로세스가 ROS 2 통신으로만 연결되는 구조**입니다. 하나의 환경(venv)으로 합칠 수 없으며, 합치려고 시도하면 안 됩니다.

```
┌───────────────┐              ┌─────────────────┐
│  Isaac Sim 5.1.0             │              │  ROS 2 Humble 노드들              │
│  (venv, Python 3.11)         │  ROS 2       │  (시스템 Python 3.10)             │
│                              │  토픽/서비스  │                                  │
│  - main_scene.usd 물리 시뮬  │ ◄─────► │  - scan_executor_node            │
│  - Script Editor 브릿지      │               │  - curobo_planner_node           │
│    (딸기 좌표 발행,          │               │  - fake_vision_node              │
│     /joint_command 수신)     │              │  - sim_executor_bridge_node      │
└───────────────┘              └─────────────────┘
```

**왜 분리되어 있나:**
- ROS 2 Humble은 Python 3.10에 묶여 있음 (rclpy 등 모든 ROS 라이브러리가 3.10용으로 빌드됨)
- Isaac Sim 5.1.0은 Python 3.11 필요 → venv에 설치
- 따라서 ROS 노드를 Isaac Sim venv 안에서 실행할 수 없고, 그 반대도 불가능

**두 세계가 섞이면 생기는 문제:**
- 터미널에 ROS 2 환경변수(`LD_LIBRARY_PATH`, `PYTHONPATH` 등)가 설정된 상태로 Isaac Sim을 실행하면, Isaac Sim이 3.10용 라이브러리를 로드하려다 즉시 크래시함
- 그래서 Isaac Sim은 반드시 아래의 `env -u ...` 명령으로 ROS 환경변수를 제거하고 실행해야 함 (아래 "Isaac Sim 실행" 참고)

**통신은 어떻게 되나:** 두 프로세스는 파이썬을 공유하지 않고, 네트워크 계층(DDS)의 ROS 2 토픽/서비스로만 데이터를 주고받습니다. 양쪽 모두 각자의 rclpy를 씁니다:
- **py3.10 세계**: 시스템 ROS 2 Humble의 rclpy
- **py3.11 세계(Isaac Sim)**: `isaacsim.ros2.bridge` 확장에 **번들된 py3.11용 humble rclpy** — 시스템 rclpy는 파이썬 버전이 달라 import 자체가 불가능하고, NVIDIA가 py3.11로 미리 빌드한 사본이 확장 폴더 안에 동봉되어 있습니다.

> **⚠️ ROS2 Bridge 확장을 반드시 켜야 합니다.** 번들 rclpy는 확장의 시동 코드가 `sys.path`에 등록해 줘야 import가 가능합니다. 확장이 꺼진 채 Script Editor에서 브릿지를 실행하면 `ModuleNotFoundError: No module named 'rclpy'`가 납니다.
> Window → Extensions → "ROS2 Bridge" 검색 → **ENABLED + AUTOLOAD** 체크. 콘솔에 `rclpy loaded`가 찍히면 정상입니다.
> 내장 humble도 시스템 Humble과 같은 배포판·같은 기본 RMW(FastDDS)라 토픽 상호운용에 문제가 없습니다.

| 무엇을 | 어디에 설치 |
|---|---|
| Isaac Sim | venv (Python 3.11) |
| ROS 2 Humble, colcon | 시스템 (apt) |
| cuRobo, PyTorch (플래닝 노드용) | 시스템 Python 3.10 |
| `strawberry_motion` 등 ROS 패키지 | `colcon build` → 이 프로젝트의 `install/` 폴더 |

### 왜 브릿지가 두 개인가 — 언어의 경계와 세계의 경계

이 파이프라인에는 이름이 비슷한 브릿지가 두 개 있습니다. 서로 **다른 종류의 경계**를 담당하기 때문에 둘 다 필요합니다.

```
[플래닝 노드들] ──dsr_msgs2 서비스──▶ [sim_executor_bridge_node] ──/joint_command──▶ [Script Editor 브릿지] ──▶ 가상 로봇 관절
 py3.10, 이식 구간   "두산 제어기 언어"     py3.10, 가짜 제어기          표준 메시지        py3.11, Isaac Sim 내부
```

**경계 1 — 언어의 경계: `sim_executor_bridge_node` (가짜 두산 제어기)**

모션 플래닝 노드들은 실제 두산 로봇용으로 작성되어, 두산 제어기 전용 서비스(`MoveJoint`, `MoveLine`, `MoveSplineJoint`, 그리퍼 서비스)를 호출합니다. 시뮬레이션에는 두산 제어기가 없으므로, 이 노드가 해당 서비스 서버를 대신 열어 **제어기인 척 응답하는 대역** 역할을 합니다. 받은 명령은 범용 `/joint_command` 토픽(표준 `JointState`)으로 번역해 발행하고, 데카르트 명령인 `MoveLine`을 관절 각도로 바꾸기 위해 cuRobo IK를 내장하고 있습니다. 이 덕분에 플래닝 노드는 **실기용 인터페이스 계약(서비스·토픽·액션)을 그대로 지킨 채** 시뮬에서 동작하며, 실기 전환 시에는 이 노드만 빼면 진짜 두산 드라이버가 같은 서비스를 제공합니다. (이식 과정에서 플래너에 가한 수정은 `PLANNER_CHANGES.md` 에 항목별로 남겼고, 실기 기본 동작은 파라미터 기본값으로 보존했습니다.)

**경계 2 — 세계의 경계: Script Editor 브릿지 (시뮬레이터 안의 손발)**

`/joint_command`가 발행되어도 ROS 토픽이 저절로 시뮬 속 로봇을 움직이지는 못합니다. 가상 로봇의 관절(articulation)과 딸기 prim은 **Isaac Sim 프로세스 내부의 물리 세계**에 있고, 이를 만질 수 있는 것은 그 프로세스 안에서 실행되는 코드뿐입니다. 이 스크립트는 Isaac Sim 안에 상주하면서 `/joint_command`를 받아 관절에 적용하고, 시뮬 로봇의 관절 상태를 `/dsr01/joint_states`로, 딸기 prim 좌표를 `/isaac_sim/strawberries`로 발행합니다. 실기에서 모터와 카메라가 하던 물리적 입출력을 시뮬 세계 안에서 대신하는 존재입니다.

**하나로 합치지 않는 이유:** Script Editor 브릿지가 두산 서비스까지 직접 받으려면 `dsr_msgs2` 커스텀 메시지와 cuRobo+PyTorch를 전부 py3.11용으로 다시 빌드해 Isaac Sim 안에 넣어야 합니다. 지금 구조에서는 Isaac Sim 쪽 브릿지가 **표준 메시지(`JointState`, `PoseArray`)만** 사용하므로 확장에 번들된 rclpy만으로 충분하고, 무겁고 까다로운 의존성은 전부 py3.10 세계에 남아 있습니다.

---

## 사전 준비

### ROS 2 Humble 설치

Ubuntu 22.04 + Python 3.10 기준 ROS 2 Humble이 필요합니다. colcon 빌드 및 노드 실행의 전제 조건입니다.

```bash
# ROS 2 공식 apt 저장소 등록 후
sudo apt install ros-humble-desktop python3-colcon-common-extensions
```

> **⚠️ 워크스페이스 경로에 한글이 있으면 안 됩니다.** rosidl 메시지 생성기가 비ASCII 경로를 처리하지 못해 인터페이스 패키지 빌드가 실패합니다. 이 프로젝트를 `~/바탕화면`에서 `~/strawberry_grasp_environment`로 이전한 이유입니다.

### 외부 패키지 클론

```bash
cd src/
git clone https://github.com/doosan-robotics/doosan-robot2.git doosan_robot2
git clone https://github.com/ROBOTIS-GIT/RH-P12-RN.git RH-P12-RN
```

### colcon 빌드

```bash
cd ~/strawberry_grasp_environment
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
    strawberry_motion strawberry_sim_core e0509_gripper_description dsr_gripper_tcp_interfaces
source install/setup.bash
```

### cuRobo 설치 (이 PC: RTX 4060 Ti 기준)

RTX 4060 Ti(Ada, sm_89)는 일반 PyTorch 배포판이 지원합니다. cuRobo v0.7.8 소스 빌드에는 nvcc(CUDA 툴킷)가 필요합니다.

```bash
# CUDA 툴킷 (nvcc) 설치
sudo apt-get install -y cuda-toolkit-12-4

# PyTorch (cu124)
python3 -m pip install torch==2.6.0 torchvision \
    --index-url https://download.pytorch.org/whl/cu124

# cuRobo v0.7.8 소스 빌드 (원본 플래너 노드가 의존하는 구버전)
cd ~
git clone https://github.com/NVlabs/curobo.git
cd curobo && git reset --hard v0.7.8
export CUDA_HOME=/usr/local/cuda-12.4
python3 -m pip install -e . --no-build-isolation

# warp-lang 버전 고정 (cuRobo 설치 후에 실행할 것)
python3 -m pip install "warp-lang==1.9.1"
```

> **⚠️ warp-lang은 1.9.1로 고정.** cuRobo 0.7.8은 warp-lang 1.10+와 호환되지 않습니다
> (`wp.torch` 지연 로딩 변경으로 MotionGen 생성 실패). 상세: [`docs/next_steps.md`](docs/next_steps.md) Phase 3.

> 실기팀 PC(RTX 5080, sm_120)에서는 CUDA 12.8 + 별도 절차가 필요했으나, 이 PC에는 해당 없음.

### Isaac Sim 실행 (ROS 2 환경변수 분리 + 내장 ROS 라이브러리 지정)

Isaac Sim은 `~/.venv`(Python 3.11)에 pip으로 설치되어 있어 `isaacsim` 명령으로 실행합니다.
시스템 ROS(py3.10) 환경변수는 제거하되, ROS2 Bridge 확장이 쓸 **내장 ROS 라이브러리 경로는 지정**해야 합니다:

```bash
source ~/.venv/bin/activate
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u ROS_VERSION -u ROS_PYTHON_VERSION \
    ROS_DISTRO=humble \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    LD_LIBRARY_PATH="$HOME/.venv/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib" \
    isaacsim
```

> **⚠️ 세 변수(`ROS_DISTRO`/`RMW_IMPLEMENTATION`/`LD_LIBRARY_PATH`)를 지정하지 않으면
> ROS2 Bridge 확장이 켜지지 않습니다.** 확장은 활성화 직후 ROS 라이브러리 로드 자가진단을 하고,
> 실패하면 스스로를 다시 비활성화합니다 — Extensions 창에서 **토글을 켜도 곧바로 꺼진 상태로
> 되돌아오는 증상**이 바로 이것입니다. `LD_LIBRARY_PATH`는 시스템 ROS 경로 대신 확장에 번들된
> humble 라이브러리 폴더만 가리키게 덮어씁니다 (py3.10 시스템 라이브러리 혼입 방지).
