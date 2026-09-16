# 이후 작업 명세

> ⚠️ **담당: 2026-07~08 세션의 작업 명세 (이력 문서).**
> **제출 범위·작업 순서는 [`../SUBMISSION_PLAN.md`](../SUBMISSION_PLAN.md)(2026-09-10) 가 기준**이고,
> 마일스톤 이력은 [`../PORTFOLIO_SPRINT.md`](../PORTFOLIO_SPRINT.md),
> 수치는 [`parameters.md`](parameters.md), 실행 절차는 [`run_guide.md`](run_guide.md) 가 기준이다.
> 아래의 설치 절차는 [`../README.md`](../README.md) 와 중복이며 README 쪽이 최신이다.

현재 완료된 작업과 남은 작업을 단계별로 정리합니다.

> **📌 우선순위 공지 (2026-08-29):** 프로젝트가 **취업 지원용 포트폴리오 제출 스프린트** 체제로 전환됨.
> 작업 우선순위·범위·완료 판정은 이제 리포 루트의 [`PORTFOLIO_SPRINT.md`](../PORTFOLIO_SPRINT.md)가 결정한다
> (M1: 3차 통합 테스트 → M2: 접근 오차 해소 → M3: 포트폴리오 산출물 → 제출. M4 항목은 제출 전 착수 금지).
> 이 문서는 **작업 기록·기술 상세·실행 절차의 원본**으로 계속 유지하며, 아래 Phase 구분과 스프린트
> 마일스톤의 대응 관계는 각 절에 `[M#]` 표기로 명시했다.

---

## 현재 상태 (2026-09-07 기준)

> 아래 상세 표는 2026-08-29 시점의 스냅샷이다. 그 뒤 바뀐 것만 먼저 적는다.
>
> **2026-09-10 주의**: 아래 표의 "672.0mm / 오차 0.0mm" 는 09-07 시점 기록이다. 09-09 에 보드 기준 y 가 **810.0mm**(원본 씬 실측값)로
> 복원되면서 "0.0mm" 표현은 폐기했다 — 현재 값은 [`parameters.md`](parameters.md), 현황은 [`../PROGRESS_REPORT.md`](../PROGRESS_REPORT.md).

| 항목 | 상태 |
|---|---|
| 접근 오차 132mm (M2) | ✅ **해소** — 씬을 실기 벽 실측값(672.0mm)에 정합, 오차 0.0mm. `clamped to 672mm` WARN 소멸 확인 (`log/m2_5/`) |
| 파지 판정 (M2.5) | ✅ **해소** — 가상 제어기가 TCP↔딸기 거리로 판정. `GRASP_CONTACT_DETECTED` |
| 배치(place) 완주 (M2.5) | ✅ **해소** — taught slot0 경로로 release까지. 익은 딸기 6개 시도 → **4개 완주** |
| 딸기·보드 관통 | 🔶 원인 3건 규명·수정 (툴 길이 모델 73mm 부족 / 커스텀 파츠 충돌 모델 부재 / cuRobo 충돌 월드 공백). **실행 검증 대기** |
| 씬 딸기 개수 | 2개 → **12개** (익은 6 / 안 익은 6), 보드 4등분 서브셀 배치 |
| 스캔 재실행 | ✅ 가능 (`_started` 래치 해제) |
| 상태 표시 창 | ✅ 신규 `status_monitor_node` (읽기 전용) |
| 포트폴리오 산출물 | 🔶 B·C·D·E 완료 (`portfolio/`), **A(시연 영상)만 남음** — 명세는 [`../SUBMISSION_PLAN.md`](../SUBMISSION_PLAN.md) §3 (클립 5종 A-1~A-5, 카메라 2대, 런 3회) |
| 플래너 작업 원칙 | v1(무수정) → **v2(수정 허용 + 한 줄 로그)** 로 전환 (`PLANNER_POLICY_v2.md`) |

---

## 상태 스냅샷 (2026-08-29 기준 — 마지막 개발 세션 2026-07-16)

| 항목 | 상태 |
|---|---|
| Isaac Sim USD 씬 (`strawberry_harvest/`) | ✅ 완료 |
| 로봇 URDF 빌드 (`robot.urdf`) | ✅ 완료 |
| `strawberry_motion` ROS 2 패키지 구조 | ✅ 완료 (파일 배치) |
| `scan_executor_node`, `scan_safety` | ✅ 배치 완료 |
| `curobo_planner_node` 및 지원 모듈 | ✅ 배치 완료 |
| `strawberry_fusion_node` | ✅ 배치 완료 |
| 가상 브릿지 노드 (`sim_executor_bridge_node`) | ✅ 이식 완료 (`src/strawberry_sim_core/`) |
| 가짜 비전 노드 (`fake_vision_node`) | ✅ 이식 완료 (`src/strawberry_sim_core/`) |
| Isaac Sim 스크립트 에디터 브릿지 | ✅ 이식 완료 (`strawberry_harvest/scripts/`) |
| `dsr_gripper_tcp_interfaces` 패키지 | ✅ 이식 완료 (`src/dsr_gripper_tcp_interfaces/`) |
| cuRobo 설정 파일 (`config/curobo/`) | ✅ 이식 완료 (`src/e0509_gripper_description/` + `src/strawberry_motion/config/curobo/`) |
| 스캔 포즈 YAML (`scan_pose_candidates.yaml`) | ✅ 이식 완료 (`src/strawberry_motion/config/`) |
| ROS 2 Humble 설치 (이 PC) | ✅ 설치 완료 (2026-07-09) |
| 프로젝트 ASCII 경로 이전 (`~/strawberry_grasp_environment`) | ✅ 완료 — rosidl 한글 경로 버그 해결 |
| colcon 빌드 (5개 패키지) | ✅ 성공 |
| 패키지 import·share·실행 검증 | ✅ 통과 |
| PyTorch + cuRobo 설치 (시스템 Python 3.10) | ✅ 완료 — CUDA 12.4 + PyTorch 2.6.0+cu124 + cuRobo v0.7.8 + warp-lang 1.9.1 고정 |
| e0509 설정 실전 IK 검증 | ✅ 통과 (왕복 IK + 배치 IK 5/5) |
| 플래닝 노드 2종 기동 확인 | ✅ 통과 (planner Ready / scan_executor ready) |
| Isaac Sim GUI 연동 (ROS2 Bridge 확장 + Script Editor 브릿지 + Play) | ✅ 완료 (2026-07-09 저녁) |
| Articulation 초기화 (D455 중첩 강체 수정) | ✅ 해결 — `robot_assembly.usd` 오버라이드 |
| 4개 노드 + Isaac Sim 동시 기동, 노드 간 배선 | ✅ 확인 — planner→bridge 그리퍼 서비스 응답까지 동작 |
| **전체 파이프라인 통합 테스트 (scan/start 트리거)** | ✅ **1·2차 완주** (2026-07-10 / 07-16) — `root/nw` 셀 END-TO-END `SCAN_COMPLETE` 도달 |
| 브릿지 MoveLine 버그 3종 수정 (flatten/TOOL 좌표계/fail-open) | ✅ 2차 테스트에서 해소 확인 (2026-07-16) — MoveLine 2회 Exception 없이 실행 |
| 로봇 관절 출렁임·떨림 (드라이브 게인) | ✅ 효과 확인 (2026-07-16) — AT_SCAN_POSE 관절 오차 0.2° → 0.0°(로그 소수1자리 기준)로 급감 |
| 브릿지 MoveLine **IK elbow-flip 다이브** (2차에서 신규 발견) | ✅ 원인 규명 + 가드 수정 + 60회 IK 검증 (2026-07-16) — **브릿지 노드 재시작 필요** |
| **3차 통합 테스트 (다이브 해소 확인) = 스프린트 M1** | ✅ **완료 (2026-08-29, 2회 실행 모두 SCAN_COMPLETE)** — elbow-flip 다이브 0건, 가드 양 경로(실행/거부) 실전 검증, 로그 `log/m1/` 보존, 런 B 화면 녹화 확보. 상세는 Phase 4 "3차 결과" |
| 접근 오차 132mm (벽 캘리브레이션 불일치) | ⏳ **다음 작업 = 스프린트 M2** (Phase 5 → M2 승격, 1~2일 예상) |
| 포트폴리오 산출물 A~E (영상·다이어그램·Before/After·지표) | ⏳ 스프린트 M3 — `PORTFOLIO_SPRINT.md` §1 참고 |
| 잔여 이슈 (J1/J2 swing, 그리퍼 파지 판정, TCP 오프셋 등) | ⏸ **M4 — 제출 전 착수 금지** (기록은 Phase 5 표에 보존) |

---

## 단계별 작업 명세

### Phase 1 — 시뮬 브릿지 이식 ✅ 완료 (2026-07-09)

`strawberry_grasp_isaac`의 가상 제어기 계층 이식 완료.

| 항목 | 배치 위치 | 비고 |
|---|---|---|
| `fake_vision_node`, `sim_executor_bridge_node` | `src/strawberry_sim_core/` | 원본 그대로 |
| `dsr_gripper_tcp_interfaces` | `src/dsr_gripper_tcp_interfaces/` | 원본 그대로 |
| `isaac_sim_script_editor_bridge.py` | `strawberry_harvest/scripts/` | 로봇 prim 경로 수정: `/World/robot_recent/strawberry_grasp_robot` → `/World/robot_assembly` (신규 씬 구조 반영) |
| `self_collision_logger_script.py` | `strawberry_harvest/scripts/` | `LOG_DIR` 하드코딩(`/home/sun/...`) → 이 프로젝트 `log/` 폴더로 수정 |

**Phase 4 테스트 시 확인 필요:**
- [x] `/World/robot_assembly` prim에서 Articulation 초기화 (2026-07-09 GUI 테스트에서 **실패 → 수정**):
  - 증상: `Rigid Body of (/World/robot_assembly/rh_p12_rn_base/rsd455/RSD455) missing xformstack reset when child of another enabled rigid body` + `Articulation needs to be initialized`
  - 원인: NVIDIA 순정 D455 애셋(`rsd455.usd`)은 독립 강체(RigidBodyAPI)로 설계됨 → 그리퍼 링크 밑에 조립하면서 "강체 안의 강체"가 되어 PhysX가 거부, articulation 텐서 초기화 실패
  - 수정: `robot_assembly.usd`에서 해당 prim에 `physics:rigidBodyEnabled=False` 오버라이드 (순정 애셋 원본은 미수정 — 조립이 일어난 assembly 레이어에서 오버라이드). ArticulationRootAPI 위치는 `/robot_assembly/root_joint`로 정상
- [x] 브릿지의 딸기 탐색 로직 — 씬에 `/World/strawberry_ripe`, `/World/strawberry_unripe` 존재, 탐색 조건(이름에 "strawberry" 포함 + "robot" 제외)과 일치함을 USD 검사로 확인 (2026-07-09)

**알려진 무해 경고 (Isaac Sim 콘솔):**
- `Unresolved reference ... robot_physics.usd@</visuals/world>` — visuals 참조 경로 어긋남. 물리와 무관, 로봇 외관이 온전하면 무시 가능 (외관 누락 시 추후 수정)
- `omni.isaac.* has been deprecated in favor of isaacsim.*` — 구 API 이름 경고일 뿐 동작함

---

### Phase 2 — cuRobo 설정 이식 ✅ 완료 (2026-07-09)

**배치 결과:**

| 파일 | 배치 위치 | 비고 |
|---|---|---|
| `e0509_gripper.yml` / `.urdf` / `e0509_spheres.yml` | `src/e0509_gripper_description/config/curobo/` + `src/strawberry_motion/config/curobo/` (이중 배치) | 검증된 legacy 160mm 모델 |
| `e0509_gripper_measured_tcp.yml` | 〃 | ⚠️ 실기팀 TCP 오프셋 확인 필요 |
| `environment.yaml` | 〃 (각 config/) | 테이블 더미 큐브 정의 |
| `scan_pose_candidates_refit_candidate.yaml` | `src/strawberry_motion/config/` | ⚠️ 이 씬의 딸기 위치 기준 재검토 필요 |
| `scan_collision_world.yaml` | `src/strawberry_motion/config/` | ⚠️ 현재 씬 prop 위치 기준 수치 검토 필요 |

**이중 배치 이유:** `planner_bootstrap.py`는 `src/strawberry_motion/config/curobo/`(소스 트리)를 1순위로 찾고, `sim_executor_bridge_node`는 `e0509_gripper_description` 패키지 share에서만 찾습니다. 두 경로 모두 지원하기 위해 데이터 전용 패키지 `src/e0509_gripper_description/`을 신설했습니다 (colcon build 시 share로 설치됨).

**하드코딩 경로 일괄 수정 (사용자 승인, 2026-07-09):**

실기팀 PC 경로가 하드코딩되어 있어 이 프로젝트 구조로 재지정했습니다. 모션 로직은 건드리지 않고 경로 문자열만 변경.

| 대상 | 변경 내용 |
|---|---|
| `scripts/` 내 18개 파일 | `~/doosan_ws/src/e0509_gripper_description` → `~/strawberry_grasp_environment/src/e0509_gripper_description` (설정 읽기 + 로그 쓰기 경로) |
| `execution/scan_executor_node.py` | `_CUROBO_DIR`: `/home/user/doosan_ws/...` → 이 프로젝트의 `src/e0509_gripper_description/config/curobo` 절대경로 |

**수정하지 않은 하드코딩 (실기 전용, 시뮬 무관):**
- `strawberry_yolo_node.py`의 `~/Downloads/share_yolo/...` (YOLO 가중치) — 시뮬은 fake_vision_node가 대체
- `object_tracking_node.py`의 `~/GroundingDINO/...`, `~/models/...` (외부 모델)
- `harvest_motion_params.py`의 `~/Downloads/share_tray/...` (트레이 셀 JSON)
- 안내 문구 속 `source ~/doosan_ws/install/setup.bash` (dsr_msgs2는 실제 두산 워크스페이스 필요)

---

### Phase 3 — 패키지 빌드 및 검증 ✅ 완료 (2026-07-09, 소프트웨어 스택 + 노드 기동 검증까지 전부 완료)

**수행 내역:**

1. ROS 2 Humble + colcon 설치 (Ubuntu 22.04, 시스템 Python 3.10)
2. **프로젝트를 `~/바탕화면/` → `~/strawberry_grasp_environment`로 이전.**
   한글 경로에서 rosidl 메시지 생성기가 경로의 한글 부분을 잘라먹어 `dsr_gripper_tcp_interfaces` 빌드가 실패했음 (비ASCII 경로 버그). 이전 후 모든 경로 참조(노드 경로 상수, xacro include 절대경로, build_urdf.sh, robot.urdf, 문서)를 일괄 재조정.
3. 빌드 성공 (5개 패키지):

```bash
cd ~/strawberry_grasp_environment
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
    strawberry_motion strawberry_sim_core e0509_gripper_description dsr_gripper_tcp_interfaces
source install/setup.bash
```

**검증 체크리스트:**
- [x] `from strawberry_motion.execution.scan_safety import ...` 정상
- [x] `dsr_gripper_tcp_interfaces` srv/action/msg import 정상
- [x] `dsr_msgs2` 서비스 (MoveLine 등) import 정상
- [x] `e0509_gripper_description` share에서 cuRobo 설정 4종 조회 정상
- [x] `strawberry_motion` share에서 스캔 YAML 3종 조회 정상
- [x] `ros2 run strawberry_sim_core fake_vision_node / sim_executor_bridge_node` 실행 가능
- [x] `curobo_planner_node` / `scan_executor_node` 실행 — 두 노드 모두 정상 기동 확인 (2026-07-09)
  - `curobo_planner_node`: MotionGen warmup 완료 후 "cuRobo Planner Ready!" 도달
  - `scan_executor_node`: 스캔 타겟 4개 로드 후 "ready; explicit /strawberry/scan/start required" 도달
- [x] e0509 설정으로 실전 IK 연산 검증 (2026-07-09): FK(retract)→IK 왕복 오차 ~1µm, 근방 5개 포즈 배치 IK 5/5 성공

**PyTorch + cuRobo 설치** — 절차가 [`../README.md`](../README.md) 의 "cuRobo 설치" 절과 중복이라 그쪽으로 합쳤다 (2026-09-09). **README 쪽이 최신**이다. warp-lang 1.9.1 고정 이유도 거기 있다.
> (cuRobo 설치 시 warp 최신판이 딸려오므로, cuRobo 설치 **후에** 다운그레이드할 것.)
> setuptools 79.0.1 고정과 같은 이유의 "버전 핀" 항목입니다.

---

### Phase 4 — 전체 파이프라인 통합 테스트

**ROS 측 dry-run 완료 (2026-07-09, Isaac Sim 없이):**

4개 노드(fake_vision, sim_executor_bridge, curobo_planner, scan_executor)를 동시 기동하여 확인:
- [x] 4개 노드 동시 기동 후 전부 생존
- [x] `/strawberry/detection/pick_pose`, `/strawberry/detection/scene_positions` 토픽 발행 확인
- [x] `/strawberry/scan/start` 호출 → `execute_motion parameter is false`로 거부 (fail-closed 설계 정상 동작)
- [x] `sim_executor_bridge_node` cuRobo IK 초기화 성공

**dry-run에서 발견해 수정한 버그 (`sim_executor_bridge_node.py`):**

| 버그 | 수정 |
|---|---|
| `collision_spheres`가 상대경로라 cuRobo가 자기 content 폴더에서 찾다 실패 | `planner_bootstrap.py`와 동일하게 share의 `e0509_spheres.yml` 절대경로로 패치 |
| `IKSolverConfig.build_from_robot_config` — cuRobo 0.7.8에 없는 API | `load_from_robot_config`로 수정 |

**GUI 통합 테스트 진행 상황: 실행 순서 ①~⑥ 전부 통과 — 1차 END-TO-END 완주 (2026-07-10)**

- ① Isaac Sim 실행 (venv `isaacsim` + 환경변수 3종) ✅
- ② ROS2 Bridge 확장 활성화 (`rclpy loaded` 확인) ✅
- ③ main_scene.usd 로드 + Script Editor 브릿지 + Play ✅ — Articulation 초기화 실패를 D455 강체 오버라이드로 해결 (아래 Phase 1 체크 항목 참고)
- ④ fake_vision + sim_executor_bridge 기동 ✅ — `cuRobo IK Solver successfully initialized`
- ⑤ planner + scan_executor 기동 ✅ — planner가 브릿지의 그리퍼 서비스를 호출하고 응답받는 것까지 확인 (`SetPosition called`)
- ⑥ `/strawberry/scan/start` 트리거 ✅ — `target_cell:=root/nw`로 재기동 후 **`SCAN_COMPLETE`까지 완주**:
  스캔 포즈 이동 → 딸기 24개 탐지 → 서브셀 순회 → 첫 타겟 pre-approach 계획·실행 성공 →
  파지 시퀀스 → 재스캔 → overview 복귀. 뷰포트에서 접근·복귀 대동작 육안 확인됨.
- 참고: `&`로 띄운 백그라운드 노드는 Ctrl+C로 안 죽는다. 재시작 전 `ros2 node list`로 중복 확인, 필요시 `kill <PID>`
- 참고: 첫 트리거는 `target_cell parameter is required`로 거부됐었음 — `scan_executor_node`는
  첫 검증을 **셀 하나씩만** 허용하는 설계라 `target_cell` 파라미터 필수 (아래 게이트 표 5번)

**1차 통합 테스트에서 발견된 문제와 조치 (2026-07-10):**

| # | 문제 | 원인 | 조치 |
|---|---|---|---|
| 1 | `MoveLine Exception: must be real number, not list` — 최종 45mm 접근·detach pull 미실행 | 브릿지 `move_line_cb`가 cuRobo IK 해(`[1,6]` 텐서)를 flatten 없이 `math.degrees`에 전달 | ✅ `flatten()` 추가. 실제 cuRobo IK로 검증 완료 (해 FK 오차 0.003mm, planner 계산값과 관절각 일치) |
| 2 | 브릿지가 MoveLine 실패에도 `success=True` 반환 → planner가 `GRASP_POSE_REACHED` 오기록 | fail-open 구현 (예외를 잡고 무조건 성공 응답) | ✅ IK 실패·예외·미지원 mode 시 `success=False` 반환하도록 수정 |
| 3 | MoveLine이 TOOL 좌표계(ref=1) 무시 → 최종 접근이 베이스 +Z(위쪽)로 실행될 뻔 | "ref=0 가정" 구현 | ✅ ref=1이면 델타를 현재 EE 자세로 회전 후 적용. TOOL +Z 45mm → 베이스 [0,45,0]mm(벽 방향) 검증 완료 |
| 4 | 로봇 팔이 출렁이고(관성 진동) 이동 중 덜덜거림 | URDF 임포트 기본 드라이브 게인이 너무 무름 (stiffness 54~2648, damping ≈0 — 감쇠 사실상 없음) | ✅ `robot_assembly.usd` 오버라이드: 팔 stiffness=1e5/damping=1e4, 그리퍼 1e4/1e3 (`docs/usd_structure.md` §3 참고) |
| 5 | `Detection Y=804mm > wall surface 672mm — clamped` 경고, 그리퍼가 딸기 132mm 앞에서 정지 | fake_vision의 딸기 좌표와 planner의 벽 모델(실기 캘리브레이션 기준)이 불일치 | ⏳ 미해결 — 씬 좌표 기준으로 planner 벽 수치 재캘리브레이션 필요 (Phase 5로 이관) |
| 6 | 둘째 타겟 `IK_FAIL` + J2 swing 초과로 전 후보 reject → ABORT | known issue 재현: 딸기 위치가 대기 자세 기준 너무 꺾인 방향 | ⏳ 미해결 — Phase 5 참고 (딸기 위치 조정 또는 시작 자세 재세팅) |
| 7 | `VERIFY_GRASP: GRASP_EMPTY` → place 게이트 차단 | 시뮬 그리퍼가 순수 mock (명령값 그대로 반향) — position-only 검증으로는 시뮬에서 파지 성공 판정 불가 | ⏳ 설계 한계 — place까지 검증하려면 SafeGrasp 경로 사용 또는 mock 개선 (`docs/pipeline_overview.md` §4 참고) |

**수정 반영 방법 (2차 테스트 전 필수):**
- 브릿지 수정(#1~3): `sim_executor_bridge_node` 재시작 (symlink-install이라 재빌드 불필요)
- 게인 수정(#4): Isaac Sim에서 main_scene.usd **재로드** 후 Play (이미 열려 있으면 File → Revert 또는 재실행)

**2차 통합 테스트 결과 (2026-07-16):**

- ✅ MoveLine 2회(`FINAL_APPROACH` TOOL+Z 45mm, `DETACH_PULL` BASE−Z 40mm) **Exception 없이 실행** — #1~3 수정 유효
- ✅ 드라이브 게인 효과 확인 — 1차에서 `AT_SCAN_POSE joints_deg=[88.0 -95.1 ...]`처럼 목표 대비 0.2°가량 처지던 것이 2차에서는 소수 1자리까지 목표와 정확히 일치 (추종 강성 상승의 객관 증거)
- ❌ **신규 이슈 #8: 로봇이 딸기 앞(그래스프 자세)까지 갔다가 땅 밑으로 고꾸라짐**

| # | 문제 | 원인 | 조치 |
|---|---|---|---|
| 8 | MoveLine 실행 순간 팔 전체가 뒤집히며 바닥 관통 | **cuRobo IK의 다중해(elbow-flip)**: 같은 EE 목표에 대해 J1이 +176°/J2 팔꿈치 반전 등 먼 브랜치 해를 반환. 실측 재현: 동일 목표 30회 솔브 중 6회(20%)가 관절 이동 176~184°짜리 해, 드물게는 8개 seed 전부 먼 브랜치로 수렴 → 최근접 선택만으로는 불충분 | ✅ 브릿지 `_solve_ik_nearest` 추가 (2026-07-16): `return_seeds=8`로 받아 **현재 관절 최근접 해 선택 + 최대 관절 이동 45° 상한 가드 + 최대 5회 재시도**, 가드 실패 시 `success=False`. 검증: 두 델타 × 30회 = 60회 전부 통과, 채택 해의 최대 관절 이동 5.8°/10.6° |

**3차 테스트에서 확인할 것 `[M1]` (결과, 2026-08-29 — 2회 실행: 런 A 관찰용 / 런 B 화면 녹화):**
- [x] `MoveLine IK ok (max joint delta N.Ndeg)` 라인 확인 — 실행된 MoveLine 4건의 delta: **5.8° / 11.7°**(런 B 타겟1 접근·당겨빼기), **13.4° / 33.7°**(런 A 타겟2). 전부 45° 가드 이내 (33.7°는 예상 ~10°보다 크지만 상한 내)
- [x] 다이브 재발 없음 — 2회 실행 전 구간에서 팔 뒤집힘·바닥 관통 0건. 런 B에서 45mm 직선 진입을 육안·녹화로 확인
- [x] `MoveLine IK Failed! (near-branch 해 없음, best delta=176.1deg)` 발생 시 planner 반응 — 런 A 타겟1에서 실제 발생: planner가 `ABORT: 직선 진입 실패`로 픽을 정상 중단하고 **다음 타겟으로 계속 진행** (fail-closed 파급 확인 완료)
- [x] 4개 노드 실행 로그 보존 — `log/m1/` (fake_vision, sim_executor_bridge, curobo_planner, scan_executor 런 2회분 + 트리거 응답)
- [~] 출렁임·덜덜거림 — 런 B 육안 관찰에서 이상 보고 없음 (2차 로그 근거와 합치)

**3차 신규 관찰 (조치 없음, 기록만):**
- **cuRobo 계획 무작위성으로 런마다 결과가 다름**: 같은 씬·같은 타겟인데 pre-approach 종료 자세가 런마다 상이 (타겟1: 런 A J4=272°→접근 거부 / 런 B J4=92°→접근 성공). 45mm 접근 성공 여부가 종료 자세의 손목 꼬임에 좌우됨
- 런 A 타겟2는 성공했으나(delta 13.4°) 런 B 타겟2는 J1/J2 swing으로 24개 후보 전체 reject → ABORT.
  단, 런 A의 타겟2 성공은 ABORT 직후의 꼬인 자세에서 출발한 우연의 산물 — 정상 스캔 자세에서 출발한 런 B가 7/10 1차와 같은 조건이며 known issue가 그대로 재현된 것 (M4 항목)
- ABORT 후 중립 복귀 없이 다음 픽을 시작하는 동작 특성 확인 (런 A에서 타겟1 abort 자세 그대로 타겟2 계획 시작)
- 시뮬 그리퍼는 화면상 움직이지 않음 — 브릿지 mock이 명령값을 변수로만 저장, Isaac Sim 그리퍼 관절 미구동 (`docs/pipeline_overview.md` §4 경고 그대로. 시연 영상 품질 관점의 개선은 M4)

**⑥번 트리거의 거부 게이트 (start 콜백이 순서대로 검사, `scan_executor_node.py` `_start_cb`):**

| 순서 | 게이트 | 거부 메시지 | 해소 방법 |
|---|---|---|---|
| 1 | `execute_motion` 파라미터 | `execute_motion parameter is false` | 기동 시 `-p execute_motion:=true` |
| 2 | YAML 승인 플래그 | authorized 관련 | `scan_pose_candidates` YAML의 `use_for_automated_motion` (현재 통과 상태) |
| 3 | live joint state 수신 여부 | joint state 관련 | Isaac Sim **Play 상태** + 브릿지가 `/dsr01/joint_states` 발행 중이어야 함 |
| 4 | 중복 시작 | `scan already started` | 노드 재기동 |
| 5 | `target_cell` 지정·허용 목록 | `target_cell parameter is required ...` | 기동 시 `-p target_cell:=all` (원안 4분면 순회). 단일 분면은 `root/nw` / `root/ne` / `root/se` / `root/sw` |
| 6 | overview 자세 일치 | START_REJECTED (joint 불일치) | 로봇을 overview 자세 `[87.98, -94.91, 129.9, 175.94, -31.34, 93.42]`(deg)로 이동 — 브릿지의 `/dsr01/motion/move_joint` 서비스 수동 호출 |

**실행 순서:**

> 📌 **최신 절차는 [`run_guide.md`](run_guide.md)를 보세요** — 터미널별 단일 명령으로 정리돼 있고,
> 상태창(`status_monitor_node`)과 재실행 방법까지 포함합니다. 아래는 당시 기록으로 남겨둡니다.

```bash
# 1. Isaac Sim 실행 (venv의 isaacsim 명령 사용, README "Isaac Sim 실행" 참고)
source ~/.venv/bin/activate
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u ROS_VERSION -u ROS_PYTHON_VERSION \
    ROS_DISTRO=humble \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    LD_LIBRARY_PATH="$HOME/.venv/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib" \
    isaacsim
#    ⚠️ 세 변수 미지정 시 ROS2 Bridge 확장이 자가진단 실패로 스스로 꺼짐 (토글이 되돌아오는 증상)

# 2. ROS2 Bridge 확장 활성화 (최초 1회, AUTOLOAD 체크 권장)
#    Window → Extensions → "ROS2 Bridge" 검색 → isaacsim.ros2.bridge ENABLED + AUTOLOAD
#    콘솔에 "Using backup internal ROS2 humble distro" + "rclpy loaded" 확인.
#    ⚠️ 이 확장이 꺼져 있으면 Script Editor에서 import rclpy가
#       ModuleNotFoundError로 실패한다 (Isaac Sim은 py3.11이라 시스템 rclpy를 못 쓰고,
#       확장에 번들된 py3.11용 rclpy가 확장 기동 시 sys.path에 등록되는 구조).
#    내장 humble도 시스템 Humble과 같은 배포판/기본 RMW(FastDDS)라 토픽 상호운용에 문제 없음.

# 3. main_scene.usd 로드 후 Script Editor에서 브릿지 실행 → Play 클릭

# 4. 가상 브릿지 노드 실행 (⚠️ 새 터미널마다 두 source 필수 — 안 하면 ModuleNotFoundError: rclpy)
cd ~/strawberry_grasp_environment
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run strawberry_sim_core fake_vision_node &
ros2 run strawberry_sim_core sim_executor_bridge_node &

# 5. 모션 플래닝 노드 실행 (같은 터미널이면 source 이미 됨)
cd src/strawberry_motion/scripts
python3 curobo_planner_node.py --ros-args -p tool_model_profile:=legacy_160mm \
    -p enable_marker_place_sequence:=true -p use_taught_slot0_place_reference:=true \
    -p execute_marker_place_release:=true -p hold_after_taught_slot0_place:=false \
    -p taught_slot_sequence:="0,0,0,0,0,0" &
#    ⚠️ 뒤쪽 파라미터들은 배치(place)까지 완주시키는 스위치. 하나라도 빠지면 그 지점에서 멈춘다
#       (설명: docs/run_guide.md T2 절). 파지까지만 볼 때는 tool_model_profile만 남긴다
python3 -m strawberry_motion.execution.scan_executor_node \
    --ros-args -p execute_motion:=true -p target_cell:=all &
#    ⚠️ target_cell 필수! 없으면 ⑥에서 "target_cell parameter is required"로 거부 (2026-07-10 확인)

# 6. 파이프라인 시작 트리거 (Isaac Sim이 Play 상태인지 먼저 확인)
ros2 service call /strawberry/scan/start std_srvs/srv/Trigger
```

> ⚠️ `scan_executor_node`는 fail-closed 설계입니다. 위 "거부 게이트" 표의 6단계를 전부
> 통과해야 스캔이 시작됩니다. START_REJECTED가 나오면 응답 message와 노드 로그의
> 거부 사유를 표에서 찾아 순서대로 해소하면 됩니다.

**⑥ 트리거 후 성공 판정 — 확인할 곳 4군데:**

1. 서비스 콜 응답: `success=True` + 수락 메시지 (거부 시 message에 사유가 그대로 나옴)
2. `scan_executor_node` 로그: `SINGLE_CELL_SCAN_STARTED target=root/nw` → 스캔 포즈 이동 → 탐지 수집 순으로 진행
3. `curobo_planner_node` 로그: 플래닝 요청 수신 → trajectory SUCCESS (실패 시 J1/J2 swing 초과, `INVALID_START_STATE_WORLD_COLLISION` 등 Phase 5 known issue 확인)
4. `sim_executor_bridge_node` 로그 + Isaac Sim 뷰포트: MoveJoint/MoveLine 서비스 호출 수신, 로봇 팔이 실제로 움직이는지 육안 확인

보조 관찰 (별도 터미널, source 후):
```bash
ros2 topic echo /strawberry/scan/status              # 스캔 상태 머신 진행 상황
ros2 topic echo /strawberry/detection/pick_pose      # fake_vision 딸기 좌표 발행 확인
```

**최종 검증 체크리스트 (1차 결과, 2026-07-10):**
- [x] `/strawberry/scan/start` → `success=True` (6개 게이트 전부 통과)
- [x] 로봇이 스캔 포즈로 이동 (`SINGLE_CELL_SCAN_STARTED` + 뷰포트에서 대동작 확인. 단, 초기 자세=스캔 포즈라 첫 이동은 미관측 — 2차에서 초기 자세를 다르게 두면 명확해짐)
- [x] Isaac Sim → `fake_vision_node` → `/strawberry/detection/pick_pose` 정상 발행 (24개 후보 탐지)
- [x] `scan_executor_node` → `curobo_planner_node` 타겟 전달 (`PICK_TRIGGER` → `=== PICK ===`)
- [x] cuRobo pre-approach 계획 성공 (첫 타겟 `Plan OK` 2회 + 스플라인 실행)
- [ ] `sim_executor_bridge_node` → Isaac Sim 로봇이 파지 동작 수행 — ⚠️ MoveLine 버그로 최종 45mm 접근 미실행 (수정 완료, **2차에서 재확인**)
- [ ] J1/J2 swing 과다 오류 없이 파지 위치 도달 — ⚠️ 첫 타겟은 reject 2건 후 통과, 둘째 타겟은 ABORT (known issue, Phase 5)

**2차 테스트에서 추가 확인할 것 (결과, 2026-07-16):**
- [x] MoveLine 2회(`FINAL_APPROACH`, `DETACH_PULL`)가 Exception 없이 실행 — 단, IK elbow-flip 다이브 발생 (신규 이슈 #8, 아래 2차 결과 참고)
- [x] 관절 추종 강성 상승 확인 — AT_SCAN_POSE 오차 0.2° → 0.0° (육안 확인은 3차에서)
- [ ] MoveLine 실패 시 planner의 반응 — 2차에서는 실패 케이스 미발생, 3차 항목으로 이월

---

### Phase 5 — 알려진 미해결 문제

`strawberry_grasp_isaac`에서 이관된 이슈 + 1차 통합 테스트(2026-07-10)에서 발견된 이슈입니다.

> **스프린트 대응 (2026-08-29):** 아래 표에서 **벽 좌표 캘리브레이션 불일치만 `[M2]`로 승격**되어
> 제출 전 해결 대상이다. 나머지는 전부 `[M4]` — **제출 전 착수 금지** (`PORTFOLIO_SPRINT.md` §2 M4).

| 이슈 | 스프린트 | 설명 | 해결 방안 |
|---|---|---|---|
| 벽 좌표 캘리브레이션 불일치 (신규 2026-07-10) | **`[M2]` 제출 전 해결** | fake_vision의 딸기 Y=786~804mm vs planner 벽 모델 672mm → 132mm 클램프되어 그리퍼가 딸기 앞에서 정지 | planner의 wall surface 수치(실기 캘리브레이션 잔재)를 이 씬의 화이트보드 실좌표 기준으로 재설정. **전후 오차 수치를 반드시 기록** (포트폴리오 산출물 C·E) |
| J1/J2 swing 초과 reject | `[M4]` 착수 금지 | 딸기 좌표가 대기 자세 기준 너무 꺾인 방향에 위치해 trajectory 안전 검사 탈락. **1차 통합 테스트에서 재현됨** (둘째 타겟 전 후보 reject → ABORT) | ① Isaac Sim에서 딸기 위치를 정면으로 이동, ② scan_pose_candidates.yaml의 시작 자세 재세팅 |
| TCP 오프셋 100mm 오차 | `[M4]` 착수 금지 | measured TCP 프로필의 100mm 오프셋이 실기와 일치하는지 미확인 | 실기팀과 `e0509_gripper_measured_tcp.yml` 수치 동기화 |
| 테이블 충돌 오판 | `[M4]` 착수 금지 | measured_tcp 프로필 사용 시 `INVALID_START_STATE_WORLD_COLLISION` 오판 | `config/curobo/environment.yaml` 더미 큐브 우회 (이미 적용됨, 이식 시 포함) |
| 시뮬 그리퍼 파지 판정 불가 (신규 2026-07-10) | `[M4]` 착수 금지 | 브릿지 그리퍼 mock이 명령값을 그대로 반향 → position-only 검증은 항상 GRASP_EMPTY → place 단계 진입 불가 | SafeGrasp 액션 경로 사용(접촉 mock 670 반환) 또는 mock을 딸기 접촉 감지형으로 개선 |

---

## 이전 리포지토리 처리

`strawberry_grasp_isaac`는 참조/아카이브 용도로만 유지합니다. 새 기능 개발 및 버그 수정은 이 프로젝트(`strawberry_grasp_environment`)에서만 진행합니다.
