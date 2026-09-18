# 전체 파이프라인 개요

> **담당: 노드·토픽 구조와 데이터 흐름.**
> 실행 명령은 [`run_guide.md`](run_guide.md), 파라미터 값은 [`parameters.md`](parameters.md) 가 기준이다.
> 이 문서의 예시 명령에 적힌 숫자는 설명용이므로 실제 실행은 run_guide 를 따를 것.

> 실행 절차는 [`run_guide.md`](run_guide.md) 참고 (터미널별 단일 명령).

딸기 수확 시뮬레이션의 전체 데이터 흐름과 노드 구성을 설명합니다.

---

## 1. 전체 파이프라인

```
[Isaac Sim 물리 엔진]
    |  딸기 3D 좌표 (ROS 2 토픽)
    v
strawberry_fusion_node          ← Realsense RGB-D + YOLO seg/pose 융합
    |  /strawberry/detection/pick_pose     (PoseStamped)
    |  /strawberry/detection/scene_positions  (Float64MultiArray)
    v
scan_executor_node              ← 스캔 포즈 이동 + 타겟 큐 관리
    |  /dsr01/curobo/pick_pose  (PoseStamped)
    v
curobo_planner_node             ← cuRobo pre-approach + pick 시퀀스
    |  Doosan motion service 호출 (MoveSplineJoint / MoveLine / SafeGrasp)
    v
[두산 로봇 제어기 or 가상 제어기]
    |  /joint_command
    v
[Isaac Sim 물리 엔진 — 로봇 관절 구동]
```

---

## 2. 노드 설명

### `strawberry_fusion_node`
- **파일**: `src/strawberry_motion/scripts/strawberry_fusion_node.py`
- **역할**: Realsense RGB-D 스트림에 YOLO seg(익음도) + YOLO pose(줄기 키포인트)를 결합하여 `base_link` 기준 안정화된 줄기 파지 타겟을 발행.
- **출력 토픽**:
  - `/strawberry/detection/pick_pose` — PoseStamped, ripe 딸기 1개씩
  - `/strawberry/detection/scene_positions` — Float64MultiArray, 주변 ripe 과실 중심 좌표

### `scan_executor_node`
- **파일**: `src/strawberry_motion/strawberry_motion/execution/scan_executor_node.py`
- **역할**: 스캔 포즈 순서대로 로봇을 이동시키고, fusion 결과를 수집하여 수확 순서 목록을 만든 뒤 planner에 1개씩 전달.
- **시작 트리거**:
  ```bash
  ros2 service call /strawberry/scan/start std_srvs/srv/Trigger
  ```
- **fail-closed 설계**: 노드를 `-p execute_motion:=true`로 기동하지 않으면 위 트리거가 거부된다.
  추가로 스캔 YAML의 검증 플래그, overview 자세와 일치하는 live joint state도 요구
  (거부 사유는 노드 로그의 START_REJECTED 메시지에 출력됨). 개념 설명: [`concepts.md`](concepts.md) §5.
- **사용 서비스**: `MoveJoint`, `MoveSplineJoint` (두산 로봇)

### `curobo_planner_node`
- **파일**: `src/strawberry_motion/scripts/curobo_planner_node.py`
- **실행**: 스크립트 직접 실행 방식 (flat import 구조)
  ```bash
  cd src/strawberry_motion/scripts
  python3 curobo_planner_node.py --ros-args -p tool_model_profile:=legacy_160mm \
      -p ee_to_tcp_offset_m:=0.236 \
      -p enable_marker_place_sequence:=true -p use_taught_slot0_place_reference:=true \
      -p execute_marker_place_release:=true -p hold_after_taught_slot0_place:=false \
      -p taught_slot_sequence:="0,0,0,0,0,0"
  ```
  `ee_to_tcp_offset_m:=0.236` 은 **시뮬에서 필수**다 (2026-09-08 에 0.208 에서 갱신 — 현재 값은 [`parameters.md`](parameters.md) 참조). 없으면 툴 길이를 짧게 보고
  딸기·보드를 관통한다. 뒤쪽 파라미터들은 **배치(place)까지 완주시키는 스위치**다.
  각각의 의미와 빠졌을 때 증상은 [`run_guide.md`](run_guide.md) T2 절 참조.
- **역할**: pick_pose 수신 → cuRobo MotionGen으로 pre-approach 계획 → MoveSplineJoint 실행 → straight approach(MoveLine) → SafeGrasp → DETACH → retreat → pick_complete 발행.
- **핵심 지원 모듈** (같은 scripts/ 디렉토리에 위치):

  | 파일 | 역할 |
  |---|---|
  | `planner_bootstrap.py` | cuRobo MotionGen 초기화, ROS 파라미터 로드 |
  | `curobo_planning_adapter.py` | cuRobo API 래퍼 |
  | `curobo_kinematics_adapter.py` | IK / FK 유틸 |
  | `doosan_motion_client.py` | 두산 motion service 클라이언트 |
  | `gripper_client.py` | 그리퍼 서비스/액션 클라이언트 |
  | `pick_sequence_executor.py` | 전체 pick 시퀀스 오케스트레이터 |
  | `final_approach_executor.py` | straight approach(MoveLine) 실행 |
  | `scene_obstacle_manager.py` | 주변 딸기를 cuRobo 장애물로 등록 |
  | `trajectory_guards.py` | 관절 swing 과다 / invalid start state 검증 |
  | `runtime_jsonl_logger.py` | 이벤트별 JSONL 로그 기록 |

---

## 3. ROS 2 인터페이스 요약

### 구독 (Subscriptions)

| 토픽 | 타입 | 발행자 |
|---|---|---|
| `/strawberry/detection/pick_pose` | PoseStamped | fusion_node |
| `/strawberry/detection/scene_positions` | Float64MultiArray | fusion_node |
| `/dsr01/joint_states` | JointState | 두산 드라이버 |
| `/dsr01/curobo/pick_complete` | Empty | curobo_planner (scan_executor 수신) |
| `/sim/grasp_event` | String (`ATTACH x y z` / `RELEASE`) | sim_executor_bridge → Isaac 브릿지 스크립트 (T2 키네마틱 부착, 시뮬 전용) |

### 서비스 클라이언트

| 서비스 | 타입 | 사용 노드 |
|---|---|---|
| `/dsr01/motion/move_joint` | `dsr_msgs2/srv/MoveJoint` | scan_executor, curobo_planner |
| `/dsr01/motion/move_spline_joint` | `dsr_msgs2/srv/MoveSplineJoint` | scan_executor, curobo_planner |
| `/dsr01/motion/move_line` | `dsr_msgs2/srv/MoveLine` | curobo_planner |
| `/dsr01/motion/change_operation_speed` | `dsr_msgs2/srv/ChangeOperationSpeed` | curobo_planner |
| `/gripper_service/set_position` | `dsr_gripper_tcp_interfaces/srv/SetPosition` | curobo_planner |

### 액션 클라이언트

| 액션 | 타입 | 사용 노드 |
|---|---|---|
| `/gripper_service/safe_grasp` | `dsr_gripper_tcp_interfaces/action/SafeGrasp` | curobo_planner |

---

## 4. 시뮬레이션 vs 실기 환경 차이

시뮬레이션에서는 실제 하드웨어 대신 **가상 브릿지**가 두산 제어기 서비스를 모방합니다. 이 구조 덕분에 `curobo_planner_node`와 `scan_executor_node` 코드는 실기/시뮬 구분 없이 동일하게 사용됩니다.

```
실기:  curobo_planner_node → 두산 드라이버 → 실제 로봇
시뮬:  curobo_planner_node → sim_executor_bridge_node → /joint_command → Isaac Sim
```

### 가상 브릿지 구성 요소

| 구성 요소 | 위치 | 역할 |
|---|---|---|
| `fake_vision_node` | `src/strawberry_sim_core/` | `/isaac_sim/strawberries`(PoseArray) 구독 → fusion_node와 동일 형식의 `/strawberry/detection/*` 토픽으로 변환·발행 |
| `sim_executor_bridge_node` | `src/strawberry_sim_core/` | 두산 motion 서비스(`MoveJoint`/`MoveLine`/`MoveSplineJoint`)와 그리퍼 액션을 가로채 `/joint_command` 발행. MoveLine은 cuRobo IK로 실제 계산 — BASE(ref=0)/TOOL(ref=1) 좌표계 지원, 상대 이동(mode=1)만, **현재 관절 최근접 해 선택 + 관절 이동 45° 상한 가드**(elbow-flip 다이브 방지, 2026-07-16). IK 실패·예외·가드 탈락 시 `success=False` 반환 |
| `isaac_sim_script_editor_bridge.py` | `strawberry_harvest/scripts/` | Isaac Sim Script Editor에서 실행. 딸기 prim 좌표를 `/isaac_sim/strawberries`로 발행하고, `/joint_command`를 받아 `/World/robot_assembly` articulation에 주입. **익은 딸기만 발행**(실기 fusion 노드가 ripe만 내보내는 것에 맞춤) — 씬에는 익은 6 / 안 익은 6, 총 12개가 있다. **[T2] `/sim/grasp_event` 를 받아 CONTACT 과실을 그리퍼 밑동에 키네마틱 부착·추종하고 RELEASE 에 정지시킨다. 부착·해제된 딸기는 발행에서 제외** |
| 뷰포트 표시 (`isaac_sim_viewport_display.py`, 09-16 까지 `isaac_sim_hud.py`) | `strawberry_harvest/scripts/` | Isaac Sim 뷰포트 위에 얹히는 상태 HUD 패널과 좌하단 그리퍼 카메라 창(D455 컬러 렌더 — 인식 결과 아님). 노드 램프·타겟 카운터·현재 단계·완주 결과를 표시. 노드 4개가 `/tmp/harvest_hud_<role>.json` 에 쓴 스냅샷을 읽기만 한다 — ROS 노드가 아니다 (2026-09-09, `status_monitor_node` 대체) |

### 그리퍼 파지 판정 — 기하 기반 (2026-09-07 변경)

초기 mock은 `SetPosition`으로 받은 명령값을 `GetState`가 그대로 되돌려줬다. 닫기 명령이
700이므로 planner의 position-only 검증(`>=695` = 빈손)은 **딸기를 실제로 잡았어도 항상
`GRASP_EMPTY`** 를 냈고, place 게이트가 늘 막혔다.

지금은 브릿지가 **TCP와 딸기 prim의 실제 거리**로 판정한다.

| 조건 | 리드백 | planner 판정 |
|---|---|---|
| `d_tcp <= grasp_capture_radius_m`(35mm) | 670 | `GRASP_CONTACT_DETECTED` |
| 그 밖 | 700 | `GRASP_EMPTY` |

덕분에 "접근이 틀리면 파지도 실패한다"가 시뮬에서도 참이 되어, 접근 정확도 개선이
파지 성공률로 드러난다. 판정 근거는 매 시도마다 브릿지 로그에 남는다.

```
GRASP_JUDGE d_tcp=14.8mm d_ee=174.8mm radius=35.0mm tool_offset=208.0mm -> CONTACT
```

> ⚠️ 브릿지의 `tool_tcp_offset_m` 은 planner의 `ee_to_tcp_offset_m` 과 **같은 값이어야 한다.**
> 어긋나면 그 차이가 판정 거리에 그대로 더해져 전 타겟이 빈손이 된다.
> 자세한 이유와 수치는 [`run_guide.md`](run_guide.md) "파지 판정 튜닝" 참고.

> **여전히 범위 밖**: 접촉력·딸기 변형·슬립 등 파지 접촉 물리. 실기 그리퍼도 전류값은
> `"logged only"`이고 조우 위치로만 판정하므로, 시뮬 경계도 position에서 끊는다
> ([`PORTFOLIO_SPRINT.md`](../PORTFOLIO_SPRINT.md) §3 참고).

### 시뮬레이션 데이터 흐름 상세

```
Isaac Sim (Script Editor 브릿지)
    /isaac_sim/strawberries (PoseArray) ──► fake_vision_node
                                                │ /strawberry/detection/pick_pose
                                                ▼
                                          scan_executor_node ──► curobo_planner_node
                                                                        │ MoveLine 등 서비스 호출
                                                                        ▼
Isaac Sim (articulation 주입) ◄── /joint_command ◄── sim_executor_bridge_node
```
