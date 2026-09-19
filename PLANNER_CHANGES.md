# 플래너 수정 로그

- 적용 원칙: [`PLANNER_POLICY_v2.md`](PLANNER_POLICY_v2.md) (2026-09-06~) — **§0-1 (2026-09-14, 최우선): 실기 노드를 고치는 이유는 "핵심 시퀀스의 설계와 맞지 않아서"뿐. 가드·임계값 완화 금지, 시뮬은 원본보다 관대하면 안 된다.**
- 형식: `- [파일:함수] 무엇을 어떻게 바꿨는지 한 문장`
- 목적: **은폐 방지가 아니라 본인 기억용.** 면접에서 "이 부분 어떻게 구현하셨어요?"에
  정확히 답할 수 있도록 남긴다.

> **이력**: 2026-09-05~09-06에는 v1 정책(FIX/PARAM/SIM 분류 + 상세 대장)이 적용됐다.
> 2026-09-06 v2가 이를 전면 대체하면서 **플래너 수정이 전면 허용**되고 로그가 한 줄 형식으로 바뀌었다.
> 아래 목록은 v1 시기의 항목까지 한 줄로 정리한 것이다.
> `_baseline/`(원본 스냅샷)은 삭제하지 않고 남겨두되 **동기화 의무는 없다**.

---

## 실기 플래너 (`src/strawberry_motion/`)

- [execution/scan_executor_node.py:_ALL_CELLS_CLOCKWISE_ORDER, _compute_scan_order · execution/cell_traversal.py 모듈 docstring] **이름·주석만 변경, 동작 불변 (2026-09-17)**: 상수 `_ALL_CELLS_ZORDER` → `_ALL_CELLS_CLOCKWISE_ORDER`. 원본 값(sw→nw_flat→ne→se)도 현재 값(nw→ne→se→sw)도 시계 방향이고, 옛 이름은 인접 이동 때문에 바꾸기 전 Z-order(nw→ne→sw→se) 시절 이름이 남은 것이다. 상수 위 주석에 순서 기준을 적음 — 깊이 1 은 인접 이동(Z-order 의 ne→sw 대각선 제거), 깊이 2 는 다른 기준(아래쪽 먼저 sw→se→nw→ne, 위쪽 목표점 오탐 경험). `_compute_scan_order` 09-09 주석과 `cell_traversal.py` docstring 의 "NW 가 빠지고 3분면만 돌았다"를 "원본 순서목록을 리포 YAML 과 함께 돌리면 빠질 수 있던 조합 결함, 실기 동작 아님(4셀 순회 실기 검증, 3분면 런 로그 0건)"으로 재작성. 따라 바꾼 곳: `check_params.py` 검사 정규식, `scripts/compute_subcell_scan_poses.py` 주석, 문서(G §5·§6-1·§8·§9, H·F·PLANNER_POLICY_v2·docs/parameters.md 이름). colcon build 반영
- [execution/scan_executor_node.py:_shortest_equivalent_joints, _derive_subcell_target] **J6 MoveJoint 등가각 창 ±359.4 → ±225 (2026-09-16, §0-1 설계 불일치)**: 실행기가 J6 를 플래너 운용 한계(±225) 밖(244.4/−266.6/−349.5/−311.7)에 세우면 다음 cuRobo 계획을 플래너가 ±225 창으로 고쳐 써 첫 점이 로봇 위치에서 360° 떨어지고, 점프·스윙 가드는 "현재 위치 → 첫 점"을 안 봐서 브릿지가 플랜지를 한 바퀴 돌렸다(런 14·09-16 런 픽당 2회, 3.3\~3.9 s). 09-14 이전은 플래너 창이 ±360 이라 0회. 원 팀 데모 순서(sw→nw_flat→ne→se)는 J6 가 셀 자세까지 전부 ±225 창 안(티칭값 계산, 끝의 overview 복귀만 −266.6 이나 뒤에 픽 없음)이고 최종 시연은 sw 한 분면만 수확해(PROJECT_GOAL §1-2 관찰) 이 조합이 안 나와 실기 미발생 — 순서를 원 팀 기록(민1 STEP 6)대로 nw 시작으로 되돌린 것 위의 픽·가지치기 overview 경유·세부 칸(시뮬 시퀀스)이 드러냈다. (09-17 정정: 종전 서술 "실기 원본 순서(sw→ne→se, NW 누락) … 4분면 순회(전부 시뮬 추가 시퀀스)"는 리포 YAML 과의 별칭 결함(아래 `cell_traversal.py` 09-09 항목)을 실기 동작으로 잘못 옮긴 것. 4셀 순회는 원 팀이 실기로 돌렸다 — 민1 STEP 6·민2 STEP 7) `_MOVE_TARGET_WRAP_WINDOW_DEG` 신설(J4 는 ±359.4 유지, 플래너 J4 창이 ±360), 세부 자세 유도 한계도 J6 ±225. 오프라인 재생 `scripts/replay_j6_wrap_window.py` → `log/m3/offline_checks/j6_wrap_window_replay_20260916.txt`: 종전 창은 로그와 일치(회전 2), 새 창 회전 0·창 밖 0, MoveJoint 두 번 136→224°·151→209° 로 길어짐, 런 J6 총회전 1312→739°. colcon build 반영. 09-16 03:46 런(`log/m3/20260916T034608-d6e265e9`)에서 재작성 0·창 밖 0·스플라인 최대 1.46 s 로 회전 소멸 확인(8 파지 / 배치 거부 4: J6 spline jump 2·J3 스윙 2). 판정·면접 답변 `portfolio/H_scope_decisions.md` §11 (2026-09-16)
- [조사 기록 · T4d 배치 변경 런, 2026-09-15] **보드 가장자리 위쪽 과실의 도달 포락선 — 수정하지 않는다(§0-1).** 서로 다른 배치 5개(`log/m3/random/pilot/`)에서 과실 중심 z 790\~897mm·|x| ≥ 185mm 인 익은 과실 8개의 파지 후보(오프셋 3\~5 × 자세 4)가 전부 `MotionGenStatus.IK_FAIL`(pre-approach 목표 y 487\~491) → `ABORT: grasp 전체 실패`. 가운데 x 113 z 827 은 잡음. 높은 과실 처리 `NW_HIGH_TARGET_*`(`harvest_motion_params.py:59~64`)는 `measured_tcp_260mm` 프로파일에만 걸려 있고 그 프로파일은 URDF `grasp_tcp_link` 부재로 로드 불가 — 파일럿 로그에 해당 줄 0(audit B6 확인). 시연 배치 익은 과실 최고 z 760 이라 처음 드러남. 배치 생성 범위도 좁히지 않는다. 그 밖에 새 거부 사유 `J6 swing 180.1 > 180`(시드 3) 1건. 기록표 `log/m3/random/README.md`, 판정 `portfolio/H_scope_decisions.md` §10 (2026-09-15)

- [execution/subcell_pose.py + execution/scan_executor_node.py + scripts/check_subcell_scan_poses.py] **깊이 2 는 보드에 가깝게 (2026-09-14, T4c 선행, 사용자 결정)**: `LAB_SUBCELL_EE_Y_M` 0.433(실기 NW 깊이 2 티칭 자세 4개의 FK ee y, 보드에서 377mm; 분면 자세는 470\~490mm) · 파라미터 `subcell_ee_y_m`(기본 0.433, 0 = 09-11 동작) · 유도 사다리 `derive_subcell_joints_tiered`(lab_plane → parent_y → 부모 자세, `SUBCELL_POSE … tier=`) · 스캔 자세 도착 시 `<cell>=VIEWING` 발행(`_dwell_collect_detections`, lab_plane 세부 자세는 세부 칸, 그 외는 부모 분면) · `CAMERA_OFFSET_IN_EE_M`(USD 에서 읽은 D455 오프셋) 으로 카메라-보드 거리 계산 · 검사 도구 `--ee-y`·`--lab-fk`. 오프라인 16칸: lab_plane 9 / parent_y 2 / IK 밖 4, 여유 ≥68mm. `SUBDIVIDE_IK_READY … subcell_ee_y=0.433` 기동 로그
- [scripts/planner_bootstrap.py + scripts/run_nodes.sh] `hold_on_place_failure` 기본값 False → **True(실기 원본 fail-closed)** 로 되돌리고, 시뮬은 run_nodes.sh 에서 `false` 명시. 09-07 에 기본값을 바꿔 둔 것은 부수 규칙 1 위반. "계속 진행"은 설계 시퀀스(PROJECT_GOAL §3 모두 파지/배치)에 맞는 시뮬 선택으로 기록 (2026-09-14, 사용자 결정)
- [scripts/harvest_motion_params.py] **원복 (2026-09-14, 원칙 §0-1)**: `MAX_HARVEST_JOINT_DELTA_DEG[J1]` 95 → **75**, `MAX_TAUGHT_PLACE_TRANSFER_JOINT_DELTA_DEG` J2 130 → **100**·J3 175 → **120** (원본). 09-08·09-10 의 상향은 시뮬을 통과시키려는 완화였고 설계 불일치가 아니다. 런 9·10·11 의 실행된 계획 109개에 원본 값 재적용 → 거부 **1건**(런 10 ripe_07→slot 0, J2 103>100; 런 9·11 은 0). 원복 뒤 오프라인 도달성(런 10 JSONL) `log/m3/20260912T013357-ea2ae427/tray_reachability_run10_guards_reverted_j6_360.txt`: retreat#1(ripe_07) 이 11칸 중 6칸 J2 거부, retreat#4(ripe_08) 전 칸 J3 거부 → 런당 미배치 1\~2개 예상. J6 도 ±225 로 되돌린 변형(`…_j6_225.txt`)은 거기에 `spline_jump J6` 가 슬롯당 1\~3 시작 자세 추가. 원본과 남은 차이: `OPERATIONAL_JOINT_LIMITS_DEG[J6]` ±225→±360(09-08, 되돌리면 런당 3\~5 계획 `spline_jump` 거부 — 결정 대기), `GRASP_RETRY_OFFSETS` 축소(09-10), `WALL_SURFACE_Y_M`(씬 값), `COLLISION_ACTIVATION_DISTANCE_M` 신설 (2026-09-14)
- [scripts/harvest_motion_params.py] **C4 원복 (2026-09-14)**: `OPERATIONAL_JOINT_LIMITS_DEG[J6]` ±360 → **±225**(실기 원본). 09-08 확장은 창을 넓혀 트레이 이송을 통과시킨 것 — 시뮬이 실기보다 관대해지는 변경. cuRobo URDF J6 ±360 은 그대로(통일안 A/B 회복 없음, 아래 조사 기록). 운용 한계·스윙 가드 전부 `_baseline` 과 동일해졌다. 예상 결과: 런 10 배정 기준 배치 4/8, 못 놓는 과실은 후퇴 자세에서 릴리스 (2026-09-14)
- [조사 기록 · J6 운용 한계, 2026-09-14] 실기 노드의 J6 설정 세 곳이 다르다(cuRobo URDF ±360 / 실행기 `_JOINT_LIMITS_RAD` ±359 / 플래너 운용 한계 원본 ±225). "cuRobo 도 ±225 로 통일하면 회복된다"를 런 10 후퇴 자세로 오프라인 A/B: 현재(둘 다 ±360) 배치 6/8, 플래너만 ±225 → 4/8, 둘 다 ±225 → **4/8 (회복 없음)**. 거부 이유가 `spline_jump J6` → `swing J6`·`limit J1` 로 바뀔 뿐 — ±225 창 안의 트레이 해는 손목 180° 초과 회전이나 반대 팔 분기가 필요하다. cuRobo 설정은 건드리지 않고 플래너 ±225 원복(C4)만 하기로. 근거 `docs/e0509_spec_audit.md` §7, 파일 `log/m3/20260912T013357-ea2ae427/tray_reachability_run10_varB_curobo_j6_225.txt` (2026-09-14)
- [조사 기록 · 런 10/11, 2026-09-12] **트레이 이송 스윙 가드가 과실을 떨어뜨리는 경로 — 사용자 결정(09-12): 실기 플래너 한계로 기록, 수정하지 않는다.** 런 10 ripe_08(ne, x +280): 파지·분리 정상 → `TAUGHT_TRAY_SLOT4_ABOVE` 계획이 `Cartesian plan rejected: J3 swing 186.3 > 175`(`MAX_TAUGHT_PLACE_TRANSFER_JOINT_DELTA_DEG`)로 거부 → `hold_on_place_failure=false` 라 그 자리에서 릴리스, 과실은 (280,738,663)mm 공중에 동결. 후퇴 자세 J3 −51°, 트레이 해 J3 +135°(팔꿈치 반대 분기, 운용 한계값). 오프라인 재현 `check_tray_slot_reachability.py`(런 10 JSONL + 시뮬 격자 플래그) → `log/m3/20260912T013357-ea2ae427/tray_reachability_run10.txt`: **retreat#4(=ripe_08) 는 계획 가능한 9개 슬롯 전부 거부**(J3 swing 5곳, J5 swing 4곳) — 슬롯 순서로 피할 수 없다. 런 11(02:13, `20260912T021353-03ef3b5c`)에서는 런 10 에서 slot 10 을 통과한 **같은 후퇴 자세**(ripe_04, J=[124,4,78,298,39,−27])가 slot 9 에서 `J3 swing 197.9` 로 거부 — 같은 시작·같은 목표라도 cuRobo `plan_single` 이 고르는 IK 분기가 호출마다 달라 **간헐적**이다(도구는 그 쌍에서 통과 해를 찾는다). 원인 코드는 실기 플래너(`curobo_planning_adapter.plan` → `MotionGen.plan_single`: 목표 IK 분기를 시작 자세와 무관하게 고르고, 가드 거부 뒤 재시도 없음). **시뮬 쪽만으로 고칠 수 있는 것은 없다**(과실 위치·`taught_slot_sequence`·hold 플래그는 원인과 무관). 후보는 전부 실기 노드 사안: ① 가드 상향(파라미터만, 팔꿈치 반전 허용 — 충돌 월드엔 보드만 있어 테이블·계란판 미검사) ② 시작 자세 시드 IK 후 관절공간 계획 또는 경유 자세(경로 계산 변경) ③ 실기 한계로 기록 — **③ 채택.** 실기 티칭 격자(플래그 없음)로 돌려도 ripe_08 후퇴 자세는 0·1·3·4·6·7칸 J3 거부(`tray_reachability_run10_realgrid.txt`) → 시뮬 격자 탓 아님. 원본 J3 상한은 120°(09-08 에 175 로 올림)라 원본이면 더 자주 거부, 원본 배치 실패 동작은 fail-closed 래치(09-07 에 '그 자리에 놓고 계속'으로 바꿈 — 공중 릴리스는 이 변경의 결과). 판정·면접 답변은 `portfolio/H_scope_decisions.md` §9. 런 11 은 ripe_08 을 직선 진입 IK 실패로 아예 파지 못했다(런 10 은 3번째 오프셋에서 성공) — 이것도 호출별 편차 (2026-09-12)
- [execution/subcell_pose.py] 신설 (T4b 적응 분할) — 세부 칸(깊이 2) 스캔 자세 유도 **계산부**. 부모 관절 FK → ee 를 **x·z 만** 세부 칸 중심으로 평행이동(y·방향 동일 = 실기 NW 세부 자세 4개가 y=433 한 평면인 성질) → 부모 관절을 시드로 cuRobo IK(보드 큐보이드 + 자기충돌, 시드 32, `res.solution` 사용 — `js_solution` 은 그리퍼 관절이 붙은 7열) → 돌아온 해 중 부모와 가장 가까운 것(J1/J4/J6 360° 등가 최근접) → 관절 변화 최대값이 한도를 넘거나 IK 실패·한계 밖이면 None. rclpy 없이 import 되며 노드와 오프라인 검사가 **같은 함수**를 쓴다. MoveJoint 관절공간 직선의 보드 여유(충돌구 FK, r=−10 패딩 제외) 계산 포함. 새 좌표 하드코딩 없음 — 전부 부모 자세에서 계산 (2026-09-11)
- [execution/scan_executor_node.py] `subdivide_min_candidates`(기본 **0** = 끔 = 실기: 쪼갤지 여부를 사람이 오프라인에서 정해 YAML 에 세부 자세를 넣던 정적 방식, 런타임 판정 없음) / `subdivide_max_joint_delta_deg`(기본 60) 신설. 분면 근거리 스캔의 중복 제거 후보가 임계 이상이면 `_subdivide_and_pick`: 2×2 중 **후보 있는 칸만** 유도 자세로 **MoveJoint** 이동(`_move_to_scan_cell_and_wait` 재사용 — cuRobo 계획 경로는 한 번도 실행된 적 없는 경로라 살리지 않았다) → 재스캔(`_dwell_collect_detections`, 분면 시야 좌표 중 **그 칸의 것만** 실행기가 거른다 — fake_vision 무수정, 이웃 장애물 등록은 분면 단위 유지) → pick. 유도 실패는 `SUBDIVIDE_REJECTED` 로 그 칸만 부모 자세 pick 으로 퇴화(다른 세부 자세에 있으면 `SUBDIVIDE_RETURN_TO_PARENT` 후). 깊이 상한 2. 세부 이동 실패는 부모 분면과 같이 시퀀스 중단(`_process_cell_detections` 가 bool 반환). `_subcell_of_pose` 신설로 그룹화·필터 단일 출처. IK 솔버(IKSolver 만, MotionGen 아님)는 파라미터>0 일 때만 기동 시 생성(`SUBDIVIDE_IK_READY`, 0.4초; 실패 시 `SUBDIVIDE_DISABLED` 로 끔). 상태 문자열: `SUBDIVIDE` / `SUBDIVIDE_SKIP` / `SUBCELL_POSE` / `SUBDIVIDE_REJECTED` / `SUBCELL_SCAN` / `SUBCELL_EMPTY`. 노드 모의 시험(실제 IK, 이동·pick 모의) 5경로 통과 — 분할·전부 거부·혼합·끔·그룹화=런 9 (2026-09-11)
- [scripts/check_subcell_scan_poses.py] 신설 (T4b) — 네 분면 × 네 세부 칸 유도 자세 오프라인 검사(노드와 같은 함수·같은 로봇·같은 보드 큐보이드). 결과(한도 60°): **sw 4/4 수락(관절 변화 23.3/25.4/33.6/26.2°, ee 오차 ≤0.1mm), se 4/4(21.9\~36.9°), nw·ne 는 아래 칸 1개씩만**(nw/se 66.3°·ne/sw 70.4° 초과, 위쪽 칸 4개는 같은 y·방향으로 IK 해 없음 → 부모 자세 퇴화). MoveJoint 관절공간 직선 보드 여유: sw 부모↔세부·세부↔세부 10쌍 **최소 183.8mm**, se 209.7, nw 193.0, ne 257.2 (2026-09-11)
- [scripts/run_nodes.sh] `-p subdivide_min_candidates:=3` + 기동 대조 `SUBDIVIDE_IK_READY min_candidates=3` 한 줄. 임계 3은 현재 배치(nw2/ne1/se0/sw3)에서 가지치기·잎·분할·2단 가지치기(sw/sw 빈 칸)가 한 런에 다 나오도록 고른 시뮬 값 — 실기 기본 0 보존 (2026-09-11)
- [scripts/tray_place_policy.py:taught_grid_slot_offset_m, scripts/planner_bootstrap.py, scripts/curobo_planner_node.py] `orthogonalize_taught_grid` ROS 파라미터 신설(기본 **false** = 실기 그대로). true 면 `TAUGHT_SLOT{0,1,3}` 세 점의 **피치 크기만** 쓰고 축을 −x/−y, z 를 수평으로 둔다(slot 0 불변) — 격자 59.8×51.2mm, 사이각 84.26° → 90.00°, 행당 z −2.5mm → 0. 근거: 세 점 수동 티칭이 만든 평행사변형·기울기는 강체 계란판이 가질 수 없는 성질이라 티칭 오차다(09-10 런 5 (B)). 시뮬 계란판은 수평·직사각(사용자 결정)이므로 시뮬에서만 켠다. 기동 로그에 값을 남기고 `run_nodes.sh` 대조에 한 줄 추가. 오프라인 시험: false 는 종전 오프셋과 동일, true 는 사이각 90.00°/z 0 (2026-09-11)
- [scripts/tray_place_policy.py:taught_grid_slot_offset_m, scripts/planner_bootstrap.py, scripts/curobo_planner_node.py] `taught_grid_shift_y_m` ROS 파라미터 신설(기본 **0.0** = 실기 그대로, |값| ≤ 0.05m 아니면 기동 거부 — m/mm 자리 착오 방어). 배치 격자 **전체(slot 0 포함)** 를 world y 로 평행이동. 근거: 계란판의 수평 중점을 테이블 중심축(y=0)에 맞추기로 한 사용자 결정(T4-3 3차) — 계란판은 배치 격자에 합동으로 따라가므로 격자를 옮겨야 판이 옮겨진다. 시뮬 값 +0.0108 의 출처는 `scene_tools/egg_carton_geom.GRID_SHIFT_Y_M`(2·행피치 − slot 0 ee y − 평균 매달림 변위 y, 런 7 기준). 기동 로그에 값을 남기고 `run_nodes.sh` 대조에 한 줄 추가. `check_tray_slot_reachability.py` 에 `--orthogonalize` / `--shift-y-m` 플래그를 넣어 시뮬 격자로도 도달성을 볼 수 있게 했다. 오프라인 시험: 기본값은 종전 오프셋과 동일(15슬롯), 시뮬 값은 직교화 오프셋 + 정확히 (0, +10.8, 0)mm (2026-09-11)
- [scripts/tray_place_policy.py:taught_grid_slot_offset_m, scripts/planner_bootstrap.py, scripts/curobo_planner_node.py] `taught_grid_pitch_override_m` ROS 파라미터 신설(기본 **0.0** = 실기 티칭 피치; 0.03\~0.15m 밖이면 기동 거부). 0 보다 크면 배치 격자 두 축 피치를 이 값의 **정사각**으로(축 −x/−y, z 수평 — 직교화를 포함하며 `orthogonalize_taught_grid` 보다 우선). 근거(T4-3 4차): 런 8 화면에서 옆의 빈 컵으로 과실 적도가 벽을 뚫고 보였다 — 시뮬 과실 애셋의 정지 자세 y 전폭 53.8mm 가 실기 행 피치 51.2 보다 넓어 인접 칸에 들어갈 수 없고, 실기 컵에는 실기 모형 과실이 인접 3칸(slot 0·1·3, 부트캠프 영상)에 들어갔으므로 애셋 치수 불일치다. 시뮬은 0.068(`scene_tools/egg_carton_geom.PITCH_M` 과 쌍, verify 대조). `check_tray_slot_reachability.py --pitch-m`. 오프라인 시험: 기본값은 종전 오프셋과 동일, 0.068 이면 slot(r,c) 오프셋 = (−68c, −68r + shift, 0)mm (2026-09-11)
- [scripts/check_tray_slot_reachability.py] 신설 (T4-1) — 트레이 슬롯 도달성 오프라인 검사. 플래너의 `build_curobo_motion_gen` 으로 같은 로봇·충돌월드를 만들고, `_compute_taught_slot_above_target` 과 같은 식(Slot0 티칭 관절 FK + SLOT0/1/3 격자 벡터 + 120mm)으로 15개 슬롯의 above/release 목표를 생성해, 같은 plan config(seeds 64, attempts 3, 2.0s, `MAX_TAUGHT_PLACE_TRANSFER_JOINT_DELTA_DEG` 스윙 가드)로 계획한다. 시작 자세는 런타임 JSONL 의 `retreat_step_complete` 실측 6개 + overview. **결과(12:02 런 기준): 0·1·3·4·6·7·9·10·12 는 7/7, 13 은 5/7, x≈400 열(2·5·8·11·14)은 틸트 유무와 무관하게 0/7 `IK_FAIL`** — `is_row2` 의 실패는 15° 틸트가 아니라 위치다. 실기 팀의 `row2_*` 파라미터가 원인을 비껴간 처방이었다는 근거 (2026-09-10)
- [scripts/run_nodes.sh, docs/run_guide.md] `taught_slot_sequence` **`0,0,0,0,0,0` → `0,1,3,4,6,7`** (T4-1). T2 로 과실이 실제 이송되자 여섯 개가 slot 0 한 칸에 겹치는 것이 드러났다 — 시뮬 결함이 아니라 실기 노드 설정 결함(같은 값이면 실기도 한 칸에 떨어뜨림; 부트캠프 최종은 과실 1\~2개라 안 드러남). 기동 로그 대조에 `slot_sequence=[0, 1, 3, 4, 6, 7]` 한 줄 추가 (2026-09-10)
- [scripts/*, execution/* 19개 파일] 설정·로그·캘리브레이션 경로 하드코딩을 `~/doosan_ws/...` → `~/strawberry_grasp_environment/...`로 일괄 치환 (2026-07-09)
- [execution/scan_executor_node.py:_start_cb] 스캔 시퀀스를 `_scan_sequence_run` 래퍼로 감싸 종료 시 `_started`를 해제 — 프로세스당 1회만 가능하던 재실행 제한을 풂 (2026-09-07)
- [scripts/pick_sequence_executor.py:maybe_execute_place_after_retreat] place 실패·게이트 차단·트레이 없음이 시퀀스 전체를 잠그던 것을, 그 자리에서 과실을 놓고 다음 타겟으로 계속하도록 변경 (`hold_on_place_failure` 파라미터, 기본 false) (2026-09-07)
- [scripts/planner_bootstrap.py:declare_and_load_params] `hold_on_place_failure` ROS 파라미터 신설 — true면 기존 fail-closed 래치 동작 (2026-09-07)
- [execution/scan_executor_node.py:declare_parameter] `pick_timeout_sec` 기본값 120 → 60초 (2026-09-07)
- [scripts/curobo_planner_node.py:_abort_pick_with_complete] 중단 시 그 자리에 선 채 `pick_complete`만 발행하던 것을, **세부영역 scan pose 로 복귀한 뒤** 발행하도록 변경 — 쿼드트리 스캔의 전제(모든 pick 은 scan pose 에서 시작)를 성공 경로만 지키고 중단 경로가 어기고 있었다 (2026-09-09)
- [scripts/curobo_planner_node.py:_return_to_pick_start_scan_pose] 신설 — 복귀 목표를 `self.pick_start_joints` 로 두고 `plan_to_fixed_joints_pose` 로 이동, 실패 시 `RETURN_TO_SCAN_FAILED` 를 남긴다 (2026-09-09)
- [scripts/pick_sequence_executor.py:handle_gripper_close_failed] 직선 후퇴만 하고 다음 타겟으로 넘기던 것을, scan pose 복귀를 거치도록 변경 — 후퇴만으로는 보드 앞을 못 벗어난다 (2026-09-09)
- [scripts/pick_sequence_executor.py] pick 시작 자세를 `node.pick_start_joints` 에 기록 — 중단 경로가 같은 복귀 목표를 쓰도록 (2026-09-09)
- [execution/cell_traversal.py] 신설 — 순회 순서를 YAML 실재 cell_id 로 해석하는 별칭 테이블. 원본 순서목록 `_ALL_CELLS_ZORDER`(현 `_ALL_CELLS_CLOCKWISE_ORDER`)의 `root/nw_flat` 이 리포 YAML 에 없어 **NW 가 조용히 빠질 수 있던** 것을 고침(리포 YAML 과의 조합 결함. 수정 전 실행 로그는 남아 있지 않다. 원 팀 실기 동작이 아니다 — H §11 09-17 정정). 누락 분면은 `TRAVERSAL_QUADRANT_MISSING` 로 드러낸다 (2026-09-09)
- [execution/scan_executor_node.py:_compute_scan_order] 위 모듈 호출로 교체 (한 줄) (2026-09-09)
- [execution/scan_transit.py] 신설 — 분면 간 스캔 이동을 보드가 든 MotionGen 으로 계획. 실패 시 종전 MoveJoint 로 폴백 (2026-09-09)
- [execution/scan_executor_node.py:_move_to_scan_cell_and_wait] 스캔 이동을 계획 경유로 전환 — 분면별 scan pose 를 부여하자 관절공간 직선이 보드를 관통했다 (sw->nw -3mm, nw->ne -23mm 실측) (2026-09-09)
- [scripts/compute_subcell_scan_poses.py] 신설 — 분면별 scan pose 오프라인 IK 산출기 (2026-09-09)
- [config/scan_pose_candidates_refit_candidate.yaml] 네 분면이 전부 overview 와 같은 값이던 것을 분면별 자세로 교체. ee y 평면은 실기 티칭 433mm 에 가장 가까운 가용값 400mm (433 은 팁-보드 23.3mm 로 cuRobo 가 거부), 자세 방향은 실기와 동일 (2026-09-09)
- [config/scan_pose_candidates_refit_candidate.yaml] 분면 자세를 **실기 v12 수동 티칭 값으로 복원**. 내가 IK 로 만든 ee y=400mm 자세는 팁-보드 56mm 라 스캔 자세에서 과실에 닿고 파지 중 보드에 박혔다. 실기 값은 97\~156mm. FK 가 기록의 TCP 5개를 0.07mm 이내로 재현해 **기록의 'TCP' 가 그리퍼 밑동**임도 확인 (2026-09-09)
- [execution/scan_executor_node.py:_ALL_CELLS_ZORDER] 데모용 sw 시작 순서를 실기 순서 nw->ne->se->sw 로 복원 (2026-09-09; 09-17 `_ALL_CELLS_CLOCKWISE_ORDER` 로 이름 변경)
- [execution/scan_executor_node.py] `plan_scan_transit` 파라미터 신설 — false 면 셀 간 이동이 실기와 동일한 순수 MoveJoint. v12 자세에서는 관절공간 직선도 보드여유 최소 43.4mm 로 안전하다 (2026-09-09)
- [execution/scan_executor_node.py:_overview_prescan_filter] `overview_prescan` 파라미터 신설(기본 false = 실기 최종본과 같은 4분면 전수 순회). true 면 트리거 직후 `root=SCANNING` 을 발행하고 overview 에서 `scene_positions` 를 분면별로 세어 **익은 과실 0개 분면을 순회에서 뺀다** (`OVERVIEW_SCAN`, `TRAVERSAL_PRUNED`). 원안 1·2단계 복원. 판정 채널이 `pick_pose` 가 아닌 이유: 실기 줄기 키포인트는 overview 거리에서 안정화되지 않는다 (2026-09-10)
- [execution/scan_executor_node.py:_move_to_scan_cell_and_wait] MoveJoint 폴백이 **비인접 분면**(가지치기로 생기는 직행)이면 overview 를 경유 — FK 실측 nw↔se 관절공간 직선 보드여유 37mm, ne→sw 스윙 234°, overview↔분면은 202mm 이상 (2026-09-10)
- [execution/scan_executor_node.py:_group_poses_by_subcell] 분면 안 2×2 중심선을 보드 전체 중심선 → **부모 분면 중심**으로. 종전엔 탐지가 항상 부모와 같은 이름의 구석(`root/nw/nw:2`)으로 몰려 2차 분할이 퇴화해 있었다 (09-08 보드 고정 격자 수정의 부작용) (2026-09-10)
- [execution/scan_executor_node.py] `plan_scan_transit` 기본값 **true → false**. 09-09 에 넣은 뒤 한 번도 실행된 적이 없었다 — 계획에 쓰는 MotionGen 이 `enable_runtime_curobo_preview` 경로에서만 생성되어 조건이 조용히 실패, 02:02·10:13 런 모두 `cuRobo 경유` 0건. 필요도 사라졌다: v12 티칭 자세의 순차 이동 관절공간 여유는 최소 181mm(보드 810, FK 20쌍), 비인접 쌍은 overview 경유. 켜져 있지만 죽어 있는 안전망을 로그 대조로 찾은 사례 (2026-09-10)
- [strawberry_harvest/assets/robot/robot_assembly.usd] 아티큘레이션 드라이브 게인 10배 — 팔 stiffness 1e5->1e6 / damping 1e4->1e5, 그리퍼 1e4->1e5 / 1e3->1e4. damping 만 올리면 오히려 느려지므로 비율을 유지한 채 같이 올려 추종 대역을 넓혔다 (2026-09-09)
- [sim_executor_bridge_node.py] `sim_speed_scale` 파라미터 신설(기본 2.0) — 스플라인/직선 실행 시간을 한 번에 조절. `SPLINE_MIN_TOTAL_SEC` 1.0->0.35, `MOVELINE_MAX_TOTAL_SEC` 5.0->2.0, `ARM_ARRIVAL_TIMEOUT_SEC` 8.0->3.0, SafeGrasp 대기 1.0->0.25s (2026-09-09)
- [execution/scan_executor_node.py] 스캔 이동 속도 60/90 -> 120/180 deg/s. 실기가 낮춘 이유는 테이블 흔들림(민1 STEP 7)이라 시뮬엔 해당 없다 (2026-09-09)
- [layout_layer.usd + config/environment.yaml x2 + scan_collision_world.yaml] 보드 콜라이더를 **실기 물리 화이트보드 1500x900(민2)** 크기로 넓히고 바닥까지 내려 `board_guard` 정지 콜라이더 신설 — 종전 콜라이더는 종이판(1090x790) 크기의 두께 0 평면이라 **보드 밑과 옆을 로봇이 그냥 지나갔다.** 앞면 y=672.0mm 는 불변, 티칭 자세/순회/트레이 여유도 불변 (2026-09-09)
- [scripts/grasp_candidate_policy.py:grasp_offsets_for_target] 오프셋 사다리 맨 앞에 **0.0** 추가 (`_exact_first`). 종전 첫 값 15mm 때문에 정상 동작에서도 조우가 파지점보다 15mm 앞(-y)에서 닫혔다. 기존 값·순서는 폴백으로 그대로 남는다. 정상 파지 d_tcp 38mm -> 35mm (2026-09-09)
- [scripts/pick_sequence_executor.py] 이웃 장애물 등록에 `straw`(bias 포함) 대신 **`raw_straw`** 를 넘긴다. 자기-제외 판정 거리가 z bias 와 정확히 같은 35mm 라 부동소수점 끝자리로 갈렸고, 익은 딸기 6개 중 5개가 **자기 자신을 반경 30mm 장애물로 등록**했다. 그래서 오프셋 0.0 이 막혀 15mm 로 물러났고 파지가 실패했다 (2026-09-10)
- [scripts/scene_obstacle_manager.py] 자기-제외 임계 35 -> **50mm** + 제외 개수 로그 (2026-09-10)
- [scripts/harvest_motion_params.py] `MAX_TAUGHT_PLACE_TRANSFER_JOINT_DELTA_DEG` J2 100 -> **130도** (**→ 2026-09-14 원복 100**). 파지·분리까지 성공하고도 트레이 이송이 막혀 과실을 보드 앞에 놓아버렸다 (실측 nw 딸기 J2 114/113도). 보드를 810mm 로 되돌리면서 팔이 더 펴져 스윙이 커진 것 (2026-09-10)
- [scripts/grasp_candidate_policy.py:_exact_first] 2026-09-09 에 넣은 오프셋 0.0 을 **되돌림**. 실측 결과 6/6 IK 실패 — ee->실제 손끝이 262.5mm 인데 플래너 TCP 는 236.0mm 라, TCP 를 딸기에 맞추면 손끝이 보드면 0.7mm 까지 들어간다. 15mm 가 기구학적 최소 (2026-09-10)
- [scripts/harvest_motion_params.py] `GRASP_RETRY_OFFSETS` / `LEFTMOST_GRASP_RETRY_OFFSETS` 를 `[15, 20, 25]mm` 로 자름. 조우가 무는 구간이 TCP 기준 -5.5\~+26.5mm 라 그보다 큰 오프셋은 물리적으로 파지 불가인데, 종전 사다리(40/50/70mm)가 그걸 성공으로 받아 ripe_02/ripe_03 이 40mm 에서 잡은 척 진행했다 (실측 along +42.9/+42.5mm) (2026-09-10)
- [scripts/planner_bootstrap.py + harvest_motion_params.py] `collision_activation_distance=0.005` 명시 (`COLLISION_ACTIVATION_DISTANCE_M`). 실기 제어기의 cuRobo 월드는 벽이 없어(더미 큐브) 여유 0 이었는데, 시뮬이 보드 큐보이드를 넣으면서 cuRobo 기본 25mm 가 따라왔다. 실기 쪽으로 되돌리되 5mm 는 남긴다 (2026-09-10)
- [조사 기록] ripe_02/ripe_03 이 40mm 로 밀린 진짜 원인은 **변형 탐색 순서**: `[-10,-5,0,+5]°` 순서에서 −5° 가 pre-approach 는 되지만 높은 딸기의 15\~25mm 종점은 도달 밖(ee +22mm), 40mm 만 안. 루프가 변형 안 첫 성공에 멈춰 0° 를 시도하지 않았다. 크레인 +33mm 포함 재현으로 T3 와 동일 패턴 확인. 사다리 25mm 절단으로 해소 (2026-09-10)
- [scripts/pick_sequence_executor.py:execute_detach_and_retreat + planner_bootstrap.py] `enable_straight_reverse_retreat` 파라미터 신설(기본 False). 설계 시퀀스 5단계 '진입 경로의 역순으로 이동' 은 실기 코드(`build_straight_retreat_steps`)에 **정상 구현돼 있으나 measured_tcp 프로파일에만** 걸려 있어, legacy 에서는 `reverse_distance = extra_advance(0)` 로 빈 리스트가 되어 통째로 생략됐다. `enable_open_stem_descent` 와 같은 방식으로 게이트만 확장 — 실기 기본 동작은 불변 (2026-09-10)
- [scripts/curobo_planner_node.py:__init__] 뷰포트 HUD 계측 부착 4줄 추가 — `harvest_probe.attach("planner", self)`. 픽 시퀀스 메서드를 밖에서 감싸 상태를 status_bus 로 내보낸다. import 실패·예외는 전부 삼켜 노드 동작에 영향이 없고, 그 4줄을 지우면 계측이 사라진다 (2026-09-09)
- [execution/scan_executor_node.py:__init__] 같은 4줄로 `harvest_probe.attach("scan", self)` — 타겟 개수·스캔 단계·시퀀스 종료를 내보낸다 (2026-09-09)
- [계측 모듈 `strawberry_harvest/scripts/hud/harvest_probe.py`] HUD 표시용 계측 확장 — `_trigger_picks_for_cell`/`_move_to_scan_cell_and_wait`/`_process_cell_detections` 의 cell_id 로 '영역'(홈/북서/북동/남서/남동)을 찍고, `execute_open_stem_descent_if_needed` 를 GRASP 단계에 포함시켰다(화면 라벨 '파지 + 하강'). 노드 파일은 건드리지 않았다 — 래핑만 늘렸다 (2026-09-09)
- [HUD `strawberry_harvest/scripts/hud/bus_merge.py`, `isaac_sim_viewport_display.py:install`(09-16 까지 `isaac_sim_hud.py`), `scripts/run_nodes.sh`] 직전 런 잔상 제거 — HUD Run 시각(`bus_merge.EPOCH`)보다 먼저 멈춘 스냅샷은 무시, `run_nodes.sh` 기동 시 `/tmp/harvest_hud_*.json` 삭제. 종전엔 planner warmup 동안 지난 런의 '수확 완료 6/6'(램프 빨강)과 새 노드의 빈 상태가 섞여 보였다. 완주 후 트리거 대기(노드 생존)는 영향 없음 (2026-09-10)
- [브릿지 `sim_executor_bridge_node.py:move_line_cb`] `MOVELINE_SHORT` 임계값 3mm 고정 → `max(3mm, 스텝 길이)`. 스텝 상한에 걸린 5mm 스텝 이동(배치 하강 120mm)에서 부족 3.5\~4.1mm 가 매번 거짓 ERROR 로 찍혔다 (11:05 런 6건, 다음 동작이 관절공간 절대 목표라 잔차 무의미). 2mm 스텝인 파지 쪽은 3mm 그대로 (2026-09-10)

## 가상 제어기 (`src/strawberry_sim_core/`) — 본인 구현물

- [fake_vision_node.py + quadrant_filter.py] **깊이 2 시야 (2026-09-14)**: `quadrant_filter.cell_bounds/in_bounds`(분면·세부 칸 경계, 실행기 `_quadrant_bounds`·`_subcell_of_pose` 와 같은 규칙) · `fake_vision` 파라미터 `subcell_view_enabled`(기본 true): `<cell>=VIEWING` 을 받으면 그 셀 경계로 발행을 좁힌다(세부 칸이면 세부 칸). SCANNING 은 종전대로 분면. → lab_plane 세부 자세에서는 scene_positions 도 세부 칸 것만 나가 플래너의 이웃 장애물이 준다 — 효과 측정은 런 12·T4d
- [sim_executor_bridge_node.py + robot.urdf + assets/robot/…/robot.usd] **제원 결정 D1·D2·D3 (2026-09-14, `docs/e0509_spec_audit.md`)**: D1 — 시뮬 로봇 J5 한계를 ±360 → **±135**, 2차로 **J2 ±360 → ±95, J3 ±155 → ±135**(전부 실기 cuRobo 운용 값; 공식보다 좁다), 브릿지 방어선 `[360, 95, 135, 360, 135, 360]`. 기록된 11개 런 사용 최대 J2 71°·J5 91° 는 여유 있고 **J3 는 135.0° 정확히(여유 0)** — 트레이 위 자세가 한계에 닿는다. **씬 재로드 필요**. D2 — 실기 속도(180/225)를 건드리지 않고 시뮬도 같은 값 유지 → 변경 없음(카탈로그 120/120/150 은 실기 설정 기록 사항). D3 — 요청 가속도를 두산 MoveIt 값으로 **자르지 않고** 초과만 기록(`DOOSAN_MOVEIT_REF` 기동 1줄, `MOVEJ_OVER_DOOSAN_MOVEIT` 호출마다). 실기 노드 무수정
- [sim_executor_bridge_node.py] **공식 제원 정합 C1·C2·C3 (2026-09-14, `docs/e0509_spec_audit.md`)**: ① `ARM_JOINT_LIMIT_DEG` 365/160 → 공식 URDF 값 360/155 (방어선이 로봇 한계보다 넓을 이유 없음) ② `move_joint_cb` 를 즉시 발행에서 **요청 vel/acc 사다리꼴 프로파일**(`movej_profile`, 동기 이동, `time` 우선, 기본 60/60)로 — 종전엔 PhysX 상한 180/225°/s 까지 움직여 실기 명령(120°/s)보다 1.5\~1.9배 빨랐다 ③ `sim_speed_scale` 기본 2.0 → **1.0** — 실기 요청 시간이 곧 실행 시간. 런 시간은 늘어난다(MoveLine 2배·스캔 이동 131° 가 1.76 s). 가속도를 두산 MoveIt 값으로 자르지는 않는다(D3 결정 대기)
- [sim_executor_bridge_node.py:_wait_for_arm_arrival] **`ARM_ARRIVAL_TOL_DEG` 1.5 → 0.3deg** (`arm_arrival_tol_deg` ROS 파라미터 신설). 1.5 는 6축 **각각** 1.5도를 도착으로 인정해 배치 자세에서 손끝 RMS 23.6mm 였고, 13:42 런의 팔 오차 10.0\~24.3mm 가 그 예산 안에 들어왔다. 드라이브 정상상태 오차는 0.02deg 수준이라 1.5는 **아직 움직이는 팔을 도착으로 인정**하는 값이다. 함께: ① **정체 감지**(개선 <0.01deg 가 0.30s 지속 → `ARM_ARRIVAL_STALLED` 로 대기 종료, 조인 값이 8초 타임아웃을 새로 만들지 않게) ② 그 정체는 **잔차 2.0deg 이하일 때만** 인정 — 팔이 얼어붙은 실패(잔차 130deg)를 "도착" 으로 삼키지 않는 상한 ③ 모든 도착에 `ARM_ARRIVAL ... 잔차 Xdeg (손끝 Xmm) / 대기 Xs` 계측. 세 경로는 오프라인 단위 시험으로 확인 (2026-09-10)
- [검증 — 런 7 14:06] 조임 효과 실측: 과실에서 역산한 **팔 오차 10.0\~24.3mm → 1.9\~5.3mm**(계통 11.0→2.5, 산포 11.0→2.1mm), 배치 하강 along 오차 −3.6\~−4.3 → **−1.0\~−1.5mm**, **`MOVELINE_SHORT` 6건 → 0건**. `ARM_ARRIVAL` 62건 관절 잔차 중앙 0.28deg / 손끝 중앙 1.6mm, `ARM_ARRIVAL_STALLED`·`ARM_ARRIVAL_TIMEOUT` **0건**, 전체 213.8 → **222.5초(+8.7초)**. 파지 lateral 은 개선 없음(2.7\~5.3mm) — 그 값은 도착 잔차가 아니라 cuRobo 가 고른 접근 자세가 지배한다 (2026-09-10)
- [sim_executor_bridge_node.py:_tip_error_mm] 신설 — 손끝오차가 **항상 0mm** 로 나오던 버그 수정. 종전에는 `kinematics.get_state()` 를 두 번 부르고 `.copy()` 없이 두 결과를 뺐는데 cuRobo 가 내부 버퍼를 재사용한다. `ARM_ARRIVAL_TIMEOUT` 이 함께 찍던 손끝 수치를 못 쓰던 이유(SUBMISSION_PLAN T1 부수 발견)가 이것이다 (2026-09-10)
- [sim_executor_bridge_node.py:move_line_cb] `MOVELINE_END_ERR` 계측 신설 — 종점 오차를 **명령 축 성분(along)과 측방 성분(lateral)** 으로 분해해 남긴다(lateral > 5mm 면 WARN). 종전 검사 `_moved` 는 시작점\~끝점의 **스칼라 거리** 하나뿐이라 옆으로 밀려도 거리만 맞으면 통과했다. 13:12 런의 배치 하강 6건이 전부 "실이동 115.8\~116.6mm / 명령 120mm" 로 정상 판정됐는데, 과실 정지 위치에서 역산한 그리퍼는 계획 자세에서 **4.3\~20.9mm** 어긋나 있었다 (계란판 배치가 들쭉날쭉해 보인 원인). 이 구간을 잡을 수 있었던 직교 편차 검사 `row2_line_check_result` 는 `is_row2` 분기에만 있는데 그 열은 IK_FAIL 이라 **한 번도 실행된 적이 없다** — `plan_scan_transit` 과 같은 부류의 죽은 안전망. **판정·동작은 바꾸지 않았다, 계측만 추가** (2026-09-10)
- [계측 결과 — 위 가설 반증] 런 6 실측: 배치 하강 6건의 lateral 은 전부 **0.1mm**(along −3.6\~−4.3mm). 직선은 정확했다. 그런데 팔은 여전히 계획 release 자세에서 **10.0\~24.3mm** 어긋난다. 진짜 원인은 `sim_executor_bridge_node.ARM_ARRIVAL_TOL_DEG = 1.5°` — 6축 **각각** 1.5° 를 도착으로 인정한다. 배치 자세에서 FK 환산하면 J1 13.7 / J2 14.2 / J3 11.4 / J4 4.5 / J5 4.5 / J6 0.0mm, **RMS 23.6mm** 로 실측 오차가 이 안에 그대로 들어온다. 드라이브 정상상태 오차는 0.02° 수준이므로 1.5° 는 **아직 움직이는 팔을 도착으로 인정**하는 것이다. 실기 Doosan 서비스는 모션이 끝나야 응답하므로 조이는 쪽이 실기에 더 가깝다. **아직 안 고쳤다** — 타이밍이 시퀀스 전체에 걸려 별도 런이 필요하다 (2026-09-10)
- [sim_executor_bridge_node.py] **T2 파지 이벤트 발행** — `/sim/grasp_event` (String). `_judge_grasp` 가 CONTACT 로 판정한 과실 중심을 `ATTACH x y z` 로, 부착 중 조우가 열리면 `RELEASE` 를 낸다 (`_on_gripper_close`/`_on_gripper_open`, `set_position_cb`·`safe_grasp_cb` 양쪽). 딸기를 좌표로 지목하는 이유는 브릿지가 prim 이름을 모르기 때문. 판정 로직은 그대로이고 Isaac 쪽에 재구현하지 않는다 (2026-09-10)
- [fake_vision_node.py:strawberry_cb] 빈 PoseArray 를 받으면 **빈 `scene_positions` 를 발행**하고 돌아간다. 종전엔 그냥 return 해서, T2 로 마지막 딸기가 부착되는 순간부터 하류(플래너 하트비트·HUD 비전 램프·브릿지 판정 목록)가 멈췄다 (12:02 런 종료 전 23.7초 공백) (2026-09-10)
- [sim_executor_bridge_node.py:move_line_cb] `MOVELINE_MAX_STEPS=24` 스텝 수 상한 신설. 실소요는 명령 속도가 아니라 스텝 수 × IK 1회(\~150ms) 가 지배한다 — 10:13 런에서 배치 하강 120mm 가 60스텝 9.2초로 자체 페이싱 1.5초의 6배, MoveLine 합계가 런의 50%. 24 로 자르면 120mm 만 5mm 스텝(관절 이동 0.3→0.75도)이 되고 파지 쪽 30/40/45mm 는 그대로. 실기 movel 에는 없는 시뮬 인공물 제거. `MoveLine ok` 로그에 **실소요** 추가 — 종전엔 계획값(1.50s)만 찍혀 9초 지연이 안 보였다 (2026-09-10)
- [sim_executor_bridge_node.py] MoveLine IKSolver 에 `collision_activation_distance=0.005` — 플래너와 동일 값 (2026-09-10)
- [sim_executor_bridge_node.py:_judge_grasp] 판정 대상을 과실 중심 -> **줄기(과실+z_bias)**, 기준을 스칼라 d_tcp -> **조우 물림 구간(along/lateral)** 으로 교체 (2026-09-10)
- [sim_executor_bridge_node.py:move_line_cb] MOVELINE_SHORT 가 항상 '실제 0.0mm' 를 찍던 것 수정 — cuRobo `get_state()` 의 내부 버퍼 재사용으로 시작 FK 가 덮어써지고 있었다. `.clone()` 으로 고정 (2026-09-10)
- [sim_executor_bridge_node.py:move_spline_cb] ★ 실행 시간에 **관절속도 기반 하한** 추가 (`SPLINE_MAX_JOINT_SPEED_DEG_S=120`). 종전 `req.time/속도배율` 만으로는 큰 스윙을 아티큘레이션이 못 따라왔고, 그 잔차가 다음 **상대** MoveLine 에 전파돼 조우가 파지점보다 앞에서 닫혔다. 실측 스캔→pre-approach 스윙: sw 40.7/62.9/43.1° (정상 동작) vs ne 108.1°, nw 195.6/196.7° (실패) — 사용자 보고와 정확히 일치 (2026-09-10)
- [sim_executor_bridge_node.py] `ARM_ARRIVAL_TIMEOUT_SEC` 3.0 -> **8.0**, 타임아웃 시 잔차를 **손끝 오차(mm)** 로 환산해 ERROR 로 남긴다 (2026-09-10)
- [sim_executor_bridge_node.py] `grasp_capture_radius_m` 55 -> **45mm 되돌림**. 55 는 도착 지연 버그의 대증요법이었고, 넓힌 탓에 **앞에서 닫힌 실패를 CONTACT 로 오판정**해 그대로 배치까지 진행했다 (사용자 보고). 45 가 정상 38.1mm 와 실패 49.5mm 를 가른다 (2026-09-10)
- [sim_executor_bridge_node.py] `grasp_capture_radius_m` 45 -> **55mm**. 정상 파지의 d_tcp 는 sqrt(15^2+35^2)=38.1mm 로 고정인데 여유가 6.9mm 뿐이라 실행 오차로 2/6 이 EMPTY 가 됐다 (2026-09-10)
- [sim_executor_bridge_node.py] `gripper_close_rad` 파라미터 신설, 닫힘 1.0 -> **1.08 rad**. 실제 메시로 재니 1.0 에서 파츠 간격이 **9.4mm** 라 줄기를 물지 못했다 (종전 스피어 측정은 패딩 19.7mm 때문에 -10.6mm 로 잘못 나왔다). 1.08 에서 0.3mm (2026-09-10)
- [robot_assembly.usd] `rh_r2`/`rh_l2` 상한 57.296 -> **63.025 deg**(1.1 rad). 종전 값은 제조사 xacro 의 **스톡 손가락** 기준이라 15cm 커스텀 파츠에는 맞지 않았다. 의도한 시뮬 편차이며 `gripper_close_rad` 와 한 쌍 (2026-09-10)
- [layout_layer.usd] 씬 초기 개도 49.11 -> **53.05 deg** (새 배율에서 stroke 600) (2026-09-10)
- [quadrant_filter.py] 신설 — 보드 고정 쿼드트리 격자. 경계 규칙을 `scan_executor_node._group_poses_by_subcell` 과 일치시켰다 (무작위 2만 점 교차검증 0 불일치) (2026-09-09)
- [fake_vision_node.py] 씬의 모든 딸기를 항상 발행하던 것을, **지금 스캔 중인 분면의 딸기만** 발행하도록 변경. 실기 카메라 시야를 격자로 대체한 것이며 실기 노드는 안 건드린다. 셀 상태는 `/strawberry/exploration/set_cell_state` 에서 받는다. `quadrant_filter_enabled:=false` 로 종전 동작 복귀 (2026-09-09)
- [status_monitor_node.py] `fake→플래너` 개수가 **현재 분면 기준**임을 라벨에 명시 (2026-09-09)
- [sim_executor_bridge_node.py:_publish_gripper_only] ★ 측정값(`current_joints`)을 명령으로 되실어 **진행 중인 팔 모션을 그 자리에서 얼려버리던** 것을, 마지막 명령값 재발행으로 교체 — 복귀 스플라인 29ms 뒤의 `SetPosition(600)` 이 J6 를 93.4 가 아닌 124.7 에 고정시켰고, 노드가 그 자세를 다음 pick 시작 자세로 저장해 사이클마다 누적됐다 (2026-09-09)
- [sim_executor_bridge_node.py:move_spline_cb] 구간당 고정 0.10s 를 `req.time` 이동량 비례 배분으로 교체 — 이동량과 무관하게 \~1.2s 만에 명령만 끝나 팔이 30도 넘게 뒤처진 채 다음 단계가 시작됐다 (2026-09-09)
- [sim_executor_bridge_node.py:_wait_for_arm_arrival] 신설 — 실기 Doosan 서비스처럼 **모션이 끝나야 응답**하도록 MoveSpline/MoveLine/MoveJoint 끝에 도착 대기 추가 (타임아웃 시 경고만, 시퀀스는 진행) (2026-09-09)
- [sim_executor_bridge_node.py:move_spline_cb] deg/rad 혼용으로 모든 관절에 57.3배 명령이 나가던 회귀 수정 (2026-09-09)
- [sim_executor_bridge_node.py:_publish_joint_command] 관절 한계 초과 명령 발행 차단 + `JOINT_COMMAND_REJECTED` 로그 (2026-09-09)

- [sim_executor_bridge_node.py:move_line_cb] MoveLine 텐서 flatten 버그 수정, fail-open 제거, TOOL 좌표계(ref=1) 지원 추가
- [sim_executor_bridge_node.py:_solve_ik_nearest] cuRobo IK elbow-flip 다이브 방지 — seed 8개 중 최근접 해 선택 + 관절 이동 45° 상한 가드 + 5회 재시도
- [sim_executor_bridge_node.py:set_position_cb / safe_grasp_cb] 명령값을 그대로 되돌려 항상 빈손 판정이던 것을, TCP↔딸기 거리 기반 판정으로 교체 (2026-09-07)
- [status_monitor_node.py] 신규 — `/strawberry/scan/status`와 `/rosout`만 구독하는 읽기 전용 상태 표시 창 (2026-09-07)
- [sim_executor_bridge_node.py:__init__] 파지 판정용 `tool_tcp_offset_m` 기본값 160 → 208mm — planner의 `ee_to_tcp_offset_m:=0.208`과 어긋나면 그 차이(48mm)가 판정 거리에 더해져 전 타겟 빈손이 되므로 한 쌍으로 맞춤 + 기동 시 값 경고 출력 (2026-09-07)
- [sim_executor_bridge_node.py:__init__] HUD 계측 4줄 — 서비스 콜백 진입을 플래너 생존 증거로, 0.3s 타이머를 제어 하트비트로 쓴다 (2026-09-09)
- [fake_vision_node.py:__init__] HUD 계측 4줄 — `strawberry_cb` 진입을 인식 하트비트로 쓴다 (2026-09-09)
- [strawberry_harvest/scripts/isaac_sim_script_editor_bridge.py:on_physics_step] 재초기화 판정을 `num_dof` → `handles_initialized` 로. Stop→Play 후 물리 뷰가 None 인데 num_dof 캐시가 남아 매 스텝 apply_action 이 AttributeError 를 던지고 관절 상태·딸기 발행이 통째로 멈추던 것을 고침 (2026-09-09)

## 씬 (`strawberry_harvest/`)

- [assets/props/egg_carton.usd, scenes/main_scene.usd, scenes/layers/layout_layer.usd] **T4-3 계란판 시각 메쉬 (2차)** — 수평·직사각·컵 15개 동일, 2,188점/984면, 물리·콜라이더 없음, 이름에 `strawberry` 없음. 피치만 플래너 상수 `TAUGHT_SLOT{0,1,3}` 의 크기(59.8×51.2mm), 판 z22 / 바닥 z1 / 스커트 −3 (컵 깊이 21mm). **1차(같은 날) 폐기**: 티칭 격자(84.26°, z 기울기)를 판에 그대로 굽고 점유 컵을 과실에 맞춘 것 — 사용자 지적대로 "균일한 컨테이너" 라는 존재 이유를 없앴다. 격자 왜곡은 티칭 오차이지 컨테이너 성질이 아니다. 배치는 사용자 결정으로 **yaw 0(베이스 축 정렬)** 이고 플래너 `orthogonalize_taught_grid=true` 로 배치 격자도 직교화해 두 격자가 합동 → translate = 직교화 slot 0 ee + 평균 ee→과실 변위(런 7) = (0.7711, 0.0915, 0). 예상 착지 치우침 ≤5.9mm(분면별 매달린 깊이 차), 림/벽 겹침 ≤0.5mm. (직교화 전 티칭 격자에 맞출 때는 Kabsch fit 이 yaw −7.24°/잔차 5.3mm, yaw 0 강제면 16.7mm 였다) (2026-09-11)
- [scripts/scene_tools/egg_carton_geom.py, gen_egg_carton_asset.py, verify_egg_carton.py] 신설 — 형상 단일 출처 / 생성기 / 검증기(일반 python3 + usd-core). 검증: 물리 0건·발행 필터·배치=fit·판 수평·직교 격자·컵 15개 동일·**예상** 착지 6개 자기 셀 안·겹침 ≤1mm. 예상 착지 = 직교화 격자 ee + 런 7 실측 매달림 변위. 다음 런 뒤 `FRUIT_REST_M` 을 새 실측으로 갱신해 재검증 (2026-09-11)
- [assets/props/egg_carton.usd, scenes/layers/layout_layer.usd, scripts/scene_tools/egg_carton_geom.py·gen·verify] **T4-3 계란판 3차** — 사용자 제공 실기 영상 정보(15구 계란판, 과실 3개를 인접 칸 slot 0·1·3 에 넣어도 닿지 않음) 반영. ① **컵 깊이 21 → 56mm**(림 z 22 → 57, 구멍 rx27/ry25 = 피치 허용 최대, 링 6단) — 시뮬 과실은 y 전폭 53.8 > 행 피치 51.2 라 이웃 행의 적도 대역 겹침은 형상으로 못 막고, 림을 그 대역(과실 중심 −1\~+13mm) 위 +20.5\~23mm 에 두어 겹침을 벽 안에 숨긴다(림 아래 15mm 이상). 밖으로는 어깨·줄기 6.5\~9mm. 과실은 키네마틱이라 바닥(z 1)은 그대로. ② **수평 중점을 테이블 중심축 y=0 에 정렬** — translate (0.7711, 0.0915→**0.1024**, 0), 격자 중점 world (711.3, −0.04)mm; 플래너 `taught_grid_shift_y_m=+0.0108` 과 쌍. geom 에 높이별 반폭 프로파일(hx/hy) · `GRID_SHIFT_Y_M`/`ideal_grid_shift_y_m()` · `carton_center_world_m()` 신설. verify 3차 게이트: 착지 6개 자기 셀 + **림 여유 ≥1mm**(최소 +3.3) + 꼭지 노출 >0 + 중점 y 편차 ≤1mm + run_nodes 인자·로그 대조 = geom 상수 + ideal 차 ≤1mm — 통과. 림 아래 벽 겹침(최대 5.9mm)은 정보로만. 3,148점/1,464면, 물리 0건. 슬롯 시퀀스는 미변경 (2026-09-11)
- [assets/props/egg_carton.usd, scenes/layers/layout_layer.usd, scripts/scene_tools/egg_carton_geom.py·gen·verify, scripts/run_nodes.sh] **T4-3 계란판 4차** — 런 8 화면에서 옆의 빈 컵으로 과실 적도가 벽을 뚫고 보임(3차의 "림 아래라 안 보인다"는 오판 — 벽은 양면 렌더). 사용자 요구 4건: ① 구멍이 과실 수평 최대 지름을 담게 → 배치 격자를 시뮬 전용 **정사각 68mm**(`taught_grid_pitch_override_m`)로, 컵 격자도 같게 ② 컵 안쪽은 **과실 프로파일 + 착지 치우침 + 여유 1.5mm** 로 유도(링 16단, 오버행 금지), 바닥 = 가장 낮게 놓인 과실 밑끝(런 8 실측 z 0.7) ③ 밑변 z 0 = 테이블 상판(밑면 제거) ④ 시퀀스 `0,1,6,7,12,13` → **`0,1,3,4,6,7`**(인접). 림 rx28.5/ry32.2(구멍 57.0×64.4), 능선 x 11.0/y 3.6, 림 z 57 그대로. `FRUIT_REST_M` 을 런 8 로 갱신하고 매달림 변위를 **수확 순서**로 재배정(`RUN_SLOT_SEQUENCE`/`RUN_GRID`/`PLACE_SLOT_SEQUENCE` 신설). translate (0.7705, 0.1360, 0), shift 0.0108 → 0.0452(판이 길어짐), 중점 world (702.5, 0.00). verify 4차 게이트: 벽 뚫림 0(최소 여유 +0.6) · 림 여유 ≥7.2 · 밑끝−바닥 +0.1\~+2.6 · 꼭지 노출 ≥6.3 · 밑변 = 상판 · 능선 ≥2 · run_nodes 의 pitch/shift/sequence = geom — 통과(컵 동일성 비교를 최근접점 방식으로 수정). 7,944점/3,863면, 물리 0건. 오프라인 도달성(런 8 retreat 시작): 6슬롯 중 slot 3 만 6/7(retreat#1 시작 swing J6 — 수확 순서상 해당 없음) (2026-09-11)
- [scenes/main_scene.usd, scenes/layers/layout_layer.usd, scenes/layers/physics_layer.usd, scripts/run_nodes.sh, scripts/scene_tools/egg_carton_geom.py·verify] **익은 딸기 6 → 8 (안 익은 6 → 4)** — 사용자 결정. `strawberry_unripe_01`(nw) → `strawberry_ripe_07`, `strawberry_unripe_03`(ne) → `strawberry_ripe_08`: 같은 애셋의 `ripeness` 변형과 prim 이름만 바꿈(위치·물리 조인트 위치 불변, 조인트 이름 `stem_ripe_07/08`, 덩굴 `vine_ripe_07/08`). 브릿지 발행 필터가 이름 기준이라 코드 수정 없이 발행 대상 8개. 총 12개 유지. 분면별 익은 수 nw 3 / ne 2 / sw 3 / se 0 → se 가지치기 유지, 적응 분할은 nw·sw. `taught_slot_sequence` `0,1,3,4,6,7,9,10`(slot 9·10 사전 검사 7/7). 계란판 도구는 착지 후보를 "8칸 × 실측 편차 6개"(landing_candidates)로 바꿔 과실→칸 매핑에 무관하게 검증 — 컵 봉투 동일, 애셋·translate 불변. 미검증: ripe_07(x −380) 파지 도달성 → 런 10 (2026-09-11)
- [관찰 — 분면별 접근 기울기가 배치 간격에 남는다] 런 7 실측: **sw 분면은 파지 접근을 10.1° 기울여 하고 nw/ne 는 0.1\~0.3°(거의 수평)** 다. 기울기가 다르면 과실이 툴 축으로 매달리는 깊이가 달라지고(248.9 vs 257.6mm, 차 8.7mm), 트레이에서는 모든 과실을 같은 그리퍼 자세로 놓으므로 그 차이가 **거의 그대로 world x**(툴 +Z 의 x 성분 0.989)로 나타난다. 그래서 분면 경계를 걸치는 열 쌍(slot 6↔7 = ne↔sw)만 중심거리 59.8 → 49.4mm, 간격 +6.0mm 다(나머지 두 쌍은 +16\~17mm, 이상값 +15.3mm). 수확 순서가 얕은 3 + 깊은 3 이라 **한 쌍은 구조적으로 두 그룹에 걸친다.** 분면별 접근 자세는 실기 티칭 값이고 T2 는 그 오차를 보존하도록 설계했으므로 **고치지 않는다.** 필요하면 `taught_slot_sequence:=0,1,7,12,13,6` 으로 그 쌍의 깊은 쪽을 먼저 놓아 벌릴 수 있으나 특정 수확 순서에 맞춘 튜닝이다 (2026-09-10)
- [assets/props/vine.usd, scenes/main_scene.usd, scenes/layers/layout_layer.usd] **T3 정적 덩굴** — 보드에서 각 딸기 줄기 끝까지 얇은 곡선 12개. 프로토타입 메쉬 1개(200점/191면)를 12번 참조하고 배치만 layout_layer 에서 준다 — 딸기 12개가 y·orient·scale 이 모두 같아 형상이 전부 동일하기 때문. **물리·콜라이더·조인트 없음**, 프림 이름에 `strawberry` 가 없어 브릿지 좌표 발행 필터에도 안 걸린다(§2 게이트: 플래너 입출력 불변). 수직 구간을 과실중심 +25\~+65mm 로 잡아 **플래너가 겨냥하는 두 점이 덩굴 중심선 위에 정확히 놓인다** — 파지 목표 +35mm(`pick_target_z_bias_m`)와 하강 시작 +65mm(`CRANE_Z_OFFSET_M`), 12개 전부 이탈 0.000mm. 열린 조우가 덩굴을 따라 내려와 물고, T2 부착으로 과실이 딸려 나가면 "덩굴 끝에서 분리"로 읽힌다. 색은 애셋 텍스처의 줄기 구간 UV 평균(sRGB 102,119,31). 끝을 (0,0) 수직으로 둔 이유는 ripe/unripe 메시 줄기가 서로 반대로 기울어 있어서다 (2026-09-10)
- [scripts/scene_tools/gen_vine_asset.py, vine_geom.py, verify_vines.py] 신설 — 덩굴 생성기와 검증기(일반 python3 + usd-core, Isaac 불필요). 검증은 조립된 씬에서 발행 필터·물리 0건·보드/줄기 접점·플래너 두 점·이웃 과실 간격 5항목을 본다. **딸기 배치를 바꾸면 다시 돌린다** — 현재 최소 여유는 `vine_ripe_06` vs `strawberry_ripe_04` 의 +1.3mm 로 얇다 (2026-09-10)
- [scripts/isaac_sim_script_editor_bridge.py] **T2 키네마틱 부착** — `/sim/grasp_event` 구독. ATTACH 면 좌표에서 60mm 안의 가장 가까운 익은 딸기 루트 prim 을 골라 그 순간의 `T_rel = T_fruit · T_gripper⁻¹` 를 잡고(그리퍼 밑동 `rh_p12_rn_base` 기준, **TCP 스냅 없음**), 매 물리 스텝 `fruit = T_rel · T_gripper` 로 따라간다. 부착 중: 강체 kinematic, 콜라이더 off, 줄기 FixedJoint off — 전부 **세션 레이어**에만 씀(저장해도 씬에 안 박힘). RELEASE 면 그 자리에 정지(kinematic 유지, 콜라이더 계속 off — 조우 사이의 부동체가 팔을 튕기지 않게). **부착·해제된 딸기는 `/isaac_sim/strawberries` 에서 제외** — 안 빼면 계란판(분면 격자상 se) 의 딸기가 다시 타겟으로 잡혀 가지치기가 깨진다. 런타임 FixedJoint 생성 없음. 그리퍼 자세는 물리 뷰(RigidPrim) 우선, 실패 시 USD xform 폴백 (2026-09-10)
- [scripts/isaac_sim_script_editor_bridge.py + scenes/lab_environment.usd + scenes/layers/physics_layer.usd] **T4c 낙하 물리 (2026-09-15)**: RELEASE 위치가 계란판 세계 상자 밖이면 과실을 동적 강체로 되돌려(kinematic 해제·콜라이더 복구·속도 0, 줄기 조인트는 꺼진 채) 중력으로 떨어뜨린다 — 이송 계획 거부로 놓은 과실이 공중에 떠 보이던 T2 표현을 없앤다. 부착 중 세션 레이어에 쓴 xform 의견은 낙하 시 제거(PhysX 루트 레이어 쓰기가 드러나도록; 헤드리스 검증 09-15, 변형 B 매 스텝 복사는 텔레포트로 속도 리셋되어 기각). 바닥 `floor`(시뮬 소품, 상판 아래 0.75m 가정) + 콜라이더 신설 — 보드 앞 릴리스 지점은 테이블 밖. Kit 로그 `RELEASE … PLACED in tray` / `DROPPED outside tray … dropped=n`
- [scripts/hud/status_bus.py + bus_merge.py + harvest_probe.py + isaac_sim_viewport_display.py(당시 isaac_sim_hud.py) + hud/make_labels.py] **T4c HUD 낙하 카운트 (2026-09-15)**: `result.dropped`(소유 planner) — 프로브가 실행기 `_release_and_continue_after_place_failure` 경계에서 +1(`PLACE_GATE_BLOCKED_*` 제외), 완료 블록 둘째 줄 `배치 n · 낙하 m`(라벨 `final_placed`·`final_dropped`). 실행기·플래너 무수정
- [scene_tools/gen_random_layout.py + scripts/run_batch.sh + scripts/isaac_batch_orchestrator.py + scripts/run_metrics.py] **T4d 무작위 배치 도구 (2026-09-15)**: 시드 → 과실 12개 x·z(익은 8/미숙 4 이름 고정, x ±0.42·z 0.35\~0.90·간격 ≥75mm·덩굴 통로·경계 15mm) → 세 파일 동시 기록/복원; 파일 신호(`/tmp/harvest_batch/`)로 Isaac 이 Stop·재로드·브릿지·HUD·Play 를 반복; ROS 쪽이 노드 재기동·트리거·완주 대기·로그 수집·지표(CSV, Wilson CI). 실행기·플래너·브릿지 무수정
- [scenes/layers/appearance_layer.usd(신설·생성물) + scenes/main_scene.usd:subLayers + scripts/scene_tools/gen_unripe_appearance.py(신설)] **안 익은 딸기 외형 (2026-09-17)**: 비대상 과실(/World 직계, 이름에 `unripe` — 브릿지 발행 필터와 같은 규칙, 개수·이름 하드코딩 없음)의 바인딩 UsdPreviewSurface `inputs:diffuseColor` 를 새 오버라이드 레이어에서 연한 녹백색 sRGB #B4D69A(linear 0.456·0.672·0.323) 상수로 덮고 텍스처 연결은 명시적 빈 연결(`connect = None`)로 막음 — 애셋(assets/strawberry/**) 파일 31개 md5 전후 동일, 노말·러프니스 연결 유지, 선택 근거 CIELAB C* 34·순백 ΔE 39·회백색 ΔE 32·익은 텍스처 평균 ΔE 59, main_scene 은 subLayers 맨 앞 한 줄 추가뿐. 생성기가 셰이더마다 `[appearance]` 한 줄 + 저장본 재오픈 검증. RTX 화면 반영은 씬 재로드 뒤 확인 필요
- [scripts/hud/scene_fruit.py(신설) + isaac_sim_viewport_display.py:_row_counts·_update_scene + hud/make_labels.py(count_targets·count_non_targets)] **HUD `타겟 N / 비대상 M` 상시 표시 (2026-09-17)**: 노드 행 아래. 숫자는 1초마다 열린 씬의 /World 직계 과실 prim 을 브릿지 발행 필터 규칙으로 세고(고정값 없음), 바뀔 때마다 Kit 로그 `[hud] scene fruit: target N / non-target M` 한 줄 + ripeness variant·이름 불일치 경고 + 비대상 상수 diffuse(외형 오버라이드 반영) 수 한 줄
- [scripts/hud/result_bar.py(신설) + status_bus.py:append·result.outcomes + bus_merge.py:load + harvest_probe.py:_attach_planner + isaac_sim_viewport_display.py:_row_result·_update_result + hud/make_labels.py·labels/] **HUD 결과 바 (2026-09-17)**: `수확 완료 n / N` 바로 아래에 씬 타겟 수만큼 칸(단계 진행 바와 같은 높이 10·간격 3·모서리 2), 시도 순서대로 초록 배치 성공(트레이 배치 실행기 `success`) / 빨강 배치 실패(그 밖의 상태) / 호박색 분리 실패(파지 판정 CONTACT 인 픽에서 `execute_detach_and_retreat` 가 None — 후퇴 실패로 시퀀스 잡힘), 결과 없는 타겟(파지 전 건너뜀·파지 판정 실패)은 회색. 범례 `배치 성공 n · 배치 실패 m · 분리 실패 k` 는 0 도 표시하고 종전 완료 둘째 줄을 대체(라벨 22px). 이 변경에서 실행기·상태 머신·ROS 인터페이스는 바꾸지 않았다 — 프로브가 메서드 반환값을 읽어 칸을 붙이고 칸마다 노드 로그 `[harvest_probe] 결과 바 n번째 = …` 한 줄, HUD 도 `[hud] result bar n/N: …` 한 줄. bus_merge 는 planner 의 run.started_at 이 scan 과 다르면 직전 런 결과를 안 가져온다. `result.dropped` 는 계속 센다(화면 미표시)
- [README.md:74, PROJECT_SUMMARY.md:60, portfolio/README.md:132·146·155·159·161, portfolio/H_scope_decisions.md:334, hud/make_labels.py(final_dropped 라벨)] **용어 정리 (2026-09-17, 사용자 지정)**: 상태는 `배치 실패`, 로봇의 대처 동작은 `낙하`, 분리 단계 실패는 `분리 실패` — 면접관에게 노출되는 화면·문서 중 HUD 표기를 가리키던 곳만 `배치 n · 낙하 m` → `배치 성공 n · 배치 실패 m · 분리 실패 k` 등으로 바꾸고, 자막 예시는 원인 없이 폴백 동작만(`트레이로 옮기지 못하면` / `그 자리에서 놓고 계속 진행`). 물리 사건·과거 런 기록·내부 문서(hud/README·run_guide 등)의 '낙하' 는 그대로
- [scripts/scene_tools/check_camera_framing.py:UI] **구도 검산 HUD 영역 아래 끝 0.43 → 0.49 (2026-09-17)**: 타겟 수 행·결과 바·범례로 HUD 패널 아래 끝이 1080 기준 약 516\~528px(스텁 레이아웃 추정). 롱샷 구도 검산 결과는 그대로 `UI overlap: none`
- [docs/result_display_audit.md(신설) + log/m3/offline_checks/result_display_20260917/(신설) + hud/README.md·docs/run_guide.md·docs/usd_structure.md·SUBMISSION_PLAN.md·PROGRESS_REPORT.md·portfolio/README.md·portfolio/H_scope_decisions.md §9·log/m3/README.md·README.md] **09-17 조사·결정·구현 기록 (2026-09-17)**: 배치 실패 4건 로그 조사(A), 분리/배치 실패 구분 가능성(B), '낙하' 표기 전수(C), 딸기 머티리얼(D), 자막·용어·결과 바 결정, 구현 1\~4 와 검증을 한 문서로 남기고 현재 화면·레이어·절차를 설명하던 문서를 새 HUD 기준으로 맞춤. 검증 보고 원문·생성기 출력·check_params 출력 보존(하네스 스크립트는 세션 스크래치라 남지 않음)
- [scripts/hud/result_bar.py:outcome_of_pick_end(신설, outcome_of_detach 삭제) + harvest_probe.py:_attach_planner(픽 상태 entered·recorded·why 로 확장, `_record` 래치, `_after_run` 신설) + status_bus.py(docstring) + make_labels.py:STATE_KO·labels/manifest.json(state_DETACH.png 재생성) + isaac_sim_viewport_display.py(영문 라벨·docstring) + hud/check_result_bar_probe.py(신설, 오프라인 검사)] **결과 바 '분리 실패' 범위 재정의 (2026-09-18, 사용자 정의)**: 트리거는 09-18 런(`run_logs/20260918_094517`) — root/ne/sw(280,783,700)mm 가 4번째 시도의 45mm 직선 진입에서 브릿지 IK 실패로 거부돼(`bridge.log:144`) 플래너가 `ABORT: 직선 진입 실패`(`planner.log:142-143`)로 접고 재스캔에서 재시도 없이 넘어갔는데, 결과 바가 이 픽을 회색으로 남겨 사용자가 기대한 호박색 칸이 안 나왔다. '분리'의 목표를 그리퍼가 과실 앞에 멈춰선 뒤(프리어프로치 도달 — 경로는 이미 계산됨) 진입·하강·파지·당김·후퇴를 거쳐 **배치 경로 계산 직전까지 과실을 집은 상태를 만드는 것**으로 다시 잡았다(1안 + B안 채택). 09-17 의 좁은 정의(CONTACT 판정 픽에서 `execute_detach_and_retreat` 가 None)는 폐기 — 보존 런 23개·182시도에서 0건이라 화면에서 자리값을 못 했고, 실제로 난 직선 진입 실패 5건·열린 조우 하강 실패 1건이 회색으로 사라지고 있었다(`log/m3/README.md` 09-18 절). 판정 지점은 실행기 `run()` 이 끝난 직후 한 곳(`result_bar.outcome_of_pick_end`)으로 모았다 — 실패 출구를 하나하나 열거하지 않으므로 빠지는 출구가 없다. 결과 칸은 픽당 정확히 하나만 붙는다(`harvest_probe.py` 의 `pick["recorded"]` 래치 — 배치 결과와 run 종료 판정이 겹쳐도 칸이 두 개 붙지 않는다). 회색 = 프리어프로치에 도달하지 못한 타겟(파지 후보 전부 IK 실패, 프리어프로치 스플라인 실패, x·z 가드 스킵) — 사용자 정의의 전제("경로를 계산해 낸 이상")를 못 넘은 시도만 회색으로 남는다. 진입 뒤 `run()` 이 예외로 끝나도 분리 실패 칸이 붙는다(`_wrap` 의 `always`, 예외는 통과). 픽 상태는 스레드별(재진입 콜백 그룹 대비). 단계 진행 바의 DETACH 라벨은 '분리' → **'당김'**(`make_labels.py` STATE_KO, `labels/state_DETACH.png` 재생성, 영문 폴백 "PULL")로 바꿔 결과 바의 '분리 실패'와 같은 글자가 한 화면에 두 뜻으로 뜨지 않게 했다. 당김 명령 하나만 실패한 경우는 실행기가 무시하고 후퇴·배치를 계속하므로 결과 칸에는 영향 없이 로그 한 줄만 남긴다(`[harvest_probe] 당김 명령 실패`). 실행기·상태 머신·ROS 인터페이스는 이번에도 바꾸지 않았다 — 프로브가 메서드 반환값을 읽어 칸을 붙일 뿐이다. 오프라인 검사 `hud/check_result_bar_probe.py`(신설, 실행기 원본 클래스에 프로브를 실제로 붙여 시나리오별 판정을 재현) 시나리오 18개 통과(09-18). 판정의 한계(동작 명령이 배치 호출까지 갔는지만 본다, 물리적 분리는 어디서도 판정하지 않는다, `DETACH_UNVERIFIED`)는 09-17 과 동일 — `hud/result_bar.py` 상단 docstring 참고
- [scripts/isaac_sim_viewport_display.py:harvest_key(신설)·_row_result·_on_update + hud/make_labels.py:HARVEST_KO·labels/(harvest_*.png 4장 신설) + hud/harvest_probe.py:_attach_scan(_run_end) + hud/status_bus.py(docstring) + scene_tools/check_camera_framing.py:UI] **HUD 결과 바 위 줄 상시 표시 (2026-09-19, 사용자 요청)**: 완료 때만 보이던 `수확 완료` 줄 자리에 트리거 전 `수확 시작 전`·런 중 `수확 진행 중`(흐린 회색)을 띄워 런 중 단계 진행 바와 결과 바가 붙어 보이지 않게 했다. 완주하면 종전대로 `수확 완료 n / N (p%)`, 완료 호출 없이 끝난 런은 `수확 중단`(프로브가 `run.ended_at` 을 찍는다, 노드 재기동부터). 런 중 패널 아래 끝 약 516(또는 528) → 578px, 구도 검산 HUD 영역 아래 끝 0.49 → 0.54(롱샷 `UI overlap: none` 그대로). 완주 때 2단 트리는 접지 않고 세부 칸·연결선만 숨겨 높이를 남긴다(사용자 결정 — 접으면 결과 바 위 줄이 상시라 단계 줄부터 범례까지 54px 올라간다; `_TreeView.update` 의 `_l2_box.visible` 토글 삭제, `tree_model.view` 는 그대로 group None). manifest 의 `final` 항목은 `harvest_done` 으로 옮겼고 옛 `labels/final.png` 는 표시 스크립트 재 Run 뒤 지웠다. 실행기 코드는 그대로
- [scripts/isaac_sim_script_editor_bridge.py:_publish_state] `/isaac_sim/strawberries` 를 **빈 배열이어도 발행**. `if len > 0` 가드가 T2 이후 "익은 과실이 하나도 안 보임" 상태를 침묵으로 만들었다 (위 fake_vision 항목과 한 쌍) (2026-09-10)
- [assets/robot/robot_assembly.usd] D455 중첩 강체 문제로 `physics:rigidBodyEnabled=False` 오버라이드, 드라이브 게인 상향(팔 1e5/1e4, 그리퍼 1e4/1e3)
- [scenes/layers/layout_layer.usd] 보드를 −109.5mm 이동해 앞면을 플래너 벽 모델(672.0mm)에 정합, 딸기 매립 해소 (2026-09-05)
- [scenes/layers/layout_layer.usd, physics_layer.usd] 딸기 2 → 12개(익은 6/안 익은 6)로 확장, 보드 4등분 서브셀에 배치 (2026-09-07)
- [assets/props/cell_markers.usd] 신규 — 서브셀 꼭지점 9곳에 300mm 마커 봉 (시각 전용, 콜라이더·탐지 대상 아님)
- [scripts/isaac_sim_script_editor_bridge.py] 안 익은 딸기를 발행 대상에서 제외 (실기 fusion 노드가 ripe만 발행하는 것에 맞춤)

- [scripts/curobo_planner_node.py] `ee_to_tcp_offset_m` ROS 파라미터 신설 — 툴 길이 모델 정정 (2026-09-07)
- [scripts/grasp_candidate_policy.py] `x > 0.25` 파지 오프셋 사다리에 표준 후보 폴백 추가 (2026-09-07)
- [config/curobo/e0509_gripper.urdf|.yml|_measured_tcp.yml|e0509_spheres.yml] 커스텀 3D 프린팅 파츠를 충돌 모델에 추가 (2026-09-07)
- [config/environment.yaml, config/scan_collision_world.yaml] cuRobo 충돌 월드에 보드 큐보이드 추가 (2026-09-07)
- [scenes/layers/physics_layer.usd] 딸기 줄기 파단력 2 → 20N / 1 → 5Nm 상향 (2026-09-07)
- [scenes/layers/layout_layer.usd] 보드를 실제로 평평한 y=672.0mm 에 정합 — 3° 기울기 제거 + 두께 0 전제 정정 (2026-09-07)

- [scripts/planner_bootstrap.py] `enable_open_stem_descent` ROS 파라미터 신설 (2026-09-08)
- [scripts/pick_sequence_executor.py] open-stem descent 게이트를 `_crane_offset_active()` 로 분리 (2026-09-08)
- [scripts/grasp_candidate_policy.py] `grasp_variant_pose` 게이트를 플래그화 + `legacy_grasp_endpoint` 에 crane offset 인자 추가 (2026-09-08)
- [scripts/grasp_search_executor.py] `try_legacy_grasp_offsets` 에 crane offset 전달 (2026-09-08)
- [config/curobo/*.yml] `lock_joints` 0.0 → 0.857 (계획 개도를 접근 개도 stroke 600 에 일치) (2026-09-08)
- [config/curobo/*, config/*.yaml] 설정 파일 전체를 **ASCII 전용**으로 정리 (2026-09-08)

- [scripts/pick_sequence_executor.py] open-stem descent 의 `reached_z` 를 TCP 기준으로 보정 (2026-09-08)
- [scripts/harvest_motion_params.py] `MAX_HARVEST_JOINT_DELTA_DEG[J1]` 75 → 95도 (2026-09-08) — **2026-09-14 원복 75**
- [scripts/harvest_motion_params.py] `MAX_TAUGHT_PLACE_TRANSFER_JOINT_DELTA_DEG[J3]` 120 → 175도 (2026-09-08) — **2026-09-14 원복 120**

- [assets/robot/robot_assembly.usd] 커스텀 파츠 부착각 6° splay 제거 (localRot0 오버라이드) (2026-09-08)
- [robot.urdf, config/curobo/e0509_gripper.urdf] 부착 조인트 rpy pitch 0.10472 → 0 (2026-09-08)

- [sim_executor_bridge_node.py] stroke→rad 배율 1.0 → URDF 한계 1.101, 시각 개도를 리드백이 아닌 **명령값** 기준으로 (2026-09-08)
- [scenes/layers/layout_layer.usd, config/curobo/*.yml] 개도 600 대응각 0.857 → 0.944 rad (2026-09-08)
- [assets/props/whiteboard.usd] 보드를 슬레이트 틸(#3E7383) 단색으로 (2026-09-08)
- [docs/run_guide.md] `pick_target_z_bias_m:=0.035` 추가 (2026-09-08)

- [sim_executor_bridge_node.py] MoveLine 을 5mm 직선 보간으로 (IK 한 번 점프 → 다단계) (2026-09-08)
- [sim_executor_bridge_node.py] 조우 상한을 파츠 접촉각 1.0826 rad 으로 (관통 방지) (2026-09-08)
- [assets/strawberry/strawberry_ripe.usd, strawberry_unripe.usd] `inputs:opacity` 오연결 제거 (2026-09-08)

- [docs/run_guide.md] `pick_target_z_bias_m` 0.035 → 0.020, `allow_generated_tray_slot_release:=true` 추가 (2026-09-08)
- [status_monitor_node.py] 딸기 수신 개수 표시 (Isaac→fake / fake→플래너) (2026-09-08)

- [sim_executor_bridge_node.py] IK 솔버에 보드 충돌 월드 주입 (MoveLine 관통 차단), IK seed 8→20 (2026-09-08)
- [assets/robot/robot_assembly.usd] 커스텀 파츠 **링크 xformOp:orient** 의 6° splay 제거 (2026-09-08)

- [execution/scan_executor_node.py] 서브셀 격자를 탐지 bbox 중점 → **보드 고정 좌표**로 (2026-09-08)

- [scenes/layers/layout_layer.usd, physics_layer.usd] ripe_01·ripe_03 을 도달 가능 위치로 이동 (2026-09-08)

- [sim_executor_bridge_node.py] ★ IK 시드를 `seed_config` 로 전달 (elbow-flip 근본 원인) (2026-09-08)
- [sim_executor_bridge_node.py] 보간 5mm→2mm, num_seeds 24→32, 판정반경 35→45mm (2026-09-08)
- [docs/run_guide.md] `pick_target_z_bias_m` 0.020 → 0.035 (2026-09-08)

- [harvest_motion_params.py] J6 운용 한계 ±225 → ±360 (URDF 실제 한계와 동일) (2026-09-08)
- [sim_executor_bridge_node.py] 그리퍼 배율 1.101 → 1.0, MoveSplineJoint 웨이포인트 보간 (2026-09-08)
- [status_monitor_node.py] `PICK COMPLETE` 오인식(→"분리") 수정 (2026-09-08)

### 2026-09-08 — 7차 관찰 4건 (시드 수정 후 파지는 6/6 성공)

**③ 가위 모양 — USD 관절 한계가 쌍마다 다르다 (반복 실패의 진짜 원인)**
`robot.urdf` 의 그리퍼 관절 상한:

| 관절 | upper |
|---|---|
| `rh_p12_rn`(r1), `rh_l1` | **1.1 rad (63.03deg)** |
| `rh_r2`, `rh_l2` | **1.0 rad (57.30deg)** |

RH-P12-RN 은 평행사변형이라 네 관절이 **같은 각도**여야 손가락이 회전하지 않는다.
직전 커밋에서 배율을 1.101 로 올린 것이 회귀였다 — 네 관절에 같은 값을 보내면 l2/r2 만
1.0 에서 잘려 각도가 어긋나고, 손가락이 기울며 **몸통은 겹치고 팁은 벌어지는 가위**가 된다.
실측(stroke 700, 비율 유지 시): 전체 최소 −33.4mm(겹침) / 팁 +21.1mm(벌어짐).
→ 배율을 **1.0** 으로. stroke 700 에서 네 관절 모두 57.30deg = l2/r2 상한에 정확히 닿고,
평행을 유지한 채 간격 9.4mm 로 닫힌다. 접촉각 클램프(1.0826)는 l2 한계를 넘으므로 제거.
씬 초기 개도 49.11deg, cuRobo `lock_joints` 0.857 로 동기화.

**① 파지 성공 후 보드 앞에서 놓음 — J6 랩(wrap)**
`spline_jump` 거부 4건. cuRobo 가 J6=273.7deg 해를 내는데 `normalize_equivalents` 가
운용 한계 안의 등가값만 고를 수 있어 −86.3deg 를 택하고, 176.6→−86.3 = **263deg 점프**가
되어 가드(270deg)에 걸린다. `robot.urdf` 의 joint_6 한계는 **∓360deg** 이므로 273.7 을
그대로 쓸 수 있다 → 운용 한계 ±225 → **±360**. 검증: 점프 263deg → **97.1deg**, 통과.

**② 배치 경로가 보드에 부딪힘 — 스플라인 다운샘플**
플래너는 cuRobo 궤적을 **12점으로 다운샘플**해 보내는데, 브릿지가 그 12점을 0.1s 간격으로
던지기만 해서 실제 경로가 웨이포인트 사이를 드라이브가 제멋대로 잇는 모양이었다.
→ 관절공간에서 2deg 간격으로 보간해 계획 궤적을 따르게 했다.

**④ 상태창이 '분리'를 오인식**
플래너는 완료를 `=== PICK COMPLETE (DETACH_SUCCESS_UNVERIFIED) ===` 로 찍는데, 상태표에
공백형 `PICK COMPLETE` 가 없어 안 걸리고 대신 부분문자열 **`DETACH`** 에 걸렸다.
→ 공백형을 먼저 추가하고, `DETACH` 는 `DETACH_PULL_DOWN`/`detach pull` 로 좁혔다.

---

### 2026-09-08 — ★ elbow-flip 의 진짜 원인: IK 시드를 안 주고 있었다

18회 픽 중 4회만 파지 성공. 실패의 대부분이 `MOTION_FAIL FINAL_APPROACH_STRAIGHT`(9)과
`MOTION_FAIL OPEN_STEM_DESCENT`(3) 이었다.

**원인** — cuRobo 시그니처는
`solve_single(goal_pose, retract_config, seed_config, ...)` 인데 브릿지는 현재 관절을
**2번째 위치인자 = `retract_config`** 로 넘기고 있었다. 그 자리는 널스페이스 정규화 목표이지
**시드가 아니다.** 그래서 cuRobo 는 매번 무작위 seed 에서 최적화를 시작했고 어느 IK 브랜치로
수렴할지 보장이 없었다. `docs/concepts.md` 가 "같은 목표 30번 중 6번이 팔꿈치 반전"이라고
적어둔 그 현상의 **원인이 바로 이것**이다. 최근접 선택(return_seeds)은 후보가 전부 먼
브랜치일 때 아무것도 못 한다.

**실측** (45mm 직선을 보간, 익은 딸기 6개, 스텝당 최대 관절 이동량):

| 방식 | 결과 |
|---|---|
| `retract_config` 만 (종전) | **6/6 실패**, 179~181deg 팔꿈치 반전 |
| `seed_config` 로 현재관절 시드 | **6/6 통과**, **1deg** |

→ `seed_config=start.view(1,1,-1).repeat(1,num_seeds,1)` 로 수정.
아울러 직전 커밋에서 `num_seeds` 를 기본값(100)에서 24 로 낮춘 것이 회귀였다 → 32 로.

**부수 수정**
- 보간 간격 5mm → **2mm** (스텝당 1deg 라 촘촘하게 해도 안전). 5mm 스텝이 화면에서
  "깔짝깔짝" 끊겨 보이던 문제 해소.
- `pick_target_z_bias_m` 0.020 → **0.035**. 딸기 2개를 옮긴 뒤 재검증하니 6개 모두 4/4
  도달한다. 20mm 는 과실 중심 위 20mm = 과실 윗면(31.3mm)보다 **아래**라 줄기가 아니라
  과실을 물고 있었다.
- `grasp_capture_radius_m` 0.035 → **0.045**. 판정은 TCP↔과실중심 거리인데 bias 35mm 면
  정상 파지에서도 d_tcp≈38mm 라 35mm 임계값과 겹쳐 동전던지기가 된다(실측 33.9~36.2mm).

---

### 2026-09-08 — 딸기 2개 이동 + 보드 거리 이력 재확인

**딸기 이동** — `ripe_01`(−250,880)·`ripe_03`(350,800) 은 작업영역 가장자리라
pre-approach IK 가 전부 실패했다(실측: IK_FAIL×7 / IK_FAIL×5+spline_jump×4).
cuRobo IK 로 (x,z) 격자를 훑어 안전 영역을 확인한 뒤 이동:

| 딸기 | 종전 | 변경 | 서브셀 | 도달 |
|---|---|---|---|---|
| ripe_01 | (−250, 880) | **(−250, 740)** | nw 유지 | 4/4 |
| ripe_03 | (350, 800) | **(200, 740)** | ne 유지 | 4/4 |

6개 전부 4/4 변형 도달, 최소 간격 67mm, `physics_layer.usd` stem joint `localPos0` 동기 확인.

**보드 거리 이력 재확인 (사용자 문의)** — 기억이 맞다. 원본 씬은 `translate.y = 0.81`
(보드 앞면 781.5mm)였고 2026-09-05 M2 에서 672mm 로 −109.5mm 옮겼다.

그런데 그 근거인 "672 는 실기 벽 실측값"에 **1차 출처가 리포 어디에도 없다.**
원본 플래너(`_baseline/A_strawberry_motion/scripts/harvest_motion_params.py:62`)에도
`WALL_SURFACE_Y_M = 0.672` 한 줄뿐, 출처 주석이 없다. 그 판정은 2026-09-05 세션이
"잔재가 아니라 실측값"이라고 **스스로 뒤집은 서술**이 유일한 근거다.

**다만 공학적으로는 672 가 낫다.** 두 거리에서 pre-approach IK 도달 범위를 실측 비교:

| 보드 y | z=880 | z=800 | z≤740 |
|---|---|---|---|
| **672mm** | 일부 도달 | 대부분 4~3변형 | 전 구간 4~3변형 |
| **810mm** | **전 구간 도달 불가** | 1~2변형으로 악화 | x 양끝 악화 |

보드를 멀리 두면 작업영역이 **줄어든다**. 따라서 **672 를 유지한다.**
실기 벽이 실제로 810mm 라는 외부 근거가 나오면, 그때는 시뮬을 실기에 맞추되
딸기를 아래로 내려 도달성을 확보해야 한다.

---

### 2026-09-08 — 서브셀 격자를 보드 기준으로 고정

**확인 결과 씬은 문제가 없었다.** 로봇 `base_link` 는 월드 원점, 보드는 `(50, 672, 660)mm`
절대 좌표에 이미 고정돼 있다. 보드를 로봇/테이블에 "연결"할 것은 없었다.

문제는 `_group_poses_by_subcell` 한 곳이었다. 격자를 **탐지된 딸기들의 bbox 중점**으로 매번
다시 계산했다 (`x_mid=(max+min)/2`). 딸기를 하나씩 수확·블랙리스트하면 남은 것들의 bbox 가
줄어 **경계선이 움직이고**, 같은 딸기가 패스마다 다른 서브셀로 분류됐다. 실측 재현:

| 남은 딸기 | ripe_04 분류 (종전) | (수정) |
|---|---|---|
| 6개 전부 | sw | sw |
| 04,05,06 | **nw** ← 물리적으로 sw 인데 | **sw** |
| 05,06 (ripe_05) | **se** | **sw** |

→ 보드 실측에서 뽑은 고정 상수로 교체:
`BOARD_SUBCELL_X_MID_M = 0.050`, `BOARD_SUBCELL_Z_MID_M = 0.660`
(보드 x[−495, +595] 중심 50, z[265, 1055] 중심 660). 1개일 때 무조건 sw 로 몰던 특수분기도 제거.

**영향 범위는 순서/라벨뿐이다.** 이 함수는 수확 순서 휴리스틱(sw→se→nw→ne)일 뿐,
어떤 딸기를 집을지는 바꾸지 않는다. 그래서 **딸기 좌표를 bbox 중점이 보드 중심과 일치하도록
맞춰 둘 이유도 사라졌다** — `PORTFOLIO_SPRINT.md` 씬 메모의 그 제약은 이제 무효다.

---

### 2026-09-08 — 6차 관찰 4건

**④ 여전히 가위 모양 — 조인트만 고치고 링크를 안 고쳤다**
앞서 `physics:localRot0`(조인트 프레임)만 6°를 뺐는데, **렌더·기구 포즈는 링크 자신의
`xformOp:orient` 를 쓴다.** 그 값에 splay 가 그대로 남아 있었다.
검증: `authored ⊗ rpy(∓90°,6°,∓90°)⁻¹` 를 계산하면 부모(`rh_p12_rn_l2/r2`)의 authored 자세
`(0.7071081, 0, 0, 0.7071055)` 가 정확히 나온다 → 6° 만 제거하면 된다.
→ `robot_assembly.usd` 에 링크 orient 오버라이드 추가 (`quatd`, 부모 속성 타입과 일치).

**② 이동 중 그리퍼가 보드를 관통 — 브릿지 IK 에 충돌 월드가 없었다**
`IKSolverConfig.load_from_robot_config(robot_cfg, tensor_args=..., ...)` — **2번째 인자
(world_model)를 생략**해 이 솔버는 장애물을 전혀 몰랐다. 플래너(MotionGen)는 보드를 피해
waypoint 를 내지만 **그 사이를 잇는 MoveLine 구간은 전부 이 솔버가 푼다** (최종 직선 진입,
줄기 하강, 분리 당김, retreat, 배치 이송). 그래서 이동 중 관통이 났다.
→ `environment.yaml` 을 읽어 플래너와 동일한 보드 큐보이드를 주입.
검증: 정상 파지 포즈(플랜지 y=394mm)는 해가 나오고, 보드 관통 포즈(y=520mm)는 거부된다.

**③ 4개만 시도한 게 아니라 6개 전부 시도했다**
런타임 로그(`curobo_planner_node_20260908T064708`): `pick_sequence_start` 6건,
`pick_target_prepared` 6건. 5·6번째가 **1초 안에 ABORT** 해서 눈에 안 보였을 뿐이다.

| # | 타겟 | 결과 |
|---|---|---|
| 1 | (−100, 440) | 계획 OK → `FINAL_APPROACH_STRAIGHT success=false` |
| 2·3·4 | (−250,500) (−220,560) (−120,760) | `GRASP_CONTACT_DETECTED` pos=670 ✅ |
| 5 | (−250, **880**) | IK_FAIL ×7 (goal z 905~981mm) |
| 6 | (**350**, 800) | IK_FAIL ×5 + `spline_jump` reject ×4 |

5·6번은 작업영역 가장자리(최상단/최우측)다. `PORTFOLIO_SPRINT.md` 씬 메모가 이미
"⚠️ 도달성 미검증 … 나면 x 범위를 줄인다"로 예고한 항목이다.

**① 첫 딸기 직선 진입 실패** — 보간은 적용됐는데도 실패했다. 브릿지 터미널 로그를 못 봐서
어느 스텝에서 깨졌는지 미확정. IK seed 를 8→20 으로 올렸고, 실패 시 `step i/n` 이 찍히므로
다음 런에서 지점이 특정된다.

---

### 2026-09-08 — 5차 관찰 7건

⚠️ **이 런(06:11 기동)은 브릿지 수정(06:23)·딸기 재질 수정(06:22) 이전 버전이다.**
씬 파일(whiteboard 06:07, layout_layer 06:05, robot_assembly 05:42)만 반영돼 있었다.

| # | 관찰 | 원인 / 조치 |
|---|---|---|
| ① | 제대로 집는데 빈손 판정 | **`z_bias`(35mm)와 capture radius(35mm)가 충돌.** TCP 를 과실중심+35mm(줄기)로 겨냥하는데 판정은 **과실 중심까지 거리**를 잰다 → d_tcp 가 설계상 ~35mm 라 임계값 위에서 동전던지기. 실측 34.9/36.2/36.0/33.9mm. → z_bias 20mm 로 낮춰 d_tcp≈25mm |
| ② | 첫 딸기 접근 중단 | `MoveLine IK Failed (best delta=175.1deg)` = elbow-flip. **직선 보간 수정이 이 런에 없었다.** 이미 반영됨 |
| ③ | 4번째 배치 후 중단, scan pose 대기 | 다음 타겟 `ripe_01`(−250,915) 이 **20개 후보 전부 IK_FAIL** → 즉시 ABORT → 대기처럼 보임. ⑤와 같은 원인 |
| ④ | ne 딸기 배치 시 바닥까지 안 감 | `TAUGHT_TRAY_SLOT1_RELEASE_BLOCKED` — slot≠0 은 **생성된(미티칭) 좌표**라 release 를 막고 Above(z=186.5mm)에서 정지. → `allow_generated_tray_slot_release:=true` |
| ⑤ | nw 좌상단 딸기 미인식 | **인식은 된다** (`PICK 딸기 raw=(-250,645,880)`). z_bias +35mm 로 pre-approach 가 996mm 가 되어 도달 불가. cuRobo IK 실측: bias 0/10/15/20mm 도달, **25mm 이상 전멸** |
| ⑥ | 딸기 수신 개수 안 보임 | 상태창에 `Isaac→fake N개 / fake→플래너 M개` 추가. 둘이 다르면 ripeness 필터·토픽 연결 문제 |
| ⑦ | 꽉 닫으면 가위 모양 | 접촉각 클램프(1.0826 rad)가 이 런에 없었다. 이미 반영됨 |

**z_bias 20mm 의 근거** — cuRobo IK 로 익은 딸기 6개 × bias 6단계를 전수 확인:

| 딸기 | z | 0mm | 10 | 15 | 20 | 25 | 35 |
|---|---|---|---|---|---|---|---|
| ripe_01 | 880 | ✓ | ✓ | ✓ | **✓** | ✗ | ✗ |
| 나머지 5개 | — | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |

20mm 가 6개 전부를 살리는 상한이다. 줄기 정확도(35mm)와 `ripe_01` 중 후자를 택했다.

---

### 2026-09-08 — 4차 관찰 4건

**① 다음 딸기로 넘어갈 때 이상한 위치에서 계획 — 설계상 그렇고, 원인은 ②다**
`return_to_pick_start_and_complete` 는 **scan pose 가 아니라 `pick_start_joints`**
(그 픽이 시작된 관절값, `pick_sequence_executor.py:793`)로 돌아간다. 주석은 "이번 pick이
시작된 scan pose"라고 적었지만 실제로는 "픽 시작 지점"이다. scan_executor 도 서브셀 안에서는
픽 사이에 scan pose 로 안 돌아간다(재스캔 패스에서만 `MOVING_TO`).
정상이라면 픽1이 scan pose 에서 시작하므로 계속 같은 자리로 수렴한다. 그런데
`_abort_pick_with_complete` 는 **자세를 복원하지 않는다**. 픽1이 직선 진입에서 abort 하면
로봇은 pre-approach 자세에 남고, 그 뒤 모든 픽의 "pick-start"가 그 자세가 된다.
실측: 픽1 start_J1=88.0(scan) → abort → 픽2 118.6 → 픽3 119.0 → 픽4 119.5.
→ **②를 고치면 표류가 사라진다.** 설계 자체는 바꾸지 않았다.

**② sw 첫 딸기 직선 진입 실패 — cuRobo 가 아니라 브릿지 가드**
`FINAL_APPROACH_STRAIGHT MoveLine failed` 는 브릿지의 `move_line_cb` 가 낸 것이다.
종전 구현은 요청 거리 **전체를 IK 한 번으로 점프**했고, 종점 해가 현재 자세에서
`MOVELINE_MAX_JOINT_DELTA_DEG`(45도)를 넘으면 통째로 실패했다. cuRobo 는 pre/grasp 두
지점 모두 `Plan OK` 를 냈으므로 도달 불가 위치가 아니다.
→ 5mm 씩 직선 보간, 직전 해를 seed 로 이어 풀도록 변경. 스텝당 이동량이 작아 가드에 걸리지
않고 **화면상으로도 실제 직선**이 된다(종전엔 순간이동이었다). 소요 시간도 요청 속도
(`req.vel[0]`)에 맞춰 `dist/vel` 로 재현한다. elbow-flip 가드 값 자체는 그대로 둔다.

**③ 조우가 서로 파고듦 — 접촉각에서 정지**
`physxArticulation:enabledSelfCollisions = False` 라 끝까지 닫으면 두 파츠가 2.1mm 관통한다.
이분법으로 측정한 **접촉각 1.0826 rad**(간격 0.00mm)을 상한으로 두어 소프트웨어 하드스톱을
만들었다. RH-P12-RN 자체 한계 1.101 은 파츠가 없을 때의 값이고, 파츠를 단 뒤의 **실효 한계**가
1.0826 이다.
> 진짜 콜라이더 물리로 막으려면 `enabledSelfCollisions=True` + 인접 링크 쌍 전부를
> filtered pair 로 제외(14링크 = 91쌍)해야 한다. 임포트된 팔 링크는 관절부에서 서로 겹쳐
> 있어 그대로 켜면 기동 즉시 발산한다. 화면 결과는 같으므로 하드스톱을 택했다.

**④ 딸기가 그리퍼 앞에 그려짐 — 재질 오연결**
딸기 `UsdPreviewSurface` 의 `inputs:opacity` 가 **`UsdPrimvarReader_float3`**(Blender
Attribute 노드 export 잔재)에 연결돼 있었다. float3 → float 타입도 안 맞는 무의미한 연결인데,
연결이 있다는 사실만으로 렌더러가 재질을 **반투명으로 분류**해 투명 패스로 그린다. 투명 패스는
불투명 지오메트리와 별도로 정렬되므로 깊이가 뒤집힌다.
→ 연결 제거, 상수 1.0 유지. 익은/안 익은 두 애셋 모두. (당시 스크래치패드 백업 `*.usd.bak` 은 보존하지 않음 — 이전 상태는 git 이력)

---

### 2026-09-08 — 3차 관찰: 완전히 닫히지 않음 / 과실 한가운데 파지

**① 조우가 완전히 닫히지 않음 — 원인 2개**

(a) **stroke→rad 배율이 기구 범위의 91%.** `gripper_joint_publisher.py:58` 의
"stroke 700 -> ~1.0 rad" 는 근사값인데 URDF `gripper_rh_r1` 한계는 **1.101 rad** 다.
KP1 은 5mm 네오디뮴 자석이고 실기는 "그냥 완전히 닫는" 계획이므로 700 = 기구 한계가 맞다.

(b) **시각 개도를 판정 리드백으로 구동하고 있었다.** 접촉 시 리드백은 670("물체에 걸려
멈춘 개도")인데 그 값으로 관절을 구동해 화면에서 덜 닫힌 채 멈췄다. 씬에 줄기 지오메트리가
없어 조우를 물리적으로 막을 것이 없으므로, 닫으라는 명령이면 끝까지 닫는 것이 맞다.
→ `gripper_command`(명령값)와 `gripper_position`(판정 리드백)을 분리.

| stroke | 용도 | 종전 팁 간격 | 수정 후 |
|---|---|---|---|
| 600 | 접근 | 25.6mm | **15.8mm** (5mm 자석을 감쌈) |
| 700 | 닫기 | 9.4mm | **−2.1mm** (완전히 닫힘) |

**② 과실 한가운데를 찌름** — 탐지가 주는 좌표는 **과실 중심**인데 파지 목표를 그대로 썼다.
KP1(줄기의 5mm 자석)은 과실 윗면(중심 +31.3mm) **위**에 있다.
→ `pick_target_z_bias_m:=0.035`. 시퀀스가 이렇게 된다 (ripe_01, 과실중심 880mm):
접근 948 → 하강 32.6mm → **닫기 915mm(줄기)** → BASE −Z 40mm → 875mm.
과실 윗면을 쓸어내리며 분리하는 실기 동작과 일치한다.

**③ 보드 색** — 흰 종이 격자에서 **커스텀 파츠(흰색 1,1,1)가 보드에 묻혀 안 보였다.**
슬레이트 틸 `#3E7383` (linear 0.0482, 0.1714, 0.2270) 단색으로 교체. 흰 파지부·밝은 회색
팔·빨간 과실(보색)·진회색 그리퍼를 모두 분리한다. 무광 유지.

---

### 2026-09-08 — ② 재진단: 조우가 안 닫히는 진짜 원인은 부착각 6°

첫 진단에서 "과실이 팁 사이로 통과한다"고 적었는데 **초점이 틀렸다.** 물리 파지를 쓰지 않는
현재 구성에서 문제는 과실 걸림이 아니라 **커스텀 파츠가 줄기를 쥘 만큼 닫히지 않는 것**이다.

`robot.urdf` 의 `l2_to_left_custom_part` / `r2_to_right_custom_part` 오리진 rpy 에 pitch
**+0.10472 rad(6°)** 가 들어 있어 두 파츠를 바깥으로 벌린다. 밑동은 닫히는데 팁만 벌어지는
깔때기가 된다:

| 부착 pitch | stroke 600 | stroke 700 | 기구한계 1.101 |
|---|---|---|---|
| **+6° (종전)** | 55.1mm | **38.9mm** | 27.4mm |
| **0° (수정)** | 25.6mm | **9.4mm** | −2.1mm (겹침) |

→ `robot_assembly.usd` 에 `localRot0` 오버라이드로 pitch 0 적용 (USD 재임포트 불필요 —
이 파일이 커스터마이징 레이어다). `robot.urdf` 와 cuRobo URDF 2사본도 함께 0 으로.
robot.urdf 원본 주석이 이 오리진을 "나중에 수정 가능"한 임시값으로 적어 두었다.

**영향 재검증**: 툴 팁 도달 260.8 → 261.6mm, cuRobo 구체 도달 267.7 → 268.8mm,
보드 여유 10.5 → **9.4mm** (여전히 양수). `ee_to_tcp_offset_m=0.236` 유지 가능.
접촉 리드백(stroke 670)에서 팁 간격 **14.3mm** — 줄기를 문 것으로 보인다.

⚠️ 1.101 rad 까지 닫으면 두 파츠가 2.1mm 파고든다. 브릿지의 `stroke/700` 매핑이 1.0 rad 에서
멈추는 것이 안전장치 역할을 하므로 그 매핑은 건드리지 않는다.

---

### 2026-09-08 — Stage 1+2 실행 후 관찰 6건 진단

**① 하강이 과실보다 46mm 아래로 (핵심 버그, Stage2 회귀)**
`compute_open_stem_descent_m` 은 `reached_z − kp1_z` 를 하강량으로 쓴다. `kp1_z` 는 딸기 z 인데
넘기던 `reached_z` 는 **ee_link(=플랜지) z** 였다. measured_tcp 는 ee_link 가 곧 TCP 라 같았지만
legacy 는 `ee_to_tcp_offset · approach_dir_z` 만큼 어긋난다. 게다가 파지 변형이 접근을 ±10°
기울여 `approach_dir_z` 가 −0.174 까지 간다.
실측: `kp1_z=500mm reached_z=574mm → descent=74mm` (정상은 30mm). 툴 끝이 과실보다 **46mm 아래**,
이어진 detach 40mm 로 86mm 아래에서 조우를 닫아 **전 타겟 GRASP_EMPTY**.
→ `reached_z = ee_z + ee_to_tcp_offset · approach_dir_z` 로 보정. 6타겟 × 4변형 전부
툴 끝↔과실 거리 **14.8~15.0mm** (판정 반경 35mm) 로 수렴.

**⑥ ne 딸기(ripe_03) 미수확** — `J1 swing 86.9deg > 75.0deg`. 목표 J1=206.7° 는 운용 한계
(−225~225) 안이고 막은 건 픽 단위 이동량 가드뿐이었다 → 95도로 완화.

**④⑤ 파지 성공 후 곧바로 놓음 / 보드 앞에 놓음** — 같은 원인.
`J3 swing 162.8deg > 120.0deg` → `TAUGHT_TRAY_SLOT0_PLACE_BLOCKED` →
`hold_on_place_failure=false` 라 그 자리(보드 앞)에서 release 하고 다음 타겟으로.
목표 J3=−123.9° 는 운용 한계(−135~135) 안 → 175도로 완화.

**② 조우가 너무 벌어짐 (구조적, 코드 수정 안 함)** — 커스텀 파츠는 밑동이 좁고 팁이 벌어진
깔때기 형상이다. TCP(플랜지+236mm)가 있는 **팁 부근** 간격은 개도 600 에서 52.8mm,
완전닫힘에서도 36.6\~40.9mm 다. 과실 단면 44.4×54mm 가 열림 상태에서 그대로 통과한다.
실제 파지 목(throat)은 플랜지+120\~140mm(닫힘 14.9mm)인데, 거기에 과실을 두려면 팁이 보드를
88mm 관통한다. **툴 길이 대비 과실↔보드 간격(27mm)이 근본적으로 부족하다.**

**③ 파지 판정 규칙** — 브릿지가 TCP(플랜지 +`tool_tcp_offset_m` 236mm)와 익은 딸기들의
최소 거리를 재서 **≤35mm 면 CONTACT(리드백 670), 아니면 EMPTY(700)**. 브릿지 로그의
`GRASP_JUDGE d_tcp=... d_ee=... -> CONTACT/EMPTY` 한 줄에 다 나온다.

---

### 2026-09-08 — Stage 1+2 첫 실행 실패 3회, 원인 4건

Stage 1+2 코드를 넣고 처음 돌렸을 때 3회 연속 실패했다. 원인이 넷이었다.

| # | 원인 | 조치 |
|---|---|---|
| ① | `_publish_joint_command` 에서 `msg.position = msg.position + [...]` → `TypeError: can only append array (not "list") to array`. rclpy 가 대입 즉시 `array.array('d')` 로 바꾸기 때문. 첫 SetPosition/MoveJoint 에서 노드 즉사 | name/position 을 **각각 한 번만** 대입하도록 수정 |
| ② | **전날 16:43 에 띄운 노드 4개가 11시간째 생존.** 그중 planner 는 `ee_to_tcp_offset_m`·보드 장애물 없는 옛 설정, 브릿지는 Stage1 코드 없는 구버전. 새 노드와 같은 토픽·서비스에서 경쟁 → **코드를 다 고쳤는데도 계속 보드 관통** | 재부팅으로 해소. `docs/run_guide.md` 에 "시작 전 30초 점검" 절 신설 |
| ③ | T1 명령의 `-p tool_tcp_offset_m:=0.208` 이 브릿지 기본값 0.236 을 덮어씀 → \|15+236−208\|=43mm > capture radius 35mm → **전 타겟 GRASP_EMPTY** | 오버라이드 제거, 문서의 0.208 전부 0.236 으로 |
| ④ | T1 이 `{ A & B; }` 구조라 B(브릿지) 사망 후에도 A(fake_vision)가 백그라운드 생존 → 재실행마다 누적 (4개까지) | 위 점검 절에 명시 |

**교훈**: ②는 코드로는 절대 안 보인다. 증상만 보면 "수정이 안 먹었다" 로 읽혀서 엉뚱한 곳을
파게 된다. 재실행 전 프로세스 확인을 습관화할 것.

---

### 2026-09-08 — Stage 1+2: 노드 시퀀스에 시뮬 정합

**Stage 1 — 시뮬 그리퍼가 한 번도 움직이지 않던 문제**

`sim_executor_bridge_node.set_position_cb` 가 `self.gripper_position` 숫자만 저장하고
Isaac 에 관절 명령을 보내지 않았다. 게다가 Isaac 스크립트가 `msg.name` 을 무시하고
`full_cmd[:len(cmd)] = cmd` 로 위치 인덱싱해서 **그리퍼 DOF 를 지정할 방법 자체가 없었다.**
결과: 씬 초기 개도(63°/57.3° = 거의 완전 닫힘)가 런 내내 고정 → cuRobo 는 완전 열림
(팁 플랜지+233.3mm)으로 계획하는데 실제는 닫힘(+262.0mm) → 27.5mm 가 보드 관통으로.

| 항목 | 전 | 후 |
|---|---|---|
| 브릿지 그리퍼 명령 | 없음 | `stroke/700` rad 로 4축 발행 |
| Isaac DOF 매핑 | 위치 인덱싱 | **이름 기반** |
| 씬 초기 개도 | 63° / 57.3° | **49.11°** (stroke 600) |
| cuRobo `lock_joints` | 0.0 (완전열림) | **0.857** (stroke 600) |
| `ee_to_tcp_offset_m` | 208mm | **236mm** |

**Stage 2 — open-stem descent 가 죽어 있던 문제**

`measured_tcp_260mm` 프로파일에 묶여 있었는데 그 프로파일은 yml 의
`ee_link: "grasp_tcp_link"` 가 URDF 에 없어 **로드가 안 된다.** 실행에 쓰는 legacy 에서는
"위로 30mm → 수평 진입 → 열린 채 하강 → 닫기" 가 통째로 생략됐다.
→ `enable_open_stem_descent` 파라미터로 분리 (기본 false = 실기 동작 불변).

⚠️ `legacy_grasp_endpoint` 에도 crane offset 을 더해야 한다. ee_pre 에만 더하면 pre→grasp
구간이 대각선이 되고, 뒤이은 하강이 한 번 더 내려가 조우가 30mm 낮게 착지한다.

**검증 (익은 딸기 6/6)**: pre z = grasp z (z편차 0.002mm) → 하강 30mm → 최종 z 목표 일치.
툴 끝 654.6mm (과실 중심 +9.8mm), 보드여유 10.5mm.
cuRobo 실측으로 구체 도달 267.7mm 확인 (= 손계산 262.7 + `collision_sphere_buffer` 5mm),
좌우 대칭 쌍으로 나와 **mimic 조인트 전파도 정상**.

**⚠️ 설정 파일은 ASCII 전용으로 유지할 것.**
cuRobo 는 `lock_joints` 가 0 이 아닌 경로에서 robot yml 을 **ASCII 코덱으로 다시 읽는다.**
한글 주석을 넣었더니 `UnicodeDecodeError: 'ascii' codec can't decode byte 0xec in position 464`
로 플래너가 기동 실패했다(재현 확인). `config/curobo/*` 와 `config/*.yaml` 의 설명은 영어로,
한글 설명은 `docs/run_guide.md` 에 둔다.

---

### 2026-09-07 — 로봇이 딸기·보드를 관통하던 문제

**증상**: 로봇이 딸기와 충돌해 과실이 튕겨나가고, 보드를 그대로 통과. 파지점 접근 시작
위치가 딸기보다 보드 쪽에 있어 과실을 뚫고 들어감.

**원인 3개** (모두 실측으로 확인):

1. **툴 길이 모델이 73mm 짧음.** `e0509_gripper.urdf` 의 `gripper_attach_joint` 오리진이
   `xyz="0 0 0"` — 플래너의 ee_link(그리퍼 베이스)는 플랜지와 같은 자리다. 거기에
   `LEGACY_EE_TO_TCP_OFFSET_M=0.160` 을 더해 TCP 를 잡았지만, 실제 파지부(커스텀 3D
   프린팅 파츠)는 플랜지 +233.3mm(열림) ~ 261.7mm(닫힘) 까지 뻗는다.
   → pre-approach 시점에 이미 툴 끝이 과실 속 35.5mm.

2. **커스텀 파츠가 어느 충돌 모델에도 없었음.** cuRobo URDF 에는 링크 자체가 없었고,
   `robot_physics.usd` 의 `/colliders/left_custom_part`·`right_custom_part` 는 비어 있다
   (`robot.urdf` 의 해당 링크에 `<collision>` 없음). 계획에서도 물리에서도 안 보였다.

3. **cuRobo 충돌 월드가 사실상 비어 있었음.** `environment.yaml` 에 10m 밖 더미 큐보이드
   하나뿐, `scan_collision_world.yaml` 은 `objects: []`. 보드를 통과하는 궤적이 정상 판정.

**부수 확인**: 2026-09-05 벽 캘리브레이션이 전제 두 개를 틀렸다 — (a) 보드에 "반두께
28.5mm" 가 있다고 봤으나 `whiteboard.usd` 의 board 는 두께 0 평면, (b) 3° 기울기 무시.
실제 보드는 `y = 700.5 − 52.4·(x − 50)` [mm] 로 672\~729mm 범위였고, 672mm 가 맞는 곳은
오른쪽 끝 모서리 한 점뿐이었다. 익은 딸기 위치 오차 +12.8 \~ +44.2mm.

**조치 결과** (`-p ee_to_tcp_offset_m:=0.208` 기준):

| | 전 | 후 |
|---|---|---|
| 보드 평면 | 672~729mm (3° 기울기) | **672.0mm 균일** (오차 0.0mm) |
| 과실 뒷면~보드 간격 | 17.8~49.2mm | **5.0mm 균일** |
| pre-approach 툴 끝 | 658.1mm (과실 속 35.5mm) | **610.1mm** (과실 앞면보다 12.5mm 앞) |
| grasp 툴 끝 | 703.1mm (보드 관통) | **655.1mm** (과실 중심 +10.3mm, 보드여유 10.0mm) |
| 계획 가능한 익은 딸기 | — | **6/6** |

**주의**: `ee_to_tcp_offset_m` 기본값은 프로파일 값(160mm)이라 **실기 경로 동작은 안 바뀐다.**
시뮬은 반드시 `:=0.208` 을 넘겨야 하고, 기동 로그의 `EE_TO_TCP_OFFSET_OVERRIDE` 경고로 확인한다.

---

## 알려진 잔여 이슈

| 이슈 | 상태 |
|---|---|
| 목표 앞 132mm 정지 | ✅ 해소 — 씬 기하를 플래너 벽 모델에 정합 (오차 0.0mm) |
| 파지 판정 빈손 | ✅ 해소 — 가상 제어기가 TCP↔딸기 거리로 판정 |
| 재실행 불가 | ✅ 해소 — `_started` 해제 |
| 배치(place) 미구동 | ✅ 해소 — 파라미터 4개로 taught slot0 경로 완주. 2026-09-07 16:44 런에서 **익은 딸기 6개 시도 → 4개 place+release 완주** (`log/m2_5/`) |
| place 실패 시 런 전체 정지 | ✅ 해소 — 래치 대신 그 자리에서 놓고 다음 타겟으로 진행 |
| 트레이 slot 1 이상에서 J6 한계 경계 초과 | ⏳ 미해결. cuRobo가 J6를 273.7°(한계 ±225°)로 내면 normalize가 357° 점프를 만들어 가드가 거부. 회피: `taught_slot_sequence:="0,0,..."`로 slot 0 고정 |
| 둘째 타겟 J1/J2 swing reject | ⚠️ 2026-09-07 16:44 런에서는 **재현 안 됨** — 4개 타겟 연속 처리. 씬 재배치(딸기 12개)로 시작 자세·타겟 분포가 달라진 결과로 보이나 미규명 |
| `ripe_03`(x=+350mm) 도달성 | ❌ **실행에서 실패 확인** — 2026-09-07 16:44 런 `grasp 전체 실패 — 8개 후보 모두 reject`. 기하학적 통과와 별개로 도달 불가 |
| 그리퍼 l2/r2 링크가 cuRobo 충돌 모델에 없음 | ⏳ 미해결. 커스텀 파츠는 추가했으나 l2/r2(z 76~116mm)는 여전히 구체 없음 |
| `measured_tcp_260mm` 프로파일 로드 불가 | ⏳ 미해결. yml 의 `ee_link: "grasp_tcp_link"` 가 URDF 에 없음. Stage2 로 우회했으므로 제출 전 수정 불필요 |
| 딸기가 그리퍼를 따라가지 않음 (분리·배치가 화면에 안 보임) | ⏳ **Stage 3 예정**. 키네마틱 부착 필요 |
| 물리 파지 불가 | ⚠️ 구조적. 조우 최소 간격 14.9mm / 팁 36.6~40.9mm vs 과실 단면 54×62.6mm |
| 커스텀 파츠에 물리 콜라이더 없음 | ⏳ 미해결. 계획 단계에서는 막히지만 물리 엔진 최후 안전망은 부재 (`robot.urdf` `<collision>` 추가 + USD 재임포트 필요) |
| 브릿지/플래너 툴 오프셋 불일치 | ✅ 해소 — 브릿지 기본값을 208mm로 맞춤. 어긋나면 파지 판정이 전 타겟 EMPTY |
| 딸기 줄기 `breakForce = 2N` | ✅ 해소 — 20N/5Nm 으로 상향(자중의 101.9배). 파지 시 물리 분리는 원래부터 없던 경로라 부작용 없음 |

## 설정 자산 메모

`config/scan_pose_candidates_refit_candidate.yaml`의 root/nw·ne·se·sw **네 셀이 모두 동일한
관절값**을 갖는다(`tcp_transform_base`도 전부 단위행렬). 어느 셀을 지정해도 같은 자세에서
스캔하므로 **셀 지정이 현재 무의미**하다. 셀별로 다른 스캔 자세가 필요해지면 이 파일을 고친다.

---

### 2026-09-09 — 보드 거리 1차 출처 확정 (사용자 문의 재조사)

**찾았다.** 2026-09-08 에 "1차 출처가 리포 어디에도 없다" 고 적었는데, 틀렸다.
출처는 코드가 아니라 **씬 에셋**에 있었다:

    strawberry_harvest/scenes/lab_environment.usd  (커밋 df0165f, 2026-07-06)
        translate = (0.05, 0.81, 0.66)
        scale     = (1.09, 0.79, 1)
        orient    = (0.70686, -0.70686, 0.01851, -0.01851)   <- 3도 기울기

즉 사용자가 실측으로 설치한 보드는 **y = 810mm (앞면 781.5mm)**, 종이판 1090x790mm 다.
같은 커밋의 딸기도 거기 붙어 있었다 — ripe y=803.7mm / unripe y=786.4mm.
사용자의 "80cm 언저리" 기억이 정확하다.

이것을 2026-09-05 M2 세션이 `layout_layer.usd` 에서 672mm 로 **109.5mm 당겼다.**
근거는 `WALL_SURFACE_Y_M = 0.672` 였다.

**그런데 실기 원본 코드도 672 를 말한다** (`_baseline/`, 내 편집이 닿지 않은 스냅샷):

    harvest_motion_params.py:62        WALL_SURFACE_Y_M = 0.672
    compute_nw_pick_ready_pose.py:17   "the wall surface (y=672mm)"
    compute_nw_pick_ready_pose.py:173  "all the way to the wall surface (y=672mm)"

**두 실측이 충돌한다.** 판단 근거를 v12 티칭 자세로 정리하면:

| 보드 앞면 | 스캔 자세 팁-보드 거리 (nw/ne/se/sw) | 사용자 증언 "못해도 15cm" |
|---|---|---|
| 672.0mm (현재) | 97 / 156 / 133 / 119 mm | nw, sw 가 미달 |
| 781.5mm (씬 실측) | 207 / 266 / 243 / 229 mm | 전부 충족 |

**사용자 결정: A안 채택 — 810mm 로 복원 (2026-09-09).**

정정: 앞면은 781.5mm 가 아니라 **810.0mm** 다. 보드는 두께 0 평면이라 뺄 반두께가 없다.
'781.5mm' 는 반두께 28.5mm 를 가정한 옛 오독이었다.

함께 옮긴 것:
- `layout_layer.usd` 보드 translate.y 0.672 -> **0.81**, board_guard 0.772 -> **0.91**
- 딸기 12개 y 0.6448 -> **0.7828** (+138.0mm, 과실 뒷면~보드 5.0mm 유지)
- `physics_layer.usd` 줄기 조인트 `localPos0` 12개 동일 이동
- `harvest_motion_params.py` `WALL_SURFACE_Y_M` 0.672 -> **0.810**
- `config/environment.yaml` x2, `scan_collision_world.yaml` 상자 y [810, 1010]

검증:
- 익은 딸기 **6/6 IK 도달**, 관절 한계 전부 여유 (2026-09-08 스윕의 우려는 딸기가
  z<=740 으로 내려온 지금은 재현되지 않는다)
- 스캔 자세 팁-보드: 235 / 294 / 271 / 257mm (672 일 때 97\~156mm)
- 순회 구간 콜라이더 여유 181\~225mm (672 일 때 43\~87mm)
- 트레이 배치 자세 465.9mm — 영향 없음
- USD 실측: 보드면 810.0mm, 과실 뒷면 805.0mm -> 간격 5.0mm 정확히 유지

**후속 (같은 날, 사용자 지적)** — 보드/딸기 이동 시 **같이 옮겨야 하는데 빠진 것들**:

- `assets/props/cell_markers.usd` 경계 막대 9개 — 중심 y 0.522 -> **0.66**
  (보드면 810mm 에서 -Y 로 300mm, 즉 y 0.810~0.510). 확인: 막대 bbox y [0.510, 0.810].
- `execution/scan_executor_node.py` 후보 순위 함수의 **하드코딩 `0.672`** ->
  로컬 상수 `BOARD_SURFACE_Y_M = 0.810`. (`harvest_motion_params` 는 이 모듈의
  import 경로에 없어 상수를 여기 둔다 — import 로 바꿨다가 기동이 깨질 뻔했다.)
- `scripts/curobo_planner_node.py` 의 낡은 기하 주석(644.8/672.0) 갱신.
- `fake_vision_node.py` 에 **`BERRY_GEOMETRY` 기동 경고** 신설. 씬을 옮긴 뒤 Isaac 을
  다시 열지 않으면 옛 좌표가 계속 발행되어 로봇이 딸기에서 -y 로 빗나간 지점을 집는데,
  코드로는 전혀 안 보이는 실패였다. 이제 발행 좌표의 y 범위와 기대값 편차를 찍고,
  20mm 넘게 어긋나면 "Isaac 씬을 다시 열 것" 을 남긴다.
- `config/environment.yaml` 의 "보드를 옮길 때 같이 고칠 파일" 목록에 위 항목들을 추가.
