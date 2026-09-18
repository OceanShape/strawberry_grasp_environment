# 파라미터 기준표 (single source of truth)

> **이 문서가 "지금 값"의 기준이다.** 다른 문서에 같은 숫자가 나오면 그쪽이 낡은 것이다.
> 최종 갱신 2026-09-09. 값은 전부 코드/에셋에서 직접 뽑아 대조했다.
>
> - 파일이 무엇을 하는지 → [`config_reference.md`](config_reference.md)
> - 어떻게 실행하는지 → [`run_guide.md`](run_guide.md)
> - 왜 그 값이 됐는지(이력) → [`../PLANNER_CHANGES.md`](../PLANNER_CHANGES.md)

**정합 검사**: 아래 값들이 실제 코드/에셋과 맞는지 기계적으로 확인한다.

```bash
python3 check_params.py
```

보드 y 하나가 5개 파일에 중복돼 있어 손으로는 반드시 빠진다 — 2026-09-09 에 실제로
`cell_markers.usd`(당시 존재, 09-10 제거) 와 `scan_executor_node.py` 가 빠져 로봇이 딸기 앞을 집었다.

---

## 1. 보드·씬 기하 — ⚠️ 5곳이 한 세트

보드를 옮기면 **다섯 곳을 전부** 고쳐야 한다. 하나라도 어긋나면 파지 목표가 클램프되거나
로봇이 보드를 관통한다. 2026-09-09 에 실제로 `cell_markers.usd`(당시 존재) 와
`scan_executor_node.py` 두 곳이 빠져 로봇이 딸기 앞을 집었다.
(2026-09-10 에 `cell_markers.usd` 를 제거해 여섯 곳 → 다섯 곳. 분면 표시는
`whiteboard.usd` 의 `highlight` 오버레이가 맡는데, 보드 prim 의 자식이라 보드와 같이 움직인다 — 따로 고칠 게 없다.)

| 값 | 현재 | 파일 |
|---|---|---|
| 보드 앞면 y | **810.0 mm** | `strawberry_harvest/scenes/layers/layout_layer.usd` (whiteboard `translate.y = 0.81`) |
| 확장 콜라이더 | y [810, 1010] · x [−700, +800] · z [0, 1110] | 같은 파일 `board_guard` (`translate.y = 0.91`) |
| 딸기 12개 y | **782.8 mm** | 같은 파일 (보드면 −27.2mm, 과실 뒷면~보드 5.0mm) |
| 줄기 조인트 앵커 | 딸기와 동일 | `scenes/layers/physics_layer.usd` `localPos0` ×12 |
| 플래너 벽 상한 | `WALL_SURFACE_Y_M = 0.810` | `src/strawberry_motion/scripts/harvest_motion_params.py` |
| 후보 순위용 보드면 | `BOARD_SURFACE_Y_M = 0.810` | `.../execution/scan_executor_node.py` |
| 발행 좌표 검증용 | `board_surface_y_m = 0.810` | `src/strawberry_sim_core/.../fake_vision_node.py` |
| cuRobo 충돌 상자 | pose `[0.05, 0.91, 0.555]` dims `[1.50, 0.20, 1.11]` | `e0509_gripper_description/config/environment.yaml`, `strawberry_motion/config/environment.yaml`, `strawberry_motion/config/scan_collision_world.yaml` |

**보드 크기**: 보이는 종이판 1090×790 mm (x [−495, +595], z [265, 1055]).
충돌 상자는 실기 물리 화이트보드 1500×900 mm 기준 + 바닥까지 — 로봇이 보드 밑·옆으로
빠져나가는 것을 막는다.

**분할 격자** (보드 고정, 딸기 배치와 무관):
`BOARD_SUBCELL_X_MID_M = 0.050` / `BOARD_SUBCELL_Z_MID_M = 0.660`
→ nw(x≤50, z≥660) / ne(x>50, z≥660) / sw(x≤50, z<660) / se(x>50, z<660).
`scan_executor_node` 와 `strawberry_sim_core/quadrant_filter.py` 가 **같은 규칙**을 쓴다
(무작위 2만 점 교차검증 0 불일치).

**익은 딸기 분포**: sw 3 / nw 2 / ne 1 / **se 0** — se 에서 `TARGET_NOT_FOUND` 는 정상이다.

---

## 2. 툴·파지 기하

| 값 | 현재 | 어디 |
|---|---|---|
| ee → 파츠 시작 | 112.5 mm | 실제 메시 기준 |
| ee → **실제 손끝** | **262.5 mm** | ⚠️ TCP 가 손끝이 아니다 — 손끝이 **26.5mm 더 앞** |
| ee → 무는 구간 | 230.5 ~ 262.5 mm | 파츠 간격 ≤2mm 인 구간. 플래너 TCP(236)가 이 안에 있다 |
| 플래너 툴 오프셋 | `ee_to_tcp_offset_m:=0.236` | T3 실행 인자 |
| 브릿지 판정 오프셋 | `tool_tcp_offset_m = 0.236` (기본값) | **T2 에서 지정 금지** — 옛 값 0.208 이 남으면 전 타겟 빈손 판정 |
| 파지 목표 z bias | `pick_target_z_bias_m:=0.035` | 과실 중심 +35mm(줄기)를 겨냥 |
| 파지 오프셋 사다리 | **`[0.015, 0.020, 0.025]`** | 하한 15mm = 기구학적 최소 (0.0 은 손끝이 보드면 0.7mm). **상한 25mm = 조우 물림 한계(+26.5mm)** — 넘으면 조우가 줄기 앞에서 닫혀 파지 자체가 불가능하다. 종전 40/50/70mm 가 그걸 성공으로 받아 감췄다 |
| 이웃 장애물 자기-제외 | **50 mm** | `scene_obstacle_manager`. 호출자는 **raw 검출 좌표**를 넘긴다 — bias 붙은 좌표를 넘기면 거리가 정확히 35mm 라 부동소수점으로 갈려 **목표 딸기가 자기 장애물이 된다** (2026-09-10 수정) |
| 파지 판정 | `jaw_capture_near_m = -0.006`<br>`jaw_capture_far_m = 0.027`<br>`grasp_lateral_tolerance_m = 0.020`<br>`grasp_target_z_bias_m = 0.035` | **줄기 기준**(2026-09-10). 조우가 무는 구간은 ee+230.5\~262.5mm, TCP 는 ee+236mm → TCP 기준 **-5.5\~+26.5mm**. 줄기가 이 안이면 CONTACT. approach_dir 이 (0,1,0) 이라 `along` 은 사실상 **채택된 오프셋** 이다 |
| pre-approach 스탠드오프 | `PRE_APPROACH_OFFSET = 0.06` | |
| 진입 역순 후퇴 (설계 5단계) | `enable_straight_reverse_retreat:=true` (T3) | 실기 코드는 정상이나 `measured_tcp` 프로파일에만 걸려 있다. legacy 는 `reverse_distance=extra_advance(0)` 이라 역진이 생략된다. 켜면 TOOL −Z 45mm 로 진입을 되짚는다 |
| cuRobo 충돌 활성거리 | `COLLISION_ACTIVATION_DISTANCE_M = 0.005` (플래너·브릿지 동일) | cuRobo 기본은 trajopt **25mm**. 실기 제어기는 벽이 월드에 없어 여유 **0** 이었다. 5mm 는 실기보다 엄격하고 `collision_sphere_buffer` 5mm 는 별도 유지 |

> **파지 변형 탐색 순서 주의** (2026-09-10 실측): 변형은 `[-10, -5, 0, +5]°` 순으로 시도하고 변형 안에서 첫 성공 오프셋에 멈춘다. 높은 딸기는 −5° 에서 15\~25mm 가 도달 한계 밖이라 (ee 가 22mm 더 올라감) 종전 사다리로는 40mm 에서 멈췄다. 사다리를 25mm 로 자르면 −5° 가 전부 실패해 0° 로 넘어가고 거기서 15mm 가 된다.
>
> **정상값은 `d_tcp ≈ 38mm`** 다. 오프셋 15mm 에서 조우의 무는 구간이 딸기 평면을
> −20mm \~ +9mm 로 감싸므로 목표는 조우 사이에 정확히 들어온다 — 화면상 TCP 마커가
> 15mm 앞서 보이는 것은 정상이다. 46mm 면 30mm 오프셋까지 밀린 것(왼쪽 끝 딸기).

---

## 3. 그리퍼

측정은 **실제 메시**(`gripper_parts.stl`) 기준이다. cuRobo 콜리전 스피어는 패딩이
최대 19.7mm 라 간격을 음수로 내놓는다 — 스피어로 재면 안 된다 (2026-09-09 오판정).

| 값 | 현재 | 파츠 간격 |
|---|---|---|
| 완전 열림 | stroke 0 | 102.8 mm |
| 접근 `GRIPPER_APPROACH_POS` | **600** (0.926 rad) | **17.8 mm** |
| 닫힘 | 700 (**1.080 rad**) | **0.3 mm** — 줄기를 문다 |

- `gripper_close_rad = 1.08` (T2 파라미터). stroke 700 이 몇 rad 인가.
- USD 관절 상한 `rh_r2` / `rh_l2` = **1.1 rad** (63.02536°) — `robot_assembly.usd` override.

> ⚠️ **의도한 시뮬 편차.** RH-P12-RN 제조사 xacro 의 `rh_r2`/`rh_l2` 상한은 **1.0 rad**
> 이고, 그건 **스톡 손가락** 기준이다. 15cm 커스텀 파츠를 붙이면 손가락이 길어져
> 1.0 에서 파츠 끝이 **9.4mm** 벌어진 채 멈춘다 — 줄기를 물지 못한다.
> 실기와 완전히 맞추려면 USD override 를 지우고 `-p gripper_close_rad:=1.0` 으로 띄운다
> (그러면 9.4mm 간격이 된다). 두 값은 **한 쌍**이라 한쪽만 바꾸면 l2/r2 만 잘려
> 손가락이 기울고 가위 모양이 된다.

파츠 각도 보정(6° 벌어짐 제거)은 `robot_assembly.usd` 의 **두 곳 모두** 적용돼 있고,
USD 조인트 프레임이 URDF 와 **0.000°** 로 일치한다. 평행도 실측 **0.0mm** — 간격이
전체 면에 균일하다.

## 4. 스캔·순회

| 값 | 현재 |
|---|---|
| 깊이 1 순회 순서 | `_ALL_CELLS_CLOCKWISE_ORDER = ["root/nw", "root/ne", "root/se", "root/sw"]` — 원 팀 실기 기록(민1 STEP 6) 순서 그대로. 기준은 인접 이동: 처음 Z-order(nw → ne → sw → se)에 끼어 있던 ne → sw 대각선 이동을 없애려고 시계 방향으로 바꿨다. 원본 이름 `_ALL_CELLS_ZORDER` 는 Z-order 시절 이름이라 09-17 에 바꿈(동작 불변). `overview_prescan:=true` 면 익은 과실 0개 분면만 빠지고 순서는 유지 |
| 깊이 2 순서 | `_group_poses_by_subcell` 고정 목록 sw → se → nw → ne (원본 "lower-first": 실기 NW 가림 런에서 시야 위쪽 목표점이 잎·과실 윗부분 오탐이었던 경험 — 낮은 목표점을 진짜 줄기 목표일 가능성이 높다고 보고 먼저 시도. 근거·면접 문장 G §8 순서 Q). 후보 0개 칸 건너뜀. 칸 안 과실은 `(x, z)` 오름차순 |
| 실행 인자 | `-p target_cell:=all` — 4분면 전부. 단일 분면은 `root/nw` 등 |
| 스캔 자세 | `config/scan_pose_candidates_refit_candidate.yaml` `version: v12_gripper_centered_manual_teach` — **실기 DART 수동 티칭 값 그대로** |
| overview | `[87.98, −94.92, 129.89, 175.94, −31.34, 93.42]` |
| 스캔 자세 팁-보드 | nw 235 / ne 294 / se 271 / sw 257 mm |
| 셀 간 이동 | `plan_scan_transit=false` (기본, 2026-09-10 변경) — 실기와 동일한 순수 MoveJoint. true 로 켜도 MotionGen 이 없으면 타지 않는다 (선언부 주석) |
| overview 1차 스캔 | `overview_prescan=true` (시뮬, `run_nodes.sh`). 익은 과실 0개 분면은 순회에서 제외. **기본 false = 실기 최종본과 같은 전수 순회** |
| 스캔 dwell | `scan_dwell_sec=3.0` (시뮬). 기본 12초는 실기 fusion 안정화용 |
| 적응 분할 (T4b) | `subdivide_min_candidates=3` (시뮬, `run_nodes.sh`). 분면 근거리 스캔 후보가 3 이상이면 2×2 로 쪼개 **후보 있는 세부 칸만** 부모 자세에서 유도한 세부 자세(FK → x·z 평행이동 → 부모 시드 IK, `execution/subcell_pose.py`)로 MoveJoint 이동·재스캔·pick. 관절 변화 > `subdivide_max_joint_delta_deg`(60) 면 `SUBDIVIDE_REJECTED` 로 부모 자세 pick. **기본 0 = 끔 = 실기(쪼갤지 여부를 사람이 오프라인에서 정해 YAML 에 넣던 정적 방식)**. 오프라인 검사 `scripts/check_subcell_scan_poses.py` |
| 비인접 분면 폴백 | MoveJoint 폴백 시 overview 경유 (`TRANSIT_VIA_OVERVIEW`). cuRobo 계획 성공 시엔 직행 |

J4/J6 는 **티칭 원본 표현 그대로** YAML 에 둔다 (`J4=−238.52` 등).
`_shortest_equivalent_joints` 가 실행 시 등가각으로 바꾼다 — 실기 로그와 같은 값이 나온다
(`J4 −238.5 → 121.5`, `J6 −190.8 → 169.2`, `J4 −97.8 → 262.2`).

---

## 5. 속도

| 값 | 현재 | 되돌리려면 |
|---|---|---|
| 아티큘레이션 게인(팔) | stiffness **1e6** / damping **1e5** | `robot_assembly.usd` |
| 〃 (그리퍼) | stiffness 1e5 / damping 1e4 | |
| 실행 속도 배율 | `sim_speed_scale = 1.0` (09-14 C3, 종전 2.0) | 실기 요청 시간 그대로. 2.0 은 영상용 시간 압축이었다 (`docs/e0509_spec_audit.md` C3) |
| 스플라인 관절속도 바닥 | `SPLINE_MAX_JOINT_SPEED_DEG_S = 120` | 실행 시간의 **하한**을 스윙으로 정한다. 없으면 큰 스윙에서 아티큘레이션이 못 따라오고, 그 잔차가 다음 **상대** MoveLine 에 전파돼 조우가 파지점 앞에서 닫힌다 |
| 도착 대기 | `ARM_ARRIVAL_TIMEOUT_SEC = 8.0` | 3.0 이면 큰 스윙에서 타임아웃 |
| 스캔 이동 | 120 / 180 deg·s | T4 `-p scan_movej_vel_deg_s:=60` = 실기 값 |

> damping 만 올리면 **느려진다** (속도에 저항하는 항). 비율을 유지한 채 같이 올려야 추종이 빨라진다.

---

## 6. 관절 가드

| 값 | 현재 |
|---|---|
| `OPERATIONAL_JOINT_LIMITS_DEG` | ±225 / ±95 / ±135 / ±360 / ±130 / **±225** — 실기 원본. 09-08 에 J6 를 ±360 으로 넓혔던 것을 **09-14 C4 로 원복**. cuRobo URDF(J6 ±360)는 그대로 — 통일해도 회복 없음(`docs/e0509_spec_audit.md` §7) |
| `MAX_HARVEST_JOINT_DELTA_DEG` | `[75, 90, 120, 150, 130, 120]` — 원본. 09-08 J1 95 는 **09-14 원복** |
| 시뮬 로봇 J2·J3·J5 한계 (`robot.urdf`·`robot.usd`) | ±95 / ±135 / ±135° (09-14 D1, 종전 ±360/±155/±360) — 실기 cuRobo 운용 값. J3 는 기록상 135.0° 까지 쓰여 여유 0. 씬 재로드 필요 |
| 브릿지 명령 방어선 `ARM_JOINT_LIMIT_DEG` | `[360, 95, 135, 360, 135, 360]` (09-14 C1·D1, 종전 `[365,100,160,365,140,365]`) |
| `subcell_ee_y_m` (실행기) | 기본 **0.433** = 실기 깊이 2 티칭 평면(ee y, 보드에서 377mm). 세부 자세 유도 사다리 lab_plane→parent_y→부모 자세. 0 이면 09-11 동작(부모 y 유지) (09-14) |
| `subcell_view_enabled` (fake_vision) | 기본 true — `<cell>=VIEWING` 에서 그 셀 경계로 발행을 좁힌다(lab_plane 세부 자세 = 세부 칸). false 면 분면 시야만 (09-14) |
| `hold_on_place_failure` | 기본 **true** (실기 원본: 배치 실패 시 과실을 든 채 런 잠금). 시뮬은 `run_nodes.sh` 에서 `false` — 설계 §3 "모두 파지/배치"상 한 개 실패로 멈추지 않는다 (09-14 결정) |
| 두산 MoveIt 참조값 (기록 전용) | vel·acc `[120,120,150,225,225,225]` — 초과해도 자르지 않고 로그만 (09-14 D3) |
| `MAX_TAUGHT_PLACE_TRANSFER_JOINT_DELTA_DEG` | `[170, 100, 120, 150, 130, 180]` — 원본. J2 130·J3 175 는 **09-14 원복** (원칙 §0-1). (이력) 100 이면 nw 딸기 2개가 트레이 이송에서 막혀 과실을 그 자리에 놓아버린다 |
| 브릿지 발행 상한 | `ARM_JOINT_LIMIT_DEG = [365, 100, 160, 365, 140, 365]` — 넘으면 발행 차단 + `JOINT_COMMAND_REJECTED` |

---

## 7. 기동 시 눈으로 대조할 줄

| 어디 | 줄 | 없거나 다르면 |
|---|---|---|
| T2 | `BERRY_GEOMETRY: n=6 y=782.8~782.8mm` | 편차 20mm↑ = Isaac 씬을 다시 열지 않은 것 |
| T2 | `tool_tcp_offset=236mm capture_radius=45mm sim_speed_scale=1.0` | 판정이 헛돈다 |
| T2 | `MOVELINE_COLLISION_WORLD: ['whiteboard']` | 이동 중 보드 관통 |
| T3 | `EE_TO_TCP_OFFSET_OVERRIDE: 160mm -> 236mm` | 툴을 짧게 본다 |
| T4 | `TRAVERSAL_SCAN_STARTED cells=[...] (4/4 quadrants)` | 분면 누락 |
| 동작 중 | `GRASP_JUDGE ... d_tcp≈35mm -> CONTACT` | 38mm 면 오프셋 폴백 |

---

## 8. 실기 제어기와 시뮬의 의도적 차이 (sim2real 대장)

기준은 **실험실**이다 (농장 → 실험실 → 시뮬). 아래 항목만 실기와 다르며, 전부 이유가 있다.
새 차이가 생기면 여기에 적는다.

| 항목 | 실기 (실험실) | 시뮬 | 왜 |
|---|---|---|---|
| cuRobo 충돌 월드 | 10m 밖 더미 큐브 하나 (**벽 없음**) | 보드 큐보이드 + 바닥까지 guard | 시뮬은 사람 감독이 없어 이송·배치 중 관통을 막을 안전망이 필요. **시뮬 측 추가** |
| 충돌 활성거리 | (벽이 없으니 사실상 0) | 5mm 명시 | 큐보이드를 넣으면 cuRobo 기본 25mm 가 따라와 벽 앞 파지를 막는다. 실기 쪽으로 되돌린 값 |
| 그리퍼 닫힘 | 1.0 rad (스톡 상한, 파츠 간격 9.4mm) | **1.08 rad** (0.3mm) | 15cm 커스텀 파츠는 스톡 스트로크로 줄기를 못 문다. 화면에서 물리는 모습이 필요 |
| 툴 오프셋 | 레거시 160mm (측정 260mm, **100mm 오차**) | 236mm | 실기는 100mm 오차를 큰 오프셋 사다리(30\~70mm)로 상쇄하던 구조. 시뮬은 모델을 바로잡고 사다리를 15\~25mm 로 재튜닝 |
| 오프셋 사다리 | `[15,30,40,50,70]` | `[15,20,25]` | 위 항목의 귀결. 26.5mm 넘는 오프셋은 바로잡은 모델에서 조우가 줄기 앞에서 닫힌다 |
| 딸기 위치 | 벽에 붙어 늘어짐 | **동일** (뒷면~보드 5mm) | 띄우지 않는다 — 실험실 기준 |
| 진입 역순 후퇴 | measured_tcp 프로파일에서만 동작 | legacy 에서도 `enable_straight_reverse_retreat` 로 켬 | 설계 시퀀스 5단계. 실기 코드는 그대로 두고 게이트만 확장 |
| 스캔 이동 | 순수 MoveJoint | **동일** (순수 MoveJoint, 비인접 분면만 overview 경유) | v12 티칭 자세 관절공간 여유 최소 181mm. cuRobo 경유(`plan_scan_transit`)는 죽은 경로라 09-10 기본 false |
| MoveLine 보간 | 제어기 내부 직선 보간 | 2mm 스텝 IK 연쇄, **스텝 수 상한 24** (`MOVELINE_MAX_STEPS`) | 스텝당 IK ~150ms 가 실소요를 지배. 상한 없이는 120mm 배치 하강이 9.2초 (시뮬 인공물) |
| 분면 순회 | 4분면 전수 | overview 1차 스캔 후 익은 과실 있는 분면만 (`overview_prescan`) | 원안 1·2단계. `false` 로 실기와 동일하게 가능 |
| 세부 칸(깊이 2) | NW 4칸 티칭 자세만 있고 쪼갤지 여부는 오프라인 결정(정적) | 후보 밀도 규칙으로 런타임 분할, 세부 자세는 부모 자세에서 계산 (`subdivide_min_candidates`) | 쿼드트리의 적응성. `0` 으로 실기와 동일하게 가능. 시뮬엔 가림 모델이 없어 이득은 수치가 아니라 판정·순회 자체 |

> 위 표의 "툴 오프셋 100mm 오차 + 큰 사다리 상쇄" 는 시뮬이 드러낸 **실기 노드의 숨은 모델 부채**다.
