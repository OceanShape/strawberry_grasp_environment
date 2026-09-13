# 무작위 과실 배치 전환 — 지금 배치를 전제로 박힌 값 (2026-09-12 조사)

**목적**: 익은 8 / 안 익은 4 를 무작위 위치에 놓고 수확 시퀀스를 시험한다. 이 문서는 그 방향과 맞지 않는
고정 좌표·개수·배치 전제 값을 모은 것이다. **여기서는 고치지 않았다** — 이 문서를 받은 세션이 고친다.

**조사 범위**: 씬 USD 레이어, `strawberry_harvest/scripts/scene_tools/`, Isaac 브릿지, `src/strawberry_sim_core`,
`scan_executor_node.py`, 플래너 파라미터·정책(`src/strawberry_motion/scripts/`), `scripts/run_nodes.sh`, 검증 도구, 문서.
줄 번호는 2026-09-12 작업 트리 기준이다.

---

## A. 반드시 바꿔야 하는 것 — 지금 배치의 좌표·측정값이 그대로 박혀 있다

| # | 위치 | 박힌 값 | 무작위와 안 맞는 이유 |
|---|---|---|---|
| A1 | `strawberry_harvest/scenes/layers/layout_layer.usd:190~` | 과실 12개 `xformOp:translate` (x·z 는 손으로 정한 값, y 0.7828) | 배치 그 자체. 파일 주석이 "좌표 변경 시 physics_layer 의 stem joint localPos0 도 반드시 함께" 라고 경고한다 |
| A2 | `layout_layer.usd:295~` (`over "vines"`) | 덩굴 12개 translate = 같은 과실의 translate | 덩굴 원점이 과실 중심이라 같은 값을 한 번 더 적어 둔다 |
| A3 | `strawberry_harvest/scenes/layers/physics_layer.usd:50~` | `stem_<이름>` PhysicsFixedJoint 12개의 `localPos0` = 과실 world 좌표 | 줄기 고정점. **A1·A2·A3 세 곳을 손으로 맞추는 구조**라 무작위 생성기가 세 곳을 한 번에 써야 한다 |
| A4 | `scene_tools/egg_carton_geom.py:78~98` | `RUN_ID`, `RUN_SLOT_SEQUENCE`, `RUN_GRID`, `FRUIT_REST_M` (런 9 에서 특정 과실 6개가 착지한 자리) | 계란판 컵 설계의 1차 출처가 특정 과실들의 매달림 변위다 |
| A5 | `egg_carton_geom.py:196` `landing_candidates()` | "실측 편차 6개는 슬롯과 무관" 가정으로 8칸 × 6 = 48 후보 봉투 | 과실 위치가 바뀌면 매달림 변위가 달라진다. 봉투 밖에 착지하면 런 8 에서 본 옆 컵 뚫림이 다시 난다. 봉투를 무작위 분포에서 다시 잡거나, 착지 편차를 위치 함수로 모델링해야 한다 |
| A6 | `scene_tools/verify_vines.py:32` | `RIPE_COUNT = 8`, 덩굴–이웃 과실 간격 (현재 최소 +1.4mm, vine_ripe_06 vs strawberry_ripe_04 — `layout_layer.usd` 주석) | 개수는 그대로여도 간격 판정은 배치마다 새로 돌려야 한다. 생성기의 거부 조건 후보 |
| A7 | `scene_tools/verify_egg_carton.py` | `FRUIT_REST_M` 기반 검증, 과실 메시 경로 `/World/strawberry_ripe_01/geo/fruit/mesh` | 검증 기준이 A4 에 묶여 있다 (메시 경로는 형상 참조용이라 위치와 무관) |

## B. 무작위 범위를 좁히는 값 — 바꿀지, 생성기 제약으로 둘지 정해야 한다

| # | 위치 | 값 | 영향 |
|---|---|---|---|
| B1 | `harvest_motion_params.py:107` `DIRECT_GRASP_TARGET_X_RANGE_M` | (−0.45, 0.45) m | 범위 밖 x 는 `ABORT` 로 그 표적을 버린다 (`pick_sequence_executor.py:649`). 보드는 x −0.495~+0.595 라 **오른쪽 145mm·왼쪽 45mm 띠에 놓인 과실은 못 딴다** |
| B2 | `grasp_candidate_policy.py:108·116` | x > 0.25 → 오른쪽 사다리(−30·0mm 먼저), x < −0.30 → 왼쪽 끝 사다리 | 실험실 배치에서 나온 위치별 분기(실기 정책). 런 1~9 는 x −250~+200mm 만 집었고, ripe_07(−380)·ripe_08(+280)은 런 10 이 첫 확인이다 |
| B3 | `harvest_motion_params.py:171` `MAX_HARVEST_JOINT_DELTA_DEG` | J1 75도 — 원본 (09-08 에 95 로 올렸다가 **09-14 원복**) | 특정 과실에서 막혀서 푼 스윙 상한. 새 위치가 다시 걸릴 수 있다 |
| B4 | `harvest_motion_params.py:187` `MAX_TAUGHT_PLACE_TRANSFER_JOINT_DELTA_DEG` | J2 100·J3 120 — 원본 (09-08·09-10 상향분 **09-14 원복**) | 같은 성격. 위쪽 과실일수록 트레이 이송 스윙이 크다. **런 10·11 에서 실제 발생** — ripe_08 slot 4 J3 186°, ripe_04 slot 9 J3 198°. 후퇴 자세가 같아도 IK 분기가 호출마다 달라 간헐적. 무작위 배치에서는 어느 과실에서든 날 수 있다 (`PLANNER_CHANGES.md` 09-12). **09-12 사용자 결정: 실기 플래너 한계로 기록, 고치지 않음** (`portfolio/H_scope_decisions.md` §9) |
| B5 | `harvest_motion_params.py:200` `COLLISION_ACTIVATION_DISTANCE_M` | 5mm | ripe_02·03 이 40mm 로 밀린 실측에서 정한 값 (보드에 붙은 과실 기준) |
| B6 | `harvest_motion_params.py:59~64` `NW_HIGH_TARGET_*` | z ≥ 0.750 등 | `measured_tcp_model` 조건이 붙어 있어 legacy_160mm 프로파일로 도는 지금 시뮬에서는 꺼져 있을 것 — **확인 필요** |
| B7 | y 고정 | 과실 중심 y = 보드면 − 27.2mm (`fake_vision_node.py:115` 가 2cm 넘게 벗어나면 경고) | 보드에 매단 구조라 무작위는 x·z 만 |
| B8 | `scan_executor_node.py:791` 중복 제거 30mm, 파라미터 `attempted_target_blacklist_radius_m` 25mm | — | 과실 중심 간격이 이보다 가까우면 한 과실로 합쳐진다. 과실 폭 약 54mm 라 겹치지 않게만 뽑으면 자동으로 지켜진다 |
| B9 | 분면·세부 칸 경계 x 0.050, z 0.660 (`scan_executor_node.py:94~95`, `quadrant_filter.py:22~23`) | 보드 고정값 | 그대로 두되, 경계선 위 과실은 분면·세부 칸 판정이 흔들린다. 생성기에서 경계로부터 띄울지 정한다 |
| B10 | 세부 자세 도달성 (`SUBMISSION_PLAN.md` T4b 오프라인 수치) | nw·ne 분면 위쪽 세부 칸은 IK 해 없음 | 막지는 않는다 — 그 칸 과실은 부모 자세에서 딴다. 결과(로그, HUD 트리의 호박색 테두리)가 배치마다 달라진다 |

## C. 지금 배치에 맞춰 고른 실행 파라미터 (`scripts/run_nodes.sh:150~178`)

- `taught_slot_sequence:=0,1,3,4,6,7,9,10` — 익은 8개용 8칸. 개수를 8로 고정하면 유지할 수 있다. 컵 봉투 문제는 A5.
- `subdivide_min_candidates:=3` — 고정 배치에서 가지치기·잎·분할이 한 런에 다 나오게 고른 값. 무작위에서는 분할이 0~2회로 달라진다.
- `overview_prescan:=true` — 빈 분면이 없는 배치에서는 가지치기가 화면에 안 나온다.
- `taught_grid_pitch_override_m:=0.068`, `taught_grid_shift_y_m:=0.0452` — 계란판 격자. 과실 배치와는 무관하지만 A4·A5 와 한 쌍으로 정해졌다.

## D. 이름 규칙 — 위치만 섞으면 그대로 둬도 된다

- 익음 판정은 prim 이름에 `unripe` 가 있는지로 한다 (`isaac_sim_script_editor_bridge.py:275·449`, verify 도구들).
- 줄기 조인트 경로 = `/World/physics/stem_` + 과실 이름에서 `strawberry_` 를 뺀 것 (`isaac_sim_script_editor_bridge.py:310`). 덩굴은 `vine_<이름>`.
- **어떤 과실이 익었는지까지 섞으면** 이름·ripeness 변형·조인트·덩굴 이름을 함께 바꿔야 한다. 12개의 위치만 섞으면 해당 없음.

## E. 문서·판정표의 고정 기대값

- `log/m3/README.md` §런 10 판정표 — `OVERVIEW_SCAN nw:3 ne:2 se:0 sw:3`, `SUBDIVIDE root/nw`·`root/sw`, 세부 칸 이름, `PICK COMPLETE 8`, ripe_07 x −380. 무작위 런은 배치(시드)에서 기대값을 뽑아 판정표를 만들어야 한다.
  - 참고: 같은 표 2·3·7행에는 SW 하나만 분할하던 옛 배치 기준이 남아 있다. 특히 3행의 `SUBDIVIDE_REJECTED 0건` 은 8/4 배치의 오프라인 수치(NW 의 se 칸 거부)와 어긋난다.
- `portfolio/G_quadtree_interview.md` 시연 레이아웃 문단, `PROGRESS_REPORT.md`·`portfolio/E_metrics.md` 의 런별 수치, `SUBMISSION_PLAN.md` T4 항목.

## F. 확인했고 배치와 무관한 것

- HUD 쿼드트리 패널 (`strawberry_harvest/scripts/hud/tree_model.py`) — 개수·위치 가정 없음, 분면 이름 4개만 쓴다.
- 충돌 월드 `src/strawberry_motion/config/environment.yaml` — 보드만 있고 과실은 없다.
- 실행기의 분면 순회·후보 수 판정·세부 칸 그룹화 — 탐지 좌표 기반.
- 과실 좌표 발행 — 브릿지가 USD 에서 매 스텝 읽는다 (D 의 이름 규칙만 따른다).
- 덩굴 형상 (`scene_tools/vine_geom.py`) — 과실 중심 기준 상대 좌표라 위치와 무관. 배치만 A2 로 따라가면 된다.
