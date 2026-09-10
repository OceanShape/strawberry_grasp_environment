# M3 런 로그 — T1 재완주 (2회)

`SUBMISSION_PLAN.md` T1 의 재완주 로그. `~/.ros/log/` 는 언제든 정리될 수 있으므로
원본을 노드별로 이름만 붙여 옮겼다 (내용 무편집). 형식은 `log/m2_5/` 와 같다.

| 런 | 폴더 | 무엇을 검증했나 |
|---|---|---|
| **런 1** 09-10 02:02 | [`20260910T020201-fc808f75/`](20260910T020201-fc808f75/) | 09-09 정합 수정본 END-TO-END 재완주 (4분면 **전수** 순회) |
| **런 2** 09-10 10:13 | [`20260910T101326-43213d77/`](20260910T101326-43213d77/) | 09-10 **쿼드트리 가지치기** 수정본 (overview 1차 스캔 → 익은 과실 있는 분면만) |
| **런 3** 09-10 11:05 | [`20260910T110524-e89f4f91/`](20260910T110524-e89f4f91/) | 브릿지 **MoveLine 스텝 상한 24** + `plan_scan_transit=false` 기본값 |

각 폴더 구성은 동일하다.

| 파일 | 원본 |
|---|---|
| `curobo_planner.log` · `sim_executor_bridge.log` · `scan_executor.log` · `fake_vision.log` | `~/.ros/log/python3_*.log` |
| `curobo_planner_node_<run_id>.jsonl` | `src/e0509_gripper_description/logs/runtime/2026-09-10/` |
| `kit_<날짜>.log` | `~/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/5.1/` (전문) |

> Isaac Kit 로그를 함께 남기는 이유: 09-09 17:23 런의 원인(물리 스텝 콜백 예외)은
> ROS 로그 어디에도 안 나오고 Kit 로그에만 있었다. 브릿지 스크립트가 조용히 죽는
> 실패 모드는 이 파일 없이는 재판정이 불가능하다. **런 2 에서 이 판단이 다시 값을 했다** — 아래 §런 2 (5).

---

## 실행 전 점검

1. 남은 노드 확인 — 이전 런의 노드가 살아 있으면 토픽이 섞인다.
   ```
   ps -ef | grep -E "curobo_planner|sim_executor|scan_executor|fake_vision" | grep -v grep
   ```
2. Isaac: 씬 로드 → **브릿지 스크립트 실행 → HUD 실행 → Play** 순서 (`docs/run_guide.md` T1).
   **Play 중에 브릿지 스크립트를 다시 Run 하지 않는다.** 그때 만들어지는 `Articulation` 은
   Isaac 의 play 콜백을 놓쳐 `initialize()` 가 안 돌고, 컨트롤러 view 가 None 인 채로 남는다 —
   09-09 17:23 런이 정확히 이 경로였다. 09-10 수정으로 이 상태에서도 복구되지만(런 2 가 실증),
   애초에 겪지 않는 편이 낫다. Stop 후 **Play 만** 다시 누르는 것은 안전하다.
3. 플래너 기동 파라미터: `-p ee_to_tcp_offset_m:=0.236 -p enable_open_stem_descent:=true`
4. 브릿지에는 `tool_tcp_offset_m` 을 **지정하지 않는다** — 기본값이 플래너와 맞춰져 있다.
   두 값이 어긋나면 그 차이가 파지 판정 거리에 그대로 더해진다.

`bash scripts/run_nodes.sh` 가 1·3·4 를 대신하고 기동 로그 대조까지 한다.

---

# 런 1 — 2026-09-10 02:02 (`20260910T020201-fc808f75`)

09-09 정합 수정본의 END-TO-END 재완주. **로그 기준 4/4 충족, 육안 항목 1개 남음.**

| 항목 | 결과 | 근거 |
|---|---|---|
| 첫 분면(nw) 스캔 이동 도착 | ✅ | `scan_executor.log` 에 `EXEC_TIMEOUT` 0건 (09-09 17:23 런은 여기서 중단) |
| Isaac 브릿지 생존 | ✅ | `kit_20260910_020051.log` 에 `get_applied_actions` 예외 **0건** |
| 좌표 발행 연속성 | ✅ | JSONL `scene_positions_received` **413건 / 413.9초, 최대간격 1.0초** |
| 팔 도착 판정 | ✅ | `ARM_ARRIVAL_TIMEOUT` 0건 |
| 관절 한계 방어선 | ✅ | `JOINT_COMMAND_REJECTED` 0건 |
| `clamped` WARN | ✅ | `curobo_planner.log` 에 0건 |
| 4분면 순회 | ✅ | `TRAVERSAL_SCAN_STARTED ... (4/4 quadrants)` → `SCAN_COMPLETE` |
| 파지 판정 | 6/6 `CONTACT` | `GRASP_JUDGE 줄기기준 along=+13.9~18.4mm lateral=1.5~7.0mm` |
| 픽 시퀀스 종료 | 6/6 `PICK COMPLETE` | 결과 코드는 전부 `DETACH_SUCCESS_UNVERIFIED` |
| 화면상 관통 | **로그로 판정 불가** | 영상/뷰포트 육안 확인 항목 |

타겟 분포: nw 2 / ne 1 / se 0 / sw 3 = 6. **se 는 익은 과실이 0개인데도 방문했다** — 이 런이
전수 순회였기 때문이고, 런 2 의 가지치기가 고친 지점이다. 전체 364.9초.

---

# 런 2 — 2026-09-10 10:13 (`20260910T101326-43213d77`)

09-10 쿼드트리 가지치기 수정본. **9/9 충족, 육안 항목 1개 남음.**
수정 항목은 `PLANNER_CHANGES.md` 09-10 3건, 설계 근거는 `portfolio/G_quadtree_interview.md`.

## 판정표

| # | 항목 | 결과 | 근거 |
|---|---|---|---|
| 1 | **overview 1차 스캔** | ✅ | `OVERVIEW_SCAN nw:2  ne:1  se:0  sw:3` (3.0초, 집계 안정 후 종료) |
| 2 | **가지치기** | ✅ | `TRAVERSAL_PRUNED skip=['root/se'] — overview 1차 스캔에서 익은 과실 0개`. **se 를 방문하지 않았다** (`AT_SCAN_POSE` 가 nw·ne·sw 만) |
| 3 | **비인접 이동 overview 경유** | ✅ | `TRANSIT_VIA_OVERVIEW root/ne -> root/sw — 비인접 분면` 1건. se 가 빠지며 생긴 유일한 비인접 쌍 |
| 4 | **2차 분할 교정** | ✅ | `root/nw/sw:1 root/nw/se:1`, `root/sw/se:1 root/sw/nw:1 root/sw/ne:1` — 부모 분면 중심선으로 실제 분산. 런 1 은 `root/nw/nw:2` 처럼 한 구석으로 몰렸다 |
| 5 | Isaac 브릿지 생존 | ✅ | `get_applied_actions` 예외 **0건**. 아래 §(5) 참고 — 다른 실패가 났고 **자동 복구됐다** |
| 6 | 좌표 발행 연속성 | ✅ | JSONL `scene_positions_received` **460건 / 460.7초, 최대간격 1.0초** (런 종료 후 대기 구간 포함) |
| 7 | 팔 도착·관절 한계 | ✅ | `ARM_ARRIVAL_TIMEOUT` 0건, `JOINT_COMMAND_REJECTED` 0건, `EXEC_TIMEOUT`·`EXEC_FAIL`·`PLANNING_FAIL` 0건 |
| 8 | `clamped` WARN | ✅ | `curobo_planner.log` 에 0건 |
| 9 | 파지·픽 완주 | ✅ 6/6 | `GRASP_JUDGE` 6건 **전부 CONTACT**, `PICK COMPLETE` 6건, 전부 `DETACH_SUCCESS_UNVERIFIED` |
| — | 화면상 관통 | **로그로 판정 불가** | 영상/뷰포트 육안 확인 항목 |

## 수확 대상 — 익은 딸기 6개 전부

`pick_target_prepared` 6건의 좌표가 씬의 익은 딸기 6개와 1:1 로 일치한다 (mm).

| 분면 | 서브셀 | 타겟 (x, z) |
|---|---|---|
| nw | sw | (-250, 740) |
| nw | se | (-120, 760) |
| ne | sw | (+200, 740) |
| sw | se | (-100, 440) |
| sw | nw | (-250, 500) |
| sw | ne | (-220, 560) |

se 분면에는 익지 않은 과실만 있다 (`layout_layer.usd`: unripe 04·05·06). 가지치기 판정이 맞다.

## 시간 — 364.9초 → 240.6초

| 구간 | 런 1 | 런 2 |
|---|---|---|
| overview 1차 스캔 | — | 3.0초 |
| 분면 이동 (3~4회) | 4.5초 | 4.4초 (nw 1.2 / ne 1.3 / **sw 2.0** — overview 경유분) |
| 재스캔 이동 | 0.1초 × 3 | 0.1초 × 3 |
| 스캔 dwell | 12.0초 × 7 = **84.0초** | 3.0초 × 6 = **18.0초** |
| pick 6건 | 214초 (건당 35.7초) | 214.3초 (건당 35.7초) |
| overview 복귀 | 0.8초 | 0.8초 |
| **전체** | **364.9초** | **240.6초** |

줄어든 124초의 내역은 dwell 66초, se 분면 1회분(이동 1.1 + dwell 12) 13초, 재스캔 dwell 감소분이다.
**분면 간 이동은 원래 1~2초였다** — 느렸던 것은 이동이 아니라 도착 후 정지였다.
pick 시간은 두 런이 같다 (건당 35.7초). 이 값이 전체의 89% 이므로, 더 줄이려면 pick 쪽을 봐야 한다.

## (5) 브릿지 초기화 실패 → 자동 복구 — 09-10 수정의 실증

`kit_20260910_101233.log` 에 **런 시작 전** 다음 3줄이 있다 (UTC 표기, 로컬 +9h).

```
01:12:59  [bridge] initialize failed (1 so far): AttributeError("'NoneType' object has no attribute 'create_articulation_view'")
01:12:59  [bridge] articulation (re)initialized
01:13:36  [bridge] apply_action recovered (after 30 failures)
```

09-09 17:23 런을 죽였던 것과 **같은 부류의 초기화 실패**가 다시 났는데, 09-10 에 넣은
(a) `apply_action` try/except + `initialize()` 재시도 + `target_action` 유지가 잡아냈다.
30회 재시도 후 복구했고, 복구 시점(01:13:36 = 로컬 10:13:36)은 스캔 트리거(10:13:50)보다 14초 앞이다.
따라서 이번 런은 정상 상태에서 시작했다. 종전 코드였다면 여기서 런이 죽었다.

`get_applied_actions` 예외가 0건인 것은 (b) 준비 검사가 그 경로로 들어가는 것을 막았기 때문이다.
Kit 로그의 `[Error]` 는 1건 — `RSD455 ... missing xformstack reset` (씬 로드 시 D455 강체 경고). 세 런 모두
동일하게 1건이고 `robot_assembly.usd` 의 `rigidBodyEnabled=0` 오버라이드 뒤에도 남는 메시지다. 아티큘레이션
초기화와 무관하며 이 런에서 처음 난 것이 아니다. (처음 정리할 때 "0건" 으로 적었던 것은 grep 패턴 오류 — 정정.)

## 이 런에 남아 있는 것 (완료 기준 밖)

- **`Plan FAIL ... IK_FAIL` 10건.** 파지 자세 후보 탐색 중의 거부이고, 이후 후보에서 계획이 잡혀
  진행됐다. 시퀀스는 멈추지 않았다. 런 1 과 같은 건수다.
- **`MOVELINE_SHORT` 3건** (명령 45mm / 실제 41.5~42.0mm, 런 1 은 5건). **전부 `TOOL dz=-45.0mm`,
  즉 후퇴 구간**이다. 메시지 문구는 09-10 시점에 이미 방향별로 갈라져 있어 "잔차가 다음 상대
  이동에 전파된다" 로 올바르게 찍혔다.
- **`SCAN_TRANSIT` 로그 0건 / MoveJoint 폴백으로 동작.** `plan_scan_transit` 기본값은 true 인데
  cuRobo 계획 경로를 한 번도 타지 않았다 (`MOVING_TO ... cuRobo 경유` 0건, 전부
  `(direct MoveJoint, YAML pose)`). 런 1 도 같다. **이 런에서 보드 회피는 v12 티칭 자세의
  관절공간 여유(실측 최소 181mm)와 비인접 쌍 overview 경유가 담당했다.**
  → **원인 확인 (09-10)**: 계획에 쓰는 MotionGen 이 `enable_runtime_curobo_preview` 경로에서만
  생성돼 `self._mg is None` 조건이 늘 실패했다. 죽은 경로라 기본값을 false 로 바꿨다 (`PLANNER_CHANGES.md`).
- **MoveLine 이 런의 50% (120.2초).** 배치 하강 120mm 가 60스텝 × IK 150ms = 9.16초 × 6건 = 55초.
  09-10 에 스텝 수 상한 24 를 넣었다 (브릿지). 다음 런에서 `MoveLine ok: 120mm / 24 steps ... 실소요` 로 확인.
- **가지치기된 분면의 셀 상태 부수 효과.** `_overview_prescan_filter` 가 `root/se=SCANNED_EMPTY`
  를 발행하는데, 시뮬 비전 모킹이 이를 분면 전이로 읽어 **0.3초간 필터가 se 로 갔다가** 곧바로
  nw 로 바뀐다 (`fake_vision.log` 4~5행, 둘 다 1789002833). nw dwell 시작(1789002834)보다 앞이라
  탐지에는 영향이 없다.
  **HUD 는 영향 없음 (2026-09-10 확인, 종전 기재 정정).** HUD 영역은 `_pub_state` 가 아니라
  `_move_to_scan_cell_and_wait`/`_process_cell_detections`/`_trigger_picks_for_cell` 래핑에서 나오는데
  (`harvest_probe.py:296`), 가지치기된 분면은 이 셋 중 무엇도 호출되지 않는다. 보드 하이라이트가
  SE 로 깜빡이는 일은 없다 — "녹화 시 확인 대상" 이라고 적었던 것을 취소한다.

> 09-09 17:23 런에서 `ARM_ARRIVAL_TIMEOUT` 이 함께 찍은 손끝오차 값은 신뢰할 수 없다
> (`_wait_for_arm_arrival` 이 `get_state()` 결과를 `.clone()` 없이 두 번 받아 뺀다).
> 이 수치는 판정 근거로도 산출물로도 쓰지 않는다.

## 아직 쓰면 안 되는 표현

- **"파지 실패도 재현된다(EMPTY 판정)"** — 이 런에도 `GRASP_JUDGE` 6건이 전부 CONTACT 다.
  `sim_executor_bridge.log` 에 `GRASP_EMPTY` 문자열이 1건 있으나 **기동 시 모델 배너의 설명 문구**이지
  판정 결과가 아니다. EMPTY 판정 사례는 여전히 확보되지 않았다.
- **"관통 없이"** — 육안 확인 전.
- 나머지 금지 목록은 `SUBMISSION_PLAN.md` §7 · `PLANNER_POLICY_v2.md` §5.4 그대로.

---

# 런 3 — 2026-09-10 11:05 (`20260910T110524-e89f4f91`)

브릿지 `MOVELINE_MAX_STEPS=24` + `plan_scan_transit` 기본 false 적용 후 첫 런. **완주, 목표 달성. 진단 임계값 1건 재검토 필요.**

## 판정표

| # | 항목 | 결과 | 근거 |
|---|---|---|---|
| 1 | 스텝 상한 동작 | ✅ | `MoveLine 120mm: 60 -> 24 steps (5.0mm/step, 자유공간 이송)` 6건. 30/40/45mm 는 16/20/23스텝 그대로 |
| 2 | 배치 하강 실소요 | ✅ **9.16 → 4.40초** | `MoveLine ok: 120mm / 24 steps / 계획 1.50s 실소요 4.25~4.90s` (새 실소요 필드) |
| 3 | 전체 | ✅ **240.6 → 208.5초** | pick 건당 35.7 → 30.4초. MoveLine 합 128.2 → 96.9초 |
| 4 | 스텝당 관절 이동 | ✅ | 120mm 하강 0.9도 (종전 0.3), 런 최대 2.2도. 가드 45도 |
| 5 | 파지 구간 무변화 | ✅ | 45mm 진입 실이동 42.1~44.1mm(런 2 와 같은 범위), 평균 3.24초. `GRASP_JUDGE` 6/6 CONTACT |
| 6 | 시퀀스 | ✅ | `OVERVIEW_SCAN nw:2 ne:1 se:0 sw:3`, se 미방문, `TRANSIT_VIA_OVERVIEW` 1건, 타겟 6건 좌표 런 2 와 1:1 동일 |
| 7 | 실패 카운터 | ✅ | `clamped`·`ARM_ARRIVAL_TIMEOUT`·`JOINT_COMMAND_REJECTED`·`EXEC_TIMEOUT`·`MoveLine IK Failed` 전부 0. `IK_FAIL` 10 (후보 탐색, 런 1·2 와 동일) |
| 8 | 브릿지 생존 | ✅ | `apply_action recovered (after 29 failures)` — 런 2 와 같은 자동 복구, 트리거 전 완료. `get_applied_actions` 0건 |
| 9 | 좌표 연속성 | ✅ | `scene_positions_received` 560건 / 560.6초 최대간격 1.0초 (종료 후 대기 포함) |
| — | 화면상 관통 | 로그 판정 불가 | 육안 |

## 재검토 — `MOVELINE_SHORT` 3 → 11건

| 구간 | 런 2 | 런 3 | 부족량 |
|---|---|---|---|
| 45mm 후퇴 (`TOOL dz=-45`) | 3 | 5 | 3.0~3.3mm (종전과 같은 범위, 런 1 은 5건) |
| **120mm 배치 하강 (`BASE dz=-120`)** | 0 | **6** | **3.5~4.1mm** (종전 2.3mm) |

새로 생긴 6건은 전부 120mm 하강이다. 스텝이 2→5mm 로 커지면서 도착 판정(관절 1.5도 이내) 순간의
드라이브 지연이 커져 부족량이 2.3 → 3.5~4.1mm 로 늘었고, 진단 임계값 **3.0mm 고정**을 넘었다.

- 실제 영향: **없다.** 이 하강 다음 동작은 release 후 **관절공간 절대 목표**로 올라가는 스플라인이라
  잔차가 전파되지 않는다 (메시지 문구 "다음 상대 이동에 전파된다" 는 이 구간엔 맞지 않는다).
  파지 쪽 진입(TOOL +z) 은 한 건도 걸리지 않았다.
- 문제: 런마다 ERROR 6줄이 거짓으로 찍혀 로그 판독을 흐린다. 임계값 3mm 는 2mm 스텝 기준으로 잡은 값이다.
- 후보 대응: 스텝 상한에 걸린 이동은 임계값을 **한 스텝 길이(5mm)** 로 — "한 스텝 이내면 도착".
  파지 쪽 2mm 스텝 이동은 3mm 그대로. **미적용 — 사용자 판단 대기.**

## 이 런에 남아 있는 것

- `IK_FAIL` 10건, 45mm 후퇴 `MOVELINE_SHORT` 5건 — 런 1·2 와 같은 성격.
- 브릿지 초기화 실패 → 자동 복구가 **세 런 연속** 재현됐다 (런 1 은 Kit 로그에 `[bridge]` 줄이 없어 미확인,
  런 2 30회, 런 3 29회). 브릿지 스크립트를 Play 전에 실행하는 문서상 순서에서 첫 물리 스텝에 view 가
  아직 없어 나는 것으로 보이며, 0.5초 안에 복구되므로 실행 절차 문제는 아니다. 근본 원인 추적은 범위 밖.
