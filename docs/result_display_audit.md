# 배치 실패 표시·용어·결과 바 — 조사와 구현 (2026-09-17)

이 문서는 2026-09-17 한 세션에서 한 일의 1차 출처다. 순서는 **로그 조사(A–D, 읽기 전용) → 사용자 결정 → 구현 1–4 → 검증** 이다.
변경마다 한 줄 기록은 [`PLANNER_CHANGES.md`](../PLANNER_CHANGES.md) 씬 절의 09-17 항목 6줄이고, 검증 보고 원문·실행 출력은
[`log/m3/offline_checks/result_display_20260917/`](../log/m3/offline_checks/result_display_20260917/) 에 있다.

**공통 전제 (사용자 지정)**: 표시·문구 계층만 바꾼다 — 플래너 로직·상태 머신·ROS 인터페이스(토픽·서비스·액션·QoS)는 바꾸지 않는다.
USD 는 상위 오버라이드 레이어에서만, 순정·임포트 애셋은 직접 고치지 않는다. 개수는 하드코딩하지 않고 씬의 실제 딸기에서 센다.
변경마다 로그 한 줄. 끝나면 `check_params.py` 통과.

---

## 0. 요약

| 항목 | 결과 |
|---|---|
| A. 배치 실패 4건 (09-16 17:15 확인 런) | 4건 모두 **같은 호출**(교시 슬롯 위 Cartesian 계획의 후처리 가드)에서 거부. 거부 가드는 **두 종류**(J6 spline jump 2 · J3 swing 2). 가드 너머 원인(왜 그 IK 해 가지가 나왔나)은 **특정 불가** |
| B. 분리 단계 실패 vs 분리 뒤 배치 경로 실패 | **부분적으로 가능** — 플래너 원시 로그(텍스트·JSONL)를 시도 순서대로 읽으면 대부분 갈리지만, 결과 코드·`pick_complete`·HUD 카운터·run_metrics 로는 안 갈린다. 과실이 물리적으로 줄기에서 떨어졌는지는 어디서도 판정하지 않는다 |
| C. '낙하' 표기 | 145줄 / 32파일 + 라벨 PNG 1장. 면접관 노출 화면·문서에서 HUD 표기를 가리키던 **9곳**만 바꿈 |
| D. 딸기 머티리얼 | 애셋 crate 2개의 UsdPreviewSurface, 색은 텍스처 연결. 상위 레이어에서 **조건부로 덮을 수 있음**(합성 값 확인, 화면은 미확인) |
| 결정 | 자막은 원인 없이 폴백 동작만. 용어: 상태 `배치 실패` / 로봇의 대처 동작 `낙하` / 분리 단계 실패 `분리 실패`. 결과 바 3색 |
| 구현 | ① 안 익은 딸기 연한 녹백색 오버라이드 레이어 ② HUD `타겟 N / 비대상 M` ③ HUD 결과 바 + 범례 ④ 노출 문서 용어 정리 |
| 검증 | 머티리얼 반박 검증 2건 must_fix 0 · HUD Kit 스텁 하네스 · 실제 실행기 코드로 17:15 런 순서 재생 · `check_params.py` 종료코드 0 |
| 남은 것 | Isaac 화면 확인(안 익은 딸기 색 반영, 타겟 수·결과 바 표시) — §8 |

---

## 1. 조사 A — 최근 런의 배치 실패 4건

### 대상 런

`log/m3/` 에서 배치 실패가 **정확히 4건인 가장 최근 런**을 골랐다. 가장 최근 런(09-17 03:45)은 실패 2건이다.
원시 로그(플래너 `PICK COMPLETE`·`_PLACE_BLOCKED`, Kit `RELEASE … PLACED/DROPPED`)로 09-15 이후 13개 런을 다시 셌고, 각 폴더 README.txt 요약과 전부 일치했다.

| 런 | 파지 / 배치 / 낙하 | 배치 실패 사유 |
|---|---|---|
| 09-17 03:45 `20260917T034554-5e41432e` | 8 / 6 / 2 | slot0·slot1 J6 spline jump |
| **09-16 17:15 `20260916T171551-df811bd1`** (17:17 확인 런) | **8 / 4 / 4** | **slot0 J6 spline jump · slot1 J6 spline jump · slot1 J3 swing · slot6 J3 swing** |
| 09-16 16:34 `72cfcaa6` | 7 / 5 / 2 | slot1 J6 spline jump · slot7 J3 swing |
| 09-16 16:17 `73dab094` · 14:37 `ea1d9ad8` | 8 / 4 / 4 | 17:15 런과 같은 네 사유 |
| 09-16 11:47 `76c9a036` | 8 / 6 / 2 | slot0 J6 spline jump · slot1 **J6 swing 180.5 > 180** |
| 09-16 03:46 `d6e265e9` | 8 / 4 / 4 | slot0·1 J6 spline jump · slot3·6 J3 swing |

### 건별 사실 (전부 `log/m3/20260916T171551-df811bd1/`)

| # | 과실 | 슬롯 | 거부 줄 (`curobo_planner.log`) | 바로 앞 J6 재작성 줄 | Kit `DROPPED` (kit_20260916_171453.log) |
|---|---|---|---|---|---|
| 1 | ripe_07 | 0 | :43 `J6 spline jump 356.6deg > 270.0deg at waypoint 29` | :42 `J6 159.9\~271.2 -> -132.2\~224.5` | :21346 (−381.3, 744.1, 674.1) |
| 2 | ripe_02 | 1 | :118 `J6 spline jump 356.1deg > 270.0deg at waypoint 42` | :117 `J6 91.1\~270.7 -> -131.3\~224.8` | :21383 (−120.0, 738.2, 720.1) |
| 3 | ripe_08 | 1 (다시 배정) | :151 `J3 swing 185.5deg > 120.0deg (start=-51.4deg -> end=134.1deg)` | 없음 | :21404 (280.3, 738.4, 663.2) |
| 4 | ripe_04 | 6 | :295 `J3 swing 201.0deg > 120.0deg (start=78.0deg -> end=-123.1deg)` | 없음 | :21485 (−220.2, 739.1, 527.1) |

바로 다음 줄은 4건 모두 `TAUGHT_TRAY_SLOTn_PLACE_BLOCKED: above plan failed; holding fruit`(:44·:119·:152·:296), 그다음 `PICK_SEQUENCE_CONTINUE place_status=failed: released fruit here …` 이다. JSONL `curobo_plan_rejected` 는 115·281·377·755행(시작·끝 관절값 포함).

### 실패 지점 — 4건 공통

- **단계**: 분리 당김(DETACH_PULL_DOWN)과 진입 역순 후퇴(RETREAT)가 끝난 직후, **트레이 이송의 첫 호출**. 하강·릴리스 단계까지 간 건은 없다 —
  브릿지 로그에서 RETREAT `MoveLine ok` 바로 다음 줄이 `SetPosition called to 600` 이다(`sim_executor_bridge.log` 43→44, 125→126, 166→167, 360→361).
- **호출** (체인은 코드, 순서·문구는 로그): 교시 슬롯 위(above) 자세로 가는 `self._plan(…)`([`tray_place_executor.py:358`](../src/strawberry_motion/scripts/tray_place_executor.py))
  → `curobo_planning_adapter.plan` 에서 MotionGen 은 해를 냈고, 후처리 `normalize_equivalents`(:163) → `in_operational_limits`(:165) → `has_no_spline_jumps`(:170) / `has_reasonable_swing`(:174) 가 거부.
- **실패 뒤**: 다시 계획하지 않았다(건당 거부 줄 1개). 2\~3ms 뒤 그 자리에서 그리퍼를 열어(hold_on_place_failure=false) Kit `DROPPED`, 3초 뒤 `DROP_REST … on floor`.
  시도는 `PICK COMPLETE (DETACH_SUCCESS_UNVERIFIED)` 로 끝난다. 슬롯 번호는 `place_status == "success"` 일 때만 오르므로([`pick_sequence_executor.py:395`](../src/strawberry_motion/scripts/pick_sequence_executor.py)) slot0·slot1 이 두 번 배정됐다.

### 원인 — 로그에 있는 만큼

- **J6 spline jump 2건**: cuRobo 궤적의 J6 가 운용 한계 +225° 를 넘었다(271.2 / 270.7). 한계 안 등가각으로 고쳐 쓰면서(−360) 궤적에 356° 불연속이 생겨 거부됐다.
  재작성 줄은 로그 근거, "한계 [−225, 225] 안의 등가값만 고른다"는 코드 근거(`trajectory_guards.py:68`, `harvest_motion_params.py:165`).
- **J3 swing 2건**: 계획 시작과 끝의 J3 부호가 반대다(한도 `MAX_TAUGHT_PLACE_TRANSFER_JOINT_DELTA_DEG[J3]` 120).
- **가드 너머(왜 그 해 가지가 나왔나)**: **특정 불가**. IK 후보·시드·비용이 로그에 없다.

### 4건이 같은 원인인가

- **실패 지점 수준 — 같다.** 모두 above 계획의 후처리 가드 거부.
- **거부 가드 수준 — 두 종류.** 다만 가드 이름으로 가르는 것은 약하다.
  (a) spline jump 2건의 궤적은 J6 swing 한도(180°)도 넘는다 — 검사 순서상 먼저 걸린 이름이 찍혔다(검증 에이전트가 JSONL 궤적으로 계산).
  (b) 09-16 11:47 런에서는 ripe_02 의 같은 시작·끝 해가 `J6 swing 180.5deg > 180.0deg` 로 거부됐다(`76c9a036/curobo_planner.log:117`).
- **가드 너머 — 특정 불가.** 로그로 확인되는 공통 사실까지만 적는다.
  1·2·4번은 거부된 계획의 끝 관절이 같은 목표에서 통과한 계획과 **다른 해 가지**다. 3번은 끝 관절이 통과한 ripe_03 과 같고 **시작 J3 부호가 반대**다(−51.4, ripe_03 은 +59.2).
  같은 과실·시작 자세·슬롯이면 다른 런(14:37·16:17·09-17 03:45)에서도 거의 같은 수치로 반복되고,
  09-17 03:45 런에서 ripe_08 이 RETREAT 뒤 J3 +51.3 에서 출발했을 때는 slot1 계획이 통과했다(`5e41432e/curobo_planner.log:151`).
  "IK 해 가지 선택 문제 하나"로 묶는 것은 추론이다. [`portfolio/H_scope_decisions.md`](../portfolio/H_scope_decisions.md) §9 의 서술(팔꿈치 분기, J6 ±225 창)과 같은 줄이다.

### 함께 발견한 문구 문제 (고치지 않음, 기록만)

- Kit `DROP_REST … N mm from below the release point` 의 N 은 **수평(XY) 거리**다(`isaac_sim_script_editor_bridge.py:421` 의 `drift`).
  `log/m3/README.md` 확인 런 절의 "릴리스점 아래 28\~47mm 에 정지"는 수직 거리로 읽힐 수 있다. 실제 낙하 높이는 약 1.1m 다.
- `…_PLACE_BLOCKED: above plan failed; holding fruit` 는 hold_on_place_failure=false 이면 곧바로 놓는데도 "holding fruit" 라고 찍힌다(실기 노드 문구).
- `PICK COMPLETE` 결과 코드는 배치에 실패한 시도도 `DETACH_SUCCESS_UNVERIFIED` 다 — grasp_result 하나로만 정한다([`harvest_result_policy.py:31`](../src/strawberry_motion/scripts/harvest_result_policy.py)).
  JSONL `pick_sequence_complete.marker_place_release_executed` 도 실행 결과가 아니라 설정값이다.

---

## 2. 조사 B — 분리 단계 실패와 분리 뒤 트레이 배치 경로 실패를 구분할 수 있는가

**결론: 부분적으로 가능.** 보존 런(log/m3)에 분리 단계 실패는 0건이라, 분리 쪽은 코드 경로로만 판단했다(당김 MoveLine 실패는 09-07·09-08 런에 4건).

코드에서 '분리 단계'는 상태 머신이 아니라 `PickSequenceExecutor.run()` 안의 호출 순서다. `execute_detach_and_retreat` 가 BASE −Z 40mm 당김과
진입 역순 후퇴를 하고, 이어지는 VERIFY_DETACH 는 센서가 없어 상수 `DETACH_UNVERIFIED` 를 기록만 한다(`pick_sequence_executor.py:1041`).

| 실패 경로 | 원시 로그에서 갈리는 신호 | HUD·집계(09-17 이전)에서 보이던 것 |
|---|---|---|
| 분리: 당김 MoveLine 실패 | `DETACH_PULL_DOWN: … -> FAIL`, JSONL `detach_pull_down.success=false` | 드러나지 않음. 실행기가 반환값을 버리고(:298 "실패해도 retreat은 항상 실행") 정상처럼 진행해 뒤 결과에 따라 배치·낙하로 셈 |
| 분리: 역진 후퇴 실패 | `ABORT: straight reverse retreat failed`, JSONL `retreat_step_complete.ok=false`, `pick_sequence_hold_latched.reason=straight_reverse_retreat_failed` | 카운터 변화 없음. 그리퍼를 열지 않고 `pick_complete` 미발행 → scan `PICK_TIMEOUT`. 단계 표시가 '후퇴'에서 멈췄다가 scan 상태에 덮임 |
| 배치: above 계획 거부 (이번 4건) | `Cartesian plan rejected: …`, `…_PLACE_BLOCKED: above plan failed`, `place_status=failed`, JSONL `curobo_plan_rejected.reason` | `result.dropped` +1(HUD `낙하 m`), Kit `DROPPED` |
| 배치: 수직 하강 실패 | `…_PLACE_BLOCKED: vertical release descend failed` | 위와 같음 |
| 배치: 슬롯에 놓은 뒤 상승 실패 | `…_RELEASED_BUT_ASCEND_FAILED`, `place_status=failed_after_release` | `result.dropped` +1 인데 Kit 은 `PLACED` — 두 수가 어긋남(보존 런 0건) |

**정보가 사라지는 곳** (위에서 아래로)
1. 실행기: 당김 반환값 폐기, 결과 코드는 grasp_result 로만 결정.
2. ROS: `/dsr01/curobo/pick_complete` 가 `std_msgs/Empty`.
3. 프로브(09-17 이전): 당김·후퇴는 진입 시점만 기록, place_status 는 `"success"` 여부만, 낙하 카운트는 label 접두어만, hold 는 감싸지 않음.
4. 버스: `result` 에 succeeded·failed·dropped·finished 만.
5. 화면: 배치·낙하 수는 완료 때만, `failed` 는 그리지 않음.
6. run_metrics: HOLD_LATCHED·DETACH FAIL·PICK_TIMEOUT 미파싱.

**없어서 구분 못 하는 것**
- **분리 판정 자체**: 실기는 센서가 없고, 시뮬은 ATTACH(파지 판정 CONTACT) 때 줄기 조인트를 꺼 "줄기에서 안 떨어짐"이라는 사건이 없다.
  예외로 Kit 가 ATTACH 를 무시하면 과실이 줄기에 남는데, 플래너·HUD 는 정상 진행하고 Kit `ATTACH ignored` 줄에만 흔적이 남는다(보존 로그 0건).
- 시뮬 브릿지 MoveLine 의 `success` 는 IK 가 끝까지 풀렸다는 뜻뿐이다 — 덜 움직여도 true 라 당김·후퇴 OK 플래그는 동작 완수의 증거가 아니다.
- 종료 이벤트에 place_status·슬롯·트레이 안/밖 필드가 없다.
- Kit `PLACED/DROPPED` 판정은 09-15(71541d6)부터라 런 11·12 로그에는 없다.

**결과 바 색 수 결정의 흐름**: 1차 지시는 "B 가 구분 불가로 오면 결과 바는 2색(배치/실패)으로 시작 — 구분을 만들려면 상태 머신을 건드려야 하는데 촬영 직전에 할 일이 아니다"였다.
B 는 "부분적으로 가능"으로 왔고, 분리 실패 칸을 채울 값이 물리 판정이 아니라는 점 때문에 2색을 권했다. 2차 지시에서 사용자가 **3색(배치 성공 / 배치 실패 / 분리 실패)** 으로 지정했다.
구현(§6-3)은 상태 머신을 건드리지 않고, 프로브가 이미 감싸던 실행기 메서드의 반환값을 읽는 방식이다 — 분리 실패는 **동작 명령 기준**이다.

> 위 '동작 명령 기준' 정의(파지 CONTACT 픽의 `execute_detach_and_retreat` 반환값)는 09-17 시점 것이다. 분리 실패의 범위는 2026-09-18 사용자 정의로 다시 잡았다 → **09-18 §9 에서 범위 재정의**.

---

## 3. 조사 C — '낙하' 표기 위치와 실제로 바꾼 곳

**전수**: `grep -rIn '낙하'`(.git 제외, build·install·run_logs 포함) 145줄 / 32파일 + `hud/labels/final_dropped.png`(글자가 픽셀). 파일명·바이너리 0건.
분류는 코드 식별자 0 · 상태값 0 · 로그 문자열 2(`label_harvest_attempt.py:196` src·_baseline, CLI 선택지) · HUD 표시 문자열 2(`make_labels.py:63`, `labels/manifest.json:65`) + PNG · 문서 141.
완전성은 검수 에이전트가 grep 결과와 목록을 대조해 누락·오탐 0 으로 확인했다. 전체 목록은 `log/m3/offline_checks/result_display_20260917/investigation_A_to_D_agents.txt`.

**용어 규칙 (사용자 09-17)**: 상태를 가리킬 때 `배치 실패`, 로봇의 대처 동작(그 자리에서 놓아 떨어뜨림)을 가리킬 때 `낙하`, 분리 단계 실패 상태는 `분리 실패`.

**고른 기준 (사용자)**: 면접관이 볼 화면·문서에 노출되는가. 내부 식별자는 안 바꾼다. 노출 = 영상(HUD·자막), `README.md`, `PROJECT_SUMMARY.md`, `portfolio/README.md`·`E`·`H`.
노출 문서 안에서도 **HUD 표기를 가리키는 줄만** 바꿨다 — 떨어지는 물리 사건을 말하는 '낙하'는 규칙상 대처 동작이라 그대로다. 과거 런 로그·확정 결과는 제외.

| # | 위치 | 바꾼 내용 |
|---|---|---|
| 1 | `hud/make_labels.py:63` → `labels/manifest.json`·`final_dropped.png` | HUD 라벨 `낙하` → `배치 실패`(범례 22px). 키 `final_dropped` 는 유지. 문구는 `hud/result_bar.py` 가 단일 출처 |
| 2 | `portfolio/README.md:146` | 자막 예시를 확정 문구로(§5) |
| 3 | `portfolio/README.md:132` | "…배치·낙하는 HUD 에 찍힌 것만 증거" → "배치 결과(배치 성공·배치 실패·분리 실패)" |
| 4 | `portfolio/README.md:155` | 완료 줄 `배치 n · 낙하 m` → 결과 바와 범례 `배치 성공 n · 배치 실패 m · 분리 실패 k` (같은 칸의 "낙하·착지가 아래쪽에서"는 물리 사건이라 유지) |
| 5 | `portfolio/README.md:159` | 자막 용어에 `타겟 N / 비대상 M`, 범례 표기, 용어 규칙 추가 |
| 6 | `portfolio/README.md:161` | 예시 `배치 4 · 낙하 4` → `배치 성공 4 · 배치 실패 4 · 분리 실패 0` |
| 7 | `README.md:74` | HUD 설명에 타겟·비대상 수·결과 바 표기 |
| 8 | `PROJECT_SUMMARY.md:60` | 같음 |
| 9 | `portfolio/H_scope_decisions.md:334` | "완료 줄에 `낙하 m` 으로 센다" → "HUD 에 센다(09-17 부터 결과 바 범례 `배치 실패 m`)" |

**노출되지만 그대로 둔 줄**: 과거 결과·완료 기록(`PROJECT_SUMMARY.md` 106·133·154·156, `portfolio/E_metrics.md` 67·72·126 — 수치는 실제로 떨어진 과실 수라 지금도 맞다),
물리 사건(`H` 164, `portfolio/README.md` 84·103·145·154), 작업 이력(`portfolio/README.md` 5).
**내부 식별자 유지**: 라벨 키 `final_dropped`, 버스 `result.dropped`, Kit `DROPPED outside tray`·`DROP_REST`, run_metrics `dropped`.

**기록 작업(같은 날) 때 추가로 갱신한 내부 문서**: `hud/README.md`, `docs/run_guide.md`, `docs/usd_structure.md` 의 **현재 화면·레이어 설명**.
용어 정리 목록 밖이었지만 옛 완료 둘째 줄을 설명하고 있어 새 HUD 기준으로 고쳤다. 그 안의 과거 이력 문장(T4c·S2·S5 날짜 서술)은 그대로다.

---

## 4. 조사 D — 딸기 머티리얼과 오버라이드 가능성

- **정의 위치** (둘 다 crate 바이너리, 줄 번호 없음): 익은 딸기 `assets/strawberry/strawberry_ripe.usd` `/strawberry/materials/frut333_Mat`,
  안 익은 딸기 `strawberry_unripe.usd` `/strawberry/materials/frut_2333_Mat`. 둘 다 UsdPreviewSurface(`Principled_BSDF`), `diffuseColor` 가 `Image_Texture`(각각 `frut333.png`·`frut_2333.png`)에 연결.
  셰이더 구조는 같고 색 텍스처만 다르다(Gloss·Normal 텍스처는 md5 동일). displayColor 미작성, MDL 출력 없음.
- **연결**: `main_scene.usd` 과실 prim → `strawberry.usd` reference → variantSet `ripeness` → 위 두 파일. 바인딩은 `geo/fruit/mesh` 에 직접(weakerThanDescendants). 과실에는 인스턴싱 없음(instanceable 27개는 전부 로봇).
- **레이어 스택**(강한 순, 09-17 이전): 세션 > `main_scene.usd` > `layout_layer` > `physics_layer` > `lighting_layer`. 씬 레이어에 과실 머티리얼 opinion 은 없었다. 어느 로컬 레이어의 opinion 이든 애셋 reference 보다 강하다.
- **오버라이드 — 조건부로 가능** (pxr 메모리 안에서 합성 값 확인, 저장 없음):
  1. 색이 텍스처 연결이라 `diffuseColor` 값만 넣으면 텍스처가 계속 이긴다. 무인자 `DisconnectSource()` 로 명시적 빈 연결(`connect = None`)을 써야 한다. `ClearSources()` 로는 안 막힌다.
  2. 텍스처를 두고 색조만: `Image_Texture` 의 `inputs:scale`/`bias`(미작성).
  3. `inputs:file` 교체는 상대 경로가 그 값을 쓴 레이어 기준으로 풀려 `./textures/…` 그대로는 깨진다.
  4. 과실 루트에 새 머티리얼을 바인딩하려면 `strongerThanDescendants` 가 필요하다(mesh 직접 바인딩이 있어서).
  5. 합성 머티리얼 경로가 과실마다 따로라(`/World/<과실>/materials/…`) 익은·안 익은·과실 하나씩 따로 바꿀 수 있고, 경로는 variant 선택에 묶인다.
  6. `layout_layer.usd` 에 넣으면 `gen_random_layout.py:62` 정규식 제약(과실 over 블록 안 translate 앞에 중첩하면 `--apply/--restore` 가 멈춤, 같은 prim 을 한 번 더 over 하면 Duplicate prim).
  7. 세션 레이어 방식은 씬을 다시 열면 다시 적용해야 한다.
  8. RTX 화면 반영은 오프라인으로 확인 불가.
- **색은 현재 시뮬 판정에 쓰이지 않는다**: 브릿지는 prim 이름으로 익음을 판정한다. 단, 이름에 `strawberry` 가 든 Xform 을 새로 만들면 브릿지 발행 루프(603–622)가 과실 타겟으로 발행한다(Material·Scope 는 해당 없음).
  실기 `strawberry_fusion_node.py` 는 HSV 빨강 필터를 쓰지만 시뮬 `run_nodes.sh` 는 이 노드를 띄우지 않는다.

---

## 5. 자막 문구 (확정)

규칙(사용자): A 결과에서 원인이 하나로 특정되면 그 원인을 넣고, "특정 불가"면 원인은 빼고 폴백 동작만 쓴다 — 없는 원인을 지어 넣는 것보다 안 쓰는 게 낫다.
A 는 거부 가드가 두 종류이고 가드 너머는 특정 불가 → **원인 없이 폴백 동작만**.

```
트레이로 옮기지 못하면
그 자리에서 놓고 계속 진행
```

- 12자 / 15자(공백 포함), 자막 규칙(한 줄 16자 안팎·최대 2줄) 안.
- 종전 예시 `이송 계획 거부 → 트레이 밖에서 놓음, 낙하` 에서 뺀 것: "이송 계획 거부"(원인에 가깝고 HUD 에 찍히지 않는 판정), "낙하"(당시 HUD 표기 변경 예정).
- "계속 진행" 근거: 4건 모두 `continuing to the next target`, 마지막 건(ripe_04)도 스캔 자세 복귀 뒤 순회를 마쳤다. "다음 과실로"는 마지막 건에서 틀려서 쓰지 않는다.
- 반영 위치: `portfolio/README.md` §편집(실패 장면 줄).

---

## 6. 구현

### 6-1. 안 익은 딸기 색 — 오버라이드 레이어

| 항목 | 내용 |
|---|---|
| 색 | sRGB **#B4D69A**(연한 녹백색) → UsdPreviewSurface 선형 (0.4564, 0.6724, 0.3231). CIELAB L\* 82 · C\* 34 |
| 선택 근거 | 순백과 ΔE76 39, 회백색(228,228,222)과 32 — 흰색 계열(병든 딸기 색)과 안 겹침. 익은 텍스처 파일 평균과 59 — 축소 크기에서 익은 딸기와 갈림. 텍스처 평균은 파일 전체(UV 밖 포함) 기준 참고값 |
| 레이어 | `scenes/layers/appearance_layer.usd`(생성물) — 비대상 과실 4개의 `…/materials/frut_2333_Mat/Principled_BSDF` 에 `inputs:diffuseColor` 값 + `connect = None` 뿐 |
| 씬 | `main_scene.usd` subLayers **맨 앞**에 한 줄, 레이어 doc 에 한 구절. 그 밖 무변경 |
| 생성기 | `scripts/scene_tools/gen_unripe_appearance.py` — 비대상 과실을 브릿지 규칙(/World 직계, 이름에 `unripe`)으로 찾고, 메시 바인딩 → surface 셰이더(UsdPreviewSurface 확인) → diffuseColor 를 대상으로. 이름·개수·머티리얼 이름 하드코딩 없음. 하나라도 못 찾으면 저장 없이 종료코드 1. 저장 뒤 씬을 새로 열어 검증(비대상 값·연결 차단, 노말 연결 유지, 타겟 텍스처 연결 유지). 셰이더마다 `[appearance]` 한 줄 |
| 유지되는 것 | 노말·러프니스 텍스처(요철·광택), 애셋 파일 31개 md5 |
| 잃는 것 | 안 익은 과실 텍스처의 색 변화가 한 색이 된다 — 텍스처의 녹색 계열 화소 약 19%(꽃받침으로 추정)도 같은 색 |
| 바꾸려면 | `UNRIPE_SRGB` hex 하나를 고치고 생성기를 다시 돌린다. 과실이 늘거나 이름이 바뀌어도 다시 돌린다. 반영은 **씬 재로드** 뒤 |

`gen_random_layout.py`(layout·physics 만 씀), `isaac_batch_orchestrator.py`(런마다 main_scene 재로드, 저장 안 함), `run_batch.sh` 는 새 레이어를 덮거나 무력화하는 경로가 없다(코드 확인).

### 6-2. HUD `타겟 N / 비대상 M`

- 노드 행 바로 아래, 항상 표시. `타겟` 흰색 24px, `비대상` 흐린 색 24px.
- 숫자는 `hud/scene_fruit.py` 가 열린 씬의 /World 직계 과실 prim 을 **브릿지 발행 필터와 같은 규칙**(이름에 `strawberry`, `robot` 아님 / `unripe` 면 비대상)으로 1초마다 센다. 고정값 없음. 스테이지가 없으면 `-`.
- 수확·낙하된 과실도 prim 은 남으므로 런 중 N 은 변하지 않는다.
- 바뀔 때마다 Kit 로그 한 줄: `[hud] scene fruit: target N / non-target M (<씬 파일>; bridge publish filter; non-target constant diffuse k/M)`.
  끝 부분은 §6-1 오버라이드가 열린 스테이지에 들어왔는지다 — `0/M` 이면 씬을 다시 불러오지 않은 것. ripeness variant 가 이름과 어긋나면 WARNING 한 줄.

### 6-3. HUD 결과 바

| 결과 | 색 | 판정 (프로브가 실행기 메서드 경계에서 읽음) |
|---|---|---|
| **배치 성공** | 초록 #5AD469 | `tray.execute_marker_place_after_retreat` 가 `"success"` (종전 `result.succeeded` 와 같은 경계·기준) |
| **배치 실패** | 빨강 #FF4D5E | 같은 메서드가 그 밖의 상태 — `failed` · `failed_after_release` · `skip` · `preview_hold` · `tray_complete`. 분리까지 끝난 과실이 배치를 끝내지 못한 **상태**. 그 뒤 놓아 떨어뜨리는 것(낙하)은 대처 동작이라 따로 세지 않는다 |
| **분리 실패** | 호박색 #F5A524 | 파지 판정이 `GRASP_CONTACT_DETECTED` 인 픽에서 `execute_detach_and_retreat` 가 `None`(후퇴 실패로 실행기가 시퀀스를 잡음) |
| (회색) | 단계 바 꺼진 색 | 결과가 나지 않은 타겟 — 파지 후보 전부 IK 실패로 건너뜀(ABORT), 직선 진입 실패, 파지 판정 실패(`PLACE_GATE_BLOCKED`), 그리퍼 닫기 실패, 빈손 판정 픽의 후퇴 실패 |

> 위 표의 '분리 실패'·'(회색)' 정의는 09-17 것이다. 분리 실패의 범위는 2026-09-18 사용자 정의로 다시 잡았다 → **09-18 §9 에서 범위 재정의**.

- **위치·모양**: `수확 완료 n / N`(09-17 당시 완료 때만 보임. 09-19 부터 이 줄은 런 내내 자리를 지킨다 — 런 전 `수확 시작 전`, 런 중 `수확 진행 중`, 끝나면 `수확 완료 n / N`, 완료 호출 없이 끝난 런은 `수확 중단`, `hud/README.md`) 바로 아래. 칸 수 = §6-2 씬 타겟 수, 바뀌면 바를 다시 만든다. 칸 높이 10·간격 3·모서리 2·꺼진 색은 단계 진행 바와 같다.
  폭은 패널 안쪽 전체(408px) — 완료 블록이 가운데 정렬이라 단계 바처럼 머리 열(60px)만큼 들여 쓰지 않았다. 그래서 두 바의 왼쪽 끝이 다르다.
- **칸 순서**: 결과가 난 순서(시도 순서)대로 앞에서부터 칠한다.
- **범례**: 바 아래 `배치 성공 n · 배치 실패 m · 분리 실패 k`, 런 내내 표시, 0 도 표시. 라벨 PNG 22px(두 자리 숫자까지 408px 안). 칸 수보다 결과가 많으면 범례는 전부 세고 바는 앞부분만 보이며 WARNING 한 줄.
- **종전 완료 둘째 줄 `배치 n · 낙하 m`(T4c 09-15)은 이 범례로 대체**했다 — 같은 숫자를 두 용어로 두 번 띄우지 않으려고. `result.dropped` 는 버스에서 계속 센다(Kit `dropped=n` 대조용, 화면 미표시).
- **로그**: 칸이 붙을 때마다 플래너 노드 로그(`curobo_planner.log` 로 보존되는 stderr) `[harvest_probe] 결과 바 n번째 = <키> (<라벨>, place_status=… | execute_detach_and_retreat returned None)` 한 줄,
  Kit 로그 `[hud] result bar n/N: <키>` 한 줄, 새 런으로 비워지면 `[hud] result bar cleared (new run)`. 완료 때 scan 후보 총계가 씬 타겟 수와 다르면 note 한 줄.
- **버스**: `status_bus.result.outcomes`(목록, 소유 planner) + 락 안에서 붙이는 `append()`. `bus_merge` 는 planner 의 `run.started_at` 이 scan 과 다르면 직전 런 결과를 가져오지 않는다
  (노드를 살려 둔 채 재트리거했을 때 첫 픽 전까지 직전 런 칸이 뜨지 않게). 손상된 스냅샷(run 이 dict 아님)은 그 값만 없는 것으로 본다.
- **판정의 한계**: 분리 실패는 **동작 명령이 끝까지 갔는지**다. 과실이 실제로 떨어졌는지는 판정하지 않는다(§2). 당김 MoveLine 하나만 실패하면 실행기가 무시하고 진행하므로 결과는 배치 쪽에서 정해진다.
  `failed_after_release` 는 과실이 슬롯에 놓였어도 상태로는 배치 실패다(Kit 는 `PLACED`, 보존 런 0건).
- 규칙·문구·색의 단일 출처는 `hud/result_bar.py`(순수 파이썬), 씬 집계는 `hud/scene_fruit.py`(pxr 만). HUD 는 칠하기만 한다.

### 6-4. 변경 파일

| 구분 | 파일 |
|---|---|
| 신설 | `strawberry_harvest/scenes/layers/appearance_layer.usd`(생성물), `strawberry_harvest/scripts/scene_tools/gen_unripe_appearance.py`, `strawberry_harvest/scripts/hud/result_bar.py`, `strawberry_harvest/scripts/hud/scene_fruit.py`, `hud/labels/count_targets.png`·`count_non_targets.png`·`final_detach_failed.png` |
| 수정(코드·씬) | `scenes/main_scene.usd`(subLayers 한 줄·doc), `hud/status_bus.py`, `hud/bus_merge.py`, `hud/harvest_probe.py`, `hud/make_labels.py` + `labels/manifest.json`·`final_placed.png`·`final_dropped.png`, `scripts/isaac_sim_viewport_display.py`(ASCII 전용 유지), `scripts/scene_tools/check_camera_framing.py`(HUD 영역 아래 끝 0.43 → 0.49) |
| 수정(문서) | §3 의 9곳, `PLANNER_CHANGES.md`(6줄), 이 문서와 기록 작업의 문서들(§3 끝) |
| 바꾸지 않음 | `src/**`(실기 노드·시뮬 브릿지 노드), `strawberry_harvest/assets/**`, layout·physics·lighting 레이어, `log/m3/<run>/` 런 로그 |

---

## 7. 검증

| 대상 | 방법 | 결과 |
|---|---|---|
| 공통 전제 | `git status`, 애셋 md5, 코드 grep | `src/` 변경 0, 애셋 31개 md5 불변, 개수·이름 하드코딩 없음 |
| `check_params.py` | 실행 | 종료코드 0 "전부 정합" (`offline_checks/…/check_params.txt`) |
| 씬 회귀 | `verify_vines.py`, `verify_egg_carton.py`, `check_camera_framing.py` | T3·T4-3 검증 통과, 구도 `UI overlap: none` |
| 머티리얼 | 생성기 자체 검증 + 실패 경로(머티리얼 없는 과실·variant 불일치·subLayers 순서) + 반박 검증 2건(합성 정확성 / 금지·회귀) | must_fix 0. 두 번 생성해도 레이어 md5 동일(aa8da9db…). 레이어를 뺀 비교에서 달라지는 합성 키는 diffuseColor 4개뿐 |
| HUD 화면 코드 | omni.* 스텁 + 진짜 pxr·main_scene 으로 `isaac_sim_viewport_display.py` 실행 — build_fn 지연·즉시 × 한국어·영문 폴백 4조합, 조합마다 85개 검사 | 수정 전 4조합 전부 통과. 수정 후 재실행은 조합마다 84 통과 + 1 — 그 1 은 "로그에 None 이 찍힌다" 결함을 확인하던 정보성 검사라 결함 수정으로 뒤집힘 |
| 프로브·버스 | **실제 `PickSequenceExecutor.run()`** 에 가짜 의존성을 넣고 프로브를 걸어 17:15 런의 8픽 순서(텍스트 로그·JSONL·Kit 세 출처 일치)를 재생. 경로별 시나리오 A–L, 래퍼 투명성, append 경합(16스레드×2000), bus_merge 조합 23개, 같은 프로세스 두 런 | outcomes `[배치 실패, 배치 성공, 배치 실패, 배치 실패, 배치 성공, 배치 성공, 배치 성공, 배치 실패]` = 로그 순서, 배치 성공 4 = succeeded, 배치 실패 4 = dropped. 수정 후 재실행 통과(T4 한 항목은 "파지 판정 없이 분리 함수만 호출해도 칸이 붙는다"는 수정 전 기대라 뒤집힘) |

> 위 검증은 09-17 정의(파지 CONTACT 픽의 `execute_detach_and_retreat` 반환값) 기준이다. 분리 실패의 범위는 2026-09-18 사용자 정의로 다시 잡혔고 판정은 `run()` 종료 시점 한 곳으로 바뀌었다 → **09-18 §9**, 오프라인 재현 검사는 `hud/check_result_bar_probe.py`.

**검증에서 찾아 고친 것** (전부 09-17 안)
1. `bus_merge`: 스냅샷의 `run` 이 dict 가 아니면 `load()` 전체가 예외 → HUD 전체 정지(HEAD 는 그 파일만 건너뜀). 그 값만 없는 것으로 처리.
2. 빈손 판정 픽의 후퇴 실패에 분리 실패 칸이 붙음 → 파지 판정 CONTACT 픽만 센다.
3. `append` 실패인데 "결과 바 0번째" 로그가 찍힘 → 경고로 바꿈.
4. 씬 집계가 안 된 상태의 HUD 로그에 `None` → `?`.
5. 씬 집계가 바뀔 때 로그 두 줄 → 한 줄로 합침.
6. `check_camera_framing.py` HUD 영역 아래 끝 0.43 → 0.49(패널이 커져서).

**고치지 않은 note**
- 영문 폴백(라벨 PNG 없을 때)의 범례 폭 약 486px 이 패널 안쪽 408px 을 넘는다 — 녹화 경로(한국어 PNG)는 361/398px.
- 결과 바 왼쪽 끝이 단계 바와 다르다(§6-3, 의도).
- `failed_after_release` 는 빨강인데 Kit 는 `PLACED`(§6-3, 규칙상 상태).

**근거 파일** — [`log/m3/offline_checks/result_display_20260917/`](../log/m3/offline_checks/result_display_20260917/):
`investigation_A_to_D_agents.txt`(조사 A–D 에이전트 12개 보고 원문), `material_override_agents.txt`(구현 + 반박 검증 2건),
`hud_result_bar_agents.txt`(HUD 스텁 하네스·프로브 재생 보고, **수정 전 코드 기준** — 수정 후 재실행 결과는 이 문서 위 표),
`gen_unripe_appearance_run.txt`, `check_params.txt`. 하네스 스크립트는 세션 스크래치에 있었고 남지 않았다.

---

## 8. 녹화 전 확인 (Isaac 화면)

1. **반영 순서**: 씬 재로드(새 서브레이어) → 뷰포트 렌더 해상도 1920×1080 맞춤(`isaac_sim_fit_viewport_1080p.py`, `portfolio/README.md` 녹화 절차 3번) → 브릿지·뷰포트 표시 스크립트 Run → `bash scripts/run_nodes.sh --kill` → `bash scripts/run_nodes.sh`(프로브는 노드가 뜰 때 읽힌다) → 트리거.
2. Kit 콘솔 `[hud] scene fruit: target 8 / non-target 4 (…; non-target constant diffuse 4/4)` — `0/4` 면 씬 재로드가 안 된 것.
3. 안 익은 딸기가 연한 녹백색으로 보이는지(텍스처 색이 보이면 Kit/RTX 가 빈 연결을 따르지 않는 경우), 조명 아래 흰색으로 날아가지 않는지, 롱샷 축소에서 익은 딸기와 갈리는지.
4. HUD: `타겟 8 / 비대상 4` 상시, 결과 바 8칸이 결과마다 칠해지고 범례가 0 부터 오르는지, 완료 때 `수확 완료 n / 8` 바로 아래 바가 있는지.
5. 플래너 로그의 `[harvest_probe] 결과 바 n번째 …` 줄 수 = 범례 합계.

---

## 9. 분리 실패 범위 재정의 (2026-09-18)

이 절은 2026-09-18 세션 기록이다. §0\~§8(2026-09-17)은 그대로 두고 이 절만 새로 붙인다. 코드 변경은 `strawberry_harvest/scripts/hud/` 아래 파일들과
`isaac_sim_viewport_display.py`(`git diff` 로 확인 가능, 이 기록 시점엔 미커밋). 관련 문서 갱신은 [`hud/README.md`](../strawberry_harvest/scripts/hud/README.md),
[`run_guide.md`](run_guide.md) 에도 있다.

### 9-1. 계기 — 09-18 런에서 난 것

런 `run_logs/20260918_094517`, 타겟 root/ne/sw `(280,783,700)mm`(런의 4번째 타겟, `scan.log:91` `PICK_SKIP_ATTEMPTED root/ne/sw pos=(280,783,700)mm`)에서
45mm 직선 진입 MoveLine 이 브릿지에서 IK 실패로 거부됐다(`bridge.log:144` `MoveLine IK Failed at step 23/23`) → 플래너 `ABORT: 직선 진입 실패`(`planner.log:142-143`)
→ 재스캔은 재시도 없이 다음 타겟으로 넘어갔다(`PICK_SKIP_ATTEMPTED`) → 과실이 씬에 남았다.

그 시점 규칙(§6-3, 09-17)으로는 이 픽이 결과 칸을 못 받는다. HUD 는 8칸 중 7칸에 색을 칠했다 — 순서대로 배치 실패·배치 성공·배치 실패·배치 성공×4
(`planner.log:45,85,122,184,222,259,296` 의 `[harvest_probe] 결과 바 n번째 = …`), 그리고 이 픽(4번째)은 회색 꼬리 칸으로 남았다. 범례는 `배치 성공 5 · 배치 실패 2 · 분리 실패 0`.
사용자는 이 칸이 호박색(분리 실패)이길 기대했다.

09-17 정의는 "파지 판정이 `GRASP_CONTACT_DETECTED` 인 픽에서 `execute_detach_and_retreat` 가 `None`" 뿐이었다(§6-3). 보존 런 23개·182시도(log/m3)에서 이 정의는 0건이다 —
실제로 난 것은 직선 진입 실패 5건, 열린 조우 하강(open stem descent) 실패 1건, 빈손(`GRASP_EMPTY`) 0건, 그리퍼 닫기 실패 0건, 후퇴 실패 0건, 배치 실패(above 계획 거부) 45건이다.

실행기(`pick_sequence_executor.py`)의 진입 시작부터 배치 호출 사이에는 실패 출구가 10개 있다: 직선 진입 :958, leftmost 추가 전진 :973, 열린 조우 하강 :1007, NW 보정 :1012,
그리퍼 닫기 :1017-1027, 파지 판정 소비(게이트 차단) :361-391, 당김 :298(반환값 버려짐), 후퇴 :316-332, 배치 호출 결과 :393-470. `run()` 은 이 경로 어디서도 반환값이 없다(bare return) —
09-17 정의는 이 중 딱 하나(후퇴 실패로 시퀀스가 잡히는 경우)만 셌고 나머지 9개는 회색으로 흘렀다.

### 9-2. 사용자 정의 — 분리 / 수확

- **분리**의 목표: 그리퍼가 과실 앞에 멈춰선 뒤(프리어프로치 도달 — 경로는 이미 계산됨) 접근·진입·하강·파지·당김·후퇴를 거쳐 **배치 경로 계산 직전까지 과실을 집은 상태를 만드는 것**이다.
  그 구간 어디서 실패하든 분리 실패다.
- **수확** = 분리 + 배치 둘 다 끝나야 한다("따도 제자리에 놔둬야 수확").
- ⛔ "미수확" — 프로젝트 용어가 아니다. 대체 용어로 제안됐으나 사용자가 거부했다. 이 문서를 포함해 어디서도 쓰지 않는다.

### 9-3. 결정

**1안(하나로 묶기) + B안**: 결과 바 키·라벨은 `detach_failed`/'분리 실패' 그대로 두고 범위만 넓힌다. 단계 진행 바의 DETACH 라벨만 '분리' → '당김'으로 바꾼다(§9-6).

### 9-4. 조사 — 출구 10개, 보존 런 수, 화면 제약

- 실패 출구 10개는 §9-1 목록과 같다. 픽당 결과 칸은 정확히 하나여야 하므로, 출구를 하나씩 나열해 세는 대신 "직선 진입이 시작된 픽이 결과 칸 없이 `run()` 을 끝내면 분리 실패"로
  한 곳에서 판정하기로 했다 — 출구를 나열하면 새 출구가 생겼을 때 또 빠질 수 있다.
- 보존 런 23개·182시도(log/m3)에서 09-17 정의는 0건, 실제 분리 구간 실패는 6건(직선 진입 5 + 열린 조우 하강 1)이었다(§9-1).
- 화면 제약: 결과 바 범례 폭은 패널 안쪽 408px 이고 항목 3개(배치 성공·배치 실패·분리 실패)가 두 자리 숫자까지 그 폭에 맞게 잡혀 있다(§6-3, `hud/README.md`). 항목을 늘리지 않고
  기존 3키 범위만 넓히는 쪽이 이 제약과 맞았다.

### 9-5. 새 규칙 (구현됨)

- `hud/result_bar.py`: `outcome_of_pick_end(entered, recorded, place_enabled)` — 직선 진입이 시작된 픽(`entered`)이 결과 칸 없이(`recorded=False`) `run()` 을 끝내면 `detach_failed`.
  배치 기능이 꺼진 구성(`place_enabled=False`)이면 판정하지 않는다. `outcome_of_detach` 는 지웠다.
- `hud/harvest_probe.py`: 픽 상태를 `{"grasp","entered","recorded","why"}` 로 늘렸다. 직선 진입 직전(`_execute_final_approach_fn` 호출 전)에 `entered=True`.
  결과 칸은 `_record` 가 픽당 1회만 붙인다(`recorded` 래치). 실행기 `run()` 이 끝난 직후 실행되는 코드(`_after_run`)가 `outcome_of_pick_end` 로 판정해 칸을 붙인다.
  기존 후퇴 실패·파지 판정·하강 감지 지점은 이제 칸을 붙이지 않고 사유 문자열(`why`)만 남긴다 — `straight_entry` / `leftmost_extra_advance` / `open_stem_descent` / `after_descent` / `nw_base_y_nudge` / `grasp=<판정>` / `retreat_failed` / `run() raised after <직전 사유>`
  (추가 전진·NW 보정은 기본 비활성 출구라 실패했을 때만 사유가 남는다). 픽 상태는 스레드별(`threading.local`)로 둔다 — 플래너 노드가 ReentrantCallbackGroup + MultiThreadedExecutor 라 `run()` 두 개가 겹칠 틈이 이론상 있고, 공유 dict 였다면 앞 픽의 종료 판정이 뒤 픽의 상태를 읽어 배치 성공 픽에 분리 실패 칸을 붙일 수 있었다(리뷰에서 스레드 재현).
  로그: `[harvest_probe] 결과 바 n번째 = detach_failed (분리 실패, <why>)`.
- **회색** = 프리어프로치에 도달하지 못한 타겟이다: 파지 후보 전부 IK 실패로 건너뜀(ABORT), 프리어프로치 스플라인 실패, x·z 가드 스킵, 계획 전용 모드 홀드(`measured_tcp_plan_only`, 녹화 구성에선 꺼짐).
  진입 뒤 `run()` 이 예외로 끝난 픽은 회색이 아니라 분리 실패다 — 프로브가 실행기 메서드를 감싸는 코드(`_wrap`)에 `always` 자리를 두어 원본이 예외를 던져도 `run()` 종료 판정이 한 번 돌게 했다(예외는 그대로 다시 던진다, 사유 `run() raised after <직전 사유>`).
  같은 과실을 두 번 시도하면 칸도 두 개다(칸은 시도 순서) — 결과 수가 칸 수를 넘으면 HUD 가 overflow 경고를 찍는다.
- **배치 게이트**: 배치 기능이 꺼진 구성에서는 정상 픽도 배치 호출 없이 끝나므로 판정하지 않는다(위 `place_enabled`).
- **예외 경로(당김 명령만 실패)**: 당김 MoveLine 하나만 실패하면 실행기가 무시하고 후퇴·배치를 계속하므로 결과는 배치 쪽에서 정해진다 — 결과 칸에는 안 붙고
  로그 한 줄만 남긴다: `[harvest_probe] 당김 명령 실패 — …`(결과 영향 없음).
- 분리 실패의 한계 문장(동작 명령이 배치 호출까지 갔는지가 기준이며 물리적 분리는 어디서도 판정하지 않는다 — `DETACH_UNVERIFIED`)은 그대로다.

### 9-6. 단계 진행 바 DETACH 라벨 — '분리' → '당김'

결과 바의 '분리 실패' 가 진입부터 배치 호출 직전까지 구간 전체(후퇴 실패 포함)를 가리키게 되면서, 단계 진행 바의 '분리'(BASE −Z 40mm 당겨 떼는 그 한 동작)와 같은 글자가 한 화면에 두 뜻으로 뜨게 됐다.
`make_labels.py` 의 `STATE_KO["DETACH"]` 를 '당김'으로 바꾸고 `labels/state_DETACH.png` 를 다시 생성했다(`labels/manifest.json` 갱신). 영문 폴백은 "PULL"(`isaac_sim_viewport_display.py`
`EN["state"]["DETACH"]`). enum 키 `DETACH`·단계 색(`PHASE["DETACH"]` 0xFF3D5C)은 그대로다 — 이름만 바꿨다.

### 9-7. 파일별 변경 (git diff 기준)

| 파일 | 변경 |
|---|---|
| `hud/harvest_probe.py` | 픽 상태 확장(`entered`·`recorded`·`why`, 스레드별), `_wrap` 에 `always`(예외로 끝나도 한 번) 추가, `_after_run`/`outcome_of_pick_end` 판정 도입, 기존 실패 지점은 칸 대신 사유만 남김(추가 전진·NW 보정 실패 사유 포함), 당김 실패 로그 분리, `_enable_marker_place` 부재 경고 |
| `hud/result_bar.py` | `outcome_of_detach` 삭제 → `outcome_of_pick_end` 신설, docstring 재작성(정의·회색 정의·한계) |
| `hud/status_bus.py` | 주석 갱신 — `detach_failed` 범위 확장 기록 |
| `hud/make_labels.py` + `labels/manifest.json` + `labels/state_DETACH.png` | DETACH 라벨 '분리' → '당김' |
| `isaac_sim_viewport_display.py` | 영문 주석·`EN["state"]["DETACH"]` 갱신("PULL"), ASCII 전용 유지 |
| `hud/check_result_bar_probe.py`(신설, 이 기록 시점엔 미커밋) | 오프라인 재현 검사 — §9-9 |

`src/**`(실기 노드·시뮬 브릿지 노드)는 이번 `git diff` 에 없다 — 프로브가 실행기 메서드를 감싸는 지점만 늘었다.

### 9-8. 09-18 런에 새 규칙을 적용하면

`run_logs/20260918_094517` 8타겟 순서 그대로에 4번째 자리만 바뀐다: root/ne/sw(§9-1, 사유 `straight_entry`)가 호박색 분리 실패로 채워져 8칸 전부 색이 나가고 회색 0,
범례는 `배치 성공 5 · 배치 실패 2 · 분리 실패 1`이 된다.

### 9-9. 검증 상태

오프라인 재현 하네스는 `strawberry_harvest/scripts/hud/check_result_bar_probe.py`(이 기록 시점엔 미커밋, 런타임 노드 아님) — 실행기 원본 클래스 `PickSequenceExecutor` 에
가짜 의존성(로거·모션 함수·그리퍼·트레이 실행기)을 넣어 `run()` 을 실제 제어 흐름 그대로 돌리고, 그 위에 HUD 프로브를 실제로 붙여 결과 칸이 규칙대로 붙는지 본다. 이 하네스의
실행 결과(2026-09-18, `python3 strawberry_harvest/scripts/hud/check_result_bar_probe.py`): **시나리오 18개 · 실패 0개** — 직선 진입 실패 / 열린 조우 하강 실패 / 그리퍼 닫기 실패(후퇴 성공·후퇴 실패 hold) /
빈손 게이트 차단 / 후퇴 실패 / 진입 뒤 `run()` 예외 → 각각 분리 실패 칸 정확히 1개와 기대 사유, 배치 성공·배치 실패·당김 명령만 실패 → 배치 쪽 칸 1개, 회색 3종(후보 전부 IK 실패·프리어프로치 스플라인 실패·x 가드 스킵)과
배치 기능 꺼짐 → 칸 0개, 두 픽 연속 → `['detach_failed','placed']`, 정상 픽 단계 순서 ENTER→GRASP→DETACH→RETREAT→PLACE→RETURN, 라벨 '당김'/'분리 실패'(89.0px), 계측 지점 11개 전부 존재.
별도로 리뷰 에이전트가 같은 프로브를 붙인 자체 하네스로 22개 경로(추가 전진·NW 보정·hold=True 게이트 차단·skip/preview_hold/tray_complete 포함)에서 칸 수 불일치 0을 확인했다(그 하네스는 보존하지 않음).
추가 전진·NW 보정 출구는 녹화 구성(run_nodes.sh)에서 비활성이라 보존 하네스에는 넣지 않았다.

### 9-10. 남은 것

노드 재기동(`bash scripts/run_nodes.sh --kill` → `bash scripts/run_nodes.sh`, 프로브는 노드가 뜰 때 읽힌다) + Kit 스크립트 Run(`isaac_sim_viewport_display.py` 가 hud 모듈을
importlib.reload 한다) → 화면이 §9-8 기대값과 맞는지 확인 → T5. 씬 재로드는 필요 없다.

### 9-11. 화면 확인 (2026-09-19)

런 `run_logs/20260919_010906`(Kit 로그 `kit_20260919_010807.log`, 노드 재기동 + Kit 스크립트 Run, 씬 재로드 없음).

| 항목 | 결과 |
|---|---|
| 새 프로브 로드 | 플래너 기동 시각이 `harvest_probe.py`·`result_bar.py` 수정 시각보다 뒤. HUD 기동 줄 `HUD lang=ko, labels=42` |
| 단계 진행 바 DETACH 라벨 | 화면에서 '당김' 확인(사용자) |
| 결과 칸 | 8픽 모두 프리어프로치 도달, 칸 8개 — 배치 실패·배치 성공·배치 실패·배치 성공 ×5. 한 픽에 칸이 둘 붙은 경우 없음 |
| 범례 | `배치 성공 6 · 배치 실패 2 · 분리 실패 0`, 회색 0 |
| 플래너·Kit 대조 | Kit `[hud] result bar n/8` 순서가 플래너 로그와 같고, Kit 브릿지 `dropped=2` 가 버스 `result.dropped` 2 와 같다 |
| 단계 순서 | 8픽 모두 경로 계산 → 접근 → 진입 → 하강 + 파지 → 당김 → 후퇴 → 배치 → 복귀 |

이 런에서는 분리 실패가 나지 않았다 — 09-18 런에서 직선 진입이 막힌 (280, 783, 700)mm 과실이 이번에는 45mm 진입에 성공해 배치까지 갔다.
그래서 호박색 칸이 실제 화면에 붙는 것은 아직 라이브로 보지 못했고, 그 경로의 근거는 §9-9 오프라인 검사다.
