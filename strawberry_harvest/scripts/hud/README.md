# 인-뷰포트 상태 HUD

Isaac Sim 뷰포트 위에 얹히는 반투명 상태 HUD. Isaac Sim 창 하나만 화면 녹화하면
로봇 동작과 시스템 상태가 한 프레임에 같이 찍힌다. 사양은 `HUD_SPEC.md`.

기존 별도 상태창(`status_monitor_node`)을 대체한다.

---

## 띄우는 법

Isaac Sim GUI → **Script Editor** → [`../isaac_sim_hud.py`](../isaac_sim_hud.py) 를 열고 Run.
브릿지 스크립트(`isaac_sim_script_editor_bridge.py`)와 같은 방식이고, 둘 다 실행해야 한다.
순서는 상관없고, 몇 번을 Run 해도 HUD 는 하나만 남는다. 내리려면 에디터에서 `uninstall()`.

**노드보다 먼저 켜도 되고 나중에 켜도 된다.** HUD 는 파일을 읽을 뿐이라
실행 순서와 무관하고, 파이프라인 도중에 껐다 켜도 즉시 다시 붙는다.

### 한글은 Kit 이 못 그린다 — 그래서 그림으로 붙인다

Isaac Sim 5.1 의 텍스트 렌더러는 한글을 `?` 로 찍는다. 스크립트 에디터에 한글 소스를
열어도 `?` 로 보이고, style 의 `"font"` 에 Noto CJK 를 넘겨도 마찬가지였다(실제로 시도).
(같은 이유로 **스크립트 에디터에서 Run 하는 두 파일은 ASCII 전용**이다 — `isaac_sim_hud.py`
와 `isaac_sim_script_editor_bridge.py`. 2026-09-10 부터 주석·문자열·print 를 전부 영문으로
두어 에디터에서 `?` 가 한 글자도 안 뜬다. 한글 설명은 이 README 와 docs/ 에 둔다.
디버깅 이력 주석은 버리지 않고 영문으로 옮겼다.)
그래서 한글 라벨은 `make_labels.py` 가 Pillow 로 2배 해상도 PNG 를 미리 그려 두고
HUD 가 `ui.Image` 로 붙인다. 숫자(`3 / 6`, `2`)는 ASCII 라 그대로 `ui.Label` 이다.
램프와 진행 바는 `ui.Circle` / `ui.Rectangle` 도형이다.

```bash
python3 ~/strawberry_grasp_environment/strawberry_harvest/scripts/hud/make_labels.py
```

`labels/` 가 없으면 영문 텍스트 라벨로 자동 전환된다. 영문으로 고정하려면 `HARVEST_HUD_LANG=en`.
라벨 문구·크기·색을 바꾸려면 `make_labels.py` 의 `ITEMS` 를 고치고 다시 실행한다.

---

## 왜 파일을 거치나

사양 2절은 계측 대상이 Isaac Sim 과 같은 프로세스에 있다고 전제하지만,
이 저장소에서 Isaac Sim 안에 있는 것은 브릿지 스크립트 하나뿐이다.
인식·제어·플래너·스캔 네 노드는 전부 별도 프로세스다. 그래서 사양 10절의
예비 경로를 쓴다 — 각 프로세스가 자기 상태를 JSON 한 파일로 0.1초마다
덮어쓰고(`os.replace`), HUD 가 읽어서 합친다.

```
 fake_vision_node ──► /tmp/harvest_hud_vision.json     ─┐
 sim_executor_bridge ► /tmp/harvest_hud_controller.json ─┤
 curobo_planner ─────► /tmp/harvest_hud_planner.json    ─┼─► isaac_sim_hud.py ─► 뷰포트
 scan_executor ──────► /tmp/harvest_hud_scan.json       ─┘
```

새 ROS 노드도 토픽도 서비스도 만들지 않는다.

---

## 파일

| 파일 | 역할 | 프로세스 |
|---|---|---|
| `../isaac_sim_hud.py` | 뷰포트 오버레이. 스크립트 에디터에서 Run. 받아서 그리기만 한다 | Isaac Sim |
| `status_bus.py` | 상태의 단일 진실 원천. 순수 파이썬 (omni·rclpy 모름) | 전부 |
| `bus_sink.py` | 자기 상태를 JSON 으로 내보내는 쓰기 측 미러 | 노드 4개 |
| `bus_merge.py` | 네 파일을 소유권 규칙대로 합치는 읽기 측 | Isaac Sim |
| `harvest_probe.py` | 노드 메서드를 밖에서 감싸는 계측 모듈 | 노드 4개 |
| `make_labels.py` → `labels/` | 한글 라벨 PNG 20장 + manifest.json 생성 (Pillow, Noto Sans CJK KR) | 오프라인 |

---|---|---|
| `status_bus.py` | 상태의 단일 진실 원천. 순수 파이썬 (omni·rclpy 모름) | 전부 |
| `bus_sink.py` | 자기 상태를 JSON 으로 내보내는 쓰기 측 미러 | 노드 4개 |
| `bus_merge.py` | 네 파일을 소유권 규칙대로 합치는 읽기 측 | Isaac Sim |
| `harvest_probe.py` | 노드 메서드를 밖에서 감싸는 계측 모듈 | 노드 4개 |

---

## 계측을 빼는 법

노드 쪽에 들어간 것은 파일당 아래 4줄뿐이다. **지우면 계측이 완전히 사라진다.**

```python
        try:    # ── HUD 계측 (제거: 이 4줄만 지우면 된다) ──
            import sys, os; sys.path.append(...)
            import harvest_probe; harvest_probe.attach("<role>", self)
        except Exception: pass
```

| 파일 | role |
|---|---|
| `src/strawberry_sim_core/strawberry_sim_core/fake_vision_node.py` | `vision` |
| `src/strawberry_sim_core/strawberry_sim_core/sim_executor_bridge_node.py` | `controller` |
| `src/strawberry_motion/scripts/curobo_planner_node.py` | `planner` |
| `src/strawberry_motion/strawberry_motion/execution/scan_executor_node.py` | `scan` |

앞의 둘과 마지막 하나는 지운 뒤 `colcon build --packages-select strawberry_sim_core strawberry_motion`
이 필요하다 (`install/` 사본에서 실행되기 때문). 플래너는 `src/` 에서 직접 돌아 빌드가 필요 없다.

일시적으로만 끄고 싶으면 빌드 없이:

```bash
HARVEST_HUD_DIR=/dev/null ros2 run strawberry_sim_core fake_vision_node
```

계측 블록은 import 실패를 삼키므로 노드는 그대로 뜬다.

---

## 화면에 나오는 것

```
노드   ● 인식   ● 플래너   ● 제어   ● 스캔      램프 (초록 = 지금 일하는 중)
──────────────────────────────────────────
영역   북서 NW                                 순회 중인 세부영역 (HOME/북서/북동/남서/남동)
타겟   3 / 6        배치   2                    카운터
──────────────────────────────────────────
단계            하강 + 파지                     현재 단계 (34px, 단계별 색, 가운데 정렬)
       ■■■■□□□□□□□                              11칸 진행 바 (스캔 이동 → 완료)
──────────────────────────────────────────
          수확 완료  5 / 6 (83%)                완료 시에만 (가운데 정렬). 괄호는 목표 대비 백분율
```

**패널 위치**는 `../isaac_sim_hud.py` 상단의 `POS_X` / `POS_Y` — 뷰포트 좌상단 (0,0) 기준
픽셀이고 기본값은 24/24 다. `PANEL_WIDTH`(440) 로 폭을, `PAD`/`ROW_GAP`/`HEAD_W` 로 안쪽
여백을 조절한다.

### "성공" 이 아니라 "배치" 인 이유

사양 4.3 의 성공 정의는 `attach 성공 AND release 완료 AND release 후 딸기 prim 이
목표 영역 안` 이다. **이 저장소에는 FixedJoint attach 코드가 없다.** 파지는
`sim_executor_bridge_node._judge_grasp()` 의 기하 판정뿐이고 딸기 prim 은
그리퍼를 따라가지 않는다.

그래서 세는 것은 **파지 판정 통과 + 트레이 슬롯에서 릴리스 실행 완료** 이고,
사양 4.3 지시대로 화면에 "성공" 이라고 쓰지 않는다. 딸기 키네마틱 부착(Stage3)이
들어오면 그때 세 조건을 다 판정하고 라벨을 바꾸면 된다.

`status_bus` 의 필드 이름은 사양 3절 계약이라 `succeeded` 그대로 둔다.

### 실패를 감추지 않는다

`타겟 3 / 6 · 배치 2` 처럼 시도와 결과를 분리해 보여준다. 스킵이나 실패가 있으면
숫자가 어긋나는 것이 정상이고, 성공 수를 시도 수에 맞춰 보정하는 코드는 없다.

---

## 상태가 이상할 때

로그는 프로세스별로 갈린다: `/tmp/harvest_hud_<role>.log`.
값이 바뀔 때만 한 줄씩 쌓이므로 완주 실패 시 마지막 30줄이 그대로 단서가 된다.

```bash
tail -n 30 /tmp/harvest_hud_*.log
```

| 증상 | 볼 곳 |
|---|---|
| 램프가 계속 빨강 | `ls -l /tmp/harvest_hud_*.json` — 파일이 없으면 그 노드에 계측이 안 붙은 것 |
| 램프가 깜빡인다 | `status_bus.NODE_TIMEOUTS` 를 늘리거나 해당 하트비트 주기를 확인 |
| 타겟 수가 0 | 노드 stderr 에 `[harvest_probe] 한 런 동안 PICK_SEQUENCE_START…` 경고가 있는지 |
| 계측 지점이 사라짐 | 노드 stderr 의 `[harvest_probe] 계측 지점 없음: …` — 메서드 이름이 바뀐 것 |

`HARVEST_HUD_DIR_RUNTIME` 로 스냅샷 디렉터리를, `HARVEST_HUD_LOG` 로 로그 경로를 바꿀 수 있다.
