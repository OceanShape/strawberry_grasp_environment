# 실행 가이드 — 터미널 3개 (단일 런) / 터미널 1개 (무작위 배치 N 런)

> **담당: 실행 절차.** 터미널 명령·기동 확인·증상별 대처.
> **모든 파라미터 값의 기준은 [`parameters.md`](parameters.md) 다.**

## 복붙용 — 터미널 3개

터미널 **3개**만 쓴다. 노드 4개는 두 번째 터미널에서 **명령 하나**로 한꺼번에 뜬다.
설명·주의사항은 전부 아래에 있고, 여기는 **명령만** 있다.

**터미널 1 — Isaac Sim**

```bash
bash scripts/run_isaacsim.sh
```

→ 뜬 뒤 GUI 에서: 씬 로드 → `isaac_sim_script_editor_bridge.py` Run → `isaac_sim_viewport_display.py` Run → **Play**
([터미널 1 절](#터미널-1--isaac-sim-venv-사용))

**터미널 2 — 노드 4개**

```bash
cd ~/strawberry_grasp_environment && bash scripts/run_nodes.sh
```

→ `전부 정합` 이 뜨면 준비 완료. `!!` 가 하나라도 있으면 트리거하지 않는다. 종료는 `Ctrl+C` 한 번.

**터미널 3 — 트리거**

```bash
cd ~/strawberry_grasp_environment && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 service call /strawberry/scan/start std_srvs/srv/Trigger
```

→ 완주 후 `READY_FOR_NEXT_START` 가 뜨면 그대로 다시 실행하면 된다.

> **2026-09-10 변경.** 종전에는 노드를 세 터미널(구 T2/T3/T4)에 나눠 띄웠다.
> 로그를 사람이 눈으로 갈라 읽어야 했기 때문이다. 지금은 `run_nodes.sh` 가
> 노드별 로그 파일을 따로 남기므로(`run_logs/latest/*.log`) 나눌 이유가 없어졌다.
> **기동 순서 의존성은 원래 없었다** — 네 노드 모두 초기화 시점에 blocking wait 이
> 하나도 없고, ROS2 디스커버리가 연결을 비동기로 맺는다. 순서가 중요한 지점은
> **트리거 하나뿐**이고, 그건 터미널 3으로 분리돼 있다.

---

## 복붙용 — 무작위 배치 N 런 (T4d, 터미널 1개 + Isaac)

위 터미널 3개 방식과 **다르다.** `run_batch.sh` 가 런마다 배치 변경 → 노드 기동(`run_nodes.sh`) → 트리거 → 완주 대기 → 노드 종료 →
로그 수집까지 **전부** 한다. `run_nodes.sh` 를 따로 켜지 않는다(켜 둔 노드가 있으면 시작할 때 끈다).
Isaac 쪽은 오케스트레이터가 런마다 Stop → 씬 재로드 → 브릿지·HUD Run → Play 를 한다. 씬 로드·브릿지·HUD·Play 를 **손으로 하지 않는다**
(미리 해 둬도 첫 런에서 다시 하므로 무해하다).

**Isaac Sim**

```bash
bash scripts/run_isaacsim.sh
```

→ 뜬 뒤 Script Editor 에서 `strawberry_harvest/scripts/isaac_batch_orchestrator.py` **하나만** Run. 콘솔에 `[batch] orchestrator armed — repo /home/…/strawberry_grasp_environment` 가 뜨면 준비 완료.
Isaac 을 새로 켤 때마다 다시 Run 한다. 파일을 고친 뒤에는 Script Editor 에서 **파일을 다시 열고** Run 한다(열어 둔 탭은 옛 내용을 실행한다).
Script Editor 에서 한글이 `?` 로 보이는 것은 Kit 폰트 표시 문제라 실행과 무관하다.

**터미널 — 배치 실행** (파일럿 5 런, 약 35분)

```bash
cd ~/strawberry_grasp_environment && bash scripts/run_batch.sh 5 --tag pilot
```

→ 끝날 때까지 Isaac·터미널을 건드리지 않는다. 첫 런에서 Isaac 이 스스로 씬을 다시 열고 콘솔에 `[batch] run 1 ready` 가 뜨는지만 본다
(재로드에서 멈추면 5분 뒤 중단된다). 끝나면 시연 배치를 파일에 되돌리므로 **T5 녹화 전에는 씬을 재로드**한다.
`오케스트레이터 하트비트가 N초 전 — 꺼져 있다` 가 뜨면 오케스트레이터가 안 떠 있는 것이다(대기 중 2초마다 `/tmp/harvest_batch/isaac_state.json` 을 갱신한다).

**요약 보기**

```bash
cd ~/strawberry_grasp_environment && python3 scripts/run_metrics.py --aggregate log/m3/random/pilot/runs.csv
```

→ 파일럿 5 런(서로 다른 배치 5개)이 본 데이터이고, 결과는 `log/m3/random/pilot/` + 배치별 기록표 `log/m3/random/README.md`(SUBMISSION_PLAN T4d). 자동화 실패로 끊긴 런은 같은 시드로 다시 돌린다.
중간에 멈추려면 `Ctrl+C` → `bash scripts/run_nodes.sh --kill` → `python3 strawberry_harvest/scripts/scene_tools/gen_random_layout.py --restore`.
자세한 동작은 [T4d 절](#t4d--무작위-배치-n-런-자동-실행-2026-09-15).

---

## 실행 순서

### 터미널 1 — Isaac Sim (venv 사용)

```bash
bash scripts/run_isaacsim.sh
```

`scripts/run_isaacsim.sh` 는 예전의 긴 한 줄(venv activate + ROS 환경변수 정리)에
**Kit 확장 하나**를 붙여서 띄운다 — `strawberry_harvest/kit_ext/strawberry.sim.setup`.
매번 손으로 하던 세 가지를 부팅 때 대신 해 준다:

| 부팅 때 자동 | 내용 |
|---|---|
| 뷰포트 HUD 끄기 | FPS·프레임타임·GPU/프로세스 메모리·해상도 오버레이 + **좌하단 카메라 속도 상자**(단위 `m` 만 보이던 것, 09-16 추가). **녹화본에 안 찍힌다** |
| 배경색 차콜 | 뷰포트 빈 공간을 돔 라이트의 흰색 대신 단색 차콜로 (09-16). **조명은 안 건드린다** — 렌더러가 빈 곳에 그리는 색만 바꾼다(`/rtx/background/source/*`). 색은 `extension.toml` 의 `background_color`(LINEAR 3값). **씬을 열 때마다 다시 적용한다** — 스테이지가 열리면 렌더러가 자기 배경을 되돌려 놓는데, 타입은 `color` 로 남아 기본색 (0,0,0) = 검정이 된다 |
| Script Editor 도킹 | `Render Settings` 가 있는 탭 모음에 탭으로 붙는다. Window 메뉴로 열 필요 없다 |
| Perspective 카메라 고정 | 씬을 **열 때마다** 녹화용 구도로 되돌린다 — 위치·회전에 더해 **초점 거리**(09-16, Kit 기본 광각 18.147 → 24; 조리개 20.955 고정이라 hfov 60° → 47°)까지 (세션 레이어에만 쓰므로 씬 파일은 안 더러워진다) |

값(카메라 위치·도킹 대상)은 `strawberry_harvest/kit_ext/strawberry.sim.setup/config/extension.toml`
의 `[settings]` 에 있고, 한 번만 다르게 띄우려면 인자로 덮어쓴다:

```bash
bash scripts/run_isaacsim.sh --/exts/strawberry.sim.setup/pin_persp_camera=false
```

카메라 구도를 바꾸고 싶으면 GUI 에서 원하는 각도로 맞춘 뒤 Stage 에서
`/OmniverseKit_Persp` 를 선택하고 Property 패널의 Translate / Rotate / Focal Length 값을
`persp_translate` / `persp_rotate_xyz` / `persp_focal_length` 에 옮겨 적으면 된다 (Rotate 는 XYZ 순서).
**고정이 켜져 있으면 GUI 에서 옮긴 카메라는 다음 씬 로드 때 되돌아간다** — 손으로 맞추는 동안은
`--/exts/strawberry.sim.setup/pin_persp_camera=false` 로 띄우거나, 값을 옮겨 적은 뒤 재기동한다.

**롱샷 구도 (2026-09-16 S7, 영상 조언 3번 → 2026-09-18 사용자가 뷰포트에서 다시 잡음).** 현재 핀은 보드 정면 기준
**왼쪽 30°, 위에서 20°, 보드 중심에서 3.8 m, 초점 거리 24**(35mm 환산 41mm, hfov 47°) — 보드 화면 폭 30%, 트레이 x 75\~84%,
`check_camera_framing.py` 로 UI 겹침 없음 확인. 구도를 바꾸면 씬 재로드 전에 Script Editor 에서
`strawberry_harvest/scripts/isaac_sim_save_camera.py` 를 Run 한다 — `extension.toml` 의 `persp_*` 네 줄이 지금 카메라 값으로
바뀌고, 이후 씬을 열 때마다 확장이 그 구도로 고정한다. 아래는 S7 때 정한 기준이다: 왼쪽 30°, 위에서 30°, 3.4 m. 왼쪽 후방이라 팔이 화면을 가로질러 오른쪽 트레이로 가는 동선이 보이고, 파지 때 손목이 팔
몸통과 안 겹친다(오른쪽 후방은 트레이가 카메라 앞에 와서 팔을 가린다). 종전 09-10 구도는 왼쪽 44°/27.5°/3.09 m 에 Kit 기본
렌즈(18.147 = hfov 60°, 31mm 환산)라 보드가 화면 폭 **24%** 였고 원근이 과장됐다. 지금 값은 보드 **34%**, 트레이 x 75~85%.
조언은 40% 였지만, 좌상단 HUD 패널(x<24%, y<43%)과 좌하단 그리퍼 카메라 창(x<27%, y>60%)을 피해 피사체를 x≥28% 에 두면
34% 가 상한이다 — 더 당기면 보드 NW 가 HUD 밑으로 들어간다. 후보 비교·재계산은
`python3 strawberry_harvest/scripts/scene_tools/check_camera_framing.py` (numpy 만; 보드·트레이·로봇 지점의 화면 위치와 UI 겹침을 표로 낸다).
구도는 녹화하면서 계속 바꿀 수 있다 — 값은 `extension.toml` 한 곳이다.

**맨손으로 `isaacsim` 을 띄워도 확장은 붙는다** (2026-09-10 등록). 두 군데가 걸려 있다:

- **링크** — `~/.venv/lib/python3.11/site-packages/isaacsim/extsUser/strawberry.sim.setup`
  → 이 리포의 `strawberry_harvest/kit_ext/strawberry.sim.setup`.
  ⚠️ **리포 폴더를 옮기면 링크가 깨진다.** 옮겼으면 `ln -s` 를 다시 걸어야 한다.
- **autoload 등록** — `~/.local/share/ov/data/Kit/Isaac-Sim Full/5.1/user.config.json` 의
  `/persistent/app/exts/enabled` (ROS2 Bridge 와 같은 자리). 끄려면 `Window → Extensions`
  에서 `strawberry.sim.setup` 의 AUTOLOAD 체크를 해제한다.

`scripts/run_isaacsim.sh` 는 그 등록에 기대지 않고 인자로 직접 확장을 넘기므로,
**다른 PC나 Isaac Sim 재설치 후에도** 그대로 동작한다. 둘 중 아무거나 써도 된다.

> 확장을 아예 빼고 띄우려면 `isaacsim --/exts/strawberry.sim.setup/hide_viewport_hud=false ...`
> 처럼 개별 기능을 끄거나, 위 AUTOLOAD 체크를 해제한다.

띄운 뒤 GUI에서:
1. **ROS2 Bridge 확장 활성화** (최초 1회, AUTOLOAD 권장) — `Window → Extensions → "ROS2 Bridge"`
2. **`strawberry_harvest/scenes/main_scene.usd` 로드**
   ⚠️ 씬 파일을 수정했다면 **반드시 재로드**해야 반영된다 (노드 재시작만으론 안 됨)
3. **Script Editor**에서 `strawberry_harvest/scripts/isaac_sim_script_editor_bridge.py` 실행
4. **Script Editor**에서 `strawberry_harvest/scripts/isaac_sim_viewport_display.py` 실행 — **상태 HUD + 그리퍼 카메라 창** (09-16 까지 이름 `isaac_sim_hud.py`)
5. **Play** 클릭

> **한 번에 실행 (2026-09-18)**: 3·4번과 뷰포트 렌더 해상도 1920×1080 맞춤(`isaac_sim_fit_viewport_1080p.py`)을 `strawberry_harvest/scripts/isaac_sim_run_all.py` 하나로 Run 할 수 있다. 순서는 브릿지 → 해상도 → 뷰포트 표시. 콘솔에 `[run_all] done -- press Play` 가 뜬 뒤 Play.

**4번 뷰포트 표시** — 뷰포트 좌상단에 반투명 상태 HUD 패널, 좌하단에 그리퍼 카메라 창이 뜬다. 별도 상태창 프로그램은 폐기했고
이것이 유일한 상태 표시다. Isaac Sim 창 하나만 녹화하면 로봇 동작과 상태가 같이 찍힌다.

```
노드   ● 인식   ● 플래너   ● 제어   ● 스캔      초록 = 지금 일하는 중 (죽거나 멈추면 빨강)
타겟 8 / 비대상 4                              (09-17) 상시, 열린 씬의 딸기 prim 수
──────────────────────────────────────────
트리               [ROOT]                      쿼드트리 순회 (숫자 = 그 칸의 후보 수)
       [NW 3] [NE 2] [SE 0] [SW 3]             둘째 줄: 없음 = 잎 / 분할 / 제외
       [nw 0][ne 0][se 1][sw 2]                가장 최근에 분할한 분면의 세부 칸
──────────────────────────────────────────
단계            파지 + 하강                     단계별 색
       ■■■■□□□□□□□                              11칸 진행 바
──────────────────────────────────────────
          수확 완료  5 / 8 (63%)                완주 후에만, 흰색 (괄호 = 목표 대비 %)
■■■■■■□□                                        (09-17) 결과 바 — 타겟 수만큼 칸, 초록 배치 성공 / 빨강 배치 실패 / 호박색 분리 실패(범위 09-18 재정의), 회색 = 결과 없음
   배치 성공 5 · 배치 실패 1 · 분리 실패 0         (09-17) 범례, 런 내내·0 도 표시 (09-15 의 둘째 줄 `배치 n · 낙하 m` 대체)
```

- **트리**(2026-09-11, 영역·타겟·배치 줄 대체)는 `scan_executor` 의 순회 결정을 실행 중에 그린다. 1차 스캔에서 후보 0 인
  분면은 `제외`, 분면 자세의 후보가 분할 기준(3) 미만이면 둘째 줄 없음(잎), 이상이면 `분할` 과 함께 세부 칸 줄이 열린다
  (09-16 부터 방위 글자 없음 — 방위는 그리퍼 카메라 창 가이드가 보여 준다).
  로봇이 있는 노드·경로는 하늘색(보드 테두리와 같은 색, 09-16), 끝난 노드는 초록 테두리, 세부 자세가 거부돼 부모 자세에서 딴 세부 칸은 호박색 테두리.
  (09-19) 로봇이 분면·세부 칸에 있는 동안 로봇이 없는 노드는 흐리게 그린다. 지우지는 않고, 홈에서는 흐리지 않는다(`hud/README.md` 트리 패널 '흐림').
  (09-19) 분면 숫자는 분면 자세 첫 방문 값에서 고정된다. 픽 뒤 재스캔이 남은 수로 덮지 않고, `분할` 도 한 번 붙으면 유지된다.
  순회가 끝나면 세부 칸 줄은 접힌다. 보드 위 하늘색 테두리 하이라이트(09-16, 살구색 면 채우기에서)도 세부 칸에서 일할 때는 그 칸 하나만 켠다(`whiteboard.usd` 세부 칸 16장,
  **씬 재로드 필요** — 옛 애셋이면 부모 분면을 켠다). 실행기 코드는 안 바뀐다(프로브가 메서드 경계에서 받는다). 규칙·색·문구는 `hud/tree_model.py`, 설명은 `hud/README.md`.
- **타겟 수·결과 바** (2026-09-17, 분리 실패 범위는 2026-09-18 재정의): `타겟 N / 비대상 M` 은 열린 씬의 딸기 prim 을 브릿지 발행 필터 규칙으로 센 값이다(고정값 없음). 결과 바는 그 타겟 수만큼 칸이고,
  프로브가 실행기 반환값을 읽어 **배치 성공**(트레이 배치 실행기 `success`) / **배치 실패**(그 밖의 상태)를 순서대로 칠한다.
  **분리 실패**(2026-09-18): 직선 진입이 시작된 픽(프리어프로치 도달)이 트레이 배치 실행기를 한 번도 부르지 못하고 `run()` 이 끝나면 어디서 막혔든 분리 실패다 —
  직선 진입 실패·열린 조우 하강 실패·NW 보정 이동 실패·그리퍼 닫기 실패·파지 판정이 CONTACT 가 아니라 배치 게이트에서 차단·진입 역순 후퇴 실패를 `run()` 종료 직후 한 곳에서 판정한다
  (09-17 정의는 이 중 마지막 것 하나만 셌다 — `docs/result_display_audit.md` §9).
  용어: 상태는 `배치 실패`, 로봇이 그 자리에서 놓아 떨어뜨리는 대처 동작은 `낙하`. 분리 실패는 동작 명령이 배치 호출까지 갔는지 기준이다(물리 분리 판정 아님).
  단계 진행 바의 DETACH 라벨은 09-18부터 `당김`(옛 `분리`) — 결과 바 `분리 실패`와 글자가 겹치지 않도록.
  확인 로그: Kit `[hud] scene fruit: target 8 / non-target 4 (…; non-target constant diffuse 4/4)` — `0/4` 면 씬 재로드가 안 돼 안 익은 딸기 색 오버라이드가 빠진 것,
  칸마다 Kit `[hud] result bar n/8: …` · 플래너 로그 `[harvest_probe] 결과 바 n번째 = <키> (<라벨>, <사유>)`. **프로브를 고친 뒤에는 노드 재기동, 씬 레이어를 고친 뒤에는 씬 재로드.**
  규칙·근거·검증 `docs/result_display_audit.md` §6·§9.
- **안 익은 딸기 색** (2026-09-17): `scenes/layers/appearance_layer.usd`(생성물)가 안 익은 딸기를 연한 녹백색 #B4D69A 로 덮는다. 애셋은 그대로다.
  색을 바꾸거나 딸기 prim 이 늘거나 이름이 바뀌면 `python3 strawberry_harvest/scripts/scene_tools/gen_unripe_appearance.py` 를 다시 돌리고 씬을 재로드한다.
- **패널 위치**는 `isaac_sim_viewport_display.py` 의 `POS_X` / `POS_Y` (뷰포트 좌상단 기준 픽셀, 기본 16/32).
  09-17 에 두 행이 늘어 패널 아래 끝이 약 516\~528px 이다 — 녹화 절차대로 뷰포트 렌더 해상도를 1920×1080 으로 맞추고 디스플레이도 1080p 로 두면(`isaac_sim_fit_viewport_1080p.py`) 좌하단 그리퍼 카메라 창 위 끝(약 648px)과 약 120px 떨어진다.
- **그리퍼 카메라 창** (2026-09-16) — 뷰포트 좌하단에 로봇 손목 D455 컬러 카메라 렌더가 작게 뜬다(640×480 렌더를 480×360 으로 표시 — 09-16 S5 에서 400×300 에서 키움).
  실기 비전 노드의 카메라 창에 대응하는 화면이지만 **렌더일 뿐 인식 결과가 아니다** — 창 제목이 `그리퍼 카메라 · D455 렌더 · 인식 없음` 이다.
  실기 창처럼 **연초록 십자 + 각 분면 바깥 모서리의 NW·NE·SW·SE** 를 그리고 캡션 끝에 현재 `영역`(깊이 2 는 `NW/sw` 처럼 둘째만 소문자)을 적는다(09-16).
  십자는 2px·`#4ADE80`·70%, **스캔 구간에만 뜨고 접근·파지 중에는 0.3초 페이드로 사라진다**. 화면 고정 가이드이지 인식이 아니다 —
  오버뷰 자세에서 실행기의 실제 분면 경계(보드 중심선)가 화면 49.3%/49.3% 에 맺혀 중앙 십자와 1% 안이다(`hud/README.md` 그리퍼 카메라 창 절).
  `fake_vision` 은 이 이미지를 보지 않고 씬 정답 좌표를 보드 사각형(분면·세부 칸)으로 잘라 발행하므로, 창에 보이는 과실과 발행되는 과실이 다를 수 있다.
  시야각은 스크립트가 쓰지 않는다 — `robot_assembly.usd` 오버라이드(초점 거리 2.346 / 수평 조리개 3.896 / 수직 조리개 2.922 → 79.41°×63.83°)가 단일 출처이고,
  Run 하면 Kit 콘솔에 `[wrist_cam] ... fov 79.41 x 63.83 deg` 가 찍힌다(실기 로그 `docs/lab_data/realsense_d455_enumerate.txt` Color 640×480: 79.41°×63.89°). 어긋나면 WARNING.
  위치·크기는 `CAM_POS_X` / `CAM_MARGIN_BOTTOM` / `CAM_WIDTH`. 끄려면 Isaac 기동 전 `export HARVEST_WRIST_CAM=0`(렌더 한 번 절약).
  **씬을 로드한 뒤 Run** 해야 카메라 prim 을 찾는다 — 못 찾으면 창 없이 HUD 만 뜨고 콘솔에 안내가 찍힌다.
- 노드 4개가 `/tmp/harvest_hud_<role>.json` 에 상태를 쓰고 HUD 는 읽기만 하므로
  **터미널 2 보다 먼저 켜도, 나중에 켜도, 도중에 다시 Run 해도** 된다. 몇 번 Run 해도 하나만 남는다.
- **직전 런의 잔상** (2026-09-10 수정): HUD 는 자기가 Run 된 시각보다 먼저 멈춘 스냅샷 파일을 무시하고,
  `run_nodes.sh` 는 기동 시 `/tmp/harvest_hud_*.json` 을 지운다. 그래서 Isaac 을 다시 열거나 터미널 2 를
  다시 띄우면 트리거 전까지 **대기 상태(램프만 순서대로 초록)** 로 보인다. 완주 후 노드를 그대로 두면
  엔딩(`수확 완료 n/m`)은 남는다 — 살아 있는 노드의 파일은 계속 갱신되기 때문이다. `--kill` 은 파일을 지우지 않는다.
- **완료 줄의 수는 "성공" 이 아니다.** 딸기를 그리퍼에 붙이는 attach 가 없어(파지는 기하 판정뿐)
  *파지 판정 통과 + 트레이 슬롯 릴리스 완료* 까지만 센다.
- **낙하 (2026-09-15, T4c)**: 이송·배치 계획이 거부되면 플래너가 그 자리에서 과실을 놓는다(플래너 `PICK_SEQUENCE_CONTINUE … released fruit here`).
  이 수는 09-15\~16 에는 완료 둘째 줄 `낙하 m`, 09-17 부터는 결과 바 범례 `배치 실패 m` 으로 나간다(상태 = 배치 실패, 놓아 떨어뜨리는 대처 동작 = 낙하).
  브릿지는 그 과실을 **떨어뜨린다** — 트레이 밖 릴리스면 kinematic 을 풀고 콜라이더를 켜서 중력에 맡기고, 바닥(`lab_environment.usd` `floor`, 상판 아래 **0.45m** — 09-16 에 0.75m 에서 올렸다, 떨어진 과실이 롱샷 화면 아래로 잘려서. **씬 재로드 필요**)에
  닿아 멈춘다. Kit 로그 `RELEASE … DROPPED outside tray at … -> falls  dropped=n`, 3초 뒤 `DROP_REST … -> on floor | caught above floor (N mm up) | BELOW FLOOR (tunnelled), N mm from below the release point (dx, dy)`.
  `BELOW FLOOR` 가 나오면 안 된다(런 13 에서 20mm 바닥을 관통해 1m 로 두껍게 하고 CCD 를 켰다). `caught` 는 아래 과실에 걸린 것 — 물리 그대로, 기록만.
  **낙하 시작점 (2026-09-16 수정)**: 09-15~09-16 런에서 떨어진 과실이 전부 **그리퍼가 놓은 자리(y 739)가 아니라 보드에 매달렸던 자리(y 782.8) 아래**에 멈췄다.
  운반 중 과실은 kinematic 이라 PhysX 가 USD 에 자세를 써 주지 않고, 루트 레이어엔 보드 위치가 남아 있었다 — 낙하 코드가 세션 레이어 자세를 지우는 순간 합성 자세가
  보드로 튀고 PhysX 가 그걸 순간이동으로 받아 보드에서 떨어뜨렸다(운반 자체는 정상: 런 뒤 PhysX = USD 8/8). 이제 세션 자세를 지우기 **전에** 운반 자세를 루트 레이어에
  써 둔다(`_pin_pose_to_root`). 확인은 `DROP_REST` 끝의 `N mm from below the release point`(릴리스점 바로 아래 지점과의 **수평 XY 거리** — 브릿지 코드의 `drift`, 09-17 확인. 낙하 높이가 아니다) — 그리퍼에서 떨어지면 수십 mm 안(아래 과실에 부딪히면 더 큼),
  보드에서 떨어지면 dy 가 +44 근처다. 낙하가 있는 런 뒤엔 스테이지가 dirty 다(PhysX 가 낙하 자세를 루트 레이어에 쓴다 — 전부터 그랬다) — **Kit 에서 저장하지 않는다**.
  트레이 안 릴리스는 종전대로 동결(`RELEASE … PLACED in tray, frozen at`). 동결된 과실이 20mm 넘게 움직이면 `PLACED_FRUIT_MOVED …` 한 줄이 찍힌다(09-16, 14:38 런에서
  6번째 과실이 계란판에서 사라졌다는 목격이 있었는데 로그에 흔적이 없어 넣은 감시 — 이후 런에서 아직 재현 안 됨).
- **제원 정합 (2026-09-14)**: 브릿지 기동 로그에 `DOOSAN_MOVEIT_REF` 한 줄이 뜨고, 스캔 MoveJoint 마다 `MOVEJ_OVER_DOOSAN_MOVEIT J2 acc 162>120 J3 acc 180>150` 이 남는다.
  **정상이다** — 실기 노드가 보낸 가속도를 자르지 않고 그대로 실행하면서 두산 공식 MoveIt 설정 초과만 기록하는 것이다(`docs/e0509_spec_audit.md` D3).
  시뮬 로봇 J2·J3·J5 한계는 실기 값 ±95·±135·±135 로 좁혔다(D1, **씬 재로드 필요**). J3 는 트레이 위 자세가 정확히 135° 라 한계에 닿는다 — 도착 잔차가 커지면 여기부터 본다.
- **Kit 은 한글을 못 그린다** (스크립트 에디터조차 `?`). 그래서 한글 라벨은 `hud/labels/` 의 PNG 를 붙인다.
  그 폴더가 없으면 `python3 strawberry_harvest/scripts/hud/make_labels.py` 로 만든다 (없으면 영문 라벨로 자동 전환).
- 램프가 전부 빨강이면 `ls -l /tmp/harvest_hud_*.json` — 없는 role 은 그 노드에 계측이
  안 붙은 것(`colcon build` 후 재기동). 상세는 [`hud/README.md`](../strawberry_harvest/scripts/hud/README.md).

> 위 환경변수 3종이 없으면 ROS2 Bridge 확장이 자가진단 실패로 스스로 꺼진다
> (Extensions 창에서 토글이 되돌아오는 증상).


### 터미널 2 — 노드 4개 (한 명령)

```bash
cd ~/strawberry_grasp_environment && bash scripts/run_nodes.sh
```

이 한 줄이 `fake_vision` · `sim_executor_bridge` · `curobo_planner` · `scan_executor` 를
올바른 파라미터로 띄운다. 파라미터는 스크립트 안에 박혀 있으므로 **손으로 옮겨 적을 일이 없다**
— 종전에 파라미터 하나가 빠져 관통·전 타겟 `GRASP_EMPTY` 로 이어진 사고가 여기서 사라진다.

스크립트가 하는 일은 순서대로 이렇다.

1. **남은 노드 점검** — 하나라도 살아 있으면 **띄우지 않고 중단**한다 (아래 "왜 중요한가")
2. **`check_params.py`** — 설정 불일치면 중단한다 (`--no-check` 로 우회 가능)
3. `source /opt/ros/humble/setup.bash` + `source install/setup.bash`
4. `fake_vision` · `bridge` 기동 → 브릿지의 `cuRobo IK Solver successfully initialized`
   를 기다린 뒤 `planner` · `scan_executor` 기동
   (bridge 와 planner 가 둘 다 CUDA 를 잡으므로 초기화를 겹치지 않게 한다)
5. `cuRobo Planner Ready!` 를 기다렸다가 **기동 로그 7줄을 자동 대조**하고 결과를 찍는다

```
──────────────────────────────────────────────────────────
 기동 로그 대조
──────────────────────────────────────────────────────────
  OK   MoveLine 충돌월드 로드 (없으면 이동 중 보드 관통)
  OK   브릿지 TCP 오프셋 236mm (다르면 전 타겟 GRASP_EMPTY)
  OK   플래너 TCP 오프셋 160→236mm (없으면 툴을 짧게 보고 관통)
  OK   열린 조우 하강 단계
  OK   진입 역순 후퇴 단계
  OK   scan_executor 기동
  OK   cuRobo Planner Ready!
──────────────────────────────────────────────────────────
 전부 정합. Isaac Sim 이 Play 상태이고 로봇이 overview 자세인지 확인한 뒤,
 터미널 3 에서:

   ros2 service call /strawberry/scan/start std_srvs/srv/Trigger
──────────────────────────────────────────────────────────
```

**`!!` 가 하나라도 있으면 트리거하지 않는다.** 무엇이 빠지면 어떤 증상이 나는지는
[기동 로그 대조 항목](#기동-로그-대조-항목-2026-09-08-갱신) 표에 있다.

#### 로그 — 터미널에는 섞여 보이고, 파일로는 갈라져 남는다

터미널에는 네 노드가 `[vision]` · `[bridge]` · `[planner]` · `[scan]` 접두사를 달고 섞여 흐른다.
같은 내용이 동시에 노드별 파일로 나뉘어 저장된다.

```
run_logs/latest/vision.log
run_logs/latest/bridge.log
run_logs/latest/planner.log
run_logs/latest/scan.log
```

`run_logs/latest` 는 그 실행의 `run_logs/<타임스탬프>/` 를 가리키는 심볼릭 링크다.
**이전 실행 로그가 지워지지 않으므로** 두 런을 나란히 비교할 수 있다.

한 노드만 따로 보려면 다른 터미널에서:

```bash
tail -f ~/strawberry_grasp_environment/run_logs/latest/planner.log
```

> 로그 파일 이름에 노드 이름(`..._node`)을 **일부러 넣지 않았다.**
> 넣으면 `pkill -f curobo_planner_node` 가 로그를 쓰는 `tee` 까지 잡는다.

#### 종료 — `Ctrl+C` 한 번이면 4개 다 죽는다

`run_nodes.sh` 는 **비대화형 셸**이라 job control 이 꺼져 있고, `&` 로 띄운 노드가
스크립트와 **같은 프로세스 그룹**에 남는다. 터미널의 `Ctrl+C` 는 포그라운드 프로세스 그룹
전체에 SIGINT 를 보내므로 네 노드가 동시에 받는다. 스크립트는 그 뒤 trap 에서
`pkill` → `pkill -9` 까지 돌려 **남은 노드가 없음을 확인하고** 끝난다.

```
[run_nodes] 종료 중 — 노드 4개를 모두 정리한다.
[run_nodes] 남은 노드 없음. 로그: /home/.../run_logs/20260910_143022
[run_nodes] Isaac Sim 은 그대로다. 재실행 전 Stop → 씬 재로드 → 브릿지 Run → Play.
```

> **종전 방식(`{ A & B; }` 을 터미널에 직접 치던 것)과 다른 점.** 그건 대화형 셸이라
> job control 이 켜져 있고, `&` 로 보낸 `fake_vision_node` 가 **자기 프로세스 그룹**을 받는다.
> 포그라운드가 아니므로 `Ctrl+C` 가 안 닿아 재실행할 때마다 하나씩 쌓였다 (실제로 4개까지).
> 스크립트 방식에는 이 문제가 없다.

터미널을 잃어버렸거나 스크립트가 비정상 종료해 노드만 남았다면:

```bash
cd ~/strawberry_grasp_environment && bash scripts/run_nodes.sh --kill
```

#### ⚠️ 왜 "남은 노드 점검" 이 첫 단계인가 (2026-09-08 — 실제로 크게 당한 항목)

2026-09-08 에 **전날 16:43 에 띄운 노드 4개가 11시간째 살아 있었다.**
그중 `curobo_planner_node` 는 `ee_to_tcp_offset_m` 도 보드 장애물도 없는 **옛 설정**으로 계획하고
있었고, 구버전 `sim_executor_bridge_node` 가 모션 서비스를 제공하고 있었다. 새 노드를 아무리
올바르게 띄워도 **옛 노드가 같은 토픽·서비스에서 경쟁**해서, 코드를 다 고쳤는데도 로봇이 계속
보드를 관통했다. 원인을 찾는 데 시간이 크게 들었다.

스크립트가 이 점검을 강제하지만, 손으로 확인하려면 (아무것도 안 나와야 정상):

```bash
ps -ef | grep -E "fake_vision|sim_executor_bridge|curobo_planner|scan_executor|status_monitor" | grep -v grep
```

설정 정합도 같이 본다 (보드 y 는 6개 파일에 중복돼 있다):

```bash
cd ~/strawberry_grasp_environment && python3 check_params.py
```

`전부 정합` 이 아니면 그 줄을 고치고 시작한다. 수치 기준은 [`parameters.md`](parameters.md).

### 터미널 3 — 트리거

터미널 2 가 `전부 정합` 을 찍었고 Isaac Sim 이 **Play** 상태인지 확인한 뒤:

```bash
cd ~/strawberry_grasp_environment && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 service call /strawberry/scan/start std_srvs/srv/Trigger
```

**재실행**: 완주 후 HUD 가 `완료` 로 바뀌고 터미널 2 에 `READY_FOR_NEXT_START` 가 뜨면
이 명령을 그대로 다시 실행하면 된다.
(2026-09-07 `PLANNER-FIX #002` 이전에는 프로세스당 1회만 가능했다.)

> ⚠️ **재트리거 ≠ 새 런 (2026-09-15 런 13 교훈).** 노드를 살려 둔 채 재트리거하면 ① 플래너의 **트레이 슬롯 포인터가 이어진다**
> (런 12 가 슬롯 0·1·3·4 를 채웠으면 다음 런은 슬롯 6 부터, 시퀀스 8칸을 넘기면 슬롯 11 → `IK_FAIL`) — 실기 플래너의 정상 동작이지만
> 시뮬 측정으로는 조건이 다르다 ② **코드를 고친 뒤**라면 노드는 옛 코드다(런 13 에서는 프로브 `dropped` 가 안 세져 완료 줄 `낙하 0`, 09-17 결과 바 이후라면 칸이 회색으로 남고 범례가 0 에 머문다).
> 측정용 런과 코드 수정 뒤 런은 **`bash scripts/run_nodes.sh --kill` → 씬 재로드 → 브릿지·HUD Run → Play → `bash scripts/run_nodes.sh`** 로 새로 띄운다.

### T4d — 무작위 배치 N 런 자동 실행 (2026-09-15)

1. Isaac Sim 을 띄우고(씬은 아무 상태나), Script Editor 에서 `strawberry_harvest/scripts/isaac_batch_orchestrator.py` 를 **한 번** Run 한다.
   콘솔에 `[batch] orchestrator armed` 가 뜬다. 이후 Isaac 은 건드리지 않는다(런마다 스스로 Stop → 재로드 → 브릿지·HUD Run → Play).
2. 터미널에서:

```bash
bash scripts/run_batch.sh 5 --tag pilot
```

   런마다 `gen_random_layout.py --seed s --apply` 로 씬 파일 3곳(과실·덩굴 translate, 줄기 joint)을 바꾸고, `/tmp/harvest_batch/request.json` 으로 Isaac 에 재로드를
   요청한 뒤 `isaac_state.json` 이 `ready` 가 되면 `run_nodes.sh` 를 새로 띄워 트리거한다. 결과는 `log/m3/random/<tag>/run_<i>_seed_<s>/`
   (로그 4개, `kit_bridge.log`, `layout.json`, `metrics.json`) 과 `log/m3/random/<tag>/runs.csv`. 끝나면 시연 배치를 파일에 되돌린다
   (`gen_random_layout.py --restore` — Isaac 씬은 다음 재로드 때 반영).
3. 카운트 확인: `python3 scripts/run_metrics.py --aggregate log/m3/random/pilot/runs.csv` — 출력의 Wilson 구간은 **보고에 쓰지 않는다**(09-15 개정, SUBMISSION_PLAN T4d).
4. 런마다 `planner.log`·`scan.log`·`bridge.log` 를 읽어 `log/m3/random/README.md` 배치별 기록표를 채운다(새로 드러난 실기 노드 동작은 H §9 유형으로).

멈추고 싶으면 터미널의 `run_batch.sh` 를 Ctrl-C 한 뒤 `bash scripts/run_nodes.sh --kill`, `python3 strawberry_harvest/scripts/scene_tools/gen_random_layout.py --restore`.
`isaac_state.json` 이 `error` 면 Isaac 콘솔의 `[batch] … failed` 줄을 본다. 시연 배치(런 12~14)는 `log/m3/random/layouts/base_layout.json` 이 원본이다.

---

### 기동 로그 대조 항목 (2026-09-08 갱신)

아래 6줄은 **`run_nodes.sh` 가 기동 직후 자동으로 대조**해 `OK` / `!!` 로 찍는다.
표는 "무엇이 빠지면 어떤 증상이 나는가" 를 찾을 때 본다. 전부 **실제로 겪은 실패**에서 나온 항목이다.

| 어디 | 나와야 할 줄 | 안 보이면 |
|---|---|---|
| `bridge.log` | `MOVELINE_COLLISION_WORLD: ['whiteboard'] loaded from ...` | MoveLine 이 보드를 모른 채 푼다 → **이동 중 관통** |
| `bridge.log` | `GRASP_JUDGE_MODEL: tool_tcp_offset=236mm capture_radius=38mm` | 브릿지에 `tool_tcp_offset_m` 을 지정했거나 install 미동기화 |
| `planner.log` | `EE_TO_TCP_OFFSET_OVERRIDE: 160mm -> 236mm` | 파라미터 누락 → **툴을 짧게 보고 관통** |
| `planner.log` | `open_stem_descent=True` | 줄기 하강 단계가 통째로 생략된다 |
| `planner.log` | `straight_reverse_retreat=True` | 분리 후 **진입 역순 후퇴**(설계 5단계)가 생략된다 |
| `scan.log` | `scan_executor_node ready` | scan_executor 가 안 떴다 |
| `planner.log` | `cuRobo Planner Ready!` | MotionGen warmup 실패 — 그 위 스택트레이스를 본다 |

**⚠️ 스크립트가 볼 수 없는 줄 — 이것만은 눈으로 본다**

| 어디 | 나와야 할 줄 | 안 보이면 |
|---|---|---|
| **터미널 1 Isaac 콘솔** | `[bridge] DOF map: {'joint_1': 0, ... 'rh_l2': 8, 'rh_r2': 9}` | Script Editor 브릿지를 다시 Run 안 함 → **그리퍼가 안 움직인다** |

Isaac Sim 내부 콘솔은 별도 프로세스라 `run_logs/` 에 안 남는다.

**동작 중 확인할 줄** (터미널 2 에 접두사와 함께 흐른다)

| 줄 | 정상 | 이상하면 |
|---|---|---|
| `MoveLine ok: 45mm / 23 steps / 0.75s (worst step joint delta 1.x deg)` | worst 가 **한 자릿수** | 두 자릿수면 IK 시드가 안 먹은 것 (elbow-flip 재발) |
| `MoveSpline ok: 11 pts -> end=[...]deg` | 끝점이 **세 자리 이하** | 네 자리(수천 도)면 deg/rad 단위 오류. 2026-09-09 회귀 |
| `JOINT_COMMAND_REJECTED` | **한 줄도 없어야 한다** | 뜨면 관절 한계를 넘는 명령이 나간 것 — 단위 오류를 의심 |
| `RETURN_TO_SCAN_OK (abort)` | 중단이 나면 **반드시 한 줄** | 없으면 로봇이 보드 앞에 선 채 다음 딸기 경로를 계획한다 |
| `TRAVERSAL_SCAN_STARTED cells=[...] (4/4 quadrants)` | **4/4** | 3/4 면 분면 하나가 빠진 것 |
| `SCAN_TRANSIT ok: N pts` | 분면 이동마다 한 줄 | 없으면 관절공간 직선으로 간 것 — 보드 관통 위험 |
| `SCAN_CELL root/xx (...) -> 분면 필터 = xx` | 분면 진입마다 한 줄 | 없으면 fake_vision 이 전 분면을 계속 발행 중 |
| `BERRY_GEOMETRY: n=6 y=...mm` | 편차가 **20mm 미만** | "Isaac 씬을 다시 열 것" 이 붙으면 보드 이동이 시뮬에 반영 안 된 것 — 로봇이 딸기 앞을 집는다 |
| `ARM_ARRIVAL_TIMEOUT` | 드물어야 한다 | 자주 뜨면 스플라인 `req.time` 이 실제 이동량에 비해 너무 짧다 |
| `GRASP_JUDGE d_tcp=…mm … -> CONTACT` | d_tcp ≈ **35~38mm** | 45mm 를 넘으면 접근이 빗나간 것 |
| `OPEN_DESCENT_DYNAMIC: … -> descent=33mm` | **≈33mm** | 70mm 대면 하강량 계산이 TCP 기준이 아닌 것 |

---


## 노드별 참고사항

> **아래 파라미터는 전부 [`scripts/run_nodes.sh`](../scripts/run_nodes.sh) 안에 이미 들어 있다.**
> 이 절은 (a) 값을 바꿔야 할 때, (b) 노드 하나만 따로 띄워 디버깅할 때 본다.
> 값을 바꿨으면 **스크립트도 같이 고친다** — 안 그러면 다음 실행에서 되돌아간다.

### 노드 하나만 따로 띄울 때

`run_nodes.sh` 를 쓰지 않고 손으로 띄우는 형태다. 각 명령은 자기 터미널에서 `source` 를 스스로 한다.

```bash
# fake_vision
cd ~/strawberry_grasp_environment && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run strawberry_sim_core fake_vision_node

# sim_executor_bridge
cd ~/strawberry_grasp_environment && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run strawberry_sim_core sim_executor_bridge_node

# curobo_planner — scripts/ 에서 실행해야 한다 (flat import 구조)
cd ~/strawberry_grasp_environment && source /opt/ros/humble/setup.bash && source install/setup.bash && cd src/strawberry_motion/scripts && python3 curobo_planner_node.py --ros-args -p tool_model_profile:=legacy_160mm -p ee_to_tcp_offset_m:=0.236 -p enable_open_stem_descent:=true -p enable_straight_reverse_retreat:=true -p pick_target_z_bias_m:=0.035 -p allow_generated_tray_slot_release:=true -p enable_marker_place_sequence:=true -p use_taught_slot0_place_reference:=true -p execute_marker_place_release:=true -p hold_after_taught_slot0_place:=false -p taught_slot_sequence:="0,1,6,7,12,13" -p orthogonalize_taught_grid:=true

# scan_executor
cd ~/strawberry_grasp_environment && source /opt/ros/humble/setup.bash && source install/setup.bash && python3 -m strawberry_motion.execution.scan_executor_node --ros-args -p execute_motion:=true -p target_cell:=all
```

#### ⚠️ 손으로 띄울 때 `&&` 와 `&` 를 한 줄에 섞지 말 것

```bash
# 틀림 — source가 서브셸에 갇힌다
cd ~/x && source a && source b && ros2 run pkg node1 & ros2 run pkg node2 &
```

bash에서 `&`는 `&&`보다 결합력이 약한 **구분자**라 위 줄은 이렇게 쪼개진다.

```
[ cd && source && source && ros2 run node1 ]  &     ← 서브셸. source가 여기 갇힘
[ ros2 run node2 ]                            &     ← source 안 된 원래 셸 → command not found
```

터미널 자체도 끝까지 `source`되지 않아 이후 모든 명령이 실패한다.
게다가 대화형 셸에서 `&` 로 보낸 노드는 자기 프로세스 그룹을 받아 **`Ctrl+C` 가 안 닿는다.**
한 터미널에서 노드를 둘 이상 띄워야 하면 `run_nodes.sh` 를 쓴다 (비대화형 셸이라 이 문제가 없다).

### fake_vision_node

파라미터 없이 띄운다. 로그는 `run_logs/latest/vision.log`.

- 분면 필터가 켜져 있어야 한다 (`quadrant_filter=True`). 꺼져 있으면 첫 분면에서
  6개를 다 시도하고 나머지 분면은 후보 없음이 된다
- Isaac 에서 받은 좌표를 그대로 발행한다. 로봇이 딸기에서 빗나간 곳을 집으면
  `bridge.log` 의 `BERRY_GEOMETRY` 줄을 먼저 본다 (편차 138mm 근처 = 씬 미재로드)

### sim_executor_bridge_node

`cuRobo IK Solver successfully initialized` 가 뜨면 준비 완료. 로그는 `run_logs/latest/bridge.log`.

> ⚠️ **`-p tool_tcp_offset_m:=...` 을 붙이지 말 것.** 브릿지 기본값이 이미 **0.236** 이고,
> 플래너의 `ee_to_tcp_offset_m:=0.236` 과 **한 쌍**이다. 명시하려다 옛 값(0.208)이 남으면
> 두 값이 43mm 어긋나 capture radius(35mm)를 넘겨 **딸기를 제대로 잡아도 전 타겟이
> `GRASP_EMPTY`** 로 판정된다 (2026-09-08 실제 발생). 값을 바꿀 일이 생기면 **양쪽을 함께**
> 바꾼다 — [파지 판정 튜닝](#파지-판정-튜닝-2026-09-07-신설) 참고.
>
> 기동 로그의 `GRASP_JUDGE_MODEL: tool_tcp_offset=236mm` 로 확인한다.

### curobo_planner_node

로그는 `run_logs/latest/planner.log`.
**`cuRobo Planner Ready!` 가 뜰 때까지 기다린다** (MotionGen warmup, 수십 초).
`run_nodes.sh` 는 이 줄을 기다렸다가 대조 결과를 찍으므로 직접 셀 필요는 없다.

#### ⚠️ 시뮬 전용 파라미터 2개 — 빠지면 관통한다 (Stage1+2, 2026-09-08)

| 파라미터 | 시뮬 값 | 기본값 | 빠졌을 때 |
|---|---|---|---|
| `ee_to_tcp_offset_m` | **0.236** | 0.160 (프로파일 값) | 툴을 100mm 짧게 보고 딸기·보드를 관통 |
| `enable_open_stem_descent` | **true** | false | "위로 올라가 수평 진입 → 열린 채 하강 → 닫기" 단계가 통째로 생략 |
| `pick_target_z_bias_m` | **0.035** | 0.0 | 과실 **한가운데**를 찔러 파지한다. KP1(줄기의 5mm 자석)은 과실 윗면(중심 +31.3mm) 위다 |
| `allow_generated_tray_slot_release` | **true** | false | slot 1 이상에서 `TAUGHT_TRAY_SLOT{n}_RELEASE_BLOCKED` — 트레이 위 Above 에서 멈추고 **내려놓지 않는다** |

> **`pick_target_z_bias_m` 는 왜 35mm 인가.** 탐지가 주는 좌표는 **과실 중심**인데
> KP1(줄기의 5mm 네오디뮴 자석)은 과실 윗면(중심 +31.3mm) 위에 있다. 35mm 는 윗면보다
> 3.7mm 위 = 줄기 부위다. 2026-09-08 에 `ripe_01`·`ripe_03` 을 도달 가능 위치로 옮긴 뒤
> cuRobo IK 로 재검증: **6개 모두 4/4 접근 변형 도달** (이동 전에는 25mm 이상에서 실패했다).
> ⚠️ 브릿지 파지 판정과 한 쌍이다. 판정은 2026-09-10 기준 **툴 축으로 분해한 along/lateral**
> 이다 — `jaw_capture_near_m`(-0.006) ~ `jaw_capture_far_m`(0.027) 구간에 줄기가 들어오고
> lateral 이 `grasp_lateral_tolerance_m`(0.020) 이내여야 CONTACT.
> `grasp_capture_radius_m`(**기본 0.038**)은 FK 실패 시 폴백(TCP↔과실 중심 스칼라 거리)으로만 쓴다.
> bias 를 바꾸면 이 값들도 함께 봐야 한다.

두 기본값 모두 **실기 legacy 동작을 바꾸지 않으려고** 그대로 두었다. 시뮬에서만 켠다.

**`ee_to_tcp_offset_m` 이 왜 필요한가.**
`legacy_160mm` 의 160mm 는 "플랜지 → 그리퍼 베이스" 거리라고 적혀 있지만,
[`e0509_gripper.urdf`](../src/strawberry_motion/config/curobo/e0509_gripper.urdf) 의
`gripper_attach_joint` 오리진이 `xyz="0 0 0"` 이라 **그리퍼 베이스는 플랜지와 같은 자리**다.
실제 파지부(커스텀 3D 프린팅 파츠)는 접근 개도(stroke 600)에서 **플랜지 +260.8mm** 까지 뻗는다.

| | 기본값 160mm | `:=0.236` |
|---|---|---|
| pre-approach 툴 끝 | 과실 속 35.5mm | 과실 앞면보다 앞 ✅ |
| grasp 툴 끝 | 보드 관통 | 654.6mm — 과실 중심 +9.8mm, 보드여유 10.5mm ✅ |

**`enable_open_stem_descent` 가 왜 필요한가.**
설계된 수확 시퀀스는 `줄기 위로 30mm 올라가 수평 진입 → 열린 조우로 BASE −Z 하강해 KP1 정렬
→ 닫기 → BASE −Z 40mm 당겨 분리` 다. 이 단계가 원래 `measured_tcp_260mm` 프로파일에 묶여
있었는데, **그 프로파일은 로드 자체가 안 된다** — yml 의 `ee_link: "grasp_tcp_link"` 가 URDF 에
없는 링크다. 그래서 실행에 쓰는 `legacy_160mm` 에서는 이 단계가 죽어 있었다.
툴 모델 종류와 무관한 동작이므로 별도 파라미터로 분리했다.

**적용 확인**: 기동 로그에 `EE_TO_TCP_OFFSET_OVERRIDE: 160mm -> 236mm` 와
`open_stem_descent=True` 와 `straight_reverse_retreat=True` 가 보여야 한다. 안 보이면 파라미터가 안 먹은 것이다.

> ⚠️ **브릿지의 `tool_tcp_offset_m` 도 같은 값이어야 한다** (현재 기본값 0.236).
> 어긋나면 딸기를 제대로 잡아도 파지 판정이 `GRASP_EMPTY` 로 난다.

#### `tool_model_profile` 뒤의 파라미터 = 배치(place)까지 완주시키는 스위치

place 실행 경로는 플래너에 **이미 전부 구현돼 있고 파라미터로 꺼져 있을 뿐**이다.
아래 네 개를 다 켜야 release까지 간다. 하나라도 빠지면 그 지점에서 멈춘다.

| 파라미터 | 기본값 | 빠졌을 때 증상 |
|---|---|---|
| `enable_marker_place_sequence` | False | 파지 후 복귀만 하고 place를 **시작조차 안 함** |
| `use_taught_slot0_place_reference` | False | `MARKER_PLACE_BLOCKED: tray cells JSON not found` → soft skip |
| `execute_marker_place_release` | False | 트레이 위 ABOVE까지만 가고 **안 놓음** (`MARKER_PLACE_PREVIEW_HOLD`) |
| `hold_after_taught_slot0_place` | **True** | release 후 `TAUGHT_TRAY_PLACE_COMPLETE_HOLD`로 정지, 다음 딸기로 안 넘어감 |
| `orthogonalize_taught_grid` | **False** | 배치 격자가 실기 티칭 그대로 — 사이각 84.26° 평행사변형 + 행당 z −2.5mm. 수평·직사각 계란판과 어긋나 행 0·4 과실이 x 로 ±16mm 벗어난다 (2026-09-11 신설, 시뮬은 true) |
| `taught_grid_pitch_override_m` | **0.0** | 배치 격자가 실기 티칭 피치(59.8×51.2)다 — 4차 계란판(정사각 68mm)과 어긋나 두 번째 열·행부터 과실이 옆 컵으로 간다. 시뮬 과실 애셋(y 전폭 53.8)이 실기 행 피치보다 넓어 인접 배치가 안 되므로 시뮬만 68mm (2026-09-11 신설, 시뮬은 0.068; `orthogonalize_taught_grid` 보다 우선) |
| `taught_grid_shift_y_m` | **0.0** | 배치 격자 전체가 world y 로 옮겨지지 않는다 — 계란판(중점을 테이블 중심축 y=0 에 맞춰 놓음)에서 과실이 y 로 45mm 벗어난다. 값의 출처는 `scene_tools/egg_carton_geom.GRID_SHIFT_Y_M`, 생성기가 출력하고 검증기가 대조 (2026-09-11 신설, 시뮬은 0.0452 — 3차 0.0108 은 피치 51.2 기준) |

**`taught_slot_sequence:="0,1,3,4,6,7,9,10"` — 배치 슬롯을 **인접 칸**으로 여덟 칸 진행한다. (2026-09-11 T4-3 4차 → 같은 날 익은 8/4 전환으로 8칸)**
익은 딸기가 8개다(unripe_01·03 을 위치 그대로 익은 것으로 전환, 총 12개 유지). slot 9·10 은 사전 검사 7/7.
실기 부트캠프 영상은 과실 3개를 인접 칸(slot 0·1·3)에 넣었고 닿지 않았다(사용자 제공 정보). 시뮬에서 행 이웃이 닿았던
원인은 과실 애셋의 정지 자세 y 전폭 53.8mm 가 실기 행 피치 51.2 보다 넓은 **애셋 치수 불일치**라, 시뮬 전용
`taught_grid_pitch_override_m:=0.068`(정사각) + 과실 형상을 따르는 4차 계란판으로 인접 배치를 되살렸다. 열 2·5·8(x≈384)은
여전히 `IK_FAIL` 이라 열 0·1 × 행 0~2 다. 사전 검사 `check_tray_slot_reachability.py --pitch-m 0.068 --shift-y-m 0.0452`.

(아래는 09-10 T4-1 의 행 건너뛰기 `0,1,6,7,12,13` 근거 — 4차로 폐기, 이력으로 남긴다)
09-10 13:12 런에서 `0,1,3,4,6,7`(행 인접)로 돌려 보니 **이웃 행끼리 과실이 닿았다.** 배치 정밀도 문제가 아니라
격자 자체의 문제다 — 행 피치가 51.2mm 인데 트레이에 놓인 과실의 y 전폭이 53.8mm 다 (과실은 파지 자세에서
수직축 기준 82° 돌아 x·y 전폭이 바뀐다; 장축은 그대로 수직). **산포가 0이어도 행 이웃 간격은 +0.3mm**, 즉 완벽히 실행해도 닿는다. 행을 건너뛰면 그 쌍이
사라지고 남는 열 이웃은 +15.3mm 다. 근거·수치는 `log/m3/README.md` §런 5 (A).
5행 × 열 2개(열 2·5·8·11·14 는 IK_FAIL)에서 과실 6개를 행 이웃 없이 놓는 조합은 (0,2,4행)×(0,1열) 하나뿐이라
**slot 13 은 뺄 수 없다.** slot 13 은 사전 검증 5/7 이라 **시퀀스 맨 뒤**에 둔다 — 실패해도 이미 5개가 놓인 뒤이고
`hold_on_place_failure=false` 라 그 자리에 놓고 끝낸다.

**배치가 평행사변형으로 보이는 것은 고치지 않는다.** 티칭 격자 두 축의 사이각이 84.26° 로 직교에서 5.74°
어긋나 있다 — 강체 계란판이 스스로 비직교일 수는 없으므로 이는 **실기에서 slot 0·1·3 을 손으로 티칭할 때 들어간
오차**이고, 플래너가 그 세 점으로 15칸을 만들기 때문에 행마다 누적된다. 실기 1차 출처를 시뮬에서 직교화하면
sim2real 주장이 깨진다 (`SUBMISSION_PLAN.md` §2 부수규칙 2). 자세한 분해는 `log/m3/README.md` §런 5 (B)(C).

(아래는 종전 `0,0,0,0,0,0` → 슬롯 진행으로 바꾼 1차 근거)
종전 slot 0 고정은 T2 부착 이후 **여섯 개가 한 칸에 겹치는 화면**을 만들었다 (12:02 런 해제 위치가 서로 수 mm 차이). 이것은 시뮬
결함이 아니라 **실기 노드 설정의 결함**이다 — 같은 값이면 실기도 한 칸에 떨어뜨리는데, 부트캠프 최종은 과실 1~2개만 배치해 드러나지 않았다.
격자는 플래너 상수 `TAUGHT_SLOT{0,1,3}_PLACE_REFERENCE_POSX_MM_DEG` 로 만든다 (열 = slot%3, 행 = slot//3; 열 피치 ≈ 59.7mm(-x), 행 피치
≈ 50.6mm(-y)). 열 2·5·8(x≈400, 베이스 최근접 열, `is_row2` — 15° 피치 틸트가 붙는다)은 09-10 03:11 런에서 `IK_FAIL` 이라 뺐다.
09-07 의 `J6 spline jump` 거부는 J6 운용 한계 ±225° 탓이었고 09-08 에 ±360 으로 넓혔었다. **09-14 에 ±225 로 원복했다(C4)** — 실기 원본 값이고, 넓힌 것은 시뮬을 실기보다 관대하게 만든 것이었다. 그래서 배치 이송 거부가 다시 난다(런 10 기준 배치 4/8). 실기 노드의 동작이며 고치지 않는다.
슬롯 인덱스는 **place 가 끝날 때마다** 시퀀스를 따라 진행하고, 시퀀스가 끝나면 다음 자동 슬롯(8 = 열 2)으로 간다 —
그래서 여섯 개보다 많이 배치하지 않는다. 런 전 사전 검증: `cd src/strawberry_motion/scripts && python3 check_tray_slot_reachability.py <runtime JSONL>`.

**`use_taught_slot0_place_reference`가 왜 필요한가**: 기본 marker place 경로는 ArUco 트레이
위치추정 결과 파일(`tray_cells_json`)을 요구하는데
([tray_place_executor.py:169](../src/strawberry_motion/scripts/tray_place_executor.py:169)),
**시뮬에는 그 파일이 없다.** 이 파라미터를 켜면 JSON 경로를 통째로 우회하고 실기 티칭 좌표
(`TAUGHT_SLOT0_PLACE_REFERENCE_POSX_MM_DEG`)로 직행한다
([:165](../src/strawberry_motion/scripts/tray_place_executor.py:165)).
ArUco 트레이 비전은 스프린트 범위 밖이므로 이 경로가 맞다.

> ⚠️ **트레이 prim이 씬에 없다.** 티칭 좌표(x=520, y=52, **z=66mm** — 바닥 근처)로 가서
> **허공에 놓는다.** "파지 → 이송 → release" 동작 자체는 다 보이므로 영상 소재로는 충분하지만,
> z=66mm에서 IK_FAIL이나 바닥 간섭이 날 수 있다.
>
> **막히면**: taught 경로를 포기하고 도달 가능한 **고정 자세에서 release**로 후퇴한다
> ([`PORTFOLIO_SPRINT.md`](../PORTFOLIO_SPRINT.md) M2.5 "알려진 리스크"에 명시된 후퇴 경로).
> [`PLANNER_POLICY_v2.md`](../PLANNER_POLICY_v2.md) §3.1에서 좌표 하드코딩은 허용되므로
> 상수로 박고 `PLANNER_CHANGES.md`에 한 줄 남기면 된다.

#### place 전에 로그에서 확인할 것 — `GRASP_JUDGE ... -> CONTACT`

place 게이트는 파지 결과가 `GRASP_CONTACT_DETECTED`일 때만 열린다
([harvest_result_policy.py:11](../src/strawberry_motion/scripts/harvest_result_policy.py:11)).
`GRASP_EMPTY`가 뜨면 위 파라미터를 다 켜도 place로 넘어가지 않는다.

> `allow_unverified_grasp_place:=true`는 이 상황의 우회책이 **아니다.**
> 그건 `GRASP_UNVERIFIED`만 통과시키고 `GRASP_EMPTY`는 여전히 막는다.

#### place가 실패해도 런은 계속된다 (2026-09-07 변경)

`hold_on_place_failure` **기본값 false**. place가 실패하거나 게이트에 막히면
**실패한 그 자리에서 과실을 놓고 다음 딸기로 진행**한다 (`..._CONTINUE` 로그).

이전에는 `PICK_SEQUENCE_HOLD_LATCHED`가 걸려 **플래너를 재시작할 때까지 모든 타겟이
무시**됐다. 딸기 하나의 배치 실패가 런 전체를 죽이므로 완주 영상을 못 찍는다.
실기용 fail-closed 동작이 필요하면 `-p hold_on_place_failure:=true`로 되돌린다.

> 여전히 래치가 걸리는 경우: `TAUGHT_TRAY_FULL`(15슬롯 소진),
> 직선 후퇴 실패처럼 **팔이 움직이지 못하는** 상황. 이건 계속 진행하는 게 더 위험하다.

#### 배치 없이 파지까지만 볼 때

place 관련 파라미터를 모두 빼고 `-p tool_model_profile:=legacy_160mm`만 남긴다 (M1·M2 때의 실행 형태).

### scan_executor_node

로그는 `run_logs/latest/scan.log`.

- `execute_motion:=true` 없으면 트리거가 거부된다 (fail-closed 설계, 옵트인)
- 순회 순서는 실기와 동일한 **nw -> ne -> se -> sw -> overview** 다
- 2026-09-10 부터 `run_nodes.sh` 는 `overview_prescan:=true` 로 띄운다. overview 에서 1차 스캔(`OVERVIEW_SCAN nw:2 ne:1 se:0 sw:3`)을 하고 **익은 과실 0개 분면은 건너뛴다**(`TRAVERSAL_PRUNED skip=[...]`). 4분면 전수 순회로 돌리려면 `-p overview_prescan:=false`
- 셀 간 이동은 실기와 똑같이 순수 MoveJoint 다 (`plan_scan_transit` 기본 false, 2026-09-10). 비인접 분면 직행만 overview 를 경유한다 (`TRANSIT_VIA_OVERVIEW`)
- **[T4b 보강 09-14] 깊이 2 는 보드에 가깝게**: 기동 대조 `SUBDIVIDE_IK_READY … subcell_ee_y=0.433`. 세부 자세 로그 `SUBCELL_POSE root/sw/se tier=lab_plane …`(실기 평면 433) 또는 `tier=parent_y`(부모 거리, 09-11 동작). 스캔 자세 도착마다 `<cell>=VIEWING` 이 발행되고 fake_vision 로그에 `SCAN_CELL root/sw/se (VIEWING) -> 시야 = x[…] z[…]` 가 세부 칸 경계로 찍히면 정상. `SUBDIVIDE_REJECTED … (lab_plane: …)` 는 두 단계 모두 실패.
- **[T4b] 적응 분할** (2026-09-11): `run_nodes.sh` 는 `subdivide_min_candidates:=3` 으로 띄운다. 분면 근거리 스캔 후보가 3 이상인 분면(현재 배치에서는 sw)만 `SUBDIVIDE root/sw candidates=3 >= 3 cells=[...]` 로 2×2 로 쪼개고, 후보 있는 세부 칸만 `SUBCELL_POSE root/sw/se …` → `MOVING_TO root/sw/se` → `AT_SCAN_POSE root/sw/se` → `SUBCELL_SCAN … unique=1` → pick 순으로 내려간다. 후보가 적은 분면은 `SUBDIVIDE_SKIP`(잎), 빈 세부 칸은 `SUBCELL_EMPTY`(2단 가지치기). 기동 로그 대조에 `SUBDIVIDE_IK_READY` 가 있어야 하며, 없으면 `SUBDIVIDE_DISABLED` 로 분할 없이 종전 흐름으로 돈다. 끄려면 `-p subdivide_min_candidates:=0`
- **[T2] 딸기 부착 확인** (2026-09-10): 브릿지 로그에 `GRASP_ATTACH 과실 (x, y, z)mm` / `GRASP_RELEASE`, Isaac Script Editor 콘솔(또는 Kit 로그)에
  `[bridge] ATTACH strawberry_ripe_NN  match N mm  offset …` / `[bridge] RELEASE … PLACED in tray, frozen at … harvested=N`
  (트레이 밖이면 `RELEASE … DROPPED outside tray at … -> falls  dropped=N`, 09-15) 이 픽마다 한 쌍씩 찍힌다. 기동 시 `[bridge] tray box x[…] y[…]` 한 줄이 계란판 상자다.
  `ATTACH ignored: no ripe fruit within 60 mm` 가 나오면 브릿지 좌표와 씬이 어긋난 것(씬 재로드 누락이 흔한 원인).
- **씬 로드가 끝나기 전에 브릿지 스크립트를 Run 하면** `Exception: Prim path expression ['/World/robot_assembly'] is invalid` 로
  스크립트가 중단된다 (12:02 런 Kit 로그). 뷰포트에 로봇이 보인 뒤 Run 하고, 이 예외가 났으면 그냥 다시 Run 하면 된다.
  부착 상태는 **브릿지 스크립트를 Run 할 때마다 초기화**되므로, 씬 재로드 → Run → Play 순서를 지키면 지난 런의 계란판 딸기가 남지 않는다.
- `target_cell` 필수. **원안 시퀀스(4분면 전부 수확)는 `all`** — PROJECT_GOAL.md §1-2
  - 단일 분면만 보려면 `root/nw` / `root/ne` / `root/se` / `root/sw`
  - `all` 은 `TRAVERSAL_SCAN_STARTED cells=[...] (4/4 quadrants)` 가 떠야 한다.
    3/4 로 뜨면 그 분면의 scan pose 가 YAML 에 없다는 뜻 (`TRAVERSAL_QUADRANT_MISSING`)
- `pick_timeout_sec` **기본 60초** (2026-09-07에 120 → 60으로 변경). 타임아웃이 나면
  같은 타겟을 **한 번 더** 기다리므로 최악의 경우 타겟당 120초다.
  바꾸려면 `-p pick_timeout_sec:=90` 식으로 지정한다

## 파지 판정 튜닝 (2026-09-07 신설)

가상 제어기가 **TCP와 딸기의 실제 거리**로 파지 성공/실패를 판정한다.
파지 시도마다 `run_logs/latest/bridge.log` (터미널에서는 `[bridge]`) 에 다음이 찍힌다.

```
GRASP_JUDGE d_tcp=14.8mm d_ee=174.8mm radius=35.0mm tool_offset=208.0mm -> CONTACT
```

- `d_tcp` — 파지점(TCP)에서 가장 가까운 딸기까지 거리. **이 값이 `radius` 이하면 성공**
- `d_ee` — 그리퍼 밑동(`gripper_rh_p12_rn_base`)에서의 거리. 참고용

**툴 오프셋이 실제와 다르면** `d_ee` 쪽이 오히려 작게 나온다. 그럴 때 오프셋을 고쳐 다시 띄운다.

```bash
cd ~/strawberry_grasp_environment && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run strawberry_sim_core sim_executor_bridge_node --ros-args -p tool_tcp_offset_m:=0.100 -p grasp_capture_radius_m:=0.045
```

| 파라미터 | 기본값 | 뜻 |
|---|---|---|
| `tool_tcp_offset_m` | **0.236** | 그리퍼 밑동 → 파지점 거리. **planner의 `ee_to_tcp_offset_m`과 같은 값이어야 한다** (아래 참고) |
| `grasp_capture_radius_m` | **0.045** | 이 거리 안에 딸기가 있으면 파지 성공. `pick_target_z_bias_m` 과 한 쌍 (아래 참고) |
| `grasp_judgement_enabled` | true | false면 예전처럼 무조건 성공 처리 |

#### ⚠️ 브릿지와 플래너의 툴 오프셋은 **반드시 같아야 한다**

플래너는 grasp 종점을 `딸기 − (grasp_offset + ee_to_tcp)·approach_dir` 로 잡는다
([grasp_candidate_policy.py:140](../src/strawberry_motion/scripts/grasp_candidate_policy.py:140)).
브릿지는 거기서 `tool_tcp_offset_m` 만큼 나간 점을 파지점으로 보고 딸기까지 거리를 잰다.
따라서 실제로 측정되는 값은

```
d_tcp = | grasp_offset + (planner 오프셋 − 브릿지 오프셋) |
```

두 오프셋이 같으면 `grasp_offset`(15~30mm)만 남아 capture radius 35mm 안에 들어온다.
어긋나면 그 차이가 그대로 더해진다.

| planner | 브릿지 | d_tcp (grasp_offset 15mm 기준) | 판정 |
|---|---|---|---|
| 236mm | 236mm | **약 38mm** | CONTACT ✅ 정상 (반경 45mm) |
| **236mm** | 160mm | 약 114mm | **전 타겟 EMPTY ❌** → place 게이트도 전부 차단 |
| 160mm | **236mm** | 약 62mm | ⚠️ EMPTY. 툴은 이미 보드를 지난 상태 |

> **왜 정상값이 15mm 가 아니라 38mm 인가.** `pick_target_z_bias_m=35mm` 로 TCP 가
> **줄기(과실중심 +35mm)** 를 겨냥하는데, 판정은 TCP↔**과실 중심** 거리를 잰다.
> 즉 정상 파지에서도 `sqrt(15² + 35²) ≈ 38mm` 가 나온다 (실측 33.9~36.2mm).
> 그래서 판정 반경을 35 → **45mm** 로 올렸다. 두 값은 한 쌍이니 한쪽만 바꾸지 말 것.

- 어긋난 방향에 따라 증상이 다르다. **플래너만 올리면** 전부 빈손이 되어 금방 눈에 띄지만,
  **브릿지만 올리면** 판정이 헛돈다. 어느 쪽이든 **기동 로그 두 줄을 눈으로 대조**한다.
- 재시도 사다리(`GRASP_RETRY_OFFSETS = [15, 30, 40, 50, 70]mm`)는 구제해주지 못한다.
  오프셋이 커질수록 `d_tcp` 도 같이 커져 후보를 다 써도 계속 EMPTY 다.

플래너를 `-p ee_to_tcp_offset_m:=0.236` 으로 띄우므로 브릿지 기본값도 **0.236** 으로 맞춰 두었다
(2026-09-08 갱신, 종전 0.208). **브릿지에서 이 값을 다시 지정하지 말 것** — 옛 값이 남으면
43mm 어긋나 전 타겟이 EMPTY 가 된다. 브릿지 기동 시 다음 줄로 확인한다.

```
[WARN] GRASP_JUDGE_MODEL: tool_tcp_offset=236mm capture_radius=38mm enabled=True sim_speed_scale=1.0 gripper_close=1.080rad — planner 의 ee_to_tcp_offset_m 과 반드시 같아야 한다 (다르면 전 타겟 GRASP_EMPTY)
```

---

## 전체 종료 / 재실행 전 초기화

**`run_nodes.sh` 로 띄웠으면 터미널 2 에서 `Ctrl+C` 한 번이면 끝이다** — 스크립트가 trap 에서
`pkill` → `pkill -9` 까지 돌리고 남은 노드가 없음을 확인해준다. 터미널을 잃어버렸거나 스크립트가
비정상 종료했다면 `bash scripts/run_nodes.sh --kill` 이 같은 일을 한다.

아래는 그 스크립트가 실제로 하는 일이자, **손으로 띄웠을 때** 필요한 절차다.
손으로 띄운 경우 `&`로 백그라운드에 보낸 노드는 `Ctrl+C`가 안 닿아 살아남는다.
남은 노드가 있으면 새로 띄운 노드와 같은 토픽에 동시에 발행해 원인 불명의 오동작이 난다.

### 1단계 — ROS 노드 전부 종료

```bash
pkill -f fake_vision_node; pkill -f sim_executor_bridge_node; pkill -f curobo_planner_node; pkill -f scan_executor_node; pkill -f status_monitor_node
```

- `ros2 run` 래퍼(`ros2 run strawberry_sim_core fake_vision_node`)와 실제 노드 프로세스
  (`install/.../fake_vision_node`)가 **둘 다** 패턴에 걸리므로 한 번에 정리된다
- `python3 curobo_planner_node.py`, `python3 -m ...scan_executor_node`처럼
  실행 형태가 달라도 `-f`가 명령줄 전체를 보므로 똑같이 잡힌다
- 안 떠 있는 노드는 그냥 넘어간다 (종료코드 1은 무시해도 된다)

### 2단계 — 실제로 죽었는지 확인 (건너뛰지 말 것)

```bash
ps -ef | grep -E "fake_vision_node|sim_executor_bridge_node|curobo_planner_node|scan_executor_node|status_monitor_node" | grep -v grep
```

**출력이 비어 있어야 한다.** 뭔가 남아 있으면 SIGKILL로 다시.

```bash
pkill -9 -f fake_vision_node; pkill -9 -f sim_executor_bridge_node; pkill -9 -f curobo_planner_node; pkill -9 -f scan_executor_node; pkill -9 -f status_monitor_node
```

> planner는 MotionGen warmup(CUDA 연산) 중에 SIGTERM을 늦게 받는다.
> `cuRobo Planner Ready!` 전에 끄면 몇 초 버티거나 아예 안 죽으므로 이때 `-9`가 필요하다.

프로세스는 다 죽었는데 `ros2 node list`에 유령 노드가 남으면 디스커버리 캐시 문제다.

```bash
ros2 daemon stop
```

### 3단계 — Isaac Sim: 끄지 말고 씬만 재로드

Isaac Sim은 **켜 둔 채로 두는 편이 낫다** (재기동 + 씬 로드 + Script Editor 재실행에 수십 초).
대신 재실행 전에 반드시 아래를 한다.

1. **Stop** (Play 해제)
2. `strawberry_harvest/scenes/main_scene.usd` **재로드** — 로봇이 overview 자세로 돌아가고
   딸기 위치가 초기화된다
3. Script Editor에서 `isaac_sim_script_editor_bridge.py` **다시 Run**
4. **Play**

> **3번을 왜 다시 하나**: 스크립트가 `Articulation(prim_path="/World/robot_assembly")` 핸들과
> physics step 콜백을 잡아 두는데, 씬을 재로드하면 그 핸들이 죽은 prim을 가리킨다.
> 스크립트에 중복 실행 방지가 들어 있어 여러 번 Run 해도 안전하다
> ([isaac_sim_script_editor_bridge.py:44](../strawberry_harvest/scripts/isaac_sim_script_editor_bridge.py:44)).

> **재로드를 건너뛰면**: 노드를 중간에 죽였을 때 로봇이 아무 자세에나 서 있으므로
> `START_REJECTED ... overview pose`가 난다.

> **Isaac Sim 안의 `isaac_sim_bridge_node`는 pkill 대상이 아니다.** 그 노드는 Isaac Sim
> 프로세스 내부에서 도는 스레드라 위 `pkill`로 안 죽고, 죽일 필요도 없다.

Isaac Sim까지 정말 끝낼 때만:

```bash
pkill -f isaacsim
```

### 정리 후 재실행

터미널 2 에서 `bash scripts/run_nodes.sh` 를 다시 실행하고, `전부 정합` 을 확인한 뒤 터미널 3 에서 트리거한다.
Isaac 쪽(터미널 1 브릿지·HUD)은 그대로 둬도 된다.
플래너 코드를 고쳤다면 그 전에 `colcon build` (바로 아래 절).

## 플래너 코드를 고친 뒤에는 반드시 빌드

`scan_executor_node`는 `python3 -m`으로 실행되어 `src/`가 아니라 **`build/`·`install/` 사본**에서
로드된다. `src/`만 고치고 빌드하지 않으면 **예전 코드가 계속 돈다.**

```bash
cd ~/strawberry_grasp_environment && source /opt/ros/humble/setup.bash && colcon build --packages-select strawberry_motion
```

정합 확인 (`감사 대상 == 실행 대상 정합` 항목을 볼 것):

```bash
cd ~/strawberry_grasp_environment && bash scripts/check_planner.sh
```

---

## 자주 겪는 문제

| 증상 | 원인 / 조치 |
|---|---|
| `ros2: command not found` | 그 터미널에 `source /opt/ros/humble/setup.bash`가 안 됨 |
| `ModuleNotFoundError: No module named 'rclpy'` | 같음 — ROS setup 미source |
| `ModuleNotFoundError: No module named 'strawberry_motion'` | `source install/setup.bash` 누락 |
| `START_REJECTED scan already started` | `PLANNER-FIX #002` 이전 코드가 돌고 있다. `colcon build` 후 터미널 2 재시작 |
| `START_REJECTED ... execute_motion` | scan_executor 인자에서 `-p execute_motion:=true` 누락 |
| `START_REJECTED ... target_cell` | scan_executor 인자에서 `-p target_cell:=all` 누락 (원안 시퀀스는 `all`) |
| `START_REJECTED ... overview pose` | 로봇이 overview 자세가 아니다. 씬 재로드 또는 이전 실행 미완료 |
| 딸기가 예전 위치에 있다 | Isaac Sim에서 `main_scene.usd`를 재로드하지 않았다 |
| 명령이 두 번 실행되는 듯 / 토픽이 겹친다 | 이전 실행의 노드가 살아 있다. "전체 종료 / 재실행 전 초기화" 1~2단계 수행 |
| `Ctrl+C` 했는데 노드가 남아 있다 | 손으로 띄운 경우다 (`&`로 보낸 노드에는 `Ctrl+C`가 안 닿는다). `bash scripts/run_nodes.sh --kill` 로 정리 |
| HUD 가 안 보인다 | 터미널 1 의 4번 `isaac_sim_viewport_display.py` 를 Run 안 했거나 활성 뷰포트가 없다. Run 하면 에디터에 오류가 그대로 찍힌다 |
| 그리퍼 카메라 창이 안 뜬다 | Kit 콘솔의 `[wrist_cam]` 줄을 본다. `camera prim not found` = 씬 로드 전에 Run 했다(로드 후 다시 Run). `inset failed` = 위젯 생성 오류(HUD 는 계속 뜬다). `HARVEST_WRIST_CAM=0` 으로 기동했으면 `inset skipped` |
| 그리퍼 카메라 창 시야각이 이상하다 | 콘솔 `[wrist_cam] ... fov` 값이 79.41 x 63.83 이 아니면 WARNING 이 같이 찍힌다. 고칠 곳은 `robot_assembly.usd` 의 `Camera_OmniVision_OV9782_Color` 초점 거리·조리개(스크립트 아님) |
| HUD 한글이 `?` 로 나온다 | `hud/labels/` PNG 없음. `python3 strawberry_harvest/scripts/hud/make_labels.py` |
| HUD 램프가 전부 빨강 | `ls -l /tmp/harvest_hud_*.json` — 파일이 없으면 그 노드에 계측이 안 붙었다. `colcon build` 후 재기동 |
| **그리퍼가 아예 안 움직인다** | Isaac Script Editor 브릿지를 다시 Run 안 했다. 콘솔에 `[bridge] DOF map:` 이 찍히는지 확인 |
| **꽉 닫으면 파츠가 가위처럼 겹친다** | 그리퍼 배율이 1.0 이 아니다. `robot.urdf` 의 `rh_r2`/`rh_l2` 상한이 1.0 rad 이라 평행사변형 상한이 1.0 이다. 1.101 로 올리면 l2/r2 만 잘려 손가락이 기운다 |
| **접근 중 팔이 뒤집히며 MoveLine 실패** | `MoveLine ok:` 의 `worst step joint delta` 가 두 자릿수면 IK 시드가 안 먹은 것. `solve_single` 에 `seed_config` 를 넘겨야 한다 (2번째 위치인자는 `retract_config` 라 시드가 아니다) |
| **기동 직후 팔이 몇 바퀴 돌며 서로 박고, 이후 시퀀스가 통째로 안 돈다** | `move_spline_cb` 의 deg/rad 혼용. `req.pos` 는 **deg**, `current_joints` 는 **rad** 인데 섞어 보간한 뒤 `math.degrees` 를 또 입혀 **57.3배** 명령이 나갔다 (스캔 포즈에서 f=1 발행값 5156/-5156/6875/9740/-1719/5443 deg). `move_spline_cb` 는 항상 `success=True` 라 로그에 흔적이 없었다. 2026-09-09 수정 — 전 계산을 deg 로 통일 + 한계 초과 명령 발행 차단 |
| **배치 후 J6 가 갸우뚱한 채로 다음 딸기로 간다** | 브릿지 `_publish_gripper_only` 가 **측정값을 명령으로 되실어** 진행 중인 복귀 모션을 그 자리에서 얼렸다. 복귀 스플라인 29ms 뒤의 `SetPosition(600)` 이 J6 를 93.4 가 아닌 124.7 에 고정. 노드는 그 자세를 다음 pick 시작 자세로 저장하므로 사이클마다 누적됐다. 2026-09-09 수정 — 마지막 **명령값** 재발행 |
| **`root/se` 에서 `TARGET_NOT_FOUND`** | **정상이다.** 씬의 익은 딸기 6개는 sw3 / nw2 / ne1 / **se0** 으로 배치돼 있다. se 에는 익은 딸기를 두지 않았다 (병든 딸기 분기는 제출 범위 밖 — [`portfolio/H_scope_decisions.md`](../portfolio/H_scope_decisions.md) §2) |
| **스캔 자세에서 그리퍼가 딸기에 닿아 있다 / 파지 중 보드에 박힌다** | 스캔 자세가 보드에 너무 가깝다. 정상값은 **팁-보드 97~156mm**(실기 v12 티칭). 56mm 짜리 자세가 들어가 있으면 그것이 원인이다. `scan_pose_candidates_refit_candidate.yaml` 의 `version: v12_gripper_centered_manual_teach` 확인 |
| **보드 거리가 옛 값(672mm)으로 보인다** | 2026-09-09 에 **810.0mm** 로 복원했다(원본 씬 실측값). 다섯 곳이 한 세트다 — `layout_layer.usd`(보드 0.81 / guard 0.91 / 딸기 0.7828), `physics_layer.usd` `localPos0`, `harvest_motion_params.py` `WALL_SURFACE_Y_M=0.810`, `config/environment.yaml` x2, `scan_collision_world.yaml`. 하나라도 어긋나면 파지 목표가 클램프되거나 관통한다 |
| **로봇이 딸기에서 −y 로 빗나간 곳을 집는다** | 발행 좌표가 옛 위치다. `bridge.log` 의 `BERRY_GEOMETRY` 줄을 본다. 편차가 138mm 근처면 **Isaac 씬을 다시 열지 않은 것** |
| **파지점보다 15mm 앞에서 조우가 닫힌다** | `GRASP_RETRY_OFFSETS` 첫 값이 0.015 라 TCP 가 목표점에서 15mm 물러난 자리에 섰다. 2026-09-09 에 사다리 맨 앞에 **0.0** 을 추가 — 정상 파지의 `d_tcp` 가 38mm -> **35mm**(z bias 만) 가 되면 제대로 붙은 것 |
| **조우가 덜 닫혀 줄기를 못 문다** | 2026-09-10 수정. 닫힘을 1.0 → **1.08 rad** 로 올렸다 (파츠 간격 9.4mm → **0.3mm**). USD `rh_r2`/`rh_l2` 상한 override 와 한 쌍이다 — 한쪽만 되돌리면 가위 모양이 된다. 기동 로그의 `gripper_close=1.080rad` 확인 |
| **두 번째 딸기부터 파지 실패 / 파지점 앞에서 닫힘** | `_register_neighbor_obstacles` 에 **bias 붙은 좌표**를 넘겨 목표 딸기가 자기 자신을 반경 30mm 장애물로 등록했다 (거리가 정확히 35mm 라 부동소수점으로 갈림 — 6개 중 5개 발생). 2026-09-10 수정. `planner.log` 의 `self-excluded 1 within 50mm of target` 확인 |
| **특정 딸기만 파지점 앞에서 닫힘 (sw 는 정상, nw/ne 만)** | **도착 지연**이다. 스캔 자세→pre-approach 스윙이 sw 40~63° / ne 108° / nw 196° 인데, 스플라인 실행 시간이 `req.time/속도배율` 뿐이라 큰 스윙을 못 따라갔다. 잔차가 다음 **상대** MoveLine 에 전파된다. 2026-09-10 수정 — 관절속도 120°/s 로 시간 하한, 도착 대기 8초. `bridge.log` 의 `ARM_ARRIVAL_TIMEOUT` 줄 확인 (손끝오차 mm 가 찍힌다) |
| **앞에서 잡았는데 CONTACT 로 뜬다** | `d_tcp` 로 판정하면 이렇게 된다 (깊이에 둔감). 2026-09-10 부터 **along/lateral** 기준이다 |
| **`along` 이 +10mm 를 넘는다** | 그만큼 조우가 딸기 **앞**에서 닫힌 것이다. 같은 런의 `MOVELINE_SHORT` / `ARM_ARRIVAL_TIMEOUT` 줄과 대조하면 어느 단계에서 밀렸는지 나온다 |
| **`GRASP_POSE_REACHED offset=+0.040m variant=-5.0°`** | 높은 딸기에서 −5° 변형의 15~25mm 가 도달 한계 밖이라 40mm 로 밀린 것. 사다리가 `[15,20,25]` 면 −5° 가 전부 실패해 0° 로 넘어가 15mm 가 된다. 40mm 가 보이면 사다리가 옛 값으로 돌아간 것 |
| **분리 후 바로 배치로 넘어간다 (역순 후퇴 없음)** | 설계 5단계 누락. `planner.log` 에 `DETACH_PULL_DOWN` 다음 `RETREAT` 줄이 있어야 한다. 없으면 `-p enable_straight_reverse_retreat:=true` 누락 — 실기 코드는 `measured_tcp` 프로파일에만 걸려 있어 legacy 는 기본적으로 생략된다 |
| **분리까지 성공했는데 배치 안 하고 놔버린다** | `TAUGHT_TRAY_SLOT0_PLACE_BLOCKED: above plan failed` 확인. J2 가드 100°가 nw 딸기(114°, 113°)를 막았다. 2026-09-10 에 **130°** 로 |
| **뷰포트 배경이 검정이다** | 배경 타입은 `color` 로 남았는데 색이 기본값 (0,0,0) 으로 되돌아간 것이다. Kit 로그의 `[strawberry.sim.setup] background type=… color=…` 줄을 본다 — `did not stick` WARN 이면 그 뒤에 누가 덮어썼다는 뜻. 09-16 에 **씬 열 때마다 재적용**하도록 고쳤으니 다음 기동부터는 자동이고, 이번 세션에서는 Script Editor 새 탭에 아래를 붙여 Run 하면 즉시 돌아온다(ASCII 만 — Script Editor 는 한글을 `?` 로 찍는다):<br>`import carb.settings`<br>`s = carb.settings.get_settings()`<br>`s.set("/rtx/background/source/type", 2)`<br>`s.set("/rtx/background/source/color", [0.027, 0.033, 0.042])` |
| **보드 위 하늘색 테두리(분면 표시)가 안 뜬다 / 안 바뀐다** | HUD 스크립트가 켜고 끈다 — Isaac 콘솔에 `[hud] 보드 하이라이트 prim 을 못 찾았다` 가 있으면 씬을 다시 로드하지 않은 것(`whiteboard.usd` 의 `highlight` 가 2026-09-10 추가). 영역 값 자체가 안 바뀌면 HUD 패널의 "영역" 도 같이 멈춰 있을 것 — scan_executor 계측 문제 |
| **동작이 너무 느리다 / 빠르다** | 브릿지 `-p sim_speed_scale:=1.0`(기본, 09-14 부터 — 실기 요청 시간 그대로. 2.0 은 검증용으로 쓰지 않는다). 3.0 이면 더 빠르고 1.0 이면 2026-09-09 이전 속도다. 스캔 이동은 scan_executor `-p scan_movej_vel_deg_s:=120`(기본, 실기는 60) |
| **HUD `타겟` 총수가 한 번에 6 이 되지 않는다** | **정상이다.** 2026-09-09 분면 필터 이후 이 값은 **지금 스캔 중인 분면**의 개수다. `Isaac→fake` 쪽이 전체(6)다 |
| **첫 분면에서 6개를 다 시도하고 나머지 분면은 후보 없음** | 분면 필터가 꺼져 있다. `fake_vision_node` 의 `quadrant_filter_enabled` 확인. 기동 로그의 `quadrant_filter=True` 도 같이 본다 |
| **파지/배치 실패 후 scan pose 로 안 돌아오고 바로 다음 딸기로 이동** | `_abort_pick_with_complete` 가 로봇을 두고 `pick_complete` 만 발행했다. scan_executor 는 그 즉시 다음 타겟을 쏜다. 쿼드트리 스캔은 **모든 pick 이 세부영역 scan pose 에서 시작**하는 것을 전제로 보드와의 y 여유를 확보한다. 2026-09-09 수정 — 중단 경로도 복귀 후 발행 |
| **파지는 성공했는데 보드 앞에서 놓는다** | `spline_jump J6` 또는 `swing J3/J6` 거부. **09-14 부터 정상 동작이다** — 운용 한계 J6 ±225(실기 원본)에서 트레이 이송 일부가 거부되고 과실을 그 자리에서 놓는다. 고치지 않는다(`docs/e0509_spec_audit.md` §7, H §9). 런당 몇 건인지는 기록한다 |
| **배치 경로가 보드를 스친다** | 플래너가 궤적을 12점으로 다운샘플해 보낸다. 브릿지 `move_spline_cb` 가 그 사이를 보간하는지 확인 |
| **이동 중 그리퍼가 보드를 통과한다** | 브릿지 기동 로그에 `MOVELINE_COLLISION_WORLD:` 가 없다. IK 솔버가 충돌 월드 없이 생성된 것 |
| **딸기를 제대로 집었는데 빈손 판정** | `d_tcp` 와 `capture_radius` 를 대조. `pick_target_z_bias_m` 을 바꿨으면 `grasp_capture_radius_m` 도 같이 봐야 한다 |
| 파지는 되는데 **place를 아예 안 함** | 플래너 인자에서 `-p enable_marker_place_sequence:=true` 누락 |
| `MARKER_PLACE_BLOCKED: tray cells JSON not found` | 플래너 인자에서 `-p use_taught_slot0_place_reference:=true` 누락 (시뮬엔 ArUco 트레이 JSON이 없다) |
| `MARKER_PLACE_PREVIEW_HOLD` — 트레이 위에서 멈춤 | 플래너 인자에서 `-p execute_marker_place_release:=true` 누락 |
| `TAUGHT_TRAY_PLACE_COMPLETE_HOLD` — 놓고 나서 정지 | 플래너 인자에서 `-p hold_after_taught_slot0_place:=false` 누락 |
| place 게이트에서 `GRASP_EMPTY`로 차단 | 파지가 실제로 실패한 것. `GRASP_JUDGE` 로그의 `d_tcp`를 보고 "파지 판정 튜닝" 절대로 조정. `allow_unverified_grasp_place`로는 안 뚫린다 |
| place 도중 `IK_FAIL` / 바닥 간섭 | 티칭 슬롯(z=66mm)이 시뮬 좌표계에서 도달 불가. 고정 자세 release로 후퇴 (`curobo_planner_node` 절 참조) |
| `Cartesian plan rejected: J6 spline jump ...deg` | J6 랩 불연속. 운용 한계 ±225(09-14 원복)에서 배치 이송·파지 접근 모두에서 날 수 있다. 실기 노드의 거부이며 기록만 한다 |
| `PICK_SEQUENCE_HOLD_LATCHED` | 2026-09-07 이전 코드가 돌고 있다. `colcon build` 후 터미널 2 재시작 (지금은 place 실패 시 놓고 계속 진행한다) |
| `Pick target ignored: sequence hold` | 위와 같음. 래치가 걸리면 **플래너 재시작 외에는 풀 방법이 없다** |
