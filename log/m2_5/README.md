# M2.5 런 로그 — 2026-09-07 16:44~16:52 (place/release 완주 런)

`~/.ros/log/`에 있던 원본을 노드별로 이름만 붙여 옮겼다 (내용 무편집).
그 디렉터리는 언제든 정리될 수 있으므로 포트폴리오 근거로 여기에 보존한다.

| 파일 | 원본 |
|---|---|
| `curobo_planner.log` | `python3_25263_1788767037185.log` |
| `sim_executor_bridge.log` | `python3_25122_1788767027854.log` |
| `scan_executor.log` | `python3_25374_1788767047148.log` |
| `fake_vision.log` | `python3_25121_1788767026155.log` |
| `status_monitor.log` | `python3_25473_1788767051148.log` |

## 이 런에서 확인된 것

| 항목 | 결과 | 근거 |
|---|---|---|
| M2 벽 정합 | ✅ **`clamped to 672mm` WARN 소멸** | `curobo_planner.log` 전 PICK 헤더가 `det_y=645mm` (M1 런은 `Detection Y=804mm ... clamped`) |
| 파지 판정 | ✅ 기하 기반으로 동작 | `sim_executor_bridge.log` `GRASP_JUDGE d_tcp=14.8mm ... -> CONTACT` → planner `GRASP_CONTACT_DETECTED present_pos=670` |
| 배치(place) | ✅ **taught slot0 place + release 완주** | `TAUGHT_TRAY_SLOT0_PLACE_RELEASE: position_cmd=600` ×4 |
| 연속 처리 | ✅ **익은 딸기 6개 시도 → 4개 완주** | `=== PICK COMPLETE (DETACH_SUCCESS_UNVERIFIED) ===` ×4 |
| 서브셀 분할 | ✅ 보드 격자와 일치 | `SUBCELL_SCAN_ORDER ... sw:3 se:0 nw:2 ne:1` = 씬 배치 의도와 동일 |

## 타겟별 결과

| # | 타겟 (mm) | 결과 |
|---|---|---|
| 1 | (−100, 645, 440) | ❌ `ABORT: 직선 진입 실패` — MoveLine IK near-branch 해 없음 |
| 2 | (−250, 645, 500) | ✅ 파지→place→release |
| 3 | (−220, 645, 560) | ✅ 파지→place→release |
| 4 | (−120, 645, 760) | ✅ 파지→place→release |
| 5 | (−250, 645, 880) | ✅ 파지→place→release (grasp 후보 17/20에서 성공) |
| 6 | (350, 645, 800) = `ripe_03` | ❌ `grasp 전체 실패 — 8개 후보 모두 reject` — 도달성 한계 (알려진 이슈) |

## ⚠️ 이 런의 한계 — 툴 길이 모델이 아직 160mm이던 시점

기동 로그에 `EE_TO_TCP_OFFSET_OVERRIDE` 가 **없다** → `ee_to_tcp_offset_m` 이 프로파일
기본값(160mm)인 채로 돌았다. 즉 **관통 3원인을 고치기 전 상태**의 런이다.
시퀀스·판정 로직은 전부 정상 동작했지만, 실제 파지부는 플래너 모델보다 73mm 더 뻗어
있으므로 **화면상 딸기·보드 관통이 남아 있는 런**이다.

수정 후 재실행 결과는 `log/m3/` 에 따로 보존한다. **현재 값은 `-p ee_to_tcp_offset_m:=0.236`** 이다 (아래 주의 참고).

> **재실행 시 주의 (2026-09-10 현재 값으로 갱신)**: 이 문단에 있던 208mm 는 옛 값이다.
> 툴 길이는 **236mm** 로 바뀌었다 — 실제 파지부가 플랜지 +260.8mm 까지 뻗는다는 실측 반영.
>
> - 플래너: `-p ee_to_tcp_offset_m:=0.236` 을 **명시**한다. 기본값은 `legacy_160mm` 프로파일의
>   160mm 이고, 실기 legacy 동작을 바꾸지 않으려고 그대로 뒀다.
> - 브릿지: `tool_tcp_offset_m` 을 **지정하지 않는다.** 기본값이 이미 0.236 이라 한 쌍이 맞는다.
>   옛 값(0.208)을 손으로 넣으면 두 값이 어긋나 그 차이가 파지 판정 거리에 그대로 더해진다.
> - 확인: 브릿지 기동 로그 `GRASP_JUDGE_MODEL: tool_tcp_offset=236mm`,
>   플래너 기동 로그 `EE_TO_TCP_OFFSET_OVERRIDE: 160mm -> 236mm`.
>
> 판정 방식도 바뀌었다. 이 런 시점의 `d_tcp`(TCP↔과실 중심 스칼라 거리) 대신, 지금은 툴 축으로
> 분해한 **along/lateral** 로 판정한다 (조우 물림 구간 TCP 기준 −6~+27mm, lateral 허용 20mm).
> `grasp_capture_radius_m`(0.038)은 FK 실패 시 폴백으로만 남아 있다.
> 따라서 이 README 본문의 `d_tcp` 수치는 **당시 판정 기준**이며 현재 런과 직접 비교하지 않는다.
