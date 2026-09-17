"""
result_bar.py — HUD 결과 바의 판정 규칙·문구·색. 순수 파이썬 (omni·rclpy import 금지).

tree_model.py 와 같은 구성이다 (2026-09-17).

  1. 판정   : 실행기 메서드 경계에서 받은 값 -> 결과 키. harvest_probe 가 planner 프로세스에서 부르고,
              결과는 status_bus 'result.outcomes' 목록에 시도 순서대로 쌓인다.
  2. 문구·색 : 범례 라벨(make_labels.py 가 PNG 로 굽는다)과 칸 색(HUD 가 칠한다)의 단일 출처.
  3. view   : outcomes 목록 + 칸 수 -> 칸별 결과 키와 범례 숫자.

HUD(../isaac_sim_viewport_display.py)는 3 을 받아 칠하기만 한다. 칸 수는 HUD 가 씬에서 센 타겟 수다
(scene_fruit.py) — 여기에 개수를 적지 않는다.

결과 키 세 가지 (사용자 지정 2026-09-17 — 상태는 '배치 실패', 로봇의 대처 동작은 '낙하', 분리 단계 실패는 '분리 실패')

  placed        배치 성공  트레이 배치 실행기(execute_marker_place_after_retreat)가 "success" 를 돌려줬다.
                           status_bus 의 result.succeeded 와 같은 경계·같은 기준이다.
  place_failed  배치 실패  같은 실행기가 "success" 가 아닌 상태를 돌려줬다 — failed(이송·하강 계획/실행 실패),
                           failed_after_release(슬롯에 놓은 뒤 상승 실패), skip(트레이 없음),
                           preview_hold, tray_complete(슬롯 소진). 분리까지 끝난 과실이 트레이 배치를 끝내지 못한
                           **상태**다. 그 뒤 그 자리에서 놓아 떨어뜨리는 것(낙하)은 대처 동작이라 따로 세지 않는다.
  detach_failed 분리 실패  파지 판정이 GRASP_CONTACT_DETECTED 인 픽에서 분리 단계 함수(execute_detach_and_retreat:
                           아래로 당겨 떼기 + 진입 역순 후퇴)가 끝까지 못 가고 None 을 돌려줬다 — 실행기가 후퇴 실패로
                           시퀀스를 잡아 둔 경우다. 빈손 판정 픽의 후퇴 실패는 떼어낼 과실이 없어 세지 않는다(회색).

어느 키에도 안 들어가는 시도는 칸이 회색으로 남는다: 파지 후보 전부 IK 실패로 건너뜀(ABORT), 직선 진입 실패,
파지 판정 실패(GRASP_EMPTY 등 -> PLACE_GATE_BLOCKED), 그리퍼 닫기 실패. 칸 수가 타겟 수이므로 끝났을 때
회색 칸 수 = 결과가 나지 않은 타겟 수다.

분리 실패의 한계 (log/m3 조사 2026-09-17): 이 판정은 **동작 명령**이 끝까지 갔는지다. 줄기에서 실제로
떨어졌는지는 실기에 센서가 없고(실행기 verify_detach 가 상수 DETACH_UNVERIFIED) 시뮬은 ATTACH 때 줄기
조인트를 꺼 두므로 어디서도 판정할 수 없다. 당김 MoveLine 하나만 실패한 경우는 실행기가 무시하고
후퇴·배치를 계속하므로(pick_sequence_executor.py 의 '실패해도 retreat은 항상 실행') 결과는 배치 쪽에서
정해진다 — 여기서도 분리 실패로 세지 않는다. 보존 런(log/m3) 에서 분리 단계 실패는 0건이었다.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Sequence

#: 결과 키 — 범례 순서이기도 하다.
OUTCOMES = ("placed", "place_failed", "detach_failed")

#: 범례 문구 (make_labels.py 가 PNG 로 굽는다). HUD 표기 = 문서·자막 표기.
LABEL_KO = {"placed": "배치 성공", "place_failed": "배치 실패", "detach_failed": "분리 실패"}
#: labels/ 가 없을 때(영문 폴백) 쓰는 글자.
LABEL_EN = {"placed": "PLACED", "place_failed": "PLACE FAILED", "detach_failed": "DETACH FAILED"}

#: 칸·범례 색 (0-255 RGBA). 초록·빨강은 HUD 의 C_OK·C_BAD 와 같은 값이다 — 빨강은 실패 전용.
#: 분리 실패는 배치 실패와 구분돼야 해서 호박색. 하늘색(#81B8DC)은 '로봇이 있는 곳' 이라 쓰지 않는다.
COLOR = {
    "placed": (0x5A, 0xD4, 0x69, 255),
    "place_failed": (0xFF, 0x4D, 0x5E, 255),
    "detach_failed": (0xF5, 0xA5, 0x24, 255),
}

#: 범례 글자 크기(px). 22px 이면 세 항목이 두 자리 숫자까지 패널 안쪽 폭 408 에 들어간다
#: (Noto Sans CJK 실측: '배치 실패' 22px = 89px). 24px 은 두 자리에서 넘친다.
LEGEND_SIZE = 22

#: 칸 높이·간격·모서리 — 단계 진행 바(isaac_sim_viewport_display.HarvestHUD._row_bar)와 같은 값.
CELL_H = 10
CELL_GAP = 3
CELL_RADIUS = 2


def outcome_of_place_status(status) -> Optional[str]:
    """execute_marker_place_after_retreat 의 첫 반환값 -> 결과 키. 문자열이 아니면 None(판정 안 함)."""
    if not isinstance(status, str):
        return None
    return "placed" if status == "success" else "place_failed"


def outcome_of_detach(result) -> Optional[str]:
    """execute_detach_and_retreat 의 반환값 -> 결과 키. None 이면 분리 실패, 아니면 판정 안 함."""
    return "detach_failed" if result is None else None


def view(outcomes: Sequence[str], n_cells: int) -> Dict:
    """outcomes(시도 순서) + 칸 수 -> {"cells": [키 또는 None] * n, "counts": {키: 수}, "overflow": 넘친 수}.

    counts 는 칸 수와 무관하게 outcomes 전체를 센다 — 범례 숫자가 칸 수에 잘리지 않게.
    overflow > 0 이면 결과가 칸 수(씬 타겟 수)보다 많다는 뜻이라 HUD 가 로그로 알린다.
    """
    valid = [o for o in (outcomes or []) if o in OUTCOMES]
    n = max(0, int(n_cells or 0))
    cells: List[Optional[str]] = list(valid[:n]) + [None] * max(0, n - len(valid))
    counts = {k: valid.count(k) for k in OUTCOMES}
    return {"cells": cells, "counts": counts, "overflow": max(0, len(valid) - n)}
