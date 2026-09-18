"""
status_bus.py — 수확 시퀀스 상태의 단일 진실 원천 (in-process).

규칙:
  * 이 모듈은 omni.* 와 rclpy 를 절대 import 하지 않는다. 순수 파이썬만.
  * 상태 '판정'은 전부 여기서. HUD 는 읽어서 그리기만 한다.
  * 어떤 함수도 예외를 밖으로 던지지 않는다. 계측 코드가 시뮬을 죽이면 안 된다.

HUD_SPEC.md 4.1 참조 구현. 아래 네 가지가 사양과 다르다.

  (1) _LOG_PATH 기본값에 role 을 붙인다.
      사양은 단일 프로세스를 전제하지만 이 저장소에서는 계측 대상 4개가
      전부 별도 프로세스다. 네 프로세스가 같은 파일에 append 하면 줄이 섞여
      완주 실패 때 원인 추적(8절)이 불가능해진다.

  (2) region 섹션 추가.
      사양 3절 스키마에 없다. 4분면 순회가 이 프로젝트 시퀀스의 뼈대라,
      "지금 어느 영역에서 일하는지" 가 없으면 화면만 보고 진행을 못 읽는다.
      소유자는 scan_executor 하나뿐이다.

  (3) result.succeeded 의 의미.
      사양 4.3 의 성공 정의는 attach 를 전제하는데 이 저장소에는 FixedJoint
      attach 코드가 없다 (딸기 prim 은 그리퍼를 따라가지 않는다). 그래서
      succeeded 는 '파지 판정 통과 + 트레이 슬롯에서 릴리스 실행 완료' 를 센다.
      사양 4.3 지시대로 화면 라벨은 '성공' 이 아니라 '배치' 다.
      필드 이름은 3절 계약이므로 바꾸지 않는다.

  (4) tree 섹션 추가 (2026-09-11).
      쿼드트리 순회(1차 스캔 가지치기·잎·분할·세부 자세)를 화면에 트리로 그리는 상태.
      소유자는 scan 하나다. 모양·전이 규칙·표시 규칙은 tree_model.py 에 있고,
      여기서는 빈 값만 만든다.

  (5) result.dropped 추가 (2026-09-15, T4c).
      분리까지 된 과실을 이송·배치 계획 거부로 트레이 밖에서 놓아 버린 수.
      실기 플래너의 `_release_and_continue_after_place_failure` 가 호출된 횟수다
      (hold_on_place_failure=false 일 때만 불린다). 파지 자체가 실패한 경우
      (PLACE_GATE_BLOCKED) 는 과실이 없으므로 세지 않는다 — 그건 failed 다.
      소유자는 planner. 완료 줄에 '배치 n · 낙하 m' 으로 나간다.

  (6) result.outcomes 추가 (2026-09-17).
      결과 바의 칸 목록. 시도 순서대로 'placed' / 'place_failed' / 'detach_failed' 를 하나씩 붙인다
      (판정 규칙·문구·색은 result_bar.py). 소유자는 planner. HUD 는 이 목록으로 칸을 칠하고 범례
      '배치 성공 n · 배치 실패 m · 분리 실패 k' 를 센다 — 이날부터 완료 블록 둘째 줄은 이 범례로 바뀌었다.
      result.dropped 는 그대로 센다(Kit 브릿지 dropped=n 과 맞춰 보는 계측값). 화면에는 더 안 나간다.
      (2026-09-18) 'detach_failed' 의 범위가 넓어졌다: 직선 진입이 시작된 픽이 트레이 배치 실행기를 부르지
      못하고 끝나면 어디서 막혔든 이 키다. 판정은 실행기 run() 종료 직후 한 곳(result_bar.outcome_of_pick_end).
"""
from __future__ import annotations

import copy
import json
import os
import threading
import time
from typing import Any, Dict

import tree_model

#: 시퀀스 상태 enum. 이 목록 밖의 값은 publish 시 거부된다.
SEQUENCE_STATES = [
    "IDLE",       # 대기 / 재시작 가능
    "SCAN_MOVE",  # 스캔 자세로 이동
    "DETECT",     # 타겟 수신
    "PLAN",       # 궤적 계획
    "APPROACH",   # 접근
    "ENTER",      # 진입
    "GRASP",      # 하강 + 파지
    # [FIX 2026-09-10] 분리를 후퇴에서 떼어냈다.
    # 설계 시퀀스 4단계(BASE -Z 40mm 당겨 분리)가 5단계(진입 역순 후퇴)와 같은
    # 'RETREAT' 로 묶여 있었다. 화면에서는 딸기가 덩굴에서 떨어지는 그 순간이
    # '후퇴' 로 표시돼, 수확에서 제일 중요한 사건에 이름이 없었다.
    "DETACH",     # 당김 (아래로 당겨 떼기). 화면 라벨은 09-18부터 '당김' — 결과 바 '분리 실패' 와 글자 구분
    "RETREAT",    # 후퇴 (진입 역순)
    "PLACE",      # 배치
    "RETURN",     # 복귀
    "DONE",       # 완료
]

#: 작업 영역. 보드를 4분면으로 나눈 세부영역 + 대기 자세(overview).
#: 이 목록 밖의 값은 publish 시 거부된다.
REGIONS = ["home", "nw", "ne", "se", "sw"]


def region_of(cell_id: str) -> str:
    """scan_executor 의 cell_id -> 영역 이름. 'root/nw_flat' -> 'nw'.

    분면을 못 알아보면 'home' 을 돌려준다 (overview 로 취급).
    """
    try:
        parts = str(cell_id).split("/")
        if len(parts) >= 2:
            quad = parts[1][:2]
            if quad in REGIONS:
                return quad
    except Exception:
        pass
    return "home"


#: 노드별 liveness 타임아웃(초).
#: 소스 성격에 맞춰 개별 지정한다 — 단일 값으로 두면 조용한 구간에서 램프가 깜빡인다.
NODE_TIMEOUTS: Dict[str, float] = {
    "vision": 3.0,        # 주기 발행
    "planner": 12.0,      # 긴 MoveJoint 구간 동안 호출이 끊긴다. 넉넉히.
    "controller": 1.5,    # 유휴 중에도 매 스텝 하트비트를 찍으므로 짧게
    "scan": 6.0,
}

#: 이 프로세스가 어떤 역할로 계측되는지. bus_sink.start() 가 채운다.
ROLE = os.environ.get("HARVEST_HUD_ROLE", "hud")

def _log_path() -> str:
    """호출 시점의 ROLE 로 경로를 정한다.

    import 시점에 고정하면 bus_sink.start() 가 ROLE 을 바꿔도 네 노드가 전부
    기본값 'hud' 파일 하나에 섞여 쓴다 (첫 실행에서 실제로 그랬다).
    """
    return os.environ.get("HARVEST_HUD_LOG", "/tmp/harvest_hud_%s.log" % ROLE)

_LOCK = threading.RLock()


def _blank() -> Dict[str, Any]:
    return {
        "nodes": {n: {"last_seen": 0.0, "timeout": t} for n, t in NODE_TIMEOUTS.items()},
        "targets": {"total": 0, "current_index": 0, "attempted": 0, "skipped": 0},
        # since = 0.0 (사양은 time.time()). bus_merge 가 sequence 를 'since 가
        # 가장 최근인 쪽' 으로 고르는데, 초기값에 현재 시각이 들어가면 아무것도
        # 발행하지 않은 HUD 프로세스의 IDLE 이 네 노드의 실제 상태를 이긴다.
        # 런 도중 Isaac Sim 을 다시 켜면 화면이 '대기' 로 굳는 경로다.
        "region": {"name": "home"},
        "tree": tree_model.blank(),
        "sequence": {"state": "IDLE", "since": 0.0},
        "result": {"succeeded": 0, "failed": 0, "dropped": 0, "outcomes": [], "finished": False},
        "run": {"started_at": None},
    }


_STATE: Dict[str, Any] = _blank()


def _log(section: str, fields: Dict[str, Any]) -> None:
    """값이 바뀔 때만 한 줄 append. 실패해도 조용히 넘어간다."""
    try:
        line = f"{time.time():.3f}|{section}|{json.dumps(fields, ensure_ascii=False)}\n"
        with open(_log_path(), "a", encoding="utf-8") as f:
            f.write(line)
    except Exception:
        pass


# ---- 쓰기 API — 계측 지점에서 호출 --------------------------------------

def heartbeat(node: str) -> None:
    """해당 노드가 지금 살아 있음을 표시. 유휴 중에도 계속 불러야 한다."""
    try:
        with _LOCK:
            entry = _STATE["nodes"].get(node)
            if entry is not None:
                entry["last_seen"] = time.time()
    except Exception:
        pass


def publish(section: str, **fields: Any) -> None:
    """섹션의 필드를 갱신. 실제로 값이 바뀐 것만 로그에 남는다."""
    try:
        with _LOCK:
            target = _STATE.get(section)
            if not isinstance(target, dict):
                return
            if section == "sequence" and "state" in fields:
                if fields["state"] not in SEQUENCE_STATES:
                    _log("ERROR", {"unknown_state": fields["state"]})
                    return
            if section == "region" and "name" in fields:
                if fields["name"] not in REGIONS:
                    _log("ERROR", {"unknown_region": fields["name"]})
                    return
            changed = {k: v for k, v in fields.items() if target.get(k) != v}
            if not changed:
                return
            target.update(changed)
            if section == "sequence" and "state" in changed:
                target["since"] = time.time()
    except Exception:
        return
    _log(section, changed)


def bump(section: str, field: str, delta: int = 1) -> None:
    """카운터를 delta 만큼 올린다.

    사양에 없는 함수다. publish 는 절대값을 받으므로, 호출부가 현재값을 읽어
    +1 해서 넘기면 읽기-쓰기 사이에 락이 풀려 두 스레드가 같은 값을 쓴다
    (플래너는 MultiThreadedExecutor 라 실제로 가능한 경합이다). 락 안에서
    증가시키는 함수를 하나 두는 편이 계약을 덜 흔든다.
    """
    try:
        with _LOCK:
            target = _STATE.get(section)
            if not isinstance(target, dict):
                return
            current = target.get(field)
            if not isinstance(current, int):
                return
            target[field] = current + delta
            changed = {field: target[field]}
    except Exception:
        return
    _log(section, changed)


def append(section: str, field: str, value: Any) -> int:
    """목록 필드 끝에 value 를 붙이고 붙인 뒤 길이를 돌려준다 (실패하면 0).

    [2026-09-17] result.outcomes(결과 바) 용. bump 와 같은 이유로 락 안에서 붙인다 — 호출부가 목록을
    읽어 새 목록을 publish 하면 읽기-쓰기 사이에 다른 스레드의 결과가 사라질 수 있다.
    로그에는 붙인 값과 몇 번째인지 한 줄을 남긴다.
    """
    try:
        with _LOCK:
            target = _STATE.get(section)
            if not isinstance(target, dict):
                return 0
            current = target.get(field)
            if not isinstance(current, list):
                return 0
            current.append(value)
            size = len(current)
    except Exception:
        return 0
    _log(section, {field + "_append": value, "index": size})
    return size


def reset() -> None:
    """새 런 시작. 시퀀스 시작 지점에서 정확히 한 번 호출한다."""
    global _STATE
    try:
        with _LOCK:
            _STATE = _blank()
            _STATE["run"]["started_at"] = time.time()
    except Exception:
        pass
    _log("run", {"reset": True})


# ---- 읽기 API — HUD 전용 -------------------------------------------------

def snapshot() -> Dict[str, Any]:
    """깊은 복사본. 호출자가 들고 있어도 내부 상태와 얽히지 않는다."""
    with _LOCK:
        return copy.deepcopy(_STATE)


def node_status() -> Dict[str, bool]:
    """노드별 살아있음 여부. 판정은 여기서 하고 HUD 는 색만 칠한다."""
    now = time.time()
    with _LOCK:
        return {
            n: (d["last_seen"] > 0.0) and ((now - d["last_seen"]) < d["timeout"])
            for n, d in _STATE["nodes"].items()
        }


def elapsed() -> float:
    with _LOCK:
        started = _STATE["run"]["started_at"]
    return 0.0 if started is None else (time.time() - started)
