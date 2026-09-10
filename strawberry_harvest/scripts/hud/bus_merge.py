"""
bus_merge.py — 프로세스별 스냅샷 파일을 하나의 상태로 합치는 읽기 측.

HUD 는 계산하지 않는다(사양 7-3). 그래서 '합치는 판단'을 hud.py 가 아니라
여기에 둔다. hud.py 는 load() 가 돌려준 것을 그리기만 한다.

소유권 — 한 필드는 한 곳만 쓴다 (사양 2절 원칙 3):

  nodes.<role>.last_seen  <- 그 role 의 파일만
  targets, region, run    <- scan  (수확 리스트와 순회 영역을 아는 것은 scan_executor 뿐)
  result.succeeded/failed <- planner (파지 판정과 릴리스를 실행하는 곳)
  result.finished         <- scan  (시퀀스 종료를 아는 곳)
  sequence                <- since 가 가장 최근인 파일
                             (아무것도 발행하지 않은 로컬 IDLE 은 since=0.0 이라
                              절대 이기지 않는다. 9절 Phase 2 더미 값 시험처럼
                              로컬에서 직접 publish 하면 그때만 이긴다.)

sequence 만 소유자가 둘(scan 은 스캔 구간, planner 는 픽 구간)이다.
두 구간은 시간상 겹치지 않으므로 '마지막으로 전이한 쪽이 현재'가 정확하다.
겹치면 판정이 어긋나는 것이 아니라 늦게 전이한 쪽이 이긴다.

파싱이나 파일 읽기가 실패하면 그 파일만 조용히 건너뛴다. 남은 파일로 계속
그리고, 빠진 노드는 last_seen 이 멈추므로 램프가 알아서 꺼진다.
"""
from __future__ import annotations

import json
import os
import sys
import time
from typing import Any, Dict, Tuple

import status_bus

ROLES = ("vision", "controller", "planner", "scan")

#: 스냅샷 파일이 이 시간보다 오래 갱신되지 않으면 그 프로세스는 죽은 것으로 본다.
#: 미러는 0.1초마다 쓰므로 20배 여유다.
STALE_SEC = 2.0

#: [FIX 2026-09-10] 이 HUD 세션이 시작된 시각. hud.install() 이 채운다.
#: 이보다 먼저 **마지막으로 쓰인** 파일은 이전 세션(직전 런)의 잔재이므로 없는 것으로 본다.
#: Isaac 을 다시 열면 지난 런의 '수확 완료 6/6' 이 램프 빨강인 채로 떠 있던 증상의 원인.
#: 살아 있는 프로세스는 0.1초마다 다시 쓰므로 written_at 이 항상 EPOCH 를 넘는다 — 영향 없음.
#: 이 세션 안에서 죽은 프로세스의 파일은 written_at > EPOCH 라 그대로 남는다(사양 8-6, 엔딩 유지).
EPOCH = 0.0

#: role -> 지금까지 스냅샷을 쓴 pid 들 (순서대로). 재시작이면 A -> B 로 한 번만
#: 바뀌고, 같은 노드가 둘 떠 있으면 A -> B -> A 로 되돌아온다. 되돌아올 때만 경고.
_pid_history = {}

#: role -> (지금까지 본 가장 최근 last_seen, 그 값을 쓴 pid)
#: last_seen 은 한 런 안에서 되돌아가지 않는다. 같은 이름의 노드가 둘 이상
#: 떠 있으면 서로의 스냅샷 파일을 덮어써서 램프가 초록↔빨강으로 깜빡이는데
#: (사양 8-4 에서 가장 안 좋아 보이는 증상), 뒤로 가는 값을 무시하면 화면이
#: 흔들리지 않는다. 대신 원인을 한 번 알린다 — 중복 노드는 HUD 문제가 아니라
#: 파이프라인이 비결정적으로 도는 실제 결함이다 (run_guide 30초 점검).
_last_seen_max = {}


_warned = set()


def _warn_once(message: str) -> None:
    if message in _warned:
        return
    _warned.add(message)
    try:
        sys.stderr.write("[harvest_hud] %s\n" % message)
    except Exception:
        pass


def _read(path: str) -> Dict[str, Any]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            payload = json.load(f)
    except Exception:
        return {}
    if not isinstance(payload, dict) or not isinstance(payload.get("state"), dict):
        return {}
    written_at = float(payload.get("written_at", 0.0))
    if written_at < EPOCH:
        return {}                      # 이전 세션의 잔재 — 없는 파일로 취급
    if (time.time() - written_at) > STALE_SEC:
        # 프로세스가 죽어 파일만 남은 경우. last_seen 은 그대로 두고 (램프는
        # 어차피 타임아웃으로 꺼진다) 카운터만 남기면 화면이 마지막 상태를
        # 유지한다 — 완주 후 엔딩이 사라지지 않아야 하므로(사양 8-6) 버리지 않는다.
        payload["stale"] = True
    return payload


def load(directory: str = None) -> Tuple[Dict[str, Any], Dict[str, bool]]:
    """(합쳐진 상태, 노드별 살아있음) 을 돌려준다.

    이 프로세스(Isaac Sim) 자신의 status_bus 도 함께 합친다. 그래야 사양 9절
    Phase 2 의 더미 값 시험이 파일 없이도 그대로 동작한다.
    """
    merged = status_bus.snapshot()          # 로컬(더미 값 시험용)이 바닥
    files = {}
    for role in ROLES:
        payload = _read(_path(role, directory))
        if payload:
            files[role] = payload

    for role, payload in files.items():
        state = payload["state"]
        entry = state.get("nodes", {}).get(role)
        if not isinstance(entry, dict):
            continue
        last_seen = float(entry.get("last_seen", 0.0))
        prev_seen, prev_pid = _last_seen_max.get(role, (0.0, None))
        pid = payload.get("pid")
        hist = _pid_history.setdefault(role, [])
        if not hist or hist[-1] != pid:
            if pid in hist:          # 이전에 보던 pid 로 되돌아왔다 = 둘이 번갈아 쓴다
                _warn_once("%s 스냅샷을 두 프로세스가 번갈아 쓰고 있다 (pid %s / %s). "
                           "같은 노드가 두 번 떠 있다 — ps -ef | grep %s 로 확인할 것"
                           % (role, hist[-1], pid, role))
            hist.append(pid)
            del hist[:-4]            # 재시작 이력이 무한히 쌓이지 않게
        if last_seen > prev_seen:
            _last_seen_max[role] = (last_seen, pid)
            prev_seen = last_seen
        if prev_seen > 0.0:
            merged["nodes"][role] = {
                "last_seen": prev_seen,
                "timeout": float(entry.get("timeout",
                                           status_bus.NODE_TIMEOUTS.get(role, 3.0))),
            }

    scan = files.get("scan", {}).get("state")
    if isinstance(scan, dict):
        for section in ("targets", "region", "run"):
            if isinstance(scan.get(section), dict):
                merged[section] = dict(scan[section])
        if isinstance(scan.get("result"), dict):
            merged["result"]["finished"] = bool(scan["result"].get("finished", False))

    planner = files.get("planner", {}).get("state")
    if isinstance(planner, dict) and isinstance(planner.get("result"), dict):
        for field in ("succeeded", "failed"):
            value = planner["result"].get(field)
            if isinstance(value, int):
                merged["result"][field] = value

    best = merged["sequence"]
    for payload in files.values():
        seq = payload["state"].get("sequence")
        if not isinstance(seq, dict):
            continue
        if seq.get("state") not in status_bus.SEQUENCE_STATES:
            continue
        if float(seq.get("since", 0.0)) > float(best.get("since", 0.0)):
            best = {"state": seq["state"], "since": float(seq["since"])}
    merged["sequence"] = best

    now = time.time()
    alive = {
        n: (d["last_seen"] > 0.0) and ((now - d["last_seen"]) < d["timeout"])
        for n, d in merged["nodes"].items()
    }
    return merged, alive


def _path(role: str, directory: str = None) -> str:
    base = directory or os.environ.get("HARVEST_HUD_DIR_RUNTIME", "/tmp")
    return os.path.join(base, "harvest_hud_%s.json" % role)
