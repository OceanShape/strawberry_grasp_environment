"""
bus_sink.py — status_bus 상태를 프로세스 밖으로 내보내는 쓰기 측 미러.

HUD_SPEC.md 10절(예비 경로). 계측 대상 네 개가 전부 Isaac Sim 과 다른
프로세스라서, 각 프로세스가 자기 status_bus 스냅샷을 JSON 한 파일로
0.1초마다 덮어쓴다. 읽기는 bus_merge.py 가 한다.

  * 반드시 임시 파일에 쓰고 os.replace 로 교체한다 (반쯤 쓰인 파일 읽기 방지).
  * 데몬 스레드 하나만 쓴다. 어떤 예외도 밖으로 나가지 않는다.
  * 파일이 갱신되는 것과 노드가 '일하는' 것은 다른 문제다. 이 스레드는
    프로세스가 살아 있는 한 계속 쓰지만, 램프를 켜는 것은 파일 안의
    last_seen 이다. 멈춘 노드는 파일이 갱신돼도 램프가 꺼진다 (사양 3절).
"""
from __future__ import annotations

import json
import os
import tempfile
import threading
import time

import status_bus

DEFAULT_DIR = os.environ.get("HARVEST_HUD_DIR_RUNTIME", "/tmp")
PERIOD_SEC = 0.1

_thread = None
_stop = threading.Event()
_path = None


def path_for(role: str, directory: str = None) -> str:
    return os.path.join(directory or DEFAULT_DIR, "harvest_hud_%s.json" % role)


def _write_once(role: str, path: str) -> None:
    payload = {
        "role": role,
        "written_at": time.time(),
        "pid": os.getpid(),
        "state": status_bus.snapshot(),
    }
    directory = os.path.dirname(path) or "."
    fd, tmp = tempfile.mkstemp(dir=directory, prefix=".harvest_hud_", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False)
        os.replace(tmp, path)
    except Exception:
        try:
            os.unlink(tmp)
        except Exception:
            pass


def _loop(role: str, path: str) -> None:
    while not _stop.wait(PERIOD_SEC):
        try:
            _write_once(role, path)
        except Exception:
            pass


def start(role: str, directory: str = None) -> str:
    """role 이름으로 미러 스레드를 띄운다. 두 번 불러도 하나만 돈다."""
    global _thread, _path
    try:
        if _thread is not None and _thread.is_alive():
            return _path
        status_bus.ROLE = role
        _path = path_for(role, directory)
        _stop.clear()
        _write_once(role, _path)          # 첫 파일을 즉시 남긴다
        _thread = threading.Thread(
            target=_loop, args=(role, _path), name="harvest_hud_sink", daemon=True)
        _thread.start()
        return _path
    except Exception:
        return _path


def stop() -> None:
    global _thread
    try:
        _stop.set()
        _thread = None
        if _path and os.path.exists(_path):
            os.unlink(_path)
    except Exception:
        pass
