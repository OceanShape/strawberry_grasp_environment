"""T4d 배치 오케스트레이터 — Isaac Sim Script Editor 에서 **한 번** Run 해 둔다.

ROS 쪽 scripts/run_batch.sh 가 /tmp/harvest_batch/request.json 에 런 요청을 쓰면, 이 스크립트가
사용자가 손으로 하던 순서를 그대로 한다:
    Stop → 씬 재로드(main_scene.usd) → 로드 완료 대기 → 브릿지 스크립트 실행 → HUD 스크립트 실행 → Play
그리고 /tmp/harvest_batch/isaac_state.json 에 {"run": n, "state": "ready", "kit_log": ..., "kit_offset": ...} 를 쓴다.
run_batch.sh 는 그 뒤 노드를 띄우고 트리거하며, Kit 로그를 kit_offset 부터 잘라 PLACED/DROPPED/DROP_REST 를 읽는다.

왜 GUI 세션 안에서 하나: 런 12~14 를 검증한 브릿지·HUD·씬을 그대로 쓰기 위해서다(헤드리스 재구성 없음). 화면도 그대로 보인다.
왜 파일 신호인가: Kit 파이썬과 ROS 노드 사이에 새 토픽·서비스를 만들지 않는다(HUD 와 같은 규칙).

멈추려면 request.json 에 {"action": "done"} 이 오거나, Script Editor 에서 builtins.harvest_batch_sub = None.
"""
import builtins
import json
import os
import time
import traceback

import carb.settings
import omni.kit.app
import omni.timeline
import omni.usd

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
SCENE = os.path.join(REPO, "strawberry_harvest/scenes/main_scene.usd")
BRIDGE = os.path.join(REPO, "strawberry_harvest/scripts/isaac_sim_script_editor_bridge.py")
HUD = os.path.join(REPO, "strawberry_harvest/scripts/isaac_sim_hud.py")
BATCH_DIR = "/tmp/harvest_batch"
REQUEST = os.path.join(BATCH_DIR, "request.json")
STATE = os.path.join(BATCH_DIR, "isaac_state.json")
ROBOT_PRIM = "/World/robot_assembly"
POLL_SEC = 0.5
SETTLE_AFTER_LOAD_SEC = 2.0      # 로드 완료 뒤 첫 프레임들이 지나가게 (씬 로드 직후 브릿지 Run 은 예외가 난다 — run_guide)
SETTLE_AFTER_BRIDGE_SEC = 1.0


def _kit_log_path():
    try:
        p = carb.settings.get_settings().get("/log/file")
        if p and os.path.exists(p):
            return p
    except Exception:
        pass
    return None


def _write_state(**kw):
    os.makedirs(BATCH_DIR, exist_ok=True)
    kw.setdefault("ts", time.time())
    tmp = STATE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(kw, f)
    os.replace(tmp, STATE)


def _exec_script(path):
    """Script Editor 의 Run 과 같은 효과 — 모듈 이름 __main__, 파일 경로 __file__ 로 실행."""
    src = open(path, encoding="utf-8").read()
    g = {"__name__": "__main__", "__file__": path, "__builtins__": builtins}
    exec(compile(src, path, "exec"), g)


class _Orchestrator:
    def __init__(self):
        self.ctx = omni.usd.get_context()
        self.tl = omni.timeline.get_timeline_interface()
        self.last_req_mtime = 0.0
        self.run = None
        self.phase = "idle"
        self.t_phase = 0.0
        self.t_poll = 0.0
        os.makedirs(BATCH_DIR, exist_ok=True)
        _write_state(run=None, state="idle", msg="orchestrator armed")
        print("[batch] orchestrator armed — waiting for %s" % REQUEST)

    # -- update loop ---------------------------------------------------------------
    def on_update(self, _e):
        now = time.time()
        try:
            if self.phase == "idle":
                if now - self.t_poll < POLL_SEC:
                    return
                self.t_poll = now
                self._poll_request()
            elif self.phase == "wait_load":
                self._wait_load(now)
            elif self.phase == "settle":
                if now - self.t_phase >= SETTLE_AFTER_LOAD_SEC:
                    self._run_scripts()
            elif self.phase == "play":
                if now - self.t_phase >= SETTLE_AFTER_BRIDGE_SEC:
                    self._play_and_report()
        except Exception as exc:
            print("[batch] run %s failed in phase %s: %r" % (self.run, self.phase, exc))
            traceback.print_exc()
            _write_state(run=self.run, state="error", msg="%s: %r" % (self.phase, exc))
            self.phase = "idle"

    def _poll_request(self):
        if not os.path.exists(REQUEST):
            return
        mt = os.path.getmtime(REQUEST)
        if mt <= self.last_req_mtime:
            return
        self.last_req_mtime = mt
        try:
            req = json.load(open(REQUEST, encoding="utf-8"))
        except Exception:
            return   # half-written; next poll
        action = req.get("action")
        if action == "done":
            print("[batch] done — stopping timeline, orchestrator stays armed")
            self.tl.stop()
            _write_state(run=None, state="idle", msg="batch done")
            return
        if action != "reload":
            return
        self.run = req.get("run")
        print("[batch] run %s (seed %s): stop -> reload %s" % (self.run, req.get("seed"), SCENE))
        _write_state(run=self.run, state="reloading")
        self.tl.stop()
        if hasattr(builtins, "my_physx_sub"):
            builtins.my_physx_sub = None      # 옛 브릿지의 물리 콜백부터 끊는다 (브릿지 Run 도 같은 일을 한다)
        ok = self.ctx.open_stage(SCENE)
        if not ok:
            raise RuntimeError("open_stage returned False")
        self.phase = "wait_load"
        self.t_phase = time.time()

    def _wait_load(self, now):
        loaded, total = self.ctx.get_stage_loading_status()[1:3] if len(self.ctx.get_stage_loading_status()) >= 3 \
            else (0, 0)
        stage = self.ctx.get_stage()
        robot_ok = bool(stage and stage.GetPrimAtPath(ROBOT_PRIM).IsValid())
        if robot_ok and loaded >= total:
            print("[batch] stage loaded (%d/%d files), settling %.1fs" % (loaded, total, SETTLE_AFTER_LOAD_SEC))
            self.phase = "settle"
            self.t_phase = now
        elif now - self.t_phase > 180.0:
            raise RuntimeError("stage load timeout (loaded %s/%s, robot %s)" % (loaded, total, robot_ok))

    def _run_scripts(self):
        print("[batch] exec bridge: %s" % BRIDGE)
        _exec_script(BRIDGE)
        if os.path.exists(HUD):
            print("[batch] exec hud: %s" % HUD)
            try:
                _exec_script(HUD)
            except Exception as exc:
                print("[batch] HUD exec failed (run continues without HUD): %r" % exc)
        self.phase = "play"
        self.t_phase = time.time()

    def _play_and_report(self):
        self.tl.play()
        kit = _kit_log_path()
        offset = os.path.getsize(kit) if kit else 0
        _write_state(run=self.run, state="ready", kit_log=kit, kit_offset=offset)
        print("[batch] run %s ready — playing; kit log %s @%d" % (self.run, kit, offset))
        self.phase = "idle"


def start():
    if getattr(builtins, "harvest_batch_sub", None) is not None:
        builtins.harvest_batch_sub = None
    orch = _Orchestrator()
    builtins.harvest_batch_orch = orch
    builtins.harvest_batch_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        orch.on_update, name="harvest_batch_orchestrator")


start()
