"""
harvest_probe.py — 계측을 노드 밖에서 감싸는 모듈. 넣었다 뺐다 하는 것이 목적.

노드 쪽에 들어가는 것은 아래 네 줄뿐이고, 이 네 줄을 지우면 계측이 완전히
사라진다. 노드의 원래 동작을 바꾸는 코드는 이 파일 어디에도 없다.

    try:    # HUD 계측 — 빼려면 이 4줄만 지운다
        import sys, os; sys.path.append(os.path.expanduser(os.environ.get(
            "HARVEST_HUD_DIR", "~/strawberry_grasp_environment/strawberry_harvest/scripts/hud")))
        import harvest_probe; harvest_probe.attach("scan", self)
    except Exception: pass

동작 방식은 메서드 래핑이다. 대상 메서드를 같은 시그니처의 래퍼로 바꾸고,
래퍼는 (1) status_bus 를 건드리고 (2) 원본을 그대로 호출해 반환값을 그대로
돌려준다. 계측 쪽 코드는 전부 try/except 로 감싸 원본 호출 경로에 예외를
새로 만들지 않는다 (사양 2절 원칙 4). 원본이 던지는 예외는 그대로 통과시킨다.

왜 로그 문자열 파싱이 아니라 래핑인가:
  /rosout 문자열 매칭은 이 저장소에서 이미 오진을 낸 적이 있다
  (status_monitor_node.py 59행 주석 — "PICK COMPLETE" 가 "DETACH" 에 걸렸다).
  메서드 경계는 이름이 바뀌면 즉시 AttributeError 로 드러나므로 조용히
  틀리지 않는다. 예외는 scan 쪽 카운터 두 개인데, 그 값은 scan_executor 의
  _pub_status 문자열에만 있어서 어쩔 수 없다. 대신 그 문자열은 같은 함수가
  내는 상수 하나뿐이고, 한 런에서 한 번도 안 맞으면 경고를 남긴다.
"""
from __future__ import annotations

import os
import re
import sys
import threading

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import bus_sink          # noqa: E402
import result_bar        # noqa: E402
import status_bus        # noqa: E402
import tree_model        # noqa: E402

ROLES = ("vision", "controller", "planner", "scan")

_attached = {}           # role -> [(owner, method_name, original), ...]
_lock = threading.RLock()
_warned = set()


# ---- 래핑 도구 -----------------------------------------------------------

def _wrap(role, owner, name, before=None, after=None, after_call=None):
    """owner.name 을 래퍼로 바꾼다. before/after/after_call 은 실패해도 무시된다.

    before(args, kwargs) 는 원본 호출 직전에, after(result) 는 직후에 불린다.
    after_call(args, kwargs, result) 는 인자와 결과가 같이 필요할 때 쓴다 (트리 계측).
    """
    try:
        original = getattr(owner, name)
    except AttributeError:
        _warn("계측 지점 없음: %s.%s — 이름이 바뀌었는지 확인할 것" % (
            type(owner).__name__, name))
        return

    def wrapper(*args, **kwargs):
        if before is not None:
            try:
                before(args, kwargs)
            except Exception:
                pass
        result = original(*args, **kwargs)
        if after is not None:
            try:
                after(result)
            except Exception:
                pass
        if after_call is not None:
            try:
                after_call(args, kwargs, result)
            except Exception:
                pass
        return result

    wrapper.__name__ = getattr(original, "__name__", name)
    wrapper.__doc__ = getattr(original, "__doc__", None)
    setattr(owner, name, wrapper)
    _attached.setdefault(role, []).append((owner, name, original))


def _warn(message):
    if message in _warned:
        return
    _warned.add(message)
    try:
        sys.stderr.write("[harvest_probe] %s\n" % message)
    except Exception:
        pass


def _note(message):
    """판정이 바뀔 때마다 한 줄 (2026-09-17). _warn 과 달리 같은 문구도 거르지 않는다.

    stderr 는 노드 로그(~/.ros/log/python3_*.log -> log/m3/<run>/curobo_planner.log)에 남는다.
    """
    try:
        sys.stderr.write("[harvest_probe] %s\n" % message)
    except Exception:
        pass


def _wrap_ros(role, node, names, before):
    """rclpy 에 이미 등록된 콜백을 감싼다.

    ★ 인스턴스 속성을 바꾸는 것으로는 안 된다.
    create_subscription/create_service 는 __init__ 안에서 `self.strawberry_cb`
    같은 **바인드된 메서드 객체를 그 자리에서 붙잡아** 보관한다. 계측 블록은
    __init__ 끝에서 도므로, 그 뒤에 인스턴스(또는 클래스) 속성을 바꿔도 rclpy 는
    이미 들고 있는 원본을 계속 부른다 — 콜백은 정상 실행되는데 하트비트만
    조용히 안 찍힌다. 실제로 그렇게 한 번 틀렸다.

    그래서 rclpy 가 보관 중인 참조 자체(Subscription.callback / Service.callback /
    ActionServer._execute_callback)를 바꾼다.
    """
    wanted = set(names)
    found = set()

    holders = []
    holders += list(getattr(node, "subscriptions", []) or [])
    holders += list(getattr(node, "services", []) or [])
    for name in dir(node):                      # 액션 서버는 노드 목록에 없다
        if name.startswith("__"):
            continue
        try:
            obj = getattr(node, name)
        except Exception:
            continue
        if hasattr(obj, "_execute_callback") and hasattr(obj, "register_execute_callback"):
            holders.append(obj)

    for holder in holders:
        attr = "callback" if hasattr(holder, "callback") else "_execute_callback"
        original = getattr(holder, attr, None)
        if not callable(original):
            continue
        name = getattr(original, "__name__", None)
        if name not in wanted:
            continue

        def make(orig):
            def wrapper(*args, **kwargs):
                try:
                    before()
                except Exception:
                    pass
                return orig(*args, **kwargs)
            wrapper.__name__ = getattr(orig, "__name__", "wrapped")
            return wrapper

        setattr(holder, attr, make(original))
        _attached.setdefault(role, []).append((holder, attr, original))
        found.add(name)

    for name in sorted(wanted - found):
        _warn("rclpy 콜백을 찾지 못함: %s — 등록 이름이 바뀌었는지 확인할 것" % name)


def _seq(state):
    return lambda *_: status_bus.publish("sequence", state=state)


def _beat(name):
    return lambda *_: status_bus.heartbeat(name)


def _timer(role, node, period, name):
    """유휴 중에도 램프가 꺼지지 않도록 노드에 하트비트 타이머를 건다.

    새 토픽/서비스/노드를 만들지 않는다 (사양 7-2). 타이머는 기본
    콜백그룹에 들어가므로, 픽 시퀀스가 ReentrantCallbackGroup 에서 몇 분씩
    돌아도 MultiThreadedExecutor 안에서 계속 뛴다 — 8-4(깜빡임 없음)의 핵심.
    """
    try:
        handle = node.create_timer(period, lambda: status_bus.heartbeat(name))
        _attached.setdefault(role, []).append((node, "__timer__", handle))
    except Exception:
        _warn("하트비트 타이머 생성 실패: %s" % name)


# ---- 역할별 계측 ---------------------------------------------------------

def _attach_vision(node):
    # Isaac 브릿지가 1초마다 PoseArray 를 보내므로 콜백 진입만으로 1Hz 하트비트가
    # 된다 (timeout 3.0s = 3배 여유). 스로틀(0.5s) 앞에서 찍히도록 메서드 자체를
    # 감싼다 — 안쪽에 넣으면 조용한 구간에서 램프가 깜빡인다.
    _wrap_ros("vision", node, ["strawberry_cb"], lambda: status_bus.heartbeat("vision"))


def _attach_controller(node):
    # 플래너가 이 서비스들을 호출한다는 것 자체가 플래너 생존의 증거다(사양 4.2).
    _wrap_ros("controller", node,
              ["move_spline_cb", "move_joint_cb", "move_line_cb",
               "change_speed_cb", "set_position_cb", "get_state_cb",
               "safe_grasp_cb"],
              lambda: status_bus.heartbeat("planner"))
    # 컨트롤러 자신은 유휴 중에도 찍어야 한다. 이 노드에는 시뮬 스텝 콜백이
    # 없으므로(물리 스텝은 Isaac 프로세스 안에 있다) 타이머로 대신한다.
    _wrap("controller", node, "_publish_joint_command", before=_beat("controller"))
    _timer("controller", node, 0.3, "controller")


def _attach_planner(node):
    _timer("planner", node, 1.0, "planner")

    executor = getattr(node, "pick_sequence_executor", None)
    if executor is None:
        _warn("pick_sequence_executor 가 아직 없다 — attach 위치를 __init__ 끝으로 옮길 것")
        return

    # 새 런 감지. reset() 은 scan 프로세스에서만 불리므로 플래너의 배치/실패 카운터는
    # 저절로 비워지지 않는다 — 첫 실행에서 2차 런의 '배치' 가 6+1=7 로 찍혔다.
    # scan 스냅샷 파일의 run.started_at 이 바뀌었으면 이쪽도 비운다. 픽마다 한 번
    # 읽는 정도라 비용은 없다.
    seen = {"started_at": None}
    # [2026-09-17] 이번 픽의 파지 판정. 분리 실패 칸은 과실을 잡았다고 판정된 픽에만 붙인다(아래 _after_detach).
    pick = {"grasp": None}

    def _sync_run(*_):
        pick["grasp"] = None
        try:
            import json
            with open(bus_sink.path_for("scan"), encoding="utf-8") as f:
                started = json.load(f)["state"]["run"]["started_at"]
        except Exception:
            return
        if started and started != seen["started_at"]:
            if seen["started_at"] is not None or status_bus.snapshot()["run"]["started_at"] is None:
                status_bus.reset()
                status_bus.publish("run", started_at=started)
            seen["started_at"] = started
        status_bus.publish("sequence", state="PLAN")

    _wrap("planner", executor, "run", before=_sync_run)
    _wrap("planner", executor, "search_grasp", after=_seq("APPROACH"))
    _wrap("planner", executor, "_execute_final_approach_fn", before=_seq("ENTER"))
    # 열린 조우로 줄기 옆을 하강하는 구간. 화면에서는 파지의 일부로 본다
    # ('파지 + 하강') — 실제로 이 하강이 끝나는 자리에서 바로 닫는다.
    _wrap("planner", executor, "execute_open_stem_descent_if_needed", before=_seq("GRASP"))
    # [FIX 2026-09-10] execute_detach_and_retreat 하나를 RETREAT 로 찍고 있었다.
    # 그 함수는 **분리(BASE -Z 40mm 당겨 떼기)와 후퇴(진입 역순)를 연달아** 하므로,
    # 화면에는 '후퇴' 라고 떠 있는 동안 실제로는 아래로 당겨 분리하는 동작이 먼저 보였다.
    # 함수 안쪽의 두 이음매를 각각 찍어 분리와 후퇴를 나눈다.
    #   _execute_pitch_detach_fn   -> 분리   (pick_sequence_executor.py:298)
    #   _execute_retreat_steps_fn  -> 후퇴   (같은 파일 :316, 그리고 파지 실패 후퇴 :268)
    # 파지 실패 경로(handle_gripper_close_failed)도 같은 후퇴 함수를 쓰는데,
    # 거기서도 '후퇴' 표시가 맞다.
    # [2026-09-17] 결과 바 — 분리 실패. 두 이음매(당김·후퇴)를 감싼 함수 전체가 None 을 돌려주면
    # 분리 단계가 끝까지 못 간 것이다(후퇴 실패로 실행기가 시퀀스를 잡음, pick_sequence_executor.py
    # execute_detach_and_retreat). 당김 하나만 실패하면 실행기가 무시하고 계속하므로 여기서도 세지 않는다.
    # 실행기 메서드를 인스턴스 속성으로 감쌀 뿐이고 run() 의 호출·반환값은 그대로다.
    def _record(outcome, why):
        if outcome is None:
            return
        index = status_bus.append("result", "outcomes", outcome)
        if index <= 0:
            _warn("결과 바 칸을 붙이지 못함: status_bus result.outcomes 가 목록이 아니다 (%s)" % outcome)
            return
        _note("결과 바 %d번째 = %s (%s, %s)" % (index, outcome, result_bar.LABEL_KO[outcome], why))

    def _after_detach(result):
        # 파지 판정이 GRASP_CONTACT_DETECTED 가 아니면(빈손 등) 떼어낼 과실이 없던 픽이라 분리 실패로 세지 않는다
        # — 실행기는 빈손이어도 당김·후퇴를 그대로 하므로 후퇴 실패가 날 수 있다. 그 픽의 칸은 회색으로 남는다.
        if pick["grasp"] != "GRASP_CONTACT_DETECTED":
            return
        _record(result_bar.outcome_of_detach(result),
                "execute_detach_and_retreat returned None")

    _wrap("planner", executor, "execute_detach_and_retreat", after=_after_detach)
    _wrap("planner", executor, "_execute_pitch_detach_fn", before=_seq("DETACH"))
    _wrap("planner", executor, "_execute_retreat_steps_fn", before=_seq("RETREAT"))
    _wrap("planner", executor, "return_to_pick_start_and_complete",
          before=_seq("RETURN"))

    # 파지 판정. GRASP_CONTACT_DETECTED 가 아니면 실패로 센다.
    # (harvest_result_policy.allow_place_after_grasp 와 같은 기준)
    def _after_grasp(result):
        try:
            grasp_result = result[0] if isinstance(result, (tuple, list)) else result
        except Exception:
            return
        pick["grasp"] = grasp_result
        if grasp_result != "GRASP_CONTACT_DETECTED":
            status_bus.bump("result", "failed")

    _wrap("planner", executor, "_close_and_verify_grasp_fn",
          before=_seq("GRASP"), after=_after_grasp)

    tray = getattr(node, "tray_place_executor", None)
    if tray is None:
        _warn("tray_place_executor 가 없다 — 배치 카운터가 올라가지 않는다")
        return

    # 배치 카운터. 이 저장소에는 FixedJoint attach 가 없어 사양 4.3 의 성공
    # 3조건을 판정할 수 없다. 여기서 세는 것은 '파지 판정을 통과하고 트레이
    # 슬롯에서 릴리스까지 실행된 것' 이고, 화면 라벨은 '배치' 다.
    def _after_place(result):
        try:
            status = result[0] if isinstance(result, (tuple, list)) else result
        except Exception:
            return
        if status == "success":
            status_bus.bump("result", "succeeded")
        # [2026-09-17] 결과 바 — 같은 반환값으로 배치 성공 / 배치 실패를 한 칸 붙인다(result_bar.py).
        _record(result_bar.outcome_of_place_status(status), "place_status=%s" % (status,))

    _wrap("planner", tray, "execute_marker_place_after_retreat",
          before=_seq("PLACE"), after=_after_place)

    # [T4c 2026-09-15] 낙하 카운터. 배치 실패로 시퀀스를 잠그지 않고 그 자리에서 조우를
    # 여는 곳은 실행기의 이 메서드 하나다(hold_on_place_failure=false 일 때만 불린다).
    # 첫 인자 label 이 PLACE_GATE_BLOCKED_* 면 파지 판정에 실패해 과실이 없는 경우라
    # 세지 않는다(그건 failed). 나머지(계획 거부·트레이 없음·preview hold)는 분리된
    # 과실을 트레이 밖에서 놓은 것이므로 '낙하' 다. Isaac 브릿지도 같은 사건을
    # 릴리스 위치로 판정해 Kit 로그에 dropped=n 으로 찍는다 — 두 수가 같아야 한다.
    def _before_release_and_continue(args, kwargs):
        label = _arg(args, kwargs, 0, "label")
        if isinstance(label, str) and label.startswith("PLACE_GATE_BLOCKED"):
            return
        status_bus.bump("result", "dropped")

    _wrap("planner", executor, "_release_and_continue_after_place_failure",
          before=_before_release_and_continue)


_RE_SEQ_START = re.compile(
    r"PICK_SEQUENCE_START\s+\S+\s+.*?(\d+)\s+candidate targets.*?skipped_attempted=(\d+)")
_RE_TRIGGER = re.compile(r"PICK_TRIGGER\s+\S+\s+(\d+)/(\d+)")
# overview 1차 스캔의 분면별 후보 수. 실행기 지역 변수라 이 상태 문자열에만 있다.
_RE_OVERVIEW = re.compile(r"\bOVERVIEW_SCAN\s+((?:(?:nw|ne|se|sw):\d+\s*)+)")


def _arg(args, kwargs, index, name):
    """위치 인자 또는 키워드 인자 — 호출 모양이 바뀌어도 조용히 틀리지 않게 둘 다 본다."""
    if len(args) > index:
        return args[index]
    return kwargs.get(name)


def _attach_scan(node):
    _timer("scan", node, 1.0, "scan")

    # 아래 state 는 이 클로저에 남아 있는 누적 변수라 status_bus.reset() 만으로는
    # 안 비워진다 — 첫 실행에서 2차 런의 타겟이 7/8/9… 로 이어졌다. 같이 비운다.
    state = {"total": 0, "skipped": 0, "index": 0, "seen": False}

    # [2026-09-11] 쿼드트리 패널. 실행기의 결정(가지치기·잎·분할·세부 자세 퇴화)을 메서드 경계에서
    # 받아 tree_model 에 쌓고 status_bus 'tree' 섹션으로 낸다. 실행기 코드는 한 줄도 안 바뀐다.
    tree = tree_model.TreeModel()

    def _tree(fn, *a):
        try:
            fn(*a)
            status_bus.publish("tree", **tree.snapshot())
        except Exception:
            pass

    def _run_start(*_):
        status_bus.reset()          # region 도 home 으로 돌아간다 (로봇이 overview 에서 시작)
        state.update(total=0, skipped=0, index=0, seen=False)
        _tree(tree.reset)

    _wrap("scan", node, "_scan_sequence_run", before=_run_start)
    def _at_cell(state):
        """cell_id 를 첫 인자로 받는 메서드용 — 영역과 단계를 함께 찍는다.

        이동 중이면 '가려는 영역', 작업 중이면 '지금 있는 영역' 이 되는데,
        둘 다 그 호출의 cell_id 라서 한 군데서 처리된다.
        """
        def before(args, _kwargs):
            if args:
                status_bus.publish("region", name=status_bus.region_of(args[0]))
            if state:
                status_bus.publish("sequence", state=state)
        return before

    at_move, at_detect, at_pick = _at_cell("SCAN_MOVE"), _at_cell("DETECT"), _at_cell(None)

    def _before_move(args, kwargs):
        at_move(args, kwargs)
        _tree(tree.arrive, _arg(args, kwargs, 0, "cell_id"))

    def _before_detect(args, kwargs):
        at_detect(args, kwargs)
        _tree(tree.detected, _arg(args, kwargs, 0, "cell_id"), _arg(args, kwargs, 1, "count"))

    def _after_detect(args, kwargs, result):
        if result:                  # False = 세부 자세 이동 실패로 시퀀스 중단
            _tree(tree.cell_done, _arg(args, kwargs, 0, "cell_id"))

    def _before_pick(args, kwargs):
        at_pick(args, kwargs)
        _tree(tree.picking, _arg(args, kwargs, 0, "cell_id"))

    _wrap("scan", node, "_move_to_scan_cell_and_wait", before=_before_move)
    _wrap("scan", node, "_process_cell_detections", before=_before_detect,
          after_call=_after_detect)
    _wrap("scan", node, "_trigger_picks_for_cell", before=_before_pick)

    # 트리 전용 지점. 시뮬에서 넣은 메서드라(09-10 가지치기, 09-11 적응 분할) 없으면 경고만 남고
    # 해당 표시만 빠진다.
    def _after_prescan(args, kwargs, result):
        order = _arg(args, kwargs, 0, "scan_order") or []
        kept = set(result or [])
        _tree(tree.pruned, [c for c in order if c not in kept])

    def _before_split(args, kwargs):
        groups = _arg(args, kwargs, 1, "subgroups") or []
        _tree(tree.split, _arg(args, kwargs, 0, "cell_id"),
              {str(sub): len(poses) for sub, poses in groups})

    def _after_derive(args, kwargs, result):
        if result is None:          # SUBDIVIDE_REJECTED — 그 칸은 부모 자세에서 딴다
            _tree(tree.rejected, _arg(args, kwargs, 0, "parent_cell"),
                  _arg(args, kwargs, 1, "subcell"))

    def _before_prescan(*_):
        # overview 1차 스캔도 dwell 로 타겟을 받는 구간이다. 이게 없으면 트리의 ROOT 는 켜졌는데
        # 단계 표시는 reset 직후의 '대기' 로 남는다.
        status_bus.publish("sequence", state="DETECT")
        _tree(tree.prescan_start)

    _wrap("scan", node, "_overview_prescan_filter",
          before=_before_prescan, after_call=_after_prescan)
    _wrap("scan", node, "_should_subdivide",
          after_call=lambda a, k, r: _tree(tree.decided, _arg(a, k, 0, "cell_id"),
                                           _arg(a, k, 1, "n_candidates"), bool(r)))
    _wrap("scan", node, "_subdivide_and_pick", before=_before_split)
    _wrap("scan", node, "_derive_subcell_target", after_call=_after_derive)

    # 타겟 개수는 scan_executor 의 상태 문자열에만 있다 (vision mock 은 '지금
    # 분면에 보이는 딸기' 만 알고 수확 리스트를 모른다 — 사양 4.2 표와 다른 점).

    def _on_status(args, _kwargs):
        text = args[0] if args else ""
        if not isinstance(text, str):
            return
        m = _RE_OVERVIEW.search(text)
        if m:
            _tree(tree.prescan_counts, {q: int(n) for q, n in
                                        re.findall(r"(nw|ne|se|sw):(\d+)", m.group(1))})
            return
        m = _RE_SEQ_START.search(text)
        if m:
            state["seen"] = True
            state["total"] += int(m.group(1))
            state["skipped"] += int(m.group(2))
            status_bus.publish("targets",
                               total=state["total"], skipped=state["skipped"])
            return
        m = _RE_TRIGGER.search(text)
        if m:
            state["seen"] = True
            state["index"] += 1
            status_bus.publish("targets",
                               current_index=state["index"], attempted=state["index"])

    _wrap("scan", node, "_pub_status", before=_on_status)

    def _finished(*_):
        status_bus.publish("region", name="home")   # 마지막은 overview 복귀
        status_bus.publish("sequence", state="DONE")
        status_bus.publish("result", finished=True)
        _tree(tree.finish)
        if not state["seen"]:
            _warn("한 런 동안 PICK_SEQUENCE_START/PICK_TRIGGER 를 한 번도 못 봤다 — "
                  "scan_executor 의 상태 문자열이 바뀌었는지 확인할 것")

    _wrap("scan", node, "_finish_scan_sequence", after=_finished)


_ATTACHERS = {
    "vision": _attach_vision,
    "controller": _attach_controller,
    "planner": _attach_planner,
    "scan": _attach_scan,
}


# ---- 공개 API ------------------------------------------------------------

def attach(role: str, obj, directory: str = None) -> None:
    """role 계측을 걸고 스냅샷 미러를 띄운다. 실패해도 노드는 그대로 돈다."""
    try:
        with _lock:
            if role not in _ATTACHERS:
                _warn("알 수 없는 role: %s (가능: %s)" % (role, ", ".join(ROLES)))
                return
            if role in _attached:
                return
            _attached[role] = []
            _ATTACHERS[role](obj)
            path = bus_sink.start(role, directory)
            _warn("계측 부착: role=%s -> %s" % (role, path))
    except Exception as exc:                                    # noqa: BLE001
        _warn("attach 실패 (노드는 계속 돈다): %r" % (exc,))


def detach() -> None:
    """원본 메서드를 되돌리고 미러를 멈춘다. 주로 테스트용."""
    try:
        with _lock:
            for entries in _attached.values():
                for owner, name, original in reversed(entries):
                    if name == "__timer__":
                        try:
                            owner.destroy_timer(original)
                        except Exception:
                            pass
                    else:
                        setattr(owner, name, original)
            _attached.clear()
        bus_sink.stop()
    except Exception:
        pass
