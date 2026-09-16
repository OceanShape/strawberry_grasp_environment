"""
tree_model.py — HUD 쿼드트리 패널의 상태·배치·표시 규칙. 순수 파이썬 (omni·rclpy import 금지).

세 가지를 한 곳에 둔다 (2026-09-11).

  1. TreeModel   : scan_executor 의 결정(1차 스캔 가지치기, 잎, 분할, 세부 자세 퇴화)을 받아 트리 상태로
                   쌓는다. harvest_probe 가 scan 프로세스에서 쓰고, 스냅샷은 status_bus 'tree' 섹션으로 나간다.
  2. tree_layout : 트리 영역 폭에 대한 노드·연결선 배치 (Kit 논리 픽셀).
  3. view        : 트리 스냅샷 + 배치 -> 노드별 스타일·숫자·라벨 키, 연결선별 색.

HUD(../isaac_sim_viewport_display.py)는 2·3 을 받아 칠하기만 한다. 라벨 생성기(make_labels.py)도 여기 문구를 쓴다.
트리는 실행기의 결정을 **그린다**. 판정은 scan_executor 가 한 것이고 여기서 다시 계산하지 않는다.

노드 상태
  root : pending(런 전) / active(overview 1차 스캔 중) / base(순회 중) / done(완료)
  분면·세부 칸 : pending / active(로봇이 지금 여기) / parent(자식 칸에 내려가 있음) / done / pruned
"""
from __future__ import annotations

import copy
import threading
from typing import Dict, Iterable, Optional, Tuple

#: 화면 순서 — 1단(분면)·2단(세부 칸) 공통 (사용자 지정 2026-09-11).
#: 실행기의 세부 칸 방문 순서(sw, se, nw, ne — 아래부터)와 다르다. 화면은 위치 순서다.
QUADS = ("nw", "ne", "se", "sw")

NODE_STATES = ("pending", "active", "parent", "done", "pruned")
ROOT_STATES = ("pending", "active", "base", "done")

# ---- 문구 (1단 노드 둘째 줄. 2단 노드는 문구 없음) ------------------------------------
#: [2026-09-16 S5] 방위 글자(북서·남동 …)는 뺐다. 방위는 손목 카메라 창의 십자·NW/NE/SW/SE 가이드가
#: 보여 주고 노드 첫 줄이 이미 NW 다. 둘째 줄은 **상태만**: 잎 = 없음, 분할 = '분할', 1차 스캔
#: 가지치기 = '제외'. (09-12 지정 '방향(분할)' 꼴은 글자를 24px 로 키우면 4열 패널에 안 들어간다.)
STATUS_KO = {"split": "분할", "pruned": "제외"}
TAG_KEYS = tuple(["dir_" + q for q in QUADS] + ["split_" + q for q in QUADS]
                 + ["pruned_" + q for q in QUADS])
TAG_SIZE = 18
TAG_COLOR = {"split": (167, 139, 250, 255), "pruned": (123, 132, 148, 255)}
#: labels/ 가 없을 때(영문 폴백) 쓰는 글자. 잎(dir_*)은 둘째 줄이 없다.
TAG_EN = dict([("dir_" + q, "") for q in QUADS] + [("split_" + q, "split") for q in QUADS]
              + [("pruned_" + q, "skipped") for q in QUADS])


def tag_parts(key: str):
    """라벨 키 -> [(글자, RGBA)]. make_labels.py 가 이걸로 PNG 를 그린다. 빈 리스트 = 둘째 줄 없음."""
    kind, _q = key.split("_", 1)
    if kind == "pruned":        # 노드 전체가 흐려지는 상태라 글자도 흐리게
        return [(STATUS_KO["pruned"], TAG_COLOR["pruned"])]
    if kind == "split":
        return [(STATUS_KO["split"], TAG_COLOR["split"])]
    return []


# ---- 색 (0-255 RGBA) ----------------------------------------------------------------
#: 로봇이 지금 있는 노드와 경로. 보드 하이라이트(옅은 주황)와 같은 계열로 맞춰
#: 보드에서 불 켜진 칸과 트리에서 불 켜진 노드가 같은 곳으로 읽히게 한다.
ACTIVE = (255, 164, 110)
TEXT = (232, 237, 245, 255)
DIM = (123, 132, 148, 255)
DIM_FAINT = (123, 132, 148, 130)
OK = (90, 212, 105, 255)


def _w(a):
    return (255, 255, 255, a)


NODE_STYLE = {
    "pending": {"fill": _w(10), "border": _w(46), "bw": 1.0, "name": DIM, "count": DIM},
    "active": {"fill": ACTIVE + (72,), "border": ACTIVE + (255,), "bw": 2.0, "name": TEXT, "count": TEXT},
    "parent": {"fill": ACTIVE + (24,), "border": ACTIVE + (150,), "bw": 1.5, "name": TEXT, "count": TEXT},
    "done": {"fill": _w(10), "border": OK[:3] + (170,), "bw": 1.5, "name": TEXT, "count": OK},
    "pruned": {"fill": _w(4), "border": _w(22), "bw": 1.0, "name": DIM_FAINT, "count": DIM_FAINT},
    "base": {"fill": _w(10), "border": _w(70), "bw": 1.0, "name": TEXT, "count": TEXT},
}
#: 세부 자세 유도가 거부돼(SUBDIVIDE_REJECTED) 부모 자세에서 딴 세부 칸은 끝났을 때 초록 대신 이 테두리.
REJECTED_BORDER = (255, 181, 71, 200)
EDGE, EDGE_DIM, EDGE_ON = _w(56), _w(18), ACTIVE + (220,)


# ---- 1. 상태 -------------------------------------------------------------------------

def blank() -> Dict:
    return {
        "root": "pending",
        "current": "",                  # "" / "sw" / "sw/se" — 로봇이 지금 있는 노드
        "nodes": {q: {"state": "pending", "count": None, "split": False} for q in QUADS},
        "split_parent": None,           # 가장 최근에 분할한 분면 — 2단 줄은 이 분면의 자식만 그린다
        "children": {},                 # split_parent 의 자식: {sub: {state, count, rejected}}
    }


def parse_cell(cell_id) -> Tuple[Optional[str], Optional[str]]:
    """'root/nw_flat' -> ('nw', None), 'root/sw/se' -> ('sw', 'se'), 'root' -> (None, None)."""
    parts = str(cell_id or "").split("/")
    if len(parts) < 2 or parts[1][:2] not in QUADS:
        return None, None
    sub = parts[2][:2] if len(parts) >= 3 and parts[2][:2] in QUADS else None
    return parts[1][:2], sub


class TreeModel:
    """scan_executor 의 결정 -> 트리 상태. 호출 순서는 실행기의 실제 호출 순서를 따른다."""

    def __init__(self):
        self._lock = threading.RLock()
        self.reset()

    def reset(self) -> None:
        with self._lock:
            self._t = blank()
            self._children: Dict[str, Dict[str, Dict]] = {}

    def snapshot(self) -> Dict:
        with self._lock:
            t = copy.deepcopy(self._t)
            sp = t["split_parent"]
            t["children"] = copy.deepcopy(self._children.get(sp, {})) if sp else {}
            return t

    # -- 1차 스캔 (overview) --
    def prescan_start(self) -> None:
        with self._lock:
            self._t["root"] = "active"

    def prescan_counts(self, counts: Dict[str, int]) -> None:
        """OVERVIEW_SCAN nw:2 ne:1 se:0 sw:3 — 아직 안 간 분면에 1차 스캔 후보 수를 적는다."""
        with self._lock:
            for q, n in counts.items():
                node = self._t["nodes"].get(q)
                if node is not None and node["state"] == "pending":
                    node["count"] = int(n)

    def pruned(self, cell_ids: Iterable[str]) -> None:
        with self._lock:
            for cell_id in cell_ids:
                q, sub = parse_cell(cell_id)
                if q is not None and sub is None:
                    self._t["nodes"][q]["state"] = "pruned"

    # -- 순회 --
    def arrive(self, cell_id) -> None:
        """스캔 자세 이동 시작. 세부 칸이면 부모는 parent, 그 칸이 active."""
        q, sub = parse_cell(cell_id)
        if q is None:
            return
        with self._lock:
            if sub is not None and sub not in self._children.get(q, {}):
                sub = None          # 분할 안 된 분면의 논리 칸 — 로봇은 분면 자세에 있다
            self._set_current(q, sub)

    def detected(self, cell_id, count) -> None:
        """분면 자세 dwell 결과. 0 이면 여기서 끝나므로 숫자를 0 으로 고친다."""
        q, sub = parse_cell(cell_id)
        if q is None or sub is not None or count is None:
            return
        with self._lock:
            if int(count) == 0:
                self._t["nodes"][q]["count"] = 0

    def decided(self, cell_id, n_candidates, split) -> None:
        """_should_subdivide 의 입력(중복 제거 후 후보 수)과 결과."""
        q, sub = parse_cell(cell_id)
        if q is None or sub is not None:
            return
        with self._lock:
            node = self._t["nodes"][q]
            if n_candidates is not None:
                node["count"] = int(n_candidates)
            node["split"] = bool(split)

    def split(self, cell_id, counts: Dict[str, int]) -> None:
        """_subdivide_and_pick 진입 — 부모 자세 시야의 세부 칸별 후보 수. 0 인 칸은 2단 가지치기."""
        q, sub = parse_cell(cell_id)
        if q is None or sub is not None:
            return
        with self._lock:
            kids = {}
            for s in QUADS:
                n = int(counts.get(s, 0) or 0)
                kids[s] = {"state": "pending" if n > 0 else "pruned", "count": n, "rejected": False}
            self._children[q] = kids
            self._t["split_parent"] = q
            self._t["nodes"][q]["split"] = True

    def rejected(self, parent_cell, sub) -> None:
        """세부 자세 유도 실패 — 그 칸은 부모 자세에서 딴다."""
        q, _ = parse_cell(parent_cell)
        with self._lock:
            kid = self._children.get(q, {}).get(sub)
            if kid is not None:
                kid["rejected"] = True

    def picking(self, cell_id) -> None:
        """_trigger_picks_for_cell 진입. 분할된 분면의 세부 칸이면 그 칸을 켠다 (부모 자세 퇴화 포함)."""
        q, sub = parse_cell(cell_id)
        if q is None or sub is None:
            return
        with self._lock:
            if sub in self._children.get(q, {}):
                self._set_current(q, sub)

    def cell_done(self, cell_id) -> None:
        """_process_cell_detections 가 True 로 끝남 = 그 분면(과 자식) 완료."""
        q, sub = parse_cell(cell_id)
        if q is None or sub is not None:
            return
        with self._lock:
            for kid in self._children.get(q, {}).values():
                if kid["state"] == "active":
                    kid["state"] = "done"
            node = self._t["nodes"][q]
            if node["state"] != "pruned":
                node["state"] = "done"
            if self._t["current"].split("/")[0] == q:
                self._t["current"] = ""

    def finish(self) -> None:
        with self._lock:
            self._close_current(None, None)
            self._t["root"] = "done"
            self._t["current"] = ""

    # -- 내부 --
    def _close_current(self, q, sub) -> None:
        """지금 켜진 노드에서 (q, sub) 로 옮겨 갈 때 떠나는 쪽을 done 으로."""
        prev = self._t["current"]
        if not prev:
            return
        pq, _, psub = prev.partition("/")
        if (pq, psub or None) == (q, sub):
            return
        if psub:
            kid = self._children.get(pq, {}).get(psub)
            if kid is not None and kid["state"] == "active":
                kid["state"] = "done"
        if pq != q:
            node = self._t["nodes"].get(pq)
            if node is not None and node["state"] in ("active", "parent"):
                node["state"] = "done"

    def _set_current(self, q, sub) -> None:
        self._close_current(q, sub)
        node = self._t["nodes"][q]
        if sub is None:
            node["state"] = "active"
        else:
            node["state"] = "parent"
            self._children[q][sub]["state"] = "active"
        self._t["current"] = q if sub is None else "%s/%s" % (q, sub)
        if self._t["root"] in ("pending", "active"):
            self._t["root"] = "base"


# ---- 2. 배치 (Kit 논리 픽셀) ---------------------------------------------------------
# [2026-09-16 S5] 1080p 녹화 가독성으로 키움 — 노드 글자 24px(1단)·20px(2단·ROOT)·태그 18px 기준.
LINE = 2.0
ROOT_W, ROOT_H = 80.0, 30.0
STEM1, DROP1 = 9.0, 7.0
L1_GAP, L1_H = 8.0, 58.0
STEM2, DROP2 = 8.0, 6.0
L2_GAP, L2_H, L2_W_MAX = 6.0, 38.0, 70.0


def _hline(points, key_fmt):
    """수평 연결선을 겹치지 않는 조각으로 나눈다 — Kit HStack 은 위젯을 겹쳐 놓지 못한다.

    각 조각의 (a, b) 는 경로 강조 판정용 구간이다.
    """
    pts = sorted(set(round(float(p), 3) for p in points))
    items, spans = [], []
    last = len(pts) - 2
    for k in range(len(pts) - 1):
        a, b = pts[k], pts[k + 1]
        x0 = a - LINE / 2 if k == 0 else a
        x1 = b + LINE / 2 if k == last else b
        key = key_fmt % k
        items.append({"x": x0, "w": x1 - x0, "kind": "line", "key": key})
        spans.append((key, a, b))
    return items, spans


def _vline(x, key):
    return {"x": x - LINE / 2, "w": LINE, "kind": "line", "key": key}


def tree_layout(tw: float) -> Dict:
    """트리 영역 폭 tw 에 대한 배치.

    bands: 위에서 아래로 쌓는 가로 띠. 띠 안의 item 은 x 순서이고 서로 겹치지 않는다
           (HUD 는 띠 하나를 HStack 하나로, item 사이를 Spacer 로 채운다).
    groups[q]: q 가 분할됐을 때 보이는 2단 띠들. 네 벌을 겹쳐 두고 하나만 켠다.
    """
    tw = float(tw)
    cx = tw / 2.0
    w1 = (tw - 3 * L1_GAP) / 4.0
    l1x = {q: i * (w1 + L1_GAP) for i, q in enumerate(QUADS)}
    c = {q: l1x[q] + w1 / 2.0 for q in QUADS}
    bus1_items, bus1 = _hline([cx] + [c[q] for q in QUADS], "bus1:%d")
    bands = [
        {"h": ROOT_H, "items": [{"x": cx - ROOT_W / 2, "w": ROOT_W, "kind": "root", "key": "root"}]},
        {"h": STEM1, "items": [_vline(cx, "stem")]},
        {"h": LINE, "items": bus1_items},
        {"h": DROP1, "items": [_vline(c[q], "drop1:" + q) for q in QUADS]},
        {"h": L1_H, "items": [{"x": l1x[q], "w": w1, "kind": "l1", "key": q} for q in QUADS]},
    ]
    top_h = sum(b["h"] for b in bands)
    w2 = min(L2_W_MAX, (tw - 3 * L2_GAP) / 4.0)
    span = 4 * w2 + 3 * L2_GAP
    groups = {}
    for q in QUADS:
        gx = min(max(c[q] - span / 2.0, 0.0), tw - span)
        l2x = {s: gx + j * (w2 + L2_GAP) for j, s in enumerate(QUADS)}
        gc = {s: l2x[s] + w2 / 2.0 for s in QUADS}
        bus2_items, bus2 = _hline([c[q]] + [gc[s] for s in QUADS], "bus2:%d")
        groups[q] = {"pc": c[q], "gc": gc, "bus2": bus2, "bands": [
            {"h": STEM2, "items": [_vline(c[q], "stem2")]},
            {"h": LINE, "items": bus2_items},
            {"h": DROP2, "items": [_vline(gc[s], "drop2:" + s) for s in QUADS]},
            {"h": L2_H, "items": [{"x": l2x[s], "w": w2, "kind": "l2", "key": s} for s in QUADS]},
        ]}
    l2_h = STEM2 + LINE + DROP2 + L2_H
    y = 0.0
    for b in bands:
        b["y"], y = y, y + b["h"]
    for g in groups.values():
        y = top_h
        for b in g["bands"]:
            b["y"], y = y, y + b["h"]
    return {"width": tw, "cx": cx, "c": c, "bands": bands, "bus1": bus1, "groups": groups,
            "top_h": top_h, "l2_h": l2_h, "height": top_h + l2_h}


# ---- 3. 표시 규칙 -------------------------------------------------------------------

def _cnt(n) -> str:
    try:
        return "" if n is None else str(int(n))
    except (TypeError, ValueError):
        return ""


def _within(a, b, p, q) -> bool:
    lo, hi = min(p, q), max(p, q)
    return a >= lo - 1e-6 and b <= hi + 1e-6


def l1_tag(q: str, state: str, split: bool) -> str:
    if state == "pruned":
        return "pruned_" + q
    return ("split_" if split else "dir_") + q


def view(tree, lay: Dict) -> Dict:
    """스냅샷 -> {"nodes": {key: {style, count, tag, border}}, "lines": {key: RGBA}, "group": 분할 분면|None}.

    노드 키: "root", 분면 "nw".., 2단 "sub:nw".. / 선 키: 1단 그대로, 2단은 "g:" 접두.
    show_l2: 순회가 끝나면(root done) False — 2단 영역을 통째로 접는다 (사용자 지정 2026-09-12).
    그 자리에 HUD 의 '수확 완료' 줄이 뜬다.
    모르는 값이 와도 예외 없이 pending 으로 그린다 — 계측이 HUD 를 죽이면 안 된다.
    """
    t = tree if isinstance(tree, dict) else {}
    nodes = t.get("nodes") if isinstance(t.get("nodes"), dict) else {}
    cq, _, cs = str(t.get("current") or "").partition("/")
    cq = cq if cq in QUADS else None
    cs = cs if (cq and cs in QUADS) else None
    sp = t.get("split_parent") if t.get("split_parent") in QUADS else None
    kids = t.get("children") if isinstance(t.get("children"), dict) else {}

    out_n, out_l = {}, {}
    root = t.get("root")
    out_n["root"] = {"style": root if root in ROOT_STATES else "pending",
                     "count": "", "tag": None, "border": None}
    show_l2 = out_n["root"]["style"] != "done"
    if not show_l2:
        sp = None
    for q in QUADS:
        n = nodes.get(q) if isinstance(nodes.get(q), dict) else {}
        st = n.get("state") if n.get("state") in NODE_STATES else "pending"
        out_n[q] = {"style": st, "count": _cnt(n.get("count")),
                    "tag": l1_tag(q, st, bool(n.get("split"))), "border": None}
        out_l["drop1:" + q] = EDGE_ON if q == cq else (EDGE_DIM if st == "pruned" else EDGE)
    out_l["stem"] = EDGE_ON if cq else EDGE
    for key, a, b in lay["bus1"]:
        out_l[key] = EDGE_ON if (cq and _within(a, b, lay["cx"], lay["c"][cq])) else EDGE
    if sp:
        g = lay["groups"][sp]
        on = cq == sp and cs is not None
        out_l["g:stem2"] = EDGE_ON if on else EDGE
        for key, a, b in g["bus2"]:
            out_l["g:" + key] = EDGE_ON if (on and _within(a, b, g["pc"], g["gc"][cs])) else EDGE
        for s in QUADS:
            k = kids.get(s) if isinstance(kids.get(s), dict) else {}
            st = k.get("state") if k.get("state") in NODE_STATES else "pending"
            border = REJECTED_BORDER if (k.get("rejected") and st == "done") else None
            out_n["sub:" + s] = {"style": st, "count": _cnt(k.get("count")), "tag": None, "border": border}
            out_l["g:drop2:" + s] = EDGE_ON if (on and s == cs) else (EDGE_DIM if st == "pruned" else EDGE)
    return {"nodes": out_n, "lines": out_l, "group": sp, "show_l2": show_l2}
