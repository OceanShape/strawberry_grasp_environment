"""
isaac_sim_hud.py -- harvest status HUD drawn over the Isaac Sim viewport.

ASCII ONLY. Kit's Script Editor renders every non-ASCII character as '?',
so this file (comments, strings, prints) is kept in plain English on purpose.
Korean text that appears on screen comes from pre-rendered PNGs (see below),
never from this source. Project-wide notes stay in Korean elsewhere
(hud/README.md, docs/).

Usage -- same as isaac_sim_script_editor_bridge.py:
  Isaac Sim GUI -> Script Editor -> open this file -> Run.
  Run the bridge script too; order does not matter.
  Running this several times leaves exactly one HUD.

Data: the four nodes (fake_vision / sim_executor_bridge / curobo_planner /
scan_executor) write a snapshot to /tmp/harvest_hud_<role>.json every 0.1 s.
This file only draws. Merging rules live in hud/bus_merge.py.

Layout:
    NODES   * vision  * planner  * control  * scan     green = working now
    ------------------------------------------
    AREA    NW                                         current / target sub-area
    TARGET  3 / 6        PLACED  2
    ------------------------------------------
    PHASE            DESCEND + GRASP                   centered, colored per phase
            [####.......]                              11-cell progress bar
    ------------------------------------------
              HARVEST DONE  5 / 6 (83%)                only after the run ends

"PLACED" is not "success": this repo has no attach that glues the fruit to the
gripper, so it can only count "grasp check passed + released at the tray slot"
(hud/README.md).

Besides the panel, the HUD also marks the working area ON THE BOARD (2026-09-10):
it toggles the quadrant overlays in whiteboard.usd (light-orange grid panes) to
match the AREA value. Home = whole board, quadrant = that pane only.

Korean labels: Kit cannot draw Hangul (Isaac Sim 5.1 prints '?' even in the
Script Editor, and passing a Korean font via style "font" did not help). So
hud/make_labels.py pre-renders the Korean labels to PNG with Pillow; this file
shows them with ui.Image and uses ui.Label only for digits. If hud/labels/ is
missing, it falls back to English text labels. To change label wording, edit
make_labels.py and re-run it -- not this file.
"""
import builtins
import importlib
import json
import os
import sys
import time

# No __file__ in the Script Editor. Pin the path like the bridge script does.
HUD_DIR = os.path.expanduser(os.environ.get(
    "HARVEST_HUD_DIR", "~/strawberry_grasp_environment/strawberry_harvest/scripts/hud"))
if HUD_DIR not in sys.path:
    sys.path.insert(0, HUD_DIR)
# Kit caches modules; reload so edits take effect on the next Run.
for _m in ("status_bus", "bus_merge"):
    if _m in sys.modules:
        importlib.reload(sys.modules[_m])

import omni.kit.app
import omni.ui as ui
import omni.usd
from omni.kit.viewport.utility import get_active_viewport_window
from omni.ui import color as cl
from pxr import Usd, UsdGeom

import bus_merge
import status_bus

# ---- placement ----------------------------------------------------------------
# Panel position: pixels from the viewport's top-left corner (0, 0).
# Larger POS_X moves right, larger POS_Y moves down.
# (To pin it to the right/bottom, swap the Spacer order in _build. Top-left is
#  fixed on purpose so the panel does not drift when the viewport is resized.)
POS_X = 16
POS_Y = 32
PANEL_WIDTH = 440

FRAME_ID = "strawberry_harvest_hud"   # fixed, so reloads do not stack frames
REFRESH_HZ = 10.0
PAD = 16                              # inner padding
ROW_GAP = 10                          # gap between rows
HEAD_W = 56                           # width of the row heads (NODES/AREA/TARGET/PHASE)
LAMP_R = 6

LABEL_DIR = os.path.join(HUD_DIR, "labels")
try:
    with open(os.path.join(LABEL_DIR, "manifest.json"), encoding="utf-8") as _f:
        LABEL_IMG = json.load(_f)          # name -> {"w", "h", "text"}
except Exception:
    LABEL_IMG = {}
LANG = os.environ.get("HARVEST_HUD_LANG", "ko" if LABEL_IMG else "en")


# ---- colors -------------------------------------------------------------------
# Always float RGBA. Packed hex is ABGR inside Kit, which swaps red and blue.
def _rgb(hexv):
    return cl((hexv >> 16 & 255) / 255.0, (hexv >> 8 & 255) / 255.0, (hexv & 255) / 255.0, 1.0)


C_BG = cl(0.07, 0.08, 0.11, 0.82)
C_LINE = cl(1.0, 1.0, 1.0, 0.10)
C_TEXT = _rgb(0xE8EDF5)
C_DIM = _rgb(0x7B8494)
C_OK = _rgb(0x5AD469)
C_BAD = _rgb(0xFF4D5E)
C_ACCENT = _rgb(0xFF6B81)
C_SEG_OFF = cl(1.0, 1.0, 1.0, 0.12)
C_SEG_DONE = cl(1.0, 1.0, 1.0, 0.35)

# Per-phase colors -- same family as the retired status window so nobody has to relearn.
PHASE_COLOR = {
    "IDLE": C_DIM,
    "SCAN_MOVE": _rgb(0x4A9EFF), "DETECT": _rgb(0x00C8C8), "PLAN": _rgb(0xA78BFA),
    "APPROACH": _rgb(0xFFB547), "ENTER": _rgb(0xFFB547), "GRASP": _rgb(0xFF7A3D),
    # DETACH is the key moment of the harvest; give it a color unlike its neighbors.
    "DETACH": _rgb(0xFF3D5C),
    "RETREAT": _rgb(0x4A9EFF), "PLACE": _rgb(0x5AD469), "RETURN": _rgb(0x4A9EFF),
    "DONE": _rgb(0x5AD469),
}

# ---- English fallback text (used only when labels/ is missing) ------------------
EN = {
    "nodes": "NODES", "region": "AREA", "targets": "TARGET",
    "placed": "PLACED", "phase": "PHASE", "final": "HARVEST DONE",
    "node": {"vision": "VISION", "planner": "PLANNER",
             "controller": "CONTROL", "scan": "SCAN"},
    "state": {"IDLE": "IDLE", "SCAN_MOVE": "SCAN MOVE", "DETECT": "DETECT",
              "PLAN": "PLAN", "APPROACH": "APPROACH", "ENTER": "ENTER",
              "GRASP": "DESCEND + GRASP", "DETACH": "DETACH",
              "RETREAT": "RETREAT", "PLACE": "PLACE",
              "RETURN": "RETURN", "DONE": "DONE"},
    "area": {"home": "HOME", "nw": "NW", "ne": "NE", "se": "SE", "sw": "SW"},
}

NODE_ORDER = ["vision", "planner", "controller", "scan"]
BAR_STATES = [s for s in status_bus.SEQUENCE_STATES if s != "IDLE"]   # 11 cells

# ---- board area highlight ------------------------------------------------------
# [2026-09-10] Replaces the quadrant corner rods (cell_markers.usd, removed).
# When the AREA value (home/nw/ne/sw/se) changes, show only the matching overlay
# pane on the board. The panes are whiteboard.usd highlight/{nw,ne,sw,se}: copies
# of the board 2 mm in front, same grid texture times a light-orange tint, so a
# lit pane turns its white cells peach while the black grid lines stay black.
# Home lights all four.
# Visibility is written to the SESSION layer, so saving the stage never bakes the
# last state into the scene file. destroy() turns everything off.
HIGHLIGHT_QUADS = ("nw", "ne", "sw", "se")
HIGHLIGHT_ON = {"home": set(HIGHLIGHT_QUADS),
                "nw": {"nw"}, "ne": {"ne"}, "sw": {"sw"}, "se": {"se"}}
HIGHLIGHT_PATH = "/World/lab_environment/whiteboard/highlight"   # fixed path; search if missing
HIGHLIGHT_LOOKUP_RETRY_SEC = 2.0


class _BoardHighlight:
    def __init__(self):
        self._prims = None          # {quad: Usd.Prim}
        self._region = None         # last region applied
        self._next_lookup = 0.0
        self._warned = False

    def _find(self):
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return None
        found = {}
        for q in HIGHLIGHT_QUADS:
            prim = stage.GetPrimAtPath("%s/%s" % (HIGHLIGHT_PATH, q))
            if not prim or not prim.IsValid():
                prim = None
                # Fallback for a re-arranged scene: accept any prim whose path ends right.
                suffix = "/whiteboard/highlight/%s" % q
                for cand in stage.Traverse():
                    if str(cand.GetPath()).endswith(suffix):
                        prim = cand
                        break
            if prim is None:
                return None
            found[q] = prim
        return found

    def _set(self, stage, quads_on):
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            for q, prim in self._prims.items():
                UsdGeom.Imageable(prim).GetVisibilityAttr().Set(
                    UsdGeom.Tokens.inherited if q in quads_on else UsdGeom.Tokens.invisible)

    def apply(self, region):
        # Reopening the scene kills the cached prims (they belong to the old stage).
        # Then re-find and re-apply even for the same region -- otherwise the overlays
        # stay at the asset default (off) after a reload.
        if self._prims is not None and not all(p.IsValid() for p in self._prims.values()):
            self._prims = None
            self._region = None
        if region == self._region and self._prims is not None:
            return
        try:
            if self._prims is None:
                now = time.time()
                if now < self._next_lookup:
                    return
                self._next_lookup = now + HIGHLIGHT_LOOKUP_RETRY_SEC
                self._prims = self._find()
                if self._prims is None:
                    if not self._warned:
                        self._warned = True
                        print("[hud] board highlight prims not found (%s/*) -- scene not "
                              "loaded yet, or whiteboard.usd is an old version. "
                              "Continuing without the board highlight." % HIGHLIGHT_PATH)
                    return
            stage = omni.usd.get_context().get_stage()
            if stage is None:
                return
            self._set(stage, HIGHLIGHT_ON.get(region, set()))
            self._region = region
        except Exception as exc:                                  # noqa: BLE001
            # The highlight must never take the HUD down. Retry on the next update.
            self._prims = None
            self._region = None
            if not self._warned:
                self._warned = True
                print("[hud] board highlight failed: %r" % (exc,))

    def clear(self):
        try:
            if self._prims is not None:
                stage = omni.usd.get_context().get_stage()
                if stage is not None:
                    self._set(stage, set())
        except Exception:                                          # noqa: BLE001
            pass
        self._prims = None
        self._region = None


def _text(color, size):
    return {"color": color, "font_size": size}


def _label(name, en_text, color, size, width=None):
    """Pre-rendered PNG (ui.Image) when Korean, else text (ui.Label).

    With width the widget occupies that width (row-head alignment); without it,
    only the content width.
    """
    meta = LABEL_IMG.get(name) if LANG == "ko" else None
    if meta:
        with ui.HStack(width=width if width is not None else 0, height=0):
            img = ui.Image(os.path.join(LABEL_DIR, name + ".png"),
                           width=meta["w"], height=meta["h"],
                           fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT,
                           alignment=ui.Alignment.LEFT_CENTER)
            if width is not None:
                ui.Spacer()
        return img
    return ui.Label(en_text, width=width if width is not None else 0,
                    style=_text(color, size), alignment=ui.Alignment.LEFT_CENTER)


class _Swappable:
    """A label whose image (or text) changes with a key -- for the phase and area slots.

    Korean: swaps the source_url to `<prefix>_<key>.png` and matches the width.
    English: just sets .text. Callers do not distinguish the two.
    """

    def __init__(self, prefix, key, en_map, color, size):
        self._prefix = prefix
        self._en = en_map
        self._size = size
        self._img = None
        self._lbl = None
        if LANG == "ko" and (prefix + "_" + key) in LABEL_IMG:
            height = max(v["h"] for k, v in LABEL_IMG.items() if k.startswith(prefix + "_"))
            self._img = ui.Image(self._url(key), width=LABEL_IMG[prefix + "_" + key]["w"],
                                 height=height,
                                 fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT,
                                 alignment=ui.Alignment.CENTER)
        else:
            self._lbl = ui.Label(self._en.get(key, key), width=0, style=_text(color, size))

    def _url(self, key):
        return os.path.join(LABEL_DIR, "%s_%s.png" % (self._prefix, key))

    def set(self, key, color):
        if self._img is not None:
            meta = LABEL_IMG.get("%s_%s" % (self._prefix, key))
            if meta is None:
                return
            self._img.source_url = self._url(key)
            self._img.width = ui.Pixel(meta["w"])
        else:
            self._lbl.text = self._en.get(key, key)
            self._lbl.style = _text(color, self._size)


class HarvestHUD:
    def __init__(self):
        self._frame = None
        self._sub = None
        self._w = {}
        self._seg = []
        self._last_draw = 0.0
        self._hl = _BoardHighlight()
        self._build()
        self._sub = (omni.kit.app.get_app().get_update_event_stream()
                     .create_subscription_to_pop(self._on_update, name="harvest_hud_update"))

    # -- build (once) ------------------------------------------------------------

    def _build(self):
        vp = get_active_viewport_window()
        if vp is None:
            raise RuntimeError("No active viewport window found.")
        self._frame = vp.get_frame(FRAME_ID)
        self._frame.clear()

        with self._frame:
            with ui.VStack():
                ui.Spacer(height=POS_Y)
                with ui.HStack(height=0):
                    ui.Spacer(width=POS_X)
                    with ui.ZStack(width=PANEL_WIDTH):
                        ui.Rectangle(style={"background_color": C_BG, "border_radius": 10})
                        with ui.VStack(height=0):
                            ui.Spacer(height=PAD)
                            with ui.HStack(height=0):
                                ui.Spacer(width=PAD)
                                with ui.VStack(height=0, spacing=ROW_GAP):
                                    self._row_nodes()
                                    self._divider()
                                    self._row_region()
                                    self._row_counters()
                                    self._divider()
                                    self._row_phase()
                                    self._row_bar()
                                    self._row_final()
                                ui.Spacer(width=PAD)
                            ui.Spacer(height=PAD)
                    ui.Spacer()
                ui.Spacer()

    @staticmethod
    def _divider():
        ui.Rectangle(height=1, style={"background_color": C_LINE})

    def _head(self, key):
        _label("head_" + key, EN[key], C_DIM, 15, width=HEAD_W)

    def _row_nodes(self):
        with ui.HStack(height=0):
            self._head("nodes")
            with ui.HStack(height=0, spacing=18):
                for name in NODE_ORDER:
                    with ui.HStack(width=0, height=0, spacing=6):
                        self._w["lamp_" + name] = ui.Circle(
                            radius=LAMP_R, width=LAMP_R * 2 + 2, height=LAMP_R * 2 + 2,
                            size_policy=ui.CircleSizePolicy.FIXED,
                            alignment=ui.Alignment.CENTER,
                            style={"background_color": C_BAD})
                        _label("node_" + name, EN["node"][name], C_TEXT, 15)

    def _row_region(self):
        with ui.HStack(height=0):
            self._head("region")
            self._w["region"] = _Swappable("region", "home", EN["area"], C_TEXT, 24)
            ui.Spacer()

    def _row_counters(self):
        with ui.HStack(height=0):
            self._head("targets")
            self._w["targets"] = ui.Label("0 / 0", width=130, style=_text(C_TEXT, 24))
            _label("head_placed", EN["placed"], C_DIM, 15, width=HEAD_W)
            self._w["placed"] = ui.Label("0", width=0, style=_text(C_OK, 24))

    def _row_phase(self):
        # Phase is centered in the width left of the row head -- same span as the bar below.
        with ui.HStack(height=0):
            self._head("phase")
            ui.Spacer()
            self._w["state"] = _Swappable("state", "IDLE", EN["state"], C_DIM, 34)
            ui.Spacer()

    def _row_bar(self):
        with ui.HStack(height=0):
            ui.Spacer(width=HEAD_W)
            with ui.HStack(height=0, spacing=3):
                for _ in BAR_STATES:
                    self._seg.append(ui.Rectangle(
                        height=8, style={"background_color": C_SEG_OFF, "border_radius": 2}))

    def _row_final(self):
        # Hidden together with its divider -- a lone line before the run ends looks odd.
        with ui.VStack(height=0, spacing=ROW_GAP) as block:
            self._divider()
            with ui.HStack(height=0, spacing=12):
                ui.Spacer()
                _label("final", EN["final"], C_ACCENT, 30)
                self._w["final_num"] = ui.Label("", width=0, style=_text(C_ACCENT, 30))
                ui.Spacer()
        self._w["final"] = block
        block.visible = False

    # -- update (values only; widgets are never rebuilt) ---------------------------

    def _on_update(self, _event):
        now = time.time()
        if (now - self._last_draw) < (1.0 / REFRESH_HZ):
            return
        self._last_draw = now
        try:
            snap, alive = bus_merge.load()
        except Exception:
            return

        for name in NODE_ORDER:
            self._w["lamp_" + name].style = {
                "background_color": C_OK if alive.get(name) else C_BAD}

        self._w["region"].set(snap["region"]["name"], C_TEXT)
        self._hl.apply(snap["region"]["name"])

        tg = snap["targets"]
        self._w["targets"].text = "%d / %d" % (tg["current_index"], tg["total"])
        self._w["placed"].text = str(snap["result"]["succeeded"])

        state = snap["sequence"]["state"]
        color = PHASE_COLOR.get(state, C_TEXT)
        self._w["state"].set(state, color)

        idx = BAR_STATES.index(state) if state in BAR_STATES else -1
        for i, seg in enumerate(self._seg):
            seg.style = {"background_color": (color if i == idx else
                                              C_SEG_DONE if i < idx else C_SEG_OFF),
                         "border_radius": 2}

        finished = bool(snap["result"]["finished"])
        if finished:
            done, total = snap["result"]["succeeded"], tg["total"]
            # Percentage of the target count; total can be 0 (no ripe fruit seen).
            pct = int(round(100.0 * done / total)) if total > 0 else 0
            self._w["final_num"].text = "%d / %d (%d%%)" % (done, total, pct)
        self._w["final"].visible = finished

    # -- teardown ----------------------------------------------------------------

    def destroy(self):
        if self._sub is not None:
            self._sub.unsubscribe()
            self._sub = None
        self._hl.clear()
        if self._frame is not None:
            self._frame.clear()
            self._frame = None
        self._w.clear()
        self._seg.clear()


def install():
    """Any number of Runs from the Script Editor leaves one HUD (bridge-style builtins)."""
    old = getattr(builtins, "harvest_hud", None)
    if old is not None:
        try:
            old.destroy()
        except Exception:
            pass
    # [FIX 2026-09-10] 이 Run 이전에 마지막으로 쓰인 스냅샷(직전 런의 엔딩)은 무시한다.
    # 다시 Run 하면 그 시점이 새 기준이 된다 — 도중에 다시 Run 해도 살아 있는 노드 파일은
    # 0.1초 안에 다시 쓰이므로 곧바로 보인다.
    bus_merge.EPOCH = time.time()
    builtins.harvest_hud = HarvestHUD()
    print("Harvest HUD started (lang=%s, labels=%d)" % (LANG, len(LABEL_IMG)))
    return builtins.harvest_hud


def uninstall():
    old = getattr(builtins, "harvest_hud", None)
    if old is not None:
        old.destroy()
        builtins.harvest_hud = None


install()
