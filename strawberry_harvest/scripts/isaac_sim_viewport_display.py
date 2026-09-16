"""
isaac_sim_viewport_display.py -- everything the harvest run shows on the Isaac Sim viewport.

[2026-09-16] Renamed from isaac_sim_hud.py when the wrist camera inset was added.
A HUD is status drawn over the main view; a camera inset is a second rendered view,
and the board highlight toggles scene prims. The file now holds all three, so it is
named for the viewport; "HUD" stays the name of the status panel inside it. The
hud/ package, HARVEST_HUD_DIR and /tmp/harvest_hud_*.json keep their names: the
probe hooks in the nodes pin that path, and renaming it would edit node code.

  1. status HUD panel (top-left)      -- HarvestHUD, data from the hud/ bus
  2. board area highlight (in scene)  -- _BoardHighlight, driven by the HUD's area value
  3. wrist camera inset (bottom-left) -- _WristCamera, live render, no data

ASCII ONLY. Kit's Script Editor renders every non-ASCII character as '?',
so this file (comments, strings, prints) is kept in plain English on purpose.
Korean text that appears on screen comes from pre-rendered PNGs (see below),
never from this source. Project-wide notes stay in Korean elsewhere
(hud/README.md, docs/).

Usage -- same as isaac_sim_script_editor_bridge.py:
  Isaac Sim GUI -> Script Editor -> open this file -> Run (after the scene is loaded,
  so the wrist camera prim exists).
  Run the bridge script too; order does not matter.
  Running this several times leaves exactly one display.

Data: the four nodes (fake_vision / sim_executor_bridge / curobo_planner /
scan_executor) write a snapshot to /tmp/harvest_hud_<role>.json every 0.1 s.
This file only draws. Merging rules live in hud/bus_merge.py.

Layout (2026-09-11: the AREA / TARGET / PLACED rows were replaced by the tree):
    NODES   * vision  * planner  * control  * scan     green = working now
    ------------------------------------------
    TREE                  [ROOT]                       the scan as a quadtree
            [NW   2] [NE   1] [SE   0] [SW   3]        number = candidates in that cell
            [dir   ] [dir   ] [dir+skip] [dir+split]   2nd line (Korean PNG)
                         [nw 1][ne 1][se 1][sw 0]      children of the latest split;
                                                       folded away when the run ends
    ------------------------------------------
    PHASE            DESCEND + GRASP                   centered, colored per phase
            [####.......]                              11-cell progress bar
    ------------------------------------------
              HARVEST DONE  5 / 6 (83%)                only after the run ends

The tree paints the scan executor's own decisions as they happen (overview
prune, leaf, split, sub-pose fallback); nothing is decided here. Geometry,
colors and wording come from hud/tree_model.py (make_labels.py uses the same
wording). Orange node + path = where the robot is now; green border = finished.

"HARVEST DONE" is not "success": this repo has no attach that glues the fruit
to the gripper, so it can only count "grasp check passed + released at the tray
slot" (hud/README.md).

Besides the panel, the HUD also marks the working area ON THE BOARD (2026-09-10):
it toggles the quadrant overlays in whiteboard.usd (light-orange grid panes) to
match the current quadrant. Home = whole board, quadrant = that pane only.

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
import math
import os
import sys
import time

# No __file__ in the Script Editor. Pin the path like the bridge script does.
HUD_DIR = os.path.expanduser(os.environ.get(
    "HARVEST_HUD_DIR", "~/strawberry_grasp_environment/strawberry_harvest/scripts/hud"))
if HUD_DIR not in sys.path:
    sys.path.insert(0, HUD_DIR)
# Kit caches modules; reload so edits take effect on the next Run.
for _m in ("tree_model", "status_bus", "bus_merge"):
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
import tree_model

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
# [2026-09-16] Color meaning, fixed for the video. RED IS FAILURE ONLY: the dead
# node lamp and the dropped count. Counts that went right are green (C_OK, the
# same green as the PLACE / DONE phases); headings and ratios stay white (C_TEXT).
# The final row was drawn in C_ACCENT (0xFF6B81, pink-red) until this date, which
# read as an error message; that was its only use, so the constant is gone.
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
    "placed": "PLACED", "phase": "PHASE", "final": "HARVEST DONE", "tree": "TREE",
    "final_placed": "PLACED", "final_dropped": "DROPPED",
    "node": {"vision": "VISION", "planner": "PLANNER",
             "controller": "CONTROL", "scan": "SCAN"},
    "state": {"IDLE": "IDLE", "SCAN_MOVE": "SCAN MOVE", "DETECT": "DETECT",
              "PLAN": "PLAN", "APPROACH": "APPROACH", "ENTER": "ENTER",
              "GRASP": "DESCEND + GRASP", "DETACH": "DETACH",
              "RETREAT": "RETREAT", "PLACE": "PLACE",
              "RETURN": "RETURN", "DONE": "DONE"},
    "area": {"home": "HOME", "nw": "NW", "ne": "NE", "se": "SE", "sw": "SW"},
    "cam_title": "WRIST CAM  D455 color render - no detection",
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
# [2026-09-12] Depth 2: while the robot works a sub-cell (tree "current" = "sw/se"),
# only that sub-cell pane (highlight/sw_se, a quarter of the quadrant pane) is lit.
# The 16 sub-cell panes are optional: with an older whiteboard.usd the parent
# quadrant pane is lit instead, so the HUD keeps working before a scene reload.
# Visibility is written to the SESSION layer, so saving the stage never bakes the
# last state into the scene file. destroy() turns everything off.
HIGHLIGHT_QUADS = ("nw", "ne", "sw", "se")
HIGHLIGHT_SUBS = tuple("%s_%s" % (q, s) for q in HIGHLIGHT_QUADS for s in HIGHLIGHT_QUADS)
HIGHLIGHT_ON = {"home": set(HIGHLIGHT_QUADS),
                "nw": {"nw"}, "ne": {"ne"}, "sw": {"sw"}, "se": {"se"}}
HIGHLIGHT_ON.update({"%s/%s" % (q, s): {"%s_%s" % (q, s)}
                     for q in HIGHLIGHT_QUADS for s in HIGHLIGHT_QUADS})


def highlight_key(region, tree):
    """Which area to light: the sub-cell while the robot works one, else the AREA value."""
    try:
        cur = str((tree or {}).get("current") or "")
    except Exception:
        cur = ""
    return cur if cur in HIGHLIGHT_ON and "/" in cur else region
HIGHLIGHT_PATH = "/World/lab_environment/whiteboard/highlight"   # fixed path; search if missing
HIGHLIGHT_LOOKUP_RETRY_SEC = 2.0

# ---- wrist camera inset ----------------------------------------------------------
# [2026-09-16] A small live render from the wrist D455 color camera, bottom-left of the
# viewport -- the sim counterpart of the camera window the real vision node shows.
#
# RENDER ONLY. Nothing is detected from this image: fake_vision publishes the scene's
# ground-truth fruit poses cut by board rectangles (quadrant / sub-cell), not by this
# camera's frustum. So the inset never carries detection marks (boxes, keypoints,
# PICK#) and its title says so. What the inset shows and what fake_vision publishes
# can differ (e.g. a neighbor quadrant's fruit in frame but not published).
#
# [2026-09-16] What IS drawn over the render is the quadrant guide the real vision
# node's window also draws: a centre cross and NW/NE/SW/SE at the crossing, plus the
# current area in the caption. It is a fixed image-space guide, not a detection --
# see CAM_GUIDE_* below for why the centre cross is honest.
# Only the color camera is shown: a render has no D455 minimum depth range
# (docs/d455_min_range.md), so a depth view would look valid where it is unconfirmed.
#
# FOV IS NOT WRITTEN HERE. A USD camera has no FOV attribute; the angle follows from
# focalLength and the apertures: fov = 2 * atan(aperture / (2 * focalLength)).
# robot_assembly.usd already overrides them for this prim (horizontalAperture 3.896 is
# the asset's own value, focalLength 2.34596, verticalAperture 2.922 = 3.896 * 480/640),
# giving 79.41 x 63.83 deg. Target = the real camera's log
# (docs/lab_data/realsense_d455_enumerate.txt, "Color" / 640x480: 79.41 x 63.89 deg;
# the 0.06 deg vertical gap is the log's fx != fy). This file reads the attributes and
# prints the FOV so a drifted scene shows in the console; writing them here would be a
# second source of truth. The log's principal point offset and lens distortion are
# not modeled.
#
# Set HARVEST_WRIST_CAM=0 before launching Isaac Sim to skip the inset (one less render).
CAM_PATH = "/World/robot_assembly/rh_p12_rn_base/rsd455/RSD455/Camera_OmniVision_OV9782_Color"
CAM_SUFFIX = "/rsd455/RSD455/Camera_OmniVision_OV9782_Color"   # fallback search
CAM_RES = (640, 480)            # render texture = the D455 color stream the FOV was fitted to
CAM_LOG_FOV_DEG = (79.41, 63.89)
CAM_FOV_TOL_DEG = 0.2
CAM_POS_X = 16                  # pixels from the viewport's left edge
CAM_MARGIN_BOTTOM = 16          # pixels from the viewport's bottom edge
CAM_WIDTH = 400                 # image width on screen; height follows the aperture ratio
CAM_PAD = 10
CAM_FRAME_ID = "strawberry_harvest_wrist_cam"
CAM_ENABLED = os.environ.get("HARVEST_WRIST_CAM", "1") != "0"

# ---- quadrant guide over the inset --------------------------------------------
# [2026-09-16] The real vision node's window draws a cross and the four quadrant
# names in OpenCV green (0, 255, 0). The inset draws the same guide so the two
# windows read as one structure side by side. Two facts keep it honest:
#
#   * It is a FIXED image-space guide. The executor's real boundary is the board's
#     midlines in robot coordinates (quadrant_filter.BOARD_SUBCELL_*_MID_M,
#     scan_executor._subcell_of_pose). At the overview pose those midlines project to
#     49.3% / 49.3% of this camera's frame and stay axis-aligned
#     (check_wrist_camera_projection.py --joints-deg 88,-94.9,129.9,175.9,-31.3,93.4),
#     so a centre cross sits on the real split to within 1%. At a quadrant pose the
#     same cross is that quadrant's own 2x2 sub-cell split -- the executor's depth-2
#     rule is the parent quadrant's centre lines, so the picture stays the same rule.
#   * Label slots follow the real window as read from the footage (user, 2026-09-16):
#     the four names cluster at the crossing, each on the FAR side of it --
#     NW bottom-right, NE bottom-left, SW top-right, SE top-left. Only
#     CAM_GUIDE_SLOT encodes that; change it if the footage says otherwise.
#
# Colour is pure #00FF00 (captures look yellow-green only from compression) with a
# 1 px black outline so it survives over unripe fruit. Label height is a fraction
# of the inset height, so a bigger inset scales the guide with it. The caption gets
# the current area (HOME / NE / SW/SE ...) from the HUD's own area value, which is
# what matters at a quadrant pose where the crossing has left the frame.
CAM_GUIDE_ON = os.environ.get("HARVEST_WRIST_GUIDE", "1") != "0"
CAM_GUIDE_GREEN = _rgb(0x00FF00)
CAM_GUIDE_OUTLINE = cl(0.0, 0.0, 0.0, 1.0)
CAM_GUIDE_LINE_PX = 1           # green core; the black outline adds 1 px each side
CAM_GUIDE_GAP_PX = 9            # label distance from the crossing (advice: 8-10)
CAM_GUIDE_TEXT_FRAC = 0.05      # label height / inset height (advice: ~5%)
CAM_GUIDE_FONT = "${fonts}/OpenSans-SemiBold.ttf"
# name -> corner of the crossing it sits in: t/b = above/below, l/r = left/right
CAM_GUIDE_SLOT = {"NW": "br", "NE": "bl", "SW": "tr", "SE": "tl"}


class _BoardHighlight:
    def __init__(self):
        self._prims = None          # {quad: Usd.Prim}
        self._region = None         # last region applied
        self._next_lookup = 0.0
        self._warned = False
        self._warned_subs = False

    def _find(self):
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return None
        found = {}
        for q in HIGHLIGHT_QUADS + HIGHLIGHT_SUBS:
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
                if q in HIGHLIGHT_QUADS:
                    return None
                continue            # sub-cell panes are optional (older whiteboard.usd)
            found[q] = prim
        if len(found) == len(HIGHLIGHT_QUADS) and not self._warned_subs:
            self._warned_subs = True
            print("[hud] board highlight: no sub-cell panes (highlight/nw_se ...) -- "
                  "whiteboard.usd predates 2026-09-12 or the scene was not reloaded; "
                  "sub-cells light their parent quadrant.")
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
            on = set(HIGHLIGHT_ON.get(region, set()))
            # Sub-cell pane missing (old asset): light its parent quadrant instead.
            on = {q if q in self._prims else q.split("_")[0] for q in on}
            self._set(stage, on)
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


def _c(rgba):
    """0-255 RGBA tuple from tree_model -> Kit float color."""
    r, g, b, a = rgba
    return cl(r / 255.0, g / 255.0, b / 255.0, a / 255.0)


class _TreeView:
    """Quadtree panel (2026-09-11).

    Built once from tree_model.tree_layout(); every refresh repaints it from
    tree_model.view(). Each layout band is one HStack with Spacers between its
    items, so only the stack primitives the rest of this HUD already uses are
    needed. The four possible 2nd-level groups sit in one ZStack and only the
    group under the latest split quadrant is visible, so the panel height never
    changes during a run. When the traversal is done the whole 2nd-level area
    folds away and the HARVEST DONE row appears in its place.
    """

    def __init__(self, width):
        self._lay = tree_model.tree_layout(width)
        self._lines = {}     # 1st-level edge key -> Rectangle
        self._nodes = {}     # "root" / quadrant -> widgets
        self._groups = {}    # parent quadrant -> {"frame", "lines", "nodes"}
        self._last = {}      # widget key -> last painted value (skip no-op writes)
        with ui.VStack(width=width, height=0):
            for band in self._lay["bands"]:
                self._band(band, self._lines, self._nodes)
            with ui.ZStack(height=self._lay["l2_h"]) as l2_box:
                for q in tree_model.QUADS:
                    grp = {"lines": {}, "nodes": {}}
                    with ui.VStack(height=0) as frame:
                        for band in self._lay["groups"][q]["bands"]:
                            self._band(band, grp["lines"], grp["nodes"])
                    frame.visible = False
                    grp["frame"] = frame
                    self._groups[q] = grp
        self._l2_box = l2_box

    def _band(self, band, lines, nodes):
        h = band["h"]
        with ui.HStack(height=h):
            cursor = 0.0
            for it in band["items"]:
                if it["x"] - cursor > 0.01:
                    ui.Spacer(width=it["x"] - cursor)
                kind, key, w = it["kind"], it["key"], it["w"]
                if kind == "line":
                    lines[key] = ui.Rectangle(
                        width=w, height=h, style={"background_color": _c(tree_model.EDGE)})
                elif kind == "root":
                    nodes["root"] = self._node(w, h, "ROOT", 13, center=True)
                elif kind == "l1":
                    nodes[key] = self._node(w, h, key.upper(), 15, tag_key="dir_" + key)
                else:
                    nodes[key] = self._node(w, h, key, 13)
                cursor = it["x"] + w
            ui.Spacer()

    @staticmethod
    def _rect_style(style, border):
        return {"background_color": _c(style["fill"]),
                "border_color": _c(border or style["border"]),
                "border_width": style["bw"], "border_radius": 6}

    @classmethod
    def _node(cls, w, h, name, size, center=False, tag_key=None):
        st = tree_model.NODE_STYLE["pending"]
        out = {"size": size}
        with ui.ZStack(width=w, height=h):
            out["rect"] = ui.Rectangle(style=cls._rect_style(st, None))
            if center:
                out["name"] = ui.Label(name, alignment=ui.Alignment.CENTER,
                                       style=_text(_c(st["name"]), size))
            else:
                pad = 9 if tag_key else 7
                with ui.VStack():
                    if tag_key:
                        ui.Spacer(height=5)
                    with ui.HStack(height=20 if tag_key else h):
                        ui.Spacer(width=pad)
                        out["name"] = ui.Label(name, width=0, alignment=ui.Alignment.LEFT_CENTER,
                                               style=_text(_c(st["name"]), size))
                        ui.Spacer()
                        out["count"] = ui.Label("", width=0, alignment=ui.Alignment.RIGHT_CENTER,
                                                style=_text(_c(st["count"]), size))
                        ui.Spacer(width=pad)
                    if tag_key:
                        with ui.HStack(height=16):
                            ui.Spacer(width=pad)
                            out["tag"] = _Swappable("tree", tag_key, tree_model.TAG_EN,
                                                    C_DIM, tree_model.TAG_SIZE)
                            ui.Spacer()
                    ui.Spacer()
        return out

    def _put(self, key, value, apply):
        if self._last.get(key) != value:
            apply(value)
            self._last[key] = value

    def update(self, tree):
        v = tree_model.view(tree, self._lay)
        group = v["group"]
        self._put(("l2",), v["show_l2"], lambda on: setattr(self._l2_box, "visible", on))
        for q, grp in self._groups.items():
            self._put(("vis", q), group == q,
                      lambda on, f=grp["frame"]: setattr(f, "visible", on))
        for key, nv in v["nodes"].items():
            if key.startswith("sub:"):
                wid = self._groups[group]["nodes"].get(key[4:]) if group else None
                wkey = ("node", group, key)
            else:
                wid, wkey = self._nodes.get(key), ("node", key)
            if wid is not None:
                self._paint(wkey, wid, nv)
        for key, col in v["lines"].items():
            if key.startswith("g:"):
                rect = self._groups[group]["lines"].get(key[2:]) if group else None
                wkey = ("line", group, key)
            else:
                rect, wkey = self._lines.get(key), ("line", key)
            if rect is not None:
                self._put(wkey, col, lambda c, r=rect: setattr(
                    r, "style", {"background_color": _c(c)}))

    def _paint(self, wkey, wid, nv):
        st = tree_model.NODE_STYLE[nv["style"]]
        size = wid["size"]
        self._put(wkey + ("rect",), (nv["style"], nv["border"]), lambda _v: setattr(
            wid["rect"], "style", self._rect_style(st, nv["border"])))
        self._put(wkey + ("name",), nv["style"], lambda _v: setattr(
            wid["name"], "style", _text(_c(st["name"]), size)))
        if "count" in wid:
            self._put(wkey + ("count",), (nv["style"], nv["count"]), lambda _v: (
                setattr(wid["count"], "text", nv["count"]),
                setattr(wid["count"], "style", _text(_c(st["count"]), size))))
        if "tag" in wid and nv["tag"]:
            self._put(wkey + ("tag",), nv["tag"], lambda k: wid["tag"].set(k, C_DIM))


class HarvestHUD:
    def __init__(self):
        self._frame = None
        self._sub = None
        self._w = {}
        self._seg = []
        self._last_draw = 0.0
        self._hl = _BoardHighlight()
        self._tree = None
        self._tree_warned = False
        self.region_listener = None          # wrist camera caption follows the area (2026-09-16)
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
                                    self._row_tree()
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

    def _row_tree(self):
        # Replaces the AREA / TARGET / PLACED rows (2026-09-11): the lit node shows the
        # area down to the sub-cell, and every node carries its own candidate count.
        with ui.HStack(height=0):
            with ui.VStack(width=HEAD_W):
                self._head("tree")
                ui.Spacer()
            self._tree = _TreeView(PANEL_WIDTH - 2 * PAD - HEAD_W)

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
        # [T4c 2026-09-15] Second line: 'placed n / dropped m' (Korean PNG labels). A fruit that was detached but
        # whose transfer plan the planner rejected is released where it stands and falls
        # (bridge drop physics); the count comes from harvest_probe (result.dropped) so the
        # ending states the failure instead of only the placed/total ratio.
        # [2026-09-16] Colors: heading and ratio white, 'placed' green, 'dropped' red
        # -- see the color block at the top. Nothing here means "success" (hud/README.md).
        with ui.VStack(height=0, spacing=ROW_GAP) as block:
            self._divider()
            with ui.HStack(height=0, spacing=12):
                ui.Spacer()
                _label("final", EN["final"], C_TEXT, 30)
                self._w["final_num"] = ui.Label("", width=0, style=_text(C_TEXT, 30))
                ui.Spacer()
            with ui.HStack(height=0, spacing=8):
                ui.Spacer()
                _label("final_placed", EN["final_placed"], C_OK, 20)
                self._w["final_placed_num"] = ui.Label("", width=0, style=_text(C_OK, 20))
                ui.Spacer(width=18)
                _label("final_dropped", EN["final_dropped"], C_BAD, 20)
                self._w["final_dropped_num"] = ui.Label("", width=0, style=_text(C_BAD, 20))
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

        area = highlight_key(snap["region"]["name"], snap.get("tree"))
        self._hl.apply(area)
        if self.region_listener is not None:
            try:
                self.region_listener(area)
            except Exception:                                      # noqa: BLE001
                pass                       # the caption must never take the HUD down
        self._update_tree(snap.get("tree"))

        tg = snap["targets"]

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
            self._w["final_placed_num"].text = "%d" % done
            self._w["final_dropped_num"].text = "%d" % int(snap["result"].get("dropped", 0))
        self._w["final"].visible = finished

    def _update_tree(self, tree):
        # The tree must never take the HUD down -- same rule as the board highlight.
        try:
            if self._tree is not None:
                self._tree.update(tree)
        except Exception as exc:
            if not self._tree_warned:
                self._tree_warned = True
                print("[hud] tree update failed: %r" % (exc,))

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
        self._tree = None


def find_wrist_camera(stage):
    prim = stage.GetPrimAtPath(CAM_PATH)
    if prim and prim.IsValid() and prim.IsA(UsdGeom.Camera):
        return prim
    for cand in stage.Traverse():
        if str(cand.GetPath()).endswith(CAM_SUFFIX) and cand.IsA(UsdGeom.Camera):
            return cand
    return None


def camera_fov_deg(cam):
    """(hfov, vfov, horizontal aperture, vertical aperture) of a UsdGeom.Camera."""
    ha = float(cam.GetHorizontalApertureAttr().Get())
    va = float(cam.GetVerticalApertureAttr().Get())
    f = float(cam.GetFocalLengthAttr().Get())
    return (math.degrees(2.0 * math.atan(ha / (2.0 * f))),
            math.degrees(2.0 * math.atan(va / (2.0 * f))), ha, va)


class _WristCamera:
    """Bottom-left inset: a ViewportWidget rendering the wrist camera into the viewport's frame.

    It sits in the main viewport's own frame (like the HUD panel), so it moves and gets
    recorded with the viewport, with no extra window title bar or viewport menus.
    """

    def __init__(self):
        self._frame = None
        self._widget = None
        self._region_lbl = None
        self._region_text = None

    def build(self, vp_window):
        """True when the inset is on screen."""
        stage = omni.usd.get_context().get_stage()
        prim = find_wrist_camera(stage) if stage is not None else None
        if prim is None:
            print("[wrist_cam] camera prim not found (%s) -- load the scene, then Run this "
                  "script again. Continuing without the inset." % CAM_PATH)
            return False
        hfov, vfov, ha, va = camera_fov_deg(UsdGeom.Camera(prim))
        print("[wrist_cam] %s  fov %.2f x %.2f deg, aperture ratio %.4f  "
              "(D455 color 640x480 log: %.2f x %.2f deg, 640/480 = %.4f)"
              % (prim.GetPath(), hfov, vfov, ha / va, CAM_LOG_FOV_DEG[0], CAM_LOG_FOV_DEG[1],
                 CAM_RES[0] / float(CAM_RES[1])))
        if (abs(hfov - CAM_LOG_FOV_DEG[0]) > CAM_FOV_TOL_DEG
                or abs(vfov - CAM_LOG_FOV_DEG[1]) > CAM_FOV_TOL_DEG
                or abs(ha / va - CAM_RES[0] / float(CAM_RES[1])) > 0.005):
            print("[wrist_cam] WARNING: camera FOV/aspect differs from the D455 log. The inset "
                  "still shows; fix robot_assembly.usd (focalLength / apertures), not this file.")

        from omni.kit.widget.viewport import ViewportWidget

        img_h = int(round(CAM_WIDTH * va / ha))
        self._frame = vp_window.get_frame(CAM_FRAME_ID)
        self._frame.clear()
        with self._frame:
            with ui.VStack():
                ui.Spacer()
                with ui.HStack(height=0):
                    ui.Spacer(width=CAM_POS_X)
                    with ui.ZStack(width=CAM_WIDTH + 2 * CAM_PAD):
                        ui.Rectangle(style={"background_color": C_BG, "border_radius": 10})
                        with ui.VStack(height=0):
                            ui.Spacer(height=CAM_PAD)
                            with ui.HStack(height=0):
                                ui.Spacer(width=CAM_PAD)
                                _label("cam_title", EN["cam_title"], C_DIM, 15)
                                if CAM_GUIDE_ON:
                                    # '   AREA NE' -- the HUD's area value, set by set_region()
                                    ui.Spacer(width=18)
                                    _label("head_region", EN["region"], C_DIM, 15)
                                    ui.Spacer(width=5)
                                    self._region_lbl = ui.Label(
                                        "", width=0, style=_text(CAM_GUIDE_GREEN, 15),
                                        alignment=ui.Alignment.LEFT_CENTER)
                                ui.Spacer()
                            ui.Spacer(height=6)
                            with ui.HStack(height=img_h):
                                ui.Spacer(width=CAM_PAD)
                                with ui.ZStack(width=CAM_WIDTH, height=img_h):
                                    self._widget = ViewportWidget(
                                        camera_path=str(prim.GetPath()), resolution=CAM_RES,
                                        width=CAM_WIDTH, height=img_h)
                                    if CAM_GUIDE_ON:
                                        self._build_guide(CAM_WIDTH, img_h)
                                ui.Spacer(width=CAM_PAD)
                            ui.Spacer(height=CAM_PAD)
                    ui.Spacer()
                ui.Spacer(height=CAM_MARGIN_BOTTOM)
        print("[wrist_cam] inset on: %dx%d render shown at %dx%d, guide %s"
              % (CAM_RES + (CAM_WIDTH, img_h, "on" if CAM_GUIDE_ON else "off")))
        return True

    # -- quadrant guide ------------------------------------------------------------

    @staticmethod
    def _build_guide(w, h):
        """Centre cross + NW/NE/SW/SE at the crossing, drawn on top of the render."""
        cx, cy = w // 2, h // 2
        core = CAM_GUIDE_LINE_PX
        edge = core + 2                               # black outline, 1 px each side
        size = max(12, int(round(h * CAM_GUIDE_TEXT_FRAC)))
        box = size * 3                                # label anchor box (text-aligned inside)
        gap = CAM_GUIDE_GAP_PX

        def bar(x, y, bw, bh, color):
            with ui.Placer(offset_x=x, offset_y=y):
                ui.Rectangle(width=bw, height=bh, style={"background_color": color})

        # lines: outline first, green core on top
        bar(cx - edge // 2, 0, edge, h, CAM_GUIDE_OUTLINE)
        bar(0, cy - edge // 2, w, edge, CAM_GUIDE_OUTLINE)
        bar(cx - core // 2, 0, core, h, CAM_GUIDE_GREEN)
        bar(0, cy - core // 2, w, core, CAM_GUIDE_GREEN)

        # labels: the anchor box touches the crossing at (gap, gap); alignment inside
        # the box pushes the text into the corner nearest the crossing.
        anchor = {
            "tl": (cx - gap - box, cy - gap - box, ui.Alignment.RIGHT_BOTTOM),
            "tr": (cx + gap,       cy - gap - box, ui.Alignment.LEFT_BOTTOM),
            "bl": (cx - gap - box, cy + gap,       ui.Alignment.RIGHT_TOP),
            "br": (cx + gap,       cy + gap,       ui.Alignment.LEFT_TOP),
        }
        for name, slot in CAM_GUIDE_SLOT.items():
            x, y, align = anchor[slot]
            # 1 px black outline = the same text four times, offset, under the green one
            for dx, dy, color in ((-1, 0, CAM_GUIDE_OUTLINE), (1, 0, CAM_GUIDE_OUTLINE),
                                  (0, -1, CAM_GUIDE_OUTLINE), (0, 1, CAM_GUIDE_OUTLINE),
                                  (0, 0, CAM_GUIDE_GREEN)):
                with ui.Placer(offset_x=x + dx, offset_y=y + dy):
                    ui.Label(name, width=box, height=box, alignment=align,
                             style={"color": color, "font_size": size,
                                    "font": CAM_GUIDE_FONT})

    def set_region(self, key):
        """Caption area text from the HUD's highlight key: 'home' -> HOME, 'sw/se' -> SW/SE."""
        if self._region_lbl is None:
            return
        text = "HOME" if not key or key == "home" else str(key).upper()
        if text != self._region_text:
            self._region_text = text
            self._region_lbl.text = text

    def destroy(self):
        # ViewportWidget does not release its render texture by itself -- destroy it first.
        if self._widget is not None:
            try:
                self._widget.destroy()
            except Exception:                                      # noqa: BLE001
                pass
            self._widget = None
        if self._frame is not None:
            self._frame.clear()
            self._frame = None


class ViewportDisplay:
    """HUD panel (+ the board highlight it drives) and the wrist camera inset.

    The inset must never take the HUD down -- same rule as the highlight and the tree.
    """

    def __init__(self):
        self.hud = HarvestHUD()
        self.cam = None
        if not CAM_ENABLED:
            print("[wrist_cam] HARVEST_WRIST_CAM=0 -- inset skipped")
            return
        cam = _WristCamera()
        try:
            if cam.build(get_active_viewport_window()):
                self.cam = cam
                self.hud.region_listener = cam.set_region
        except Exception as exc:                                   # noqa: BLE001
            cam.destroy()
            print("[wrist_cam] inset failed, HUD continues: %r" % (exc,))

    def destroy(self):
        if self.cam is not None:
            self.cam.destroy()
            self.cam = None
        self.hud.destroy()


# builtins names this file has used. harvest_hud: isaac_sim_hud.py before the 2026-09-16
# rename, so a Kit session that ran the old file still ends up with one display.
_BUILTIN_NAMES = ("harvest_display", "harvest_hud")


def uninstall():
    for name in _BUILTIN_NAMES:
        old = getattr(builtins, name, None)
        if old is not None:
            try:
                old.destroy()
            except Exception:                                      # noqa: BLE001
                pass
            setattr(builtins, name, None)


def install():
    """Any number of Runs from the Script Editor leaves one display (bridge-style builtins)."""
    uninstall()
    # [FIX 2026-09-10] Ignore snapshots written before this Run (the previous run's ending).
    # This Run becomes the new baseline; live node files are rewritten within 0.1 s, so a
    # Run in the middle of a pipeline shows up right away.
    bus_merge.EPOCH = time.time()
    builtins.harvest_display = ViewportDisplay()
    print("Viewport display started (HUD lang=%s, labels=%d, wrist camera=%s)"
          % (LANG, len(LABEL_IMG), "on" if builtins.harvest_display.cam else "off"))
    return builtins.harvest_display


install()


