"""
isaac_sim_viewport_display.py -- everything the harvest run shows on the Isaac Sim viewport.

[2026-09-16] Renamed from isaac_sim_hud.py when the wrist camera inset was added.
A HUD is status drawn over the main view; a camera inset is a second rendered view,
and the board highlight toggles scene prims. The file now holds all three, so it is
named for the viewport; "HUD" stays the name of the status panel inside it. The
hud/ package, HARVEST_HUD_DIR and /tmp/harvest_hud_*.json keep their names: the
probe code added to the nodes pins that path, and renaming it would edit node code.

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

Layout (2026-09-11: the AREA / TARGET / PLACED rows were replaced by the tree;
2026-09-17: TARGET / NON-TARGET counts and the result bar added):
    NODES   * vision  * planner  * control  * scan     green = working now
    TARGET 8 / NON-TARGET 4                            always; counted from the scene
    ------------------------------------------
    TREE                  [ROOT]                       the scan as a quadtree
            [NW   2] [NE   1] [SE   0] [SW   3]        number = candidates in that cell
                                                       (first look at the quadrant pose;
                                                       rescans after picks keep it, 09-19)
            [dir   ] [dir   ] [dir+skip] [dir+split]   2nd line (Korean PNG)
                         [nw 1][ne 1][se 1][sw 0]      children of the latest split;
                                                       emptied when the run finishes
                                                       (its height stays, 2026-09-19)
    ------------------------------------------
    PHASE            DESCEND + GRASP                   centered, colored per phase
            [####.......]                              11-cell progress bar
    ------------------------------------------
              HARVEST IN PROGRESS                      one row, always (2026-09-19): NOT STARTED
          (or) HARVEST DONE  5 / 8 (62%)               / IN PROGRESS / STOPPED / DONE + counts
    [##|##|##|##|  |  ]                                result bar, one cell per target, always
      PLACED 4   PLACE FAILED 1   DETACH FAILED 0      legend, zeros shown

The tree paints the scan executor's own decisions as they happen (overview
prune, leaf, split, sub-pose fallback); nothing is decided here. Geometry,
colors and wording come from hud/tree_model.py (make_labels.py uses the same
wording). Cyan node + path = where the robot is now (same cyan as the board's lit
border, 2026-09-16); green border = finished. [2026-09-19] While the robot is in a
quadrant, every node off its path is drawn at 35% opacity -- faded, never removed.
At home (overview scan, return at the end) nothing is faded.

"HARVEST DONE" is not "success": this repo has no attach that glues the fruit
to the gripper, so it can only count "grasp check passed + released at the tray
slot" (hud/README.md).

[2026-09-19] The HARVEST line above the result bar is shown the whole run (user
request). While it was hidden, the phase bar and the result bar sat one divider apart
and read as one block. Before the scan node starts a run it says NOT STARTED, during
the run IN PROGRESS (both dim); a finished run says DONE + counts, a run that ended
without the finish call (scan move failed, exception) says STOPPED (both white) --
harvest_key() picks which. All four are 34 px, so the line itself never changes height.
When a run finishes the tree's 2nd level is emptied but keeps its 54 px (user,
2026-09-19; until then it folded away, 2026-09-12, and the DONE row appearing at that
same moment took most of the freed height). With the HARVEST line always there, a fold
would have moved the phase bar, the result bar and the legend up 54 px at the finish;
now no row on the panel changes position from the first frame to the last.

[2026-09-17] Result bar. The number of cells is the number of TARGET fruit counted in
the open scene (hud/scene_fruit.py, the bridge's own publish filter) -- never a fixed
number. Each pick that reaches a result paints the next cell: green = placed, red =
place failed, amber = detach failed (rules, wording and colors: hud/result_bar.py; the
planner-side probe appends the results, bus key result.outcomes). [2026-09-18] Amber
means the pick started its straight entry but ended without ever calling the tray place
executor -- any step between the straight entry and the place call failed (or run()
raised); judged once when the executor's run() ends, and skipped when marker place is
disabled. A target that never reached the pre-approach pose (all grasp candidates
IK-failed, pre-approach spline failed, guard skip) leaves its cell grey. The stage label for
DETACH is "PULL" (Korean "dang-gim") so the Korean word for detach ("bun-ri") is not
used twice on screen. The legend
under the bar replaces the old second line of the ending (placed n / dropped m) and is
shown the whole run, zeros included. result.dropped is still counted on the bus (it
matches the Kit bridge's dropped=n) but is no longer drawn.

Besides the panel, the HUD also marks the working area ON THE BOARD (2026-09-10):
it toggles the quadrant overlays in whiteboard.usd (cyan glowing borders) to
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
for _m in ("tree_model", "result_bar", "scene_fruit", "status_bus", "bus_merge"):
    if _m in sys.modules:
        importlib.reload(sys.modules[_m])

import omni.kit.app
import omni.ui as ui
import omni.usd
from omni.kit.viewport.utility import get_active_viewport_window
from omni.ui import color as cl
from pxr import Usd, UsdGeom

import bus_merge
import result_bar
import scene_fruit
import status_bus
import tree_model

# ---- placement ----------------------------------------------------------------
# Panel position: pixels from the viewport's top-left corner (0, 0).
# Larger POS_X moves right, larger POS_Y moves down.
# (To pin it to the right/bottom, swap the Spacer order in _build. Top-left is
#  fixed on purpose so the panel does not drift when the viewport is resized.)
POS_X = 16
POS_Y = 32
PANEL_WIDTH = 440                     # 23% of 1920 -- the price of 24 px text in a 4-column tree

FRAME_ID = "strawberry_harvest_hud"   # fixed, so reloads do not stack frames
REFRESH_HZ = 10.0
PAD = 16                              # inner padding
ROW_GAP = 12                          # gap between rows
HEAD_W = 60                           # width of the row heads (NODES/TREE/PHASE)
LAMP_R = 8
# [2026-09-16 S5] Text sizes for a 1080p recording played at ~60% width on a slide:
# body 24, row heads 22, key numbers (placed / dropped) 32, phase 40, done title 34,
# tree 24 / 20 / tags 18, camera caption 20. make_labels.py renders the Korean PNGs at
# the same sizes -- change both. The panel stays 440 wide; the tree's four columns
# are what set the floor.

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
# [2026-09-17] The ending's second line became the result bar legend: placed green,
# place failed red (still failure-only), detach failed amber so the two failures differ.
# Those three colors live in hud/result_bar.py (make_labels.py uses the same values).
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
    "placed": "PLACED", "phase": "PHASE", "tree": "TREE",
    "harvest": {"idle": "HARVEST NOT STARTED", "running": "HARVEST IN PROGRESS",
                "stopped": "HARVEST STOPPED", "done": "HARVEST DONE"},
    "final_placed": result_bar.LABEL_EN["placed"],
    "final_dropped": result_bar.LABEL_EN["place_failed"],
    "final_detach_failed": result_bar.LABEL_EN["detach_failed"],
    "count_targets": "TARGET", "count_non_targets": "NON-TARGET",
    "node": {"vision": "VISION", "planner": "PLANNER",
             "controller": "CONTROL", "scan": "SCAN"},
    "state": {"IDLE": "IDLE", "SCAN_MOVE": "SCAN MOVE", "DETECT": "DETECT",
              "PLAN": "PLAN", "APPROACH": "APPROACH", "ENTER": "ENTER",
              "GRASP": "DESCEND + GRASP", "DETACH": "PULL",
              "RETREAT": "RETREAT", "PLACE": "PLACE",
              "RETURN": "RETURN", "DONE": "DONE"},
    "area": {"home": "HOME", "nw": "NW", "ne": "NE", "se": "SE", "sw": "SW"},
    "cam_title": "GRIPPER CAM  D455 render - no detection",
}

NODE_ORDER = ["vision", "planner", "controller", "scan"]
SCENE_POLL_SEC = 1.0          # recount the scene's fruit prims at most this often (cheap: /World children)
# legend: result key -> label PNG key (the PNG keys predate the result bar and stay as widget keys)
LEGEND_LABEL = {"placed": "final_placed", "place_failed": "final_dropped",
                "detach_failed": "final_detach_failed"}
BAR_STATES = [s for s in status_bus.SEQUENCE_STATES if s != "IDLE"]   # 11 cells
# [2026-09-19] HARVEST line: not started / in progress are status notes (dim); the two end
# states are white. make_labels.py HARVEST_KO bakes the same colors into the Korean PNGs.
HARVEST_COLOR = {"idle": C_DIM, "running": C_DIM, "stopped": C_TEXT, "done": C_TEXT}


def harvest_key(snap):
    """Which HARVEST line to show. All values come from the scan node's snapshot:
      done     result.finished (probe, after _finish_scan_sequence returns)
      stopped  run.ended_at without finished -- the scan thread ended some other way
               (the probe stamps ended_at however _scan_sequence_run ends, 2026-09-19)
      running  run.started_at (status_bus.reset() at the start of _scan_sequence_run)
      idle     none of these (no trigger yet, or a fresh scan node)
    A new trigger resets run and finished, so the line goes back to running. A scan node
    that dies mid-run leaves its last snapshot, so the line stays running (its lamp goes red).
    Nodes started before ended_at existed never write it: an aborted run then stays running."""
    if snap["result"]["finished"]:
        return "done"
    run = snap.get("run")
    if not isinstance(run, dict) or not run.get("started_at"):
        return "idle"
    return "stopped" if run.get("ended_at") else "running"


# ---- board area highlight ------------------------------------------------------
# [2026-09-10] Replaces the quadrant corner rods (cell_markers.usd, removed).
# When the AREA value (home/nw/ne/sw/se) changes, show only the matching overlay
# pane on the board. The panes are whiteboard.usd highlight/{nw,ne,sw,se}.
# [2026-09-16 S6] They are now rectangular RINGS, not filled panes: a cyan (#38BDF8)
# emissive border 20 mm wide sitting exactly on the board's black grid lines, so a
# lit quadrant reads as "its grid lines turned cyan" while the fruit inside keeps
# its own colour. Generated by scene_tools/gen_board_highlight.py; prim names are
# unchanged, so this file only toggles visibility. Home lights all four.
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
CAM_WIDTH = 480                 # image width on screen (25% of 1920); height follows the aperture ratio (480x360)
CAM_PAD = 10
CAM_FRAME_ID = "strawberry_harvest_wrist_cam"
CAM_ENABLED = os.environ.get("HARVEST_WRIST_CAM", "1") != "0"

# ---- quadrant guide over the inset --------------------------------------------
# [2026-09-16] The real vision node's window draws a cross and the four quadrant
# names over its camera view. The inset draws the same guide so the two windows read
# as one structure side by side. Two facts keep it honest:
#
#   * It is a FIXED image-space guide. The executor's real boundary is the board's
#     midlines in robot coordinates (quadrant_filter.BOARD_SUBCELL_*_MID_M,
#     scan_executor._subcell_of_pose). At the overview pose those midlines project to
#     49.3% / 49.3% of this camera's frame and stay axis-aligned
#     (check_wrist_camera_projection.py --joints-deg 88,-94.9,129.9,175.9,-31.3,93.4),
#     so a centre cross sits on the real split to within 1%. At a quadrant pose the
#     same cross is that quadrant's own 2x2 sub-cell split -- the executor's depth-2
#     rule is the parent quadrant's centre lines, so the picture stays the same rule.
#   * Each name sits in the OUTER corner of its own quadrant, which is where that
#     quadrant actually is: at the overview pose the camera's right is world +X and
#     its up is world +Z, so NW (board -X, +Z) lands top-left, NE top-right, SW
#     bottom-left, SE bottom-right -- the corners check_wrist_camera_projection.py
#     prints for the board corners (NW 5.9% / 7.3% ... SE 89.2% / 88.0%).
#     (Until 2026-09-16 the four names were clustered at the crossing, each on the
#     far side of it; on screen that read as the board being mirrored.)
#
# Colour (2026-09-16, user): a softer green #4ADE80. The cross is 2 px at 70% alpha
# with no outline -- it marks a boundary, it should not compete with the fruit. The
# names keep a 1 px black outline because light green on the white board is otherwise
# hard to read. Label height is a fraction of the inset height, so a bigger inset
# scales the guide with it.
#
# The guide fades out while the arm is picking (CAM_GUIDE_STATES): reading which
# quadrant the view belongs to matters during the scan, and during the approach the
# cross would just sit across the fruit. The caption keeps the current area at all
# times, which is what matters once the crossing has left the frame.
CAM_GUIDE_ON = os.environ.get("HARVEST_WRIST_GUIDE", "1") != "0"
CAM_GUIDE_GREEN = _rgb(0x4ADE80)                     # caption area value (no fade)
CAM_GUIDE_LINE_RGBA = (0.290, 0.871, 0.502, 0.70)    # #4ADE80, 70% -- the cross
CAM_GUIDE_TEXT_RGBA = (0.290, 0.871, 0.502, 1.0)     # #4ADE80 -- the four names
CAM_GUIDE_OUTLINE_RGBA = (0.0, 0.0, 0.0, 0.85)       # 1 px outline under the names only
CAM_GUIDE_LINE_PX = 2
CAM_GUIDE_MARGIN_PX = 10        # label distance from the inset edge
CAM_GUIDE_TEXT_FRAC = 0.065     # label height / inset height (480x360 -> 23 px)
CAM_GUIDE_FONT = "${fonts}/OpenSans-SemiBold.ttf"
CAM_GUIDE_FADE_SEC = 0.3        # phase change -> guide fades in / out over this long
#: Sequence states that keep the guide up -- the arm is at (or returning to) a scan
#: pose and the view is the board. Everything else is the pick, where it fades out.
CAM_GUIDE_STATES = ("IDLE", "SCAN_MOVE", "DETECT", "PLAN", "RETURN", "DONE")
# name -> the corner of the inset it sits in: t/b = top/bottom, l/r = left/right
CAM_GUIDE_SLOT = {"NW": "tl", "NE": "tr", "SW": "bl", "SE": "br"}


def _fade(rgba, alpha):
    """Guide colour at a fade level. rgba is a plain float tuple, alpha 0..1."""
    r, g, b, a = rgba
    return cl(r, g, b, a * alpha)


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
    """A label whose image (or text) changes with a key -- for the phase, tree-tag and HARVEST slots.

    Korean: swaps the source_url to `<prefix>_<key>.png` and matches the width. A key
    with no PNG hides the image (the tree's leaf tag has no second line since S5).
    English: just sets .text. Callers do not distinguish the two.
    """

    def __init__(self, prefix, key, en_map, color, size):
        self._prefix = prefix
        self._en = en_map
        self._size = size
        self._img = None
        self._lbl = None
        mine = {k: v for k, v in LABEL_IMG.items() if k.startswith(prefix + "_")}
        if LANG == "ko" and mine:
            first = next(iter(mine))
            self._img = ui.Image(self._url(first[len(prefix) + 1:]), width=mine[first]["w"],
                                 height=max(v["h"] for v in mine.values()),
                                 fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT,
                                 alignment=ui.Alignment.CENTER)
        else:
            self._lbl = ui.Label(self._en.get(key, key), width=0, style=_text(color, size))
        self.set(key, color)

    def _url(self, key):
        return os.path.join(LABEL_DIR, "%s_%s.png" % (self._prefix, key))

    def set(self, key, color, alpha=None):
        """alpha (0..1) fades the Korean PNG through the image tint -- the tree dims nodes
        off the robot's path (2026-09-19). None leaves the tint alone (phase label).
        The English label takes its fade through color instead."""
        if self._img is not None:
            if alpha is not None:
                self._img.style = {"color": cl(1.0, 1.0, 1.0, alpha)}
            meta = LABEL_IMG.get("%s_%s" % (self._prefix, key))
            if meta is None:
                self._img.visible = False
                return
            self._img.visible = True
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
    changes during a run. When the traversal is done every 2nd-level group is hidden
    (tree_model.view returns group None) but the ZStack keeps its fixed height, so
    that area is left empty and nothing below it moves (2026-09-19, user; it used to
    fold away). The HARVEST line below the phase bar turns to DONE at the same time.

    [2026-09-19] Nodes off the robot's path are drawn faded (tree_model.DIM_ALPHA on
    every alpha: fill, border, text, second-line PNG tint). Nothing is removed, so the
    first-level skip / split / done marks stay on screen in every frame (the 2nd-level
    row still shows only the latest split quadrant's cells, as before). The rule and the
    colors live in tree_model (view() sets "dim", node_paint() returns the colors);
    this class only paints them.
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
            # Fixed height: hidden groups (before the first split, after the run) leave it empty.
            with ui.ZStack(height=self._lay["l2_h"]):
                for q in tree_model.QUADS:
                    grp = {"lines": {}, "nodes": {}}
                    with ui.VStack(height=0) as frame:
                        for band in self._lay["groups"][q]["bands"]:
                            self._band(band, grp["lines"], grp["nodes"])
                    frame.visible = False
                    grp["frame"] = frame
                    self._groups[q] = grp

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
                    nodes["root"] = self._node(w, h, "ROOT", 20, center=True)
                elif kind == "l1":
                    nodes[key] = self._node(w, h, key.upper(), 24, tag_key="dir_" + key)
                else:
                    nodes[key] = self._node(w, h, key, 20)
                cursor = it["x"] + w
            ui.Spacer()

    @staticmethod
    def _rect_style(paint):
        return {"background_color": _c(paint["fill"]),
                "border_color": _c(paint["border"]),
                "border_width": paint["bw"], "border_radius": 6}

    @classmethod
    def _node(cls, w, h, name, size, center=False, tag_key=None):
        st = tree_model.node_paint("pending")
        out = {"size": size}
        with ui.ZStack(width=w, height=h):
            out["rect"] = ui.Rectangle(style=cls._rect_style(st))
            if center:
                out["name"] = ui.Label(name, alignment=ui.Alignment.CENTER,
                                       style=_text(_c(st["name"]), size))
            else:
                pad = 6
                with ui.VStack():
                    if tag_key:
                        ui.Spacer(height=4)
                    with ui.HStack(height=30 if tag_key else h):
                        ui.Spacer(width=pad)
                        out["name"] = ui.Label(name, width=0, alignment=ui.Alignment.LEFT_CENTER,
                                               style=_text(_c(st["name"]), size))
                        ui.Spacer()
                        out["count"] = ui.Label("", width=0, alignment=ui.Alignment.RIGHT_CENTER,
                                                style=_text(_c(st["count"]), size))
                        ui.Spacer(width=pad)
                    if tag_key:
                        with ui.HStack(height=22):
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
        dim = bool(nv.get("dim"))
        p = tree_model.node_paint(nv["style"], nv["border"], dim)
        size = wid["size"]
        self._put(wkey + ("rect",), (nv["style"], nv["border"], dim), lambda _v: setattr(
            wid["rect"], "style", self._rect_style(p)))
        self._put(wkey + ("name",), (nv["style"], dim), lambda _v: setattr(
            wid["name"], "style", _text(_c(p["name"]), size)))
        if "count" in wid:
            self._put(wkey + ("count",), (nv["style"], nv["count"], dim), lambda _v: (
                setattr(wid["count"], "text", nv["count"]),
                setattr(wid["count"], "style", _text(_c(p["count"]), size))))
        if "tag" in wid and nv["tag"]:
            self._put(wkey + ("tag",), (nv["tag"], dim), lambda _v: wid["tag"].set(
                nv["tag"], _c(p["tag"]), p["tag_alpha"]))


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
        # [2026-09-17] scene counts + result bar
        self._bar_frame = None
        self._cells = []                     # ui.Rectangle per target
        self._cell_keys = []                 # result key (or None) per cell, last painted
        self._n_cells = None                 # scene target count the bar was built for
        self._scene_key = None               # ((targets, non_targets), stage key) last logged
        self._next_scene = 0.0
        self._scene_warned = False
        self._outcomes_seen = 0
        self._overflow_warned = False
        self._total_noted = False
        self.region_listener = None          # wrist camera caption follows the area (2026-09-16)
        self.phase_listener = None           # ... and its guide fades with the phase
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
                                    self._row_counts()
                                    self._divider()
                                    self._row_tree()
                                    self._divider()
                                    self._row_phase()
                                    self._row_bar()
                                    self._row_result()
                                ui.Spacer(width=PAD)
                            ui.Spacer(height=PAD)
                    ui.Spacer()
                ui.Spacer()

    @staticmethod
    def _divider():
        ui.Rectangle(height=1, style={"background_color": C_LINE})

    def _head(self, key):
        _label("head_" + key, EN[key], C_DIM, 22, width=HEAD_W)

    def _row_nodes(self):
        with ui.HStack(height=0):
            self._head("nodes")
            with ui.HStack(height=0, spacing=10):
                for name in NODE_ORDER:
                    with ui.HStack(width=0, height=0, spacing=5):
                        self._w["lamp_" + name] = ui.Circle(
                            radius=LAMP_R, width=LAMP_R * 2 + 2, height=LAMP_R * 2 + 2,
                            size_policy=ui.CircleSizePolicy.FIXED,
                            alignment=ui.Alignment.CENTER,
                            style={"background_color": C_BAD})
                        _label("node_" + name, EN["node"][name], C_TEXT, 24)

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
            self._w["state"] = _Swappable("state", "IDLE", EN["state"], C_DIM, 40)
            ui.Spacer()

    def _row_bar(self):
        with ui.HStack(height=0):
            ui.Spacer(width=HEAD_W)
            with ui.HStack(height=0, spacing=3):
                for _ in BAR_STATES:
                    self._seg.append(ui.Rectangle(
                        height=10, style={"background_color": C_SEG_OFF, "border_radius": 2}))

    def _row_counts(self):
        # [2026-09-17] Always on: 'TARGET N / NON-TARGET M'. Both numbers are counted from the open
        # scene's fruit prims every SCENE_POLL_SEC (hud/scene_fruit.py -- the same filter the
        # bridge uses to publish targets), so a changed layout or a reloaded scene shows up
        # by itself. '-' until a stage with /World is open.
        with ui.HStack(height=0, spacing=6):
            _label("count_targets", EN["count_targets"], C_TEXT, 24)
            self._w["count_targets_num"] = ui.Label("-", width=0, style=_text(C_TEXT, 24))
            ui.Spacer(width=4)
            ui.Label("/", width=0, style=_text(C_DIM, 24))
            ui.Spacer(width=4)
            _label("count_non_targets", EN["count_non_targets"], C_DIM, 24)
            self._w["count_non_targets_num"] = ui.Label("-", width=0, style=_text(C_DIM, 24))
            ui.Spacer()

    def _row_result(self):
        # The HARVEST DONE line was hidden until the run ends (2026-09-10).
        # [2026-09-19] It is shown the whole run now: NOT STARTED / IN PROGRESS until the end, then
        # DONE + counts (or STOPPED) in the same slot (harvest_key). Hidden, it left the phase bar
        # and the result bar one divider apart, and the two bars read as one (user request).
        # [2026-09-17] Right under it: the result bar and its legend, shown the whole run.
        # They replace the ending's second line 'placed n / dropped m' (T4c 2026-09-15): the
        # legend carries the same placed count and the same failure count under the terms
        # fixed on 2026-09-17, plus the detach failure count. The bar has one cell per scene
        # target and is styled like the phase bar above (height, gap, corner radius); it spans
        # the full inner width because this block is centered, not aligned to the row heads.
        with ui.VStack(height=0, spacing=ROW_GAP):
            self._divider()
            with ui.HStack(height=0, spacing=12):
                ui.Spacer()
                self._w["harvest"] = _Swappable("harvest", "idle", EN["harvest"],
                                                HARVEST_COLOR["idle"], 34)
                self._w["final_num"] = ui.Label("", width=0, style=_text(C_TEXT, 34))
                ui.Spacer()
            self._w["final_num"].visible = False
            # Rebuilt only when the scene target count changes (_update_scene); painting
            # a result never rebuilds it.
            self._bar_frame = ui.Frame(height=result_bar.CELL_H, build_fn=self._build_cells)
            size = result_bar.LEGEND_SIZE
            with ui.HStack(height=0, spacing=4):
                ui.Spacer()
                for i, key in enumerate(result_bar.OUTCOMES):
                    if i:
                        ui.Spacer(width=10)
                    col = _c(result_bar.COLOR[key])
                    _label(LEGEND_LABEL[key], EN[LEGEND_LABEL[key]], col, size)
                    self._w["legend_" + key] = ui.Label("0", width=0, style=_text(col, size))
                ui.Spacer()

    @staticmethod
    def _cell_style(key):
        return {"background_color": C_SEG_OFF if key is None else _c(result_bar.COLOR[key]),
                "border_radius": result_bar.CELL_RADIUS}

    def _build_cells(self):
        """build_fn of the result bar frame: one cell per scene target, painted from the last view."""
        self._cells = []
        n = int(self._n_cells or 0)
        with ui.HStack(height=result_bar.CELL_H, spacing=result_bar.CELL_GAP):
            if n <= 0:
                ui.Spacer()
                return
            for i in range(n):
                key = self._cell_keys[i] if i < len(self._cell_keys) else None
                self._cells.append(ui.Rectangle(height=result_bar.CELL_H,
                                                style=self._cell_style(key)))

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
        self._update_scene(now)
        self._update_result(snap)

        tg = snap["targets"]

        state = snap["sequence"]["state"]
        color = PHASE_COLOR.get(state, C_TEXT)
        self._w["state"].set(state, color)
        if self.phase_listener is not None:
            try:
                self.phase_listener(state)
            except Exception:                                      # noqa: BLE001
                pass                       # the inset must never take the HUD down

        idx = BAR_STATES.index(state) if state in BAR_STATES else -1
        for i, seg in enumerate(self._seg):
            seg.style = {"background_color": (color if i == idx else
                                              C_SEG_DONE if i < idx else C_SEG_OFF),
                         "border_radius": 2}

        finished = bool(snap["result"]["finished"])
        hkey = harvest_key(snap)
        self._w["harvest"].set(hkey, HARVEST_COLOR[hkey])
        if finished:
            done, total = snap["result"]["succeeded"], tg["total"]
            # Percentage of the target count; total can be 0 (no ripe fruit seen).
            pct = int(round(100.0 * done / total)) if total > 0 else 0
            self._w["final_num"].text = "%d / %d (%d%%)" % (done, total, pct)
            # [2026-09-17] N here is the scan executor's candidate total, the bar's is the scene
            # target count. They were equal in every kept run; say so once if they are not.
            if not self._total_noted and self._n_cells is not None and total != self._n_cells:
                self._total_noted = True
                print("[hud] note: HARVEST DONE total %d (scan candidates) != scene targets %d "
                      "(result bar cells)" % (total, self._n_cells))
        self._w["final_num"].visible = finished

    def _update_scene(self, now):
        """Recount the scene's target / non-target fruit; one log line whenever the count changes."""
        if now < self._next_scene:
            return
        self._next_scene = now + SCENE_POLL_SEC
        try:
            ctx = omni.usd.get_context()
            stage = ctx.get_stage()
            fruit = scene_fruit.classify(stage)
            try:
                stage_id = ctx.get_stage_id()
            except Exception:                                      # noqa: BLE001
                stage_id = None
            layer = stage.GetRootLayer().identifier if stage is not None else ""
        except Exception as exc:                                   # noqa: BLE001
            if not self._scene_warned:
                self._scene_warned = True
                print("[hud] scene fruit count failed: %r" % (exc,))
            return
        n, m = len(fruit["targets"]), len(fruit["non_targets"])
        key = ((n, m), (stage_id, layer))
        if key == self._scene_key:
            return
        self._scene_key = key
        have = stage is not None and bool(layer)
        self._w["count_targets_num"].text = "%d" % n if have else "-"
        self._w["count_non_targets_num"].text = "%d" % m if have else "-"
        # One line per change. The appearance part says whether the non-target color override
        # (layers/appearance_layer.usd) is in the open stage: 0/M = scene not reloaded since then.
        const = sum(1 for p in fruit["non_targets"] if scene_fruit.diffuse_is_constant(stage, p))
        print("[hud] scene fruit: target %d / non-target %d (%s; bridge publish filter; "
              "non-target constant diffuse %d/%d)"
              % (n, m, os.path.basename(layer) if layer else "no stage", const, m))
        if fruit["mismatch"]:
            print("[hud] WARNING: ripeness variant disagrees with the prim name for %s -- "
                  "the pipeline follows the name" % ", ".join(fruit["mismatch"]))
        if n != self._n_cells:
            self._n_cells = n
            self._cell_keys = []
            if self._bar_frame is not None:
                self._bar_frame.rebuild()

    def _update_result(self, snap):
        """Paint the result bar and legend; one log line per new result."""
        outcomes = [o for o in (snap["result"].get("outcomes") or []) if isinstance(o, str)]
        v = result_bar.view(outcomes, self._n_cells or 0)
        cells_text = "?" if self._n_cells is None else "%d" % self._n_cells
        if v["cells"] != self._cell_keys and len(self._cells) == len(v["cells"]):
            self._cell_keys = list(v["cells"])
            for rect, key in zip(self._cells, self._cell_keys):
                rect.style = self._cell_style(key)
        for key in result_bar.OUTCOMES:
            text = "%d" % v["counts"][key]
            lbl = self._w.get("legend_" + key)
            if lbl is not None and lbl.text != text:
                lbl.text = text
        if len(outcomes) > self._outcomes_seen:
            for i in range(self._outcomes_seen, len(outcomes)):
                print("[hud] result bar %d/%s: %s" % (i + 1, cells_text, outcomes[i]))
        elif len(outcomes) < self._outcomes_seen:
            print("[hud] result bar cleared (new run)")
            self._overflow_warned = False
            self._total_noted = False
        self._outcomes_seen = len(outcomes)
        if v["overflow"] and not self._overflow_warned:
            self._overflow_warned = True
            print("[hud] WARNING: %d results but only %s scene targets -- the legend counts all, "
                  "the bar shows the first %s" % (len(outcomes), cells_text, cells_text))

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
        self._cells = []
        self._bar_frame = None
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
        self._guide = []            # [(widget, style dict, colour key, base rgba)]
        self._guide_sub = None
        self._alpha = 1.0           # current guide fade level
        self._alpha_target = 1.0
        self._last_tick = 0.0
        self._fade_warned = False

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
                                _label("cam_title", EN["cam_title"], C_DIM, 20)
                                if CAM_GUIDE_ON:
                                    # '   AREA NE' -- the HUD's area value, set by set_region()
                                    ui.Spacer(width=18)
                                    _label("head_region", EN["region"], C_DIM, 20)
                                    ui.Spacer(width=6)
                                    self._region_lbl = ui.Label(
                                        "", width=0, style=_text(CAM_GUIDE_GREEN, 20),
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
        if self._guide:
            self._last_tick = time.time()
            self._guide_sub = (omni.kit.app.get_app().get_update_event_stream()
                               .create_subscription_to_pop(self._on_guide_tick,
                                                           name="wrist_cam_guide_fade"))
        print("[wrist_cam] inset on: %dx%d render shown at %dx%d, guide %s"
              % (CAM_RES + (CAM_WIDTH, img_h, "on" if CAM_GUIDE_ON else "off")))
        return True

    # -- quadrant guide ------------------------------------------------------------

    def _build_guide(self, w, h):
        """Centre cross + NW/NE/SW/SE in each quadrant's outer corner, over the render."""
        cx, cy = w // 2, h // 2
        t = CAM_GUIDE_LINE_PX
        size = max(12, int(round(h * CAM_GUIDE_TEXT_FRAC)))
        box = size * 3                                # label anchor box (text aligned inside)
        m = CAM_GUIDE_MARGIN_PX

        def keep(widget, style, key, rgba):
            # Every guide widget is repainted by _apply_alpha, so its style dict and
            # its colour at full strength are kept next to it.
            self._guide.append((widget, dict(style), key, rgba))

        def bar(x, y, bw, bh):
            style = {"background_color": _fade(CAM_GUIDE_LINE_RGBA, 1.0)}
            with ui.Placer(offset_x=x, offset_y=y):
                keep(ui.Rectangle(width=bw, height=bh, style=style), style,
                     "background_color", CAM_GUIDE_LINE_RGBA)

        bar(cx - t // 2, 0, t, h)
        bar(0, cy - t // 2, w, t)

        corner = {
            "tl": (m, m, ui.Alignment.LEFT_TOP),
            "tr": (w - m - box, m, ui.Alignment.RIGHT_TOP),
            "bl": (m, h - m - box, ui.Alignment.LEFT_BOTTOM),
            "br": (w - m - box, h - m - box, ui.Alignment.RIGHT_BOTTOM),
        }
        for name, slot in CAM_GUIDE_SLOT.items():
            x, y, align = corner[slot]
            # outline = the same text four times, offset, under the green one
            for dx, dy, rgba in ((-1, 0, CAM_GUIDE_OUTLINE_RGBA), (1, 0, CAM_GUIDE_OUTLINE_RGBA),
                                 (0, -1, CAM_GUIDE_OUTLINE_RGBA), (0, 1, CAM_GUIDE_OUTLINE_RGBA),
                                 (0, 0, CAM_GUIDE_TEXT_RGBA)):
                style = {"color": _fade(rgba, 1.0), "font_size": size, "font": CAM_GUIDE_FONT}
                with ui.Placer(offset_x=x + dx, offset_y=y + dy):
                    keep(ui.Label(name, width=box, height=box, alignment=align, style=style),
                         style, "color", rgba)

    def set_phase(self, state):
        """Guide up while the view is the board, faded out during the pick."""
        self._alpha_target = 1.0 if state in CAM_GUIDE_STATES else 0.0

    def _on_guide_tick(self, _event):
        now = time.time()
        dt, self._last_tick = now - self._last_tick, now
        if abs(self._alpha - self._alpha_target) < 1e-3:
            return
        step = max(0.0, dt) / CAM_GUIDE_FADE_SEC
        self._alpha += step if self._alpha_target > self._alpha else -step
        self._alpha = min(1.0, max(0.0, self._alpha))
        try:
            self._apply_alpha()
        except Exception as exc:                                   # noqa: BLE001
            # The fade must never take the inset (or the HUD) down: stop ticking.
            if not self._fade_warned:
                self._fade_warned = True
                print("[wrist_cam] guide fade stopped: %r" % (exc,))
            if self._guide_sub is not None:
                self._guide_sub.unsubscribe()
                self._guide_sub = None

    def _apply_alpha(self):
        visible = self._alpha > 0.01
        for widget, style, key, rgba in self._guide:
            widget.visible = visible
            if visible:
                style[key] = _fade(rgba, self._alpha)
                widget.style = dict(style)

    def set_region(self, key):
        """Caption area from the HUD's highlight key: 'home' -> HOME, 'nw/sw' -> NW/sw.

        The sub-cell stays lower case, the same way the HUD tree writes depth-2 nodes.
        """
        if self._region_lbl is None:
            return
        if not key or key == "home":
            text = "HOME"
        else:
            text = "/".join(p.upper() if i == 0 else p.lower()
                            for i, p in enumerate(str(key).split("/")))
        if text != self._region_text:
            self._region_text = text
            self._region_lbl.text = text

    def destroy(self):
        if self._guide_sub is not None:
            self._guide_sub.unsubscribe()
            self._guide_sub = None
        self._guide = []
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
                self.hud.phase_listener = cam.set_phase
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


