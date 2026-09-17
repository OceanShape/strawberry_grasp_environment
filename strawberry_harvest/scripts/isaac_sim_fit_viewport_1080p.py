"""
isaac_sim_fit_viewport_1080p.py -- size the viewport for 1:1 1080p recording.

[2026-09-17] Two numbers have to be 1920x1080 for a 1:1 capture:
  1. the render resolution (what the viewport renders), and
  2. the viewport's content area on screen.
The HUD panel and the wrist camera inset (isaac_sim_viewport_display.py) are drawn
in screen pixels, not render pixels. The monitor is 2560x1440, so setting only the
render resolution (viewport menu -> Resolution) letterboxes or stretches the render
while the HUD keeps its pixel size. This script sets both.

ASCII ONLY. Kit's Script Editor renders every non-ASCII character as '?'.

Usage:
  1. Drag the Viewport tab out so the window is undocked
     (a docked window ignores width/height).
  2. Isaac Sim GUI -> Script Editor -> open this file -> Run.
  3. Check the console line: "content: 1920.0 x 1080.0 | render: (1920, 1080)".
     If it is off by a pixel or two, Run once more.
  4. Run isaac_sim_viewport_display.py after this (re-running it is safe;
     it always leaves exactly one display).
Record only that window (or a 1920x1080 region over it), never the whole
2560x1440 screen scaled down (portfolio/README.md, "resolution and crop").
"""
import asyncio

import omni.kit.app
from omni.kit.viewport.utility import get_active_viewport_window

W, H = 1920, 1080
POS_X, POS_Y = 50, 50          # keeps the whole window on the 2560x1440 screen
SETTLE_FRAMES = 3              # frames to wait before reading the laid-out size


async def _settle(app):
    for _ in range(SETTLE_FRAMES):
        await app.next_update_async()


async def fit_viewport():
    app = omni.kit.app.get_app()
    win = get_active_viewport_window()
    if win is None:
        raise RuntimeError("No active viewport window found.")
    win.viewport_api.resolution = (W, H)          # 1. render resolution
    win.position_x, win.position_y = POS_X, POS_Y
    win.width, win.height = W, H
    await _settle(app)
    # 2. the window size includes the title bar -> correct by the measured content size
    win.width += W - win.frame.computed_width
    win.height += H - win.frame.computed_height
    await _settle(app)
    print("[fit_viewport] content:", win.frame.computed_width, "x", win.frame.computed_height,
          "| render:", win.viewport_api.resolution)


asyncio.ensure_future(fit_viewport())
