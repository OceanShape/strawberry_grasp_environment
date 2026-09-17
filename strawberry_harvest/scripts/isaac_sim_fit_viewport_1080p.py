"""
isaac_sim_fit_viewport_1080p.py -- set the viewport render resolution to 1080p.

[2026-09-17] Recording is 1:1 at 1920x1080, so the viewport has to render at
1920x1080 (the viewport menu -> Resolution entry, done here in one Run).
[2026-09-18] This script no longer resizes or undocks the viewport window: the
display resolution is switched to 1920x1080 for recording, so the whole Kit
window is captured and a separate 1080p window is not needed. The window stays
where it is, docked or not.

ASCII ONLY. Kit's Script Editor renders every non-ASCII character as '?'.

Usage:
  1. Isaac Sim GUI -> Script Editor -> open this file -> Run.
  2. Check the console line: "render: (1920, 1080)".
  3. Run isaac_sim_viewport_display.py after this (re-running it is safe;
     it always leaves exactly one display).
The printed content size is the viewport's on-screen size -- the HUD panel and
the wrist camera inset are drawn in screen pixels, so they keep their designed
proportions only while the screen is at 1920x1080 (portfolio/README.md,
"resolution and crop").
"""
import asyncio

import omni.kit.app
from omni.kit.viewport.utility import get_active_viewport_window

W, H = 1920, 1080
SETTLE_FRAMES = 3              # frames to wait before reading the laid-out size


async def _settle(app):
    for _ in range(SETTLE_FRAMES):
        await app.next_update_async()


async def fit_viewport():
    app = omni.kit.app.get_app()
    win = get_active_viewport_window()
    if win is None:
        raise RuntimeError("No active viewport window found.")
    win.viewport_api.resolution = (W, H)
    await _settle(app)
    print("[fit_viewport] render:", win.viewport_api.resolution,
          "| content on screen:", win.frame.computed_width, "x", win.frame.computed_height)


asyncio.ensure_future(fit_viewport())
