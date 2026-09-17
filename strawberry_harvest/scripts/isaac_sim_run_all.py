"""
isaac_sim_run_all.py -- run the three Script Editor scripts with one Run.

[2026-09-18] Replaces opening and running these one by one:
  1. isaac_sim_script_editor_bridge.py  (ROS 2 bridge)
  2. isaac_sim_fit_viewport_1080p.py    (viewport render resolution 1920x1080)
  3. isaac_sim_viewport_display.py      (HUD panel + wrist camera inset)
The order is the one in portfolio/README.md: the display runs after the viewport
resolution is set. That script is async, so this one waits for its task to finish
before running the display.

ASCII ONLY. Kit's Script Editor renders every non-ASCII character as '?'.

Usage:
  1. Load strawberry_harvest/scenes/main_scene.usd (reload after scene edits).
  2. Isaac Sim GUI -> Script Editor -> open this file -> Run.
  3. Wait for "[run_all] done" in the console, then press Play.
Re-running is as safe as re-running the three scripts: the bridge and the display
each keep a single instance in builtins.

Each script is executed like the Script Editor's Run (__name__ "__main__").
Paths are pinned (no usable __file__ in the Script Editor); override with HARVEST_REPO.
"""
import asyncio
import builtins
import os
import traceback

REPO = os.path.expanduser(os.environ.get("HARVEST_REPO", "~/strawberry_grasp_environment"))
SCRIPT_DIR = os.path.join(REPO, "strawberry_harvest/scripts")
BRIDGE = os.path.join(SCRIPT_DIR, "isaac_sim_script_editor_bridge.py")
FIT = os.path.join(SCRIPT_DIR, "isaac_sim_fit_viewport_1080p.py")
DISPLAY = os.path.join(SCRIPT_DIR, "isaac_sim_viewport_display.py")


def _exec_script(path):
    """Same effect as the Script Editor's Run (as in isaac_batch_orchestrator.py)."""
    print("[run_all] exec: %s" % path)
    src = open(path, encoding="utf-8").read()
    g = {"__name__": "__main__", "__file__": path, "__builtins__": builtins}
    exec(compile(src, path, "exec"), g)


async def _exec_and_await(path):
    """Run a script that schedules its work with asyncio.ensure_future, then wait for that work."""
    before = asyncio.all_tasks()
    _exec_script(path)
    new = [t for t in asyncio.all_tasks() - before if not t.done()]
    for result in await asyncio.gather(*new, return_exceptions=True):
        if isinstance(result, BaseException):
            raise result


async def _fit_then_display():
    try:
        await _exec_and_await(FIT)
    except Exception:
        print("[run_all] WARNING: setting the viewport resolution failed, running the display anyway")
        traceback.print_exc()
    _exec_script(DISPLAY)
    print("[run_all] done -- press Play")


_exec_script(BRIDGE)
asyncio.ensure_future(_fit_then_display())
