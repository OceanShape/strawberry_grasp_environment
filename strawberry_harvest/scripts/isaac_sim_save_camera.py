"""
isaac_sim_save_camera.py -- keep the current viewport framing for every scene load.

[2026-09-18] The perspective camera is pinned by the strawberry.sim.setup
extension on every stage open, from four values in
strawberry_harvest/kit_ext/strawberry.sim.setup/config/extension.toml:
  persp_translate, persp_rotate_xyz, persp_focal_length, persp_horizontal_aperture
Moving the camera in the viewport does not change them, so the next scene load
puts the old framing back. This script reads /OmniverseKit_Persp as it is now,
writes those four lines of extension.toml (comments untouched), and updates the
running session's settings so a reload in this session keeps the new framing too.

ASCII ONLY. Kit's Script Editor renders every non-ASCII character as '?'.

Usage:
  1. Frame the shot in the viewport (Perspective camera).
     Do NOT reload the scene first -- the reload re-pins the old framing.
  2. Isaac Sim GUI -> Script Editor -> open this file -> Run.
  3. Check the console lines "[save_camera] ..." (old -> new values).
Paths are pinned (no usable __file__ in the Script Editor); override with HARVEST_REPO.
"""
import math
import os
import re

import carb.settings
import omni.usd
from pxr import Usd, UsdGeom

REPO = os.path.expanduser(os.environ.get("HARVEST_REPO", "~/strawberry_grasp_environment"))
TOML = os.path.join(REPO, "strawberry_harvest/kit_ext/strawberry.sim.setup/config/extension.toml")
SETTING_ROOT = "/exts/strawberry.sim.setup"
PERSP_PATH = "/OmniverseKit_Persp"


def _rot_col(a, b, c):
    """Column-vector matrix of USD rotateXYZ (a, b, c) degrees: X first, then Y, then Z."""
    a, b, c = (math.radians(v) for v in (a, b, c))
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    return [
        [cc * cb, cc * sb * sa - sc * ca, cc * sb * ca + sc * sa],
        [sc * cb, sc * sb * sa + cc * ca, sc * sb * ca - cc * sa],
        [-sb, cb * sa, cb * ca],
    ]


def read_camera():
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(PERSP_PATH) if stage else None
    if not prim or not prim.IsValid():
        raise RuntimeError("%s not found -- is a stage open?" % PERSP_PATH)
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    # USD matrices are row-vector: the rotation block is the transpose of the column form.
    r = [[m[j][i] for j in range(3)] for i in range(3)]
    b = math.degrees(math.asin(max(-1.0, min(1.0, -r[2][0]))))
    a = math.degrees(math.atan2(r[2][1], r[2][2]))
    c = math.degrees(math.atan2(r[1][0], r[0][0]))
    err = max(abs(x - y) for rr, qq in zip(r, _rot_col(a, b, c)) for x, y in zip(rr, qq))
    if err > 1e-6:
        raise RuntimeError("camera has scale/shear or gimbal lock (residual %.2e); not saved" % err)
    cam = UsdGeom.Camera(prim)
    return {
        "persp_translate": [round(m[3][0], 4), round(m[3][1], 4), round(m[3][2], 4)],
        "persp_rotate_xyz": [round(v, 3) + 0.0 for v in (a, b, c)],  # + 0.0 turns -0.0 into 0.0
        "persp_focal_length": round(float(cam.GetFocalLengthAttr().Get()), 3),
        "persp_horizontal_aperture": round(float(cam.GetHorizontalApertureAttr().Get()), 3),
    }


def _fmt(v):
    if isinstance(v, list):
        return "[" + ", ".join(repr(float(x)) for x in v) + "]"
    return repr(float(v))


def save(values):
    text = open(TOML, encoding="utf-8").read()
    for key, value in values.items():
        pattern = re.compile(r'^(exts\."strawberry\.sim\.setup"\.%s = )(.*)$' % key, re.M)
        found = pattern.search(text)
        if found is None:
            raise RuntimeError("%s not found in %s; nothing written" % (key, TOML))
        print("[save_camera] %s: %s -> %s" % (key, found.group(2), _fmt(value)))
        text = text[:found.start(2)] + _fmt(value) + text[found.end(2):]
    with open(TOML, "w", encoding="utf-8") as f:
        f.write(text)
    settings = carb.settings.get_settings()
    for key, value in values.items():
        path = "%s/%s" % (SETTING_ROOT, key)
        if isinstance(value, list):
            settings.set_float_array(path, value)
        else:
            settings.set_float(path, value)
    print("[save_camera] written: %s (this session's settings updated too)" % TOML)


save(read_camera())
