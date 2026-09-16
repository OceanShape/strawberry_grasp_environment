"""
strawberry_sim_setup -- startup tweaks so the sim is recording-ready on boot.

ASCII ONLY, English only: same rule as the Script Editor scripts in
strawberry_harvest/scripts/. Korean notes live in docs/run_guide.md.

Four things, all of which had to be redone by hand every launch:

1. Viewport HUD off (FPS / frame time / device memory / process memory /
   resolution / render progress / camera speed). Those overlays must not show
   up in the portfolio video. Same switches as the viewport toolbar
   "Display -> Heads Up Display" checkboxes, so toggling them by hand still
   works during a session; this only sets the boot state.
2. Script Editor docked into the tab group that holds Render Settings, so it
   no longer has to be opened from the Window menu on every launch.
3. Perspective camera pinned to the framing used for recording, re-applied on
   every stage open (the run loop reloads main_scene.usd between runs).
4. Viewport background forced to a flat charcoal instead of the dome light's
   white, so the empty space around the set reads as margin, not as an
   unfinished room. A renderer setting only -- the lighting does not move.

Nothing here touches the scene file: the camera transform is authored on the
session layer, which is where Kit keeps /OmniverseKit_Persp anyway, and the
background is an /rtx setting that lives in the app, not in the stage.
"""
import asyncio

import carb
import carb.settings
import omni.ext
import omni.kit.app
import omni.kit.commands
import omni.ui as ui
import omni.usd
from pxr import Gf, Usd, UsdGeom

SETTING_ROOT = "/exts/strawberry.sim.setup"
PERSP_PATH = "/OmniverseKit_Persp"

# The HUD entries drawn over the viewport. Keys match omni.kit.viewport.window
# (persistent per-viewport settings). What an install actually has is listed in
# user.config.json under /persistent/app/viewport/<window>/<viewport>/hud/.
#
# [2026-09-16] cameraSpeed added -- it is the box in the BOTTOM-LEFT corner of
# the viewport (camera move velocity plus the stage unit, which reads as a lone
# "m" at the default 0.05 speed). It sat on top of the wrist camera inset while
# the T5 framing was being checked. In omni.kit.viewport.window 107.2.0
# (stats/__init__.py) the left-aligned stats group holds ViewportSpeed
# (cameraSpeed) and ViewportMessage (toastMessage); the right one holds the six
# readouts below it.
#
# Two HUD entries are left alone by this extension (they stay at whatever the
# Display -> Heads Up Display menu last set), because both only appear when
# something is wrong and this repo does not hide failures:
#   toastMessage        -- Kit's own transient warnings (same bottom-left corner;
#                          turn it off by hand for a take if one lands in frame)
#   metrics/assembler   -- the USD unit / up-axis mismatch warning
HUD_ITEMS = (
    "renderFPS",
    "renderResolution",
    "renderProgress",
    "deviceMemory",
    "hostMemory",
    "processMemory",
    "cameraSpeed",
)

# Fallbacks if the [settings] block in extension.toml is missing.
DEFAULT_TRANSLATE = (-1.86373, -1.15613, 2.08606)
DEFAULT_ROTATE_XYZ = (58.1076, 0.0, -46.70705)
# [2026-09-16 S7] Lens is pinned too. Kit's default perspective camera is wide
# (focalLength 18.147 on a 20.955 aperture = 60 deg hfov, a 31 mm-equivalent), which
# makes the set look small and stretches the perspective. Only the ratio
# aperture / focalLength sets the field of view, so both are written: the aperture
# stays at Kit's 20.955 and the focal length carries the choice.
#   hfov = 2 * atan(20.955 / (2 * F));  "35 mm-equivalent" = F 20.4,  "50 mm" = F 29.1
# The framing numbers behind the defaults are in docs/run_guide.md (camera section).
DEFAULT_FOCAL_LENGTH = 18.147
DEFAULT_HORIZONTAL_APERTURE = 20.955

# The camera is re-applied a few times after a stage opens: Kit resets
# /OmniverseKit_Persp while the stage is still loading, and the last write
# wins. Values are frame counts to wait before each attempt.
CAMERA_RETRY_FRAMES = (10, 30, 90)

# Docking is retried the same way, for the same reason.
DOCK_ATTEMPTS = 3
DOCK_RETRY_FRAMES = 60

# [2026-09-16] Background tone for the video (video review: "a light grey void
# reads as empty, a charcoal one reads as margin").
#
# What paints the empty space is the dome light in
# scenes/layers/lighting_layer.usd -- white at intensity 300. Darkening the dome
# would darken the fill with it, because a dome light's background and its
# ambient contribution are the same number. Render Settings > Common >
# Background > "Background Override" changes what primary rays see when they
# miss geometry, and nothing else: the dome keeps lighting the scene exactly as
# the 09-16 key/fill pass left it.
#
#   /rtx/background/source/type   0 = dome light (default), 1 = texture, 2 = color
#   /rtx/background/source/color  three LINEAR floats (the Render Settings color
#                                 widgets are linear), so sRGB #2E333A charcoal
#                                 is (0.027, 0.033, 0.042)
#
# Both are plain app settings, not persistent ones, so they are set on every launch.
# [2026-09-16] They are also re-applied on every STAGE OPEN, the same as the camera.
# On the first run the boot-time write held (the log read back the charcoal), and the
# viewport still turned black after the scene loaded 23 s later: opening a stage puts
# the renderer's own background back, and with type still "color" the default colour
# is (0, 0, 0). Setting it once at startup is therefore not enough. The retry frames
# cover the reset that lands while the stage is still loading.
BACKGROUND_TYPE_KEY = "/rtx/background/source/type"
BACKGROUND_COLOR_KEY = "/rtx/background/source/color"
BACKGROUND_SOURCE_COLOR = 2
DEFAULT_BACKGROUND_COLOR = (0.027, 0.033, 0.042)
BACKGROUND_RETRY_FRAMES = CAMERA_RETRY_FRAMES


class StrawberrySimSetupExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self._settings = carb.settings.get_settings()
        self._tasks = []
        self._stage_sub = None
        self._background_on = False
        self._pin_camera_on = False
        self._background_warned = False

        if self._get_bool("hide_viewport_hud", True):
            self._hide_viewport_hud()

        self._background_on = self._get_bool("set_background", True)
        self._pin_camera_on = self._get_bool("pin_persp_camera", True)

        if self._background_on:
            self._spawn(self._apply_background_async())

        if self._get_bool("dock_script_editor", True):
            self._spawn(self._dock_script_editor_async())

        if self._pin_camera_on:
            # Also covers the stage that is already open (extension reload).
            self._spawn(self._pin_camera_async())

        if self._pin_camera_on or self._background_on:
            # Opening a stage resets both the viewport camera and the renderer's
            # background, so both are re-applied on every stage open.
            self._stage_sub = (
                omni.usd.get_context()
                .get_stage_event_stream()
                .create_subscription_to_pop(self._on_stage_event,
                                            name="strawberry_sim_setup_stage")
            )

    def on_shutdown(self):
        for task in self._tasks:
            if not task.done():
                task.cancel()
        self._tasks = []
        self._stage_sub = None

    # -- settings helpers ---------------------------------------------------

    def _get_bool(self, name: str, default: bool) -> bool:
        value = self._settings.get(f"{SETTING_ROOT}/{name}")
        return default if value is None else bool(value)

    def _get_str(self, name: str, default: str) -> str:
        value = self._settings.get(f"{SETTING_ROOT}/{name}")
        return default if not value else str(value)

    def _get_float(self, name: str, default: float) -> float:
        value = self._settings.get(f"{SETTING_ROOT}/{name}")
        try:
            return default if value is None else float(value)
        except (TypeError, ValueError):
            return default

    def _get_vec3(self, name: str, default) -> tuple:
        value = self._settings.get(f"{SETTING_ROOT}/{name}")
        if not value or len(value) != 3:
            return tuple(default)
        return tuple(float(v) for v in value)

    def _spawn(self, coro):
        self._tasks.append(asyncio.ensure_future(coro))

    # -- 1. viewport HUD ----------------------------------------------------

    def _hide_viewport_hud(self):
        """Turn the HUD stat readouts off for every viewport, now and later.

        omni.kit.viewport.window resolves each entry from the most specific key
        to the most general, so the persistent per-viewport keys (which already
        exist in user.config.json and say True) have to be overwritten, not just
        the /app/viewport/defaults ones.
        """
        for item in HUD_ITEMS:
            self._settings.set(f"/app/viewport/defaults/hud/{item}/visible", False)

        viewport_ids = set()
        tree = self._settings.get("/persistent/app/viewport") or {}
        if isinstance(tree, dict):
            for window_name, window in tree.items():
                if not isinstance(window, dict):
                    continue
                for viewport_name, viewport in window.items():
                    if isinstance(viewport, dict) and "hud" in viewport:
                        viewport_ids.add(f"{window_name}/{viewport_name}")
        # The viewport this app always has, in case nothing is persisted yet.
        viewport_ids.add("Viewport/Viewport0")

        for viewport_id in sorted(viewport_ids):
            for item in HUD_ITEMS:
                self._settings.set(f"/persistent/app/viewport/{viewport_id}/hud/{item}/visible", False)

        carb.log_info(f"[strawberry.sim.setup] HUD hidden for: {sorted(viewport_ids)}")

    # -- 1b. viewport background -------------------------------------------

    async def _apply_background_async(self):
        app = omni.kit.app.get_app()
        self._apply_background()
        waited = 0
        for target_frame in BACKGROUND_RETRY_FRAMES:
            for _ in range(target_frame - waited):
                await app.next_update_async()
            waited = target_frame
            self._apply_background()
        self._check_background()

    def _check_background(self):
        """Read back what the renderer actually holds -- a black viewport means the
        colour was reset to (0, 0, 0) while the type stayed at "color"."""
        asked = self._get_vec3("background_color", DEFAULT_BACKGROUND_COLOR)
        got = self._settings.get(BACKGROUND_COLOR_KEY) or ()
        kind = self._settings.get(BACKGROUND_TYPE_KEY)
        line = f"background type={kind} color={list(got)} (asked {list(asked)})"
        drifted = (len(got) < 3
                   or any(abs(float(a) - float(b)) > 1e-3 for a, b in zip(asked, got[:3])))
        if drifted and not self._background_warned:
            self._background_warned = True
            carb.log_warn(f"[strawberry.sim.setup] {line} -- did not stick")
        else:
            carb.log_info(f"[strawberry.sim.setup] {line}")

    def _apply_background(self):
        color = self._get_vec3("background_color", DEFAULT_BACKGROUND_COLOR)
        try:
            self._settings.set(BACKGROUND_TYPE_KEY, BACKGROUND_SOURCE_COLOR)
            self._settings.set(BACKGROUND_COLOR_KEY, [float(c) for c in color])
        except Exception as exc:  # noqa: BLE001 -- never break app startup
            carb.log_warn(f"[strawberry.sim.setup] background color failed: {exc!r}")

    # -- 2. Script Editor docking ------------------------------------------

    async def _dock_script_editor_async(self):
        app = omni.kit.app.get_app()
        target_title = self._get_str("dock_target_window", "Render Settings")

        # Let the app finish building its own layout first (isaacsim.app.setup
        # docks the stage / console / content windows on startup).
        for _ in range(30):
            await app.next_update_async()

        editor = await self._show_window_async("Script Editor")
        target = await self._show_window_async(target_title)
        if editor is None or target is None:
            carb.log_warn(
                f"[strawberry.sim.setup] cannot dock Script Editor into '{target_title}' "
                f"(script editor={editor is not None}, target={target is not None})"
            )
            return

        # Retried: other extensions keep rearranging the layout for a while
        # after startup, and a dock that lands too early gets undone.
        for _ in range(DOCK_ATTEMPTS):
            editor.dock_in(target, ui.DockPosition.SAME)
            for _ in range(DOCK_RETRY_FRAMES):
                await app.next_update_async()
            editor = ui.Workspace.get_window("Script Editor") or editor
            target = ui.Workspace.get_window(target_title) or target
            if editor.dock_id and editor.dock_id == target.dock_id:
                break

        if self._get_bool("focus_script_editor", True):
            editor.focus()
        else:
            target.focus()
        carb.log_info(
            f"[strawberry.sim.setup] Script Editor dock_id={editor.dock_id} "
            f"'{target_title}' dock_id={target.dock_id}"
        )

    async def _show_window_async(self, title: str):
        app = omni.kit.app.get_app()
        window = ui.Workspace.get_window(title)
        if window is None:
            # Registered by omni.kit.menu.utils, i.e. the same call the
            # Window menu entry makes.
            ui.Workspace.show_window(title, True)
            for _ in range(10):
                await app.next_update_async()
                window = ui.Workspace.get_window(title)
                if window is not None:
                    break
        if window is not None:
            window.visible = True
        return window

    # -- 3. perspective camera ---------------------------------------------

    def _on_stage_event(self, event):
        if event.type != int(omni.usd.StageEventType.OPENED):
            return
        if self._pin_camera_on:
            self._spawn(self._pin_camera_async())
        if self._background_on:
            self._spawn(self._apply_background_async())

    async def _pin_camera_async(self):
        app = omni.kit.app.get_app()
        waited = 0
        for target_frame in CAMERA_RETRY_FRAMES:
            for _ in range(target_frame - waited):
                await app.next_update_async()
            waited = target_frame
            self._pin_camera()

    def _pin_camera(self) -> bool:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return False
        prim = stage.GetPrimAtPath(PERSP_PATH)
        if not prim or not prim.IsValid():
            return False

        translate = self._get_vec3("persp_translate", DEFAULT_TRANSLATE)
        rotate = self._get_vec3("persp_rotate_xyz", DEFAULT_ROTATE_XYZ)
        focal = self._get_float("persp_focal_length", DEFAULT_FOCAL_LENGTH)
        aperture = self._get_float("persp_horizontal_aperture", DEFAULT_HORIZONTAL_APERTURE)

        try:
            # Kit defines /OmniverseKit_Persp on the session layer. Authoring
            # there keeps main_scene.usd clean -- moving the viewport camera
            # must never mark the scene as modified.
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                omni.kit.commands.execute(
                    "TransformPrimSRT",
                    path=PERSP_PATH,
                    new_translation=Gf.Vec3d(*translate),
                    new_rotation_euler=Gf.Vec3d(*rotate),
                    new_rotation_order=Gf.Vec3i(0, 1, 2),  # XYZ, same as the property panel
                    new_scale=Gf.Vec3d(1.0, 1.0, 1.0),
                )
                cam = UsdGeom.Camera(prim)
                cam.GetFocalLengthAttr().Set(float(focal))
                cam.GetHorizontalApertureAttr().Set(float(aperture))
        except Exception as exc:  # noqa: BLE001 -- never break app startup
            carb.log_warn(f"[strawberry.sim.setup] pinning {PERSP_PATH} failed: {exc!r}")
            return False
        return True
