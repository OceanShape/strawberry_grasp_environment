"""
strawberry_sim_setup -- startup tweaks so the sim is recording-ready on boot.

ASCII ONLY, English only: same rule as the Script Editor scripts in
strawberry_harvest/scripts/. Korean notes live in docs/run_guide.md.

Three things, all of which had to be redone by hand every launch:

1. Viewport HUD off (FPS / frame time / device memory / process memory /
   resolution / render progress). Those overlays must not show up in the
   portfolio video. Same switches as the viewport toolbar
   "Display -> Heads Up Display" checkboxes, so toggling them by hand still
   works during a session; this only sets the boot state.
2. Script Editor docked into the tab group that holds Render Settings, so it
   no longer has to be opened from the Window menu on every launch.
3. Perspective camera pinned to the framing used for recording, re-applied on
   every stage open (the run loop reloads main_scene.usd between runs).

Nothing here touches the scene file: the camera transform is authored on the
session layer, which is where Kit keeps /OmniverseKit_Persp anyway.
"""
import asyncio

import carb
import carb.settings
import omni.ext
import omni.kit.app
import omni.kit.commands
import omni.ui as ui
import omni.usd
from pxr import Gf, Usd

SETTING_ROOT = "/exts/strawberry.sim.setup"
PERSP_PATH = "/OmniverseKit_Persp"

# The HUD entries drawn over the viewport. Keys match
# omni.kit.viewport.window (persistent per-viewport settings).
HUD_ITEMS = (
    "renderFPS",
    "renderResolution",
    "renderProgress",
    "deviceMemory",
    "hostMemory",
    "processMemory",
)

# Fallbacks if the [settings] block in extension.toml is missing.
DEFAULT_TRANSLATE = (-1.86373, -1.15613, 2.08606)
DEFAULT_ROTATE_XYZ = (58.1076, 0.0, -46.70705)

# The camera is re-applied a few times after a stage opens: Kit resets
# /OmniverseKit_Persp while the stage is still loading, and the last write
# wins. Values are frame counts to wait before each attempt.
CAMERA_RETRY_FRAMES = (10, 30, 90)

# Docking is retried the same way, for the same reason.
DOCK_ATTEMPTS = 3
DOCK_RETRY_FRAMES = 60


class StrawberrySimSetupExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self._settings = carb.settings.get_settings()
        self._tasks = []
        self._stage_sub = None

        if self._get_bool("hide_viewport_hud", True):
            self._hide_viewport_hud()

        if self._get_bool("dock_script_editor", True):
            self._spawn(self._dock_script_editor_async())

        if self._get_bool("pin_persp_camera", True):
            self._stage_sub = (
                omni.usd.get_context()
                .get_stage_event_stream()
                .create_subscription_to_pop(self._on_stage_event, name="strawberry_sim_setup_camera")
            )
            # Also covers the stage that is already open (extension reload).
            self._spawn(self._pin_camera_async())

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
        if event.type == int(omni.usd.StageEventType.OPENED):
            self._spawn(self._pin_camera_async())

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
        except Exception as exc:  # noqa: BLE001 -- never break app startup
            carb.log_warn(f"[strawberry.sim.setup] pinning {PERSP_PATH} failed: {exc!r}")
            return False
        return True
