"""
isaac_sim_script_editor_bridge.py -- ROS 2 bridge run inside Isaac Sim.

ASCII ONLY. Kit's Script Editor renders every non-ASCII character as '?', so
this file (comments, strings, prints) is kept in plain English on purpose.
Project-wide notes stay in Korean elsewhere (docs/, PLANNER_CHANGES.md).
Same rule as isaac_sim_viewport_display.py.

Usage:
  Isaac Sim GUI -> Script Editor -> open this file -> Run, then press Play.
  Run this BEFORE Play. Re-running it while the timeline is already playing is
  the failure path described under _apply_ready() below (it recovers now, but
  the log stays cleaner if you avoid it). Stop -> Play alone is safe.

What it does, once per physics step:
  1. applies the latest /joint_command to the robot articulation,
  2. publishes /dsr01/joint_states,
  3. publishes ripe strawberry world positions on /isaac_sim/strawberries at 1 Hz,
  4. [T2 2026-09-10] kinematic attach: on "ATTACH x y z" from /sim/grasp_event
     (published by sim_executor_bridge when it judges CONTACT) the nearest ripe
     strawberry root prim is captured relative to the gripper base link and
     follows it every step; on "RELEASE" inside the tray it is frozen where it is,
     [T4c 2026-09-15] on "RELEASE" outside the tray (planner gave up the place and
     opened the gripper where it stood) it becomes a dynamic body again and falls
     under gravity onto the floor collider -- the failure is shown, not hidden.
     Attached and released fruit are dropped from /isaac_sim/strawberries --
     otherwise a fruit sitting in the tray would be re-detected as a target (the
     tray lies in the SE quadrant of the board grid). No FixedJoint is created at
     runtime.
"""
import sys
import threading
import numpy as np
import builtins
import os
import time

# [IMPORTANT] Keep the system ROS 2 packages from clashing with the ones built
# into Isaac Sim.
sys.path = [p for p in sys.path if '/opt/ros' not in p]

import omni.kit.app
manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

import omni.physx
import omni.timeline
import omni.usd                 # used by _publish_state; do not rely on a side-effect import
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction

class ScriptBridgeNode(Node):
    def __init__(self):
        super().__init__('isaac_sim_bridge_node')
        self.joint_states_pub = self.create_publisher(JointState, '/dsr01/joint_states', 10)
        self.joint_cmd_sub = self.create_subscription(JointState, '/joint_command', self.joint_command_cb, 10)
        self.strawberry_pub = self.create_publisher(PoseArray, '/isaac_sim/strawberries', 10)

        self.target_action = None
        self.pending_grasp_events = []      # [T2] filled by grasp_event_cb, drained in the physics step
        self.grasp_event_lock = threading.Lock()
        self.dof_names_cache = None
        self.dof_index_cache = None      # {dof_name: index} -- Stage1 name mapping
        self.warned_unknown_dof = False
        self.stiffness_set = False
        self.last_pub_time = 0.0

    def grasp_event_cb(self, msg):
        # [T2] Only queue here. USD edits happen in the physics-step callback (main thread).
        with self.grasp_event_lock:
            self.pending_grasp_events.append(str(msg.data))

    def joint_command_cb(self, msg):
        # [Stage1 2026-09-07] Keep the joint names along with the positions.
        # Before this, only the position array was stored and on_physics_step
        # indexed it positionally (full_cmd[:len(cmd)] = cmd). That assumed the
        # six arm axes sit at the front of the DOF array in that order, and it
        # left NO WAY TO ADDRESS THE GRIPPER DOFs -- which is why the sim gripper
        # never moved once.
        self.target_action = (list(msg.name), np.array(msg.position))

def start_bridge():
    if not rclpy.ok():
        rclpy.init()

    # [IMPORTANT] Never create the node and its spin thread twice (guards against
    # clicking Run again).
    if not hasattr(builtins, "my_ros_node") or builtins.my_ros_node is None:
        node = ScriptBridgeNode()
        builtins.my_ros_node = node
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    else:
        node = builtins.my_ros_node

    # Make sure a reused node also has the newer fields -- re-running this script
    # while an older node instance is still alive in builtins is the common case.
    for attr, default in (("last_error_print", 0.0),
                          ("last_init_attempt", 0.0),
                          ("fail_count", 0),
                          ("pending_grasp_events", []),
                          ("grasp_event_lock", threading.Lock())):
        if not hasattr(node, attr):
            setattr(node, attr, default)
    # [T2] A node instance reused from an earlier Run predates grasp_event_cb on its
    # class, so subscribe with a closure instead of the bound method.
    if getattr(node, "grasp_event_sub", None) is None:
        def _grasp_event_cb(msg):
            with node.grasp_event_lock:
                node.pending_grasp_events.append(str(msg.data))
        node.grasp_event_sub = node.create_subscription(String, '/sim/grasp_event', _grasp_event_cb, 10)
    # [T2] Attach state is reset on every Run. The documented order is scene reload ->
    # Run this script -> Play, so a Run means "fresh scene"; fruit frozen in the tray by
    # an earlier run would otherwise stay excluded from publishing forever.
    node.attached = None          # {"path": str, "T_rel": Gf.Matrix4d} while a fruit follows the gripper
    node.harvested = set()        # prim paths released (tray or not) -- never published again
    node.dropped = 0              # [T4c] releases outside the tray -> fell under gravity
    node.tray_bounds = None       # [T4c] lazily computed world-space box of the egg carton
    node.falling = {}             # [T4c] prim path -> timeline time of the drop; rest pose logged once
    node.gripper_rigid = None     # lazily created physics view of the gripper base link

    if hasattr(builtins, "my_physx_sub"):
        builtins.my_physx_sub = None

    # Robot articulation path for main_scene.usd
    # (old scene: /World/robot_recent/strawberry_grasp_robot)
    robot = Articulation(prim_path="/World/robot_assembly", name="doosan_robot")
    builtins.my_robot = robot

    # ---- readiness check and recovery (FIX 2026-09-10) -------------------------
    #
    # Run of 2026-09-09 17:23: on the first MoveJoint, apply_action raised
    #   AttributeError: 'NoneType' object has no attribute 'get_applied_actions'
    # and repeated it 2,486 times over 21 seconds. The exception aborted the whole
    # callback, SO THE joint_states AND STRAWBERRY PUBLISHES BELOW IT STOPPED TOO;
    # the arm stayed frozen at the overview pose and scan_executor's arrival check
    # timed out after 90 s. The ROS logs only showed "did not arrive", so nothing
    # pointed at Isaac as the cause.
    #
    # Three fixes:
    #  (a) wrap apply_action in try/except; on failure re-initialize and KEEP
    #      target_action so the next physics step retries it,
    #  (b) let the readiness check look at the controller view too (see _apply_ready),
    #  (c) keep publishing state even when applying the command fails.
    ERROR_LOG_INTERVAL_SEC = 2.0
    INIT_RETRY_INTERVAL_SEC = 0.5

    def _note_error(tag, exc):
        """Rate-limit repeats of the same failure (it was 2,486 lines before)."""
        node.fail_count += 1
        now = time.time()
        if now - node.last_error_print >= ERROR_LOG_INTERVAL_SEC:
            node.last_error_print = now
            print("[bridge] %s (%d so far): %r" % (tag, node.fail_count, exc))

    def _apply_ready():
        """Is apply_action actually possible right now?

        [FIX 2026-09-09] Check handles_initialized, not num_dof. On Stop -> Play,
        Isaac invalidates the physics view to None while num_dof stays cached, so
        the old check passed. That check is kept.

        [FIX 2026-09-10] But handles_initialized alone is not enough. It reflects
        the robot's own prim view, while apply_action actually uses
        ArticulationController._articulation_view, and that handle is only wired
        up inside robot.initialize(). Re-running this script WHILE THE TIMELINE IS
        PLAYING makes Isaac auto-initialize the prim view only: handles_initialized
        becomes True and the controller view stays None. That is exactly what the
        2026-09-09 17:23 run did (17:23:17 onStop -> Play -> 17:23:22 script re-run
        -> 17:23:30 exception on the first command).
        """
        try:
            if not robot.handles_initialized:
                return False
        except Exception:        # when the view itself is None this property raises too
            return False
        try:
            ctrl = robot.get_articulation_controller()
        except Exception:
            ctrl = None
        if ctrl is None:
            ctrl = getattr(robot, "_articulation_controller", None)
        if ctrl is None:
            return False
        if not hasattr(ctrl, "_articulation_view"):
            # If a future Isaac release renames the attribute, returning False here
            # forever would be worse -- no command would ever go out. Let (a)'s
            # try/except handle it instead.
            return True
        return ctrl._articulation_view is not None

    def _try_initialize():
        """Re-initialize so the controller view gets wired up (rate-limited)."""
        now = time.time()
        if now - node.last_init_attempt < INIT_RETRY_INTERVAL_SEC:
            return False
        node.last_init_attempt = now
        try:
            robot.initialize()
        except Exception as exc:     # e.g. RigidPrimView AttributeError
            _note_error("initialize failed", exc)
            return False
        node.dof_index_cache = None      # new view, so rebuild the DOF map as well
        node.dof_names_cache = None
        print("[bridge] articulation (re)initialized")
        return True

    def _apply_target_action():
        """(a) Do not throw the command away when it fails."""
        names, positions = node.target_action
        try:
            current_pos = robot.get_joint_positions()
        except Exception as exc:
            _note_error("get_joint_positions failed", exc)
            return
        if current_pos is None:
            return
        full_cmd = np.array(current_pos, dtype=float).copy()

        # Apply the action -- DOFs are located by name (Stage1)
        if node.dof_index_cache is None:
            try:
                node.dof_index_cache = {
                    robot.dof_names[i]: i for i in range(robot.num_dof)
                }
            except Exception as exc:
                _note_error("building the DOF map failed", exc)
                return
            print("[bridge] DOF map:", node.dof_index_cache)
        unknown = []
        for name, value in zip(names, positions):
            idx = node.dof_index_cache.get(name)
            if idx is None:
                unknown.append(name)
                continue
            full_cmd[idx] = value
        if unknown and not node.warned_unknown_dof:
            node.warned_unknown_dof = True
            print(f"[bridge] WARNING: unknown joint names ignored: {unknown} "
                  f"(articulation DOFs: {list(node.dof_index_cache)})")

        try:
            robot.apply_action(ArticulationAction(joint_positions=full_cmd))
        except Exception as exc:
            # *** Do NOT clear target_action. Re-initialize and retry next step.
            _note_error("apply_action failed -- re-initializing and retrying", exc)
            _try_initialize()
            return
        if node.fail_count:
            print("[bridge] apply_action recovered (after %d failures)" % node.fail_count)
            node.fail_count = 0
        node.target_action = None

    # ---- [T2 2026-09-10] kinematic attach ---------------------------------------
    #
    # Design (SUBMISSION_PLAN T2): capture T_rel = T_fruit_world * inv(T_gripper_world)
    # at CONTACT and replay fruit_world = T_rel * gripper_world every step. The fruit is
    # NOT snapped to the TCP -- an approach that missed by 5 mm leaves the fruit hanging
    # 5 mm off, which is the point of a verification loop. While attached: rigid body
    # set kinematic, stem joint disabled, colliders off. On RELEASE inside the tray
    # everything stays as is (kinematic + colliders off), so the fruit is a frozen prop
    # in the tray and the retreating gripper cannot be kicked by an immovable body
    # between its jaws (the egg carton has no collider, so a dynamic fruit would fall
    # through it).
    # [T4c 2026-09-15] On RELEASE outside the tray -- the planner rejected the transfer
    # plan and opened the gripper where it stood (hold_on_place_failure=false) -- the
    # fruit is made dynamic again (kinematic off, colliders on, stem joint stays off,
    # velocity zero) and falls onto the floor collider. Freezing it in mid-air hid a
    # placement failure behind a fruit that looked "held". The parts that touch the
    # fruit (custom fingertips) have no colliders, so re-enabling the fruit's collider
    # between the open jaws injects no contact impulse.
    # All physics/xform opinions go to the SESSION layer, so saving the stage never
    # bakes a harvested state into the scene file (same rule as the HUD highlight).
    GRIPPER_BASE_PRIM = "/World/robot_assembly/rh_p12_rn_base"   # cuRobo ee_link
    ATTACH_MATCH_MAX_M = 0.06     # bridge coordinate vs prim: farther than this = disagreement, ignore
    TRAY_PRIM = "/World/egg_carton"   # in-tray test = inside this prim's world box (x, y) + margin
    TRAY_XY_MARGIN_M = 0.03           # a release hanging over the carton rim still counts as "in tray"
    TRAY_Z_ABOVE_M = 0.30             # ... and no higher than this above the carton top
    DROP_REST_AFTER_S = 3.0           # log where a dropped fruit ended up this long after the release
    # lab_environment.usd floor top; used only to label the rest pose. Raised from
    # -0.75 on 2026-09-16 so a dropped fruit stays inside the recorded frame.
    FLOOR_TOP_M = -0.45

    def _stage():
        return omni.usd.get_context().get_stage()

    def _ripe_berry_roots(stage):
        """Same filter as the publish loop, restricted to the root prims under /World."""
        out = []
        for prim in stage.Traverse():
            n = prim.GetName().lower()
            if ("strawberry" in n and "robot" not in n and "unripe" not in n
                    and prim.GetParent().GetPath().pathString == "/World"
                    and prim.IsA(UsdGeom.Xformable)):
                out.append(prim)
        return out

    def _usd_world(prim):
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _gripper_world(stage):
        """Gripper base pose from the physics view if available (authoritative while
        playing), otherwise from USD (relies on PhysX->USD write-back)."""
        try:
            if node.gripper_rigid is None:
                # Isaac 5.x: isaacsim.core.prims.SingleRigidPrim. Older: omni.isaac.core.prims.RigidPrim
                # (this install keeps it under extsDeprecated, same as the Articulation import above).
                try:
                    from isaacsim.core.prims import SingleRigidPrim as _GripView
                except Exception:
                    from omni.isaac.core.prims import RigidPrim as _GripView
                node.gripper_rigid = _GripView(prim_path=GRIPPER_BASE_PRIM, name="grip_base_view")
                print("[bridge] gripper pose source: %s" % type(node.gripper_rigid).__name__)
            pos, quat = node.gripper_rigid.get_world_pose()          # quat = wxyz (Isaac convention)
            m = Gf.Matrix4d()
            m.SetTransform(Gf.Rotation(Gf.Quatd(float(quat[0]),
                                                 Gf.Vec3d(float(quat[1]), float(quat[2]), float(quat[3])))),
                           Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
            return m
        except Exception as exc:
            _note_error("gripper physics view unavailable, using USD xform", exc)
            node.gripper_rigid = None
            return _usd_world(stage.GetPrimAtPath(GRIPPER_BASE_PRIM))

    def _set_attached_physics(stage, berry):
        """Kinematic body, colliders off, stem joint off. Session layer only."""
        joint_path = "/World/physics/stem_" + berry.GetName().replace("strawberry_", "")
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            UsdPhysics.RigidBodyAPI(berry).CreateKinematicEnabledAttr(True)
            for p in Usd.PrimRange(berry):
                if p.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
            joint = stage.GetPrimAtPath(joint_path)
            if joint and joint.IsValid():
                UsdPhysics.Joint(joint).CreateJointEnabledAttr(False)
            else:
                print("[bridge] WARNING: stem joint not found: %s" % joint_path)

    def _set_dropped_physics(stage, berry):
        """[T4c] Dynamic body again, colliders on, stem joint stays off. Session layer only.
        The fruit was carried kinematically (no contact), so it starts the fall from rest
        at the release pose instead of inheriting a kinematic target velocity."""
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            rb = UsdPhysics.RigidBodyAPI(berry)
            rb.CreateKinematicEnabledAttr(False)
            rb.CreateVelocityAttr(Gf.Vec3f(0.0, 0.0, 0.0))
            rb.CreateAngularVelocityAttr(Gf.Vec3f(0.0, 0.0, 0.0))
            # Run 13 (09-15): a fruit falling 1.4 m (5.3 m/s = 88 mm per 60 Hz step) tunnelled
            # through the 20 mm floor slab and vanished. CCD on this body (scene flag in
            # physics_layer.usd) plus the 1 m thick floor stop that.
            berry.CreateAttribute("physxRigidBody:enableCCD", Sdf.ValueTypeNames.Bool).Set(True)
            for p in Usd.PrimRange(berry):
                if p.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(True)

    def _remove_session_xform_opinions(stage, path):
        """[T4c] Drop the session-layer translate/orient opinions written by _follow_attached.
        PhysX writes simulated poses back to the ROOT layer; a stronger session opinion
        would mask that write-back and the falling fruit would still look frozen in the
        viewport (headless check 2026-09-15; the check scripts were not kept, results in SUBMISSION_PLAN T4c: variant B --
        copying the PhysX pose into USD each step -- made omni.physx teleport the body every
        step and reset its velocity; removing the opinions instead shows the fall with no
        jump, because PhysX kept writing the carried pose to the root layer meanwhile)."""
        spec = stage.GetSessionLayer().GetPrimAtPath(path)
        if spec is None:
            return
        for name in ("xformOp:translate", "xformOp:orient"):
            if name in spec.properties:
                spec.RemoveProperty(spec.properties[name])

    def _watch_falling(stage):
        """[T4c] Once per dropped fruit, DROP_REST_AFTER_S after the release, print where it
        came to rest (USD pose = PhysX write-back once the session opinions are gone). Run 13:
        one fruit landed on an unripe fruit below it, one vanished -- the log now says which."""
        if not node.falling:
            return
        now = omni.timeline.get_timeline_interface().get_current_time()
        for path, t0 in list(node.falling.items()):
            if now - t0 < DROP_REST_AFTER_S:
                continue
            del node.falling[path]
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                continue
            pos = _usd_world(prim).ExtractTranslation()
            if pos[2] < FLOOR_TOP_M - 0.10:
                where = "BELOW FLOOR (tunnelled)"
            elif pos[2] < FLOOR_TOP_M + 0.06:
                where = "on floor"
            else:
                where = "caught above floor (%.0f mm up)" % ((pos[2] - FLOOR_TOP_M) * 1000)
            print("[bridge] DROP_REST %s  at (%.1f, %.1f, %.1f) mm after %.1f s -> %s"
                  % (path.split("/")[-1], pos[0] * 1000, pos[1] * 1000, pos[2] * 1000,
                     now - t0, where))

    def _tray_bounds(stage):
        """World-space box of the egg carton, computed once per Run (the tray never moves)."""
        if node.tray_bounds is None:
            prim = stage.GetPrimAtPath(TRAY_PRIM)
            if not prim or not prim.IsValid():
                print("[bridge] WARNING: tray prim not found: %s -- every release counts as outside"
                      % TRAY_PRIM)
                node.tray_bounds = False
            else:
                box = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                        ["default", "render"]).ComputeWorldBound(prim)
                r = box.ComputeAlignedRange()
                lo, hi = r.GetMin(), r.GetMax()
                node.tray_bounds = (lo[0] - TRAY_XY_MARGIN_M, hi[0] + TRAY_XY_MARGIN_M,
                                    lo[1] - TRAY_XY_MARGIN_M, hi[1] + TRAY_XY_MARGIN_M,
                                    hi[2] + TRAY_Z_ABOVE_M)
                print("[bridge] tray box x[%.3f, %.3f] y[%.3f, %.3f] z<=%.3f (from %s)"
                      % (node.tray_bounds + (TRAY_PRIM,)))
        return node.tray_bounds

    def _in_tray(stage, pos):
        b = _tray_bounds(stage)
        if not b:
            return False
        return b[0] <= pos[0] <= b[1] and b[2] <= pos[1] <= b[3] and pos[2] <= b[4]

    def _set_world_xform(stage, prim, world_m):
        """Write translate/orient ops (scale untouched) so the prim lands at world_m."""
        parent_w = _usd_world(prim.GetParent()) if prim.GetParent() else Gf.Matrix4d(1.0)
        local_m = world_m * parent_w.GetInverse()
        t = Gf.Transform()
        t.SetMatrix(local_m)
        q = t.GetRotation().GetQuat()
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
                name = op.GetOpName()
                if name == "xformOp:translate":
                    op.Set(Gf.Vec3d(t.GetTranslation()))
                elif name == "xformOp:orient":
                    op.Set(Gf.Quatf(float(q.GetReal()), Gf.Vec3f(q.GetImaginary())))

    def _attach(stage, xyz):
        if node.attached is not None:
            print("[bridge] ATTACH ignored: already attached to %s" % node.attached["path"])
            return
        target = Gf.Vec3d(*xyz)
        best, best_d = None, 1e9
        for prim in _ripe_berry_roots(stage):
            if prim.GetPath().pathString in node.harvested:
                continue
            d = (_usd_world(prim).ExtractTranslation() - target).GetLength()
            if d < best_d:
                best, best_d = prim, d
        if best is None or best_d > ATTACH_MATCH_MAX_M:
            print("[bridge] ATTACH ignored: no ripe fruit within %.0f mm of (%.3f, %.3f, %.3f) (nearest %.0f mm)"
                  % (ATTACH_MATCH_MAX_M * 1000, xyz[0], xyz[1], xyz[2], best_d * 1000))
            return
        grip_w = _gripper_world(stage)
        fruit_w = _usd_world(best)
        t_rel = fruit_w * grip_w.GetInverse()
        _set_attached_physics(stage, best)
        node.attached = {"path": best.GetPath().pathString, "T_rel": t_rel}
        off = t_rel.ExtractTranslation()
        print("[bridge] ATTACH %s  match %.1f mm  offset from gripper base (%.1f, %.1f, %.1f) mm"
              % (best.GetName(), best_d * 1000, off[0] * 1000, off[1] * 1000, off[2] * 1000))

    def _release(stage):
        if node.attached is None:
            print("[bridge] RELEASE ignored: nothing attached")
            return
        path = node.attached["path"]
        prim = stage.GetPrimAtPath(path)
        pos = _usd_world(prim).ExtractTranslation() if prim and prim.IsValid() else Gf.Vec3d(0, 0, 0)
        node.harvested.add(path)
        node.attached = None
        name = path.split("/")[-1]
        if _in_tray(stage, pos):
            print("[bridge] RELEASE %s  PLACED in tray, frozen at (%.1f, %.1f, %.1f) mm  harvested=%d"
                  % (name, pos[0] * 1000, pos[1] * 1000, pos[2] * 1000, len(node.harvested)))
            return
        # [T4c] outside the tray: let it fall. The count is printed so the Kit log agrees
        # with the HUD's dropped count (harvest_probe counts the planner side).
        if prim and prim.IsValid():
            _set_dropped_physics(stage, prim)
            _remove_session_xform_opinions(stage, path)
            node.falling[path] = omni.timeline.get_timeline_interface().get_current_time()
        node.dropped += 1
        print("[bridge] RELEASE %s  DROPPED outside tray at (%.1f, %.1f, %.1f) mm -> falls  harvested=%d dropped=%d"
              % (name, pos[0] * 1000, pos[1] * 1000, pos[2] * 1000, len(node.harvested), node.dropped))

    def _drain_grasp_events(stage):
        with node.grasp_event_lock:
            events, node.pending_grasp_events = node.pending_grasp_events, []
        for ev in events:
            parts = ev.split()
            try:
                if parts and parts[0] == "ATTACH" and len(parts) == 4:
                    _attach(stage, tuple(float(v) for v in parts[1:4]))
                elif parts and parts[0] == "RELEASE":
                    _release(stage)
                else:
                    print("[bridge] unknown grasp event: %r" % ev)
            except Exception as exc:
                _note_error("grasp event %r failed" % ev, exc)

    def _follow_attached(stage):
        if node.attached is None:
            return
        prim = stage.GetPrimAtPath(node.attached["path"])
        if not prim or not prim.IsValid():
            print("[bridge] attached prim vanished: %s" % node.attached["path"])
            node.attached = None
            return
        try:
            _set_world_xform(stage, prim, node.attached["T_rel"] * _gripper_world(stage))
        except Exception as exc:
            _note_error("follow failed", exc)

    def _publish_state():
        """(c) This block keeps running even when applying the command failed.

        Publishing has to stay alive so the planner's arrival wait and fake_vision
        keep going, and so the logs can tell "the arm did not move" apart from
        "the bridge died".
        """
        try:
            if node.dof_names_cache is None:
                node.dof_names_cache = [robot.dof_names[i] for i in range(robot.num_dof)]
            positions = robot.get_joint_positions()
        except Exception as exc:
            _note_error("reading joint state failed", exc)
            return
        if positions is None:
            return  # physics view not initialized yet, or shutting down

        js_msg = JointState()
        js_msg.header.stamp = node.get_clock().now().to_msg()
        js_msg.name = node.dof_names_cache
        js_msg.position = positions.tolist()
        node.joint_states_pub.publish(js_msg)

        # Strawberry positions, once per second
        current_time = time.time()
        if current_time - node.last_pub_time <= 1.0:
            return
        node.last_pub_time = current_time
        try:
            stage = omni.usd.get_context().get_stage()
            pose_array = PoseArray()
            pose_array.header.stamp = node.get_clock().now().to_msg()
            pose_array.header.frame_id = "world"

            for prim in stage.Traverse():
                # Find strawberry objects: name contains "strawberry", not the robot.
                #
                # [2026-09-07] Unripe fruit is excluded.
                # On the real robot, strawberry_fusion_node runs YOLO ripeness
                # classification and publishes ONLY ripe fruit as pick_pose.
                # Publishing everything here would make the sim more permissive than
                # the hardware, and the planner would try to harvest unripe fruit --
                # something that never happens on the real robot. Unripe fruit stays
                # in the scene purely as a visual obstacle.
                prim_name = prim.GetName().lower()
                if ("strawberry" in prim_name
                        and "robot" not in prim_name
                        and "unripe" not in prim_name):
                    # [T2] harvested / attached fruit is no longer a target
                    path = prim.GetPath().pathString
                    if path in node.harvested or (node.attached is not None
                                                  and node.attached["path"] == path):
                        continue
                    if prim.IsA(UsdGeom.Xformable):
                        xform = UsdGeom.Xformable(prim)
                        # World transform
                        time_code = omni.timeline.get_timeline_interface().get_current_time()
                        world_transform = xform.ComputeLocalToWorldTransform(time_code)
                        translation = world_transform.ExtractTranslation()

                        pose = Pose()
                        pose.position.x = float(translation[0])
                        pose.position.y = float(translation[1])
                        pose.position.z = float(translation[2])
                        pose.orientation.w = 1.0
                        pose_array.poses.append(pose)
        except Exception as exc:
            _note_error("collecting strawberry positions failed", exc)
            return

        # [T2 fix 2026-09-10] Always publish, even when the array is empty. Once the last
        # visible ripe fruit is attached, an empty array is the honest "nothing left"
        # signal. Skipping it froze the downstream state: fake_vision stopped publishing
        # (HUD vision lamp red after 3 s), the planner's scene_positions heartbeat
        # stopped, and sim_executor_bridge kept the harvested fruit's old board
        # position in its judgement list (12:02 run: 23.7 s gap at the end).
        node.strawberry_pub.publish(pose_array)

    def on_physics_step(step_size):
        if not omni.timeline.get_timeline_interface().is_playing():
            return

        ready = _apply_ready()
        if not ready:
            ready = _try_initialize() and _apply_ready()

        if ready and node.target_action is not None:
            _apply_target_action()

        # [T2] attach/release events, then keep the attached fruit on the gripper.
        try:
            stage = _stage()
            if stage is not None:
                _drain_grasp_events(stage)
                _follow_attached(stage)
                _watch_falling(stage)
        except Exception as exc:
            _note_error("attach step failed", exc)

        # (c) Publish state no matter what happened above.
        _publish_state()

    builtins.my_physx_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(on_physics_step)
    print("Script Editor ROS 2 Bridge Started Successfully!")

start_bridge()
