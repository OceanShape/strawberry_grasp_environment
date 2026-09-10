"""
isaac_sim_script_editor_bridge.py -- ROS 2 bridge run inside Isaac Sim.

ASCII ONLY. Kit's Script Editor renders every non-ASCII character as '?', so
this file (comments, strings, prints) is kept in plain English on purpose.
Project-wide notes stay in Korean elsewhere (docs/, PLANNER_CHANGES.md).
Same rule as isaac_sim_hud.py.

Usage:
  Isaac Sim GUI -> Script Editor -> open this file -> Run, then press Play.
  Run this BEFORE Play. Re-running it while the timeline is already playing is
  the failure path described under _apply_ready() below (it recovers now, but
  the log stays cleaner if you avoid it). Stop -> Play alone is safe.

What it does, once per physics step:
  1. applies the latest /joint_command to the robot articulation,
  2. publishes /dsr01/joint_states,
  3. publishes ripe strawberry world positions on /isaac_sim/strawberries at 1 Hz.
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
from pxr import UsdGeom

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
        self.dof_names_cache = None
        self.dof_index_cache = None      # {dof_name: index} -- Stage1 name mapping
        self.warned_unknown_dof = False
        self.stiffness_set = False
        self.last_pub_time = 0.0

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
                          ("fail_count", 0)):
        if not hasattr(node, attr):
            setattr(node, attr, default)

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

        if len(pose_array.poses) > 0:
            node.strawberry_pub.publish(pose_array)

    def on_physics_step(step_size):
        if not omni.timeline.get_timeline_interface().is_playing():
            return

        ready = _apply_ready()
        if not ready:
            ready = _try_initialize() and _apply_ready()

        if ready and node.target_action is not None:
            _apply_target_action()

        # (c) Publish state no matter what happened above.
        _publish_state()

    builtins.my_physx_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(on_physics_step)
    print("Script Editor ROS 2 Bridge Started Successfully!")

start_bridge()
