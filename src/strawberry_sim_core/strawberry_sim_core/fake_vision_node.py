"""실기 비전 노드(strawberry_fusion_node)의 시뮬 대역.

Isaac Sim 이 주는 딸기 좌표를 실기와 **동일한 토픽 형태**로 발행한다.

[2026-09-09] 분면(쿼드트리 세부영역) 필터 추가.
  실기 카메라는 세부영역 scan pose 에서 **그 분면만** 담기 때문에 fusion 노드가
  그 분면의 딸기만 발행한다. 종전 이 노드는 씬의 모든 딸기를 항상 발행했고,
  그래서 첫 분면에서 6개를 전부 시도하고 나머지 세 분면은 후보가 없었다
  (원안 시퀀스 3/5/6 단계 불성립 — PROJECT_GOAL.md section 1-2).
  이제 scan_executor 가 발행하는 셀 상태를 듣고 그 분면의 딸기만 내보낸다.
  카메라 시야를 보드 고정 격자로 대체하는 것이며, 실기 노드는 손대지 않는다.
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Float64MultiArray, String

from strawberry_sim_core.quadrant_filter import (
    parse_cell_state, quadrant_from_cell_id, quadrant_of, cell_bounds, in_bounds)


class FakeVisionNode(Node):
    def __init__(self):
        super().__init__('fake_vision_node')

        # 필터를 끄면 종전(전부 발행) 동작으로 돌아간다.
        self.declare_parameter("quadrant_filter_enabled", True)
        # [2026-09-14] 깊이 2 시야. scan_executor 가 세부 자세에 물리적으로 도착해 `root/sw/se=VIEWING` 을
        # 발행하면 그 세부 칸의 딸기만 발행한다 — 세부 자세가 보드에 더 가까우므로(실기 티칭 평면 433mm)
        # 시야가 좁아진다는 모델. 부모 자세에서 pick 하는 논리 세부 칸(SCANNING)은 종전대로 분면 시야.
        # False 면 09-11 동작(세부 칸도 분면 시야).
        self.declare_parameter("subcell_view_enabled", True)
        self.subcell_view_enabled = bool(self.get_parameter("subcell_view_enabled").value)
        self.quadrant_filter_enabled = bool(
            self.get_parameter("quadrant_filter_enabled").value)

        # 실제 비전 노드(strawberry_fusion_node.py)가 발행하는 토픽과 동일한 퍼블리셔 생성
        self.pick_pub = self.create_publisher(
            PoseStamped, "/strawberry/detection/pick_pose", 20)
        self.scene_pub = self.create_publisher(
            Float64MultiArray, "/strawberry/detection/scene_positions", 10)

        # Isaac Sim으로부터 딸기 좌표 수신
        self.strawberry_sub = self.create_subscription(
            PoseArray, '/isaac_sim/strawberries', self.strawberry_cb, 10)

        # 지금 어느 세부영역을 스캔 중인지. scan_executor 가 'root/nw=SCANNING'
        # 형태로 발행한다 (scan_executor_node._pub_state).
        self.cell_state_sub = self.create_subscription(
            String, '/strawberry/exploration/set_cell_state',
            self.cell_state_cb, 10)
        # None = 필터 없음(전 분면). overview(home) 1차 스캔은 보드 전체를 보므로
        # 셀 상태가 오기 전까지는 전부 발행하는 것이 맞다.
        self.active_quadrant = None
        self.active_cell_id = None
        self.active_bounds = None      # (x0, x1, z0, z1) 또는 None = 전 분면

        self.get_logger().info(
            "Fake Vision Node initialized. (Mocking strawberry_fusion_node) "
            "quadrant_filter=%s subcell_view=%s" % (self.quadrant_filter_enabled, self.subcell_view_enabled))
        self.last_pub_time = 0.0
        self._geom_logged = False

        # 보드 앞면 y (m). scan_executor_node.BOARD_SURFACE_Y_M /
        # harvest_motion_params.WALL_SURFACE_Y_M 과 같은 값이어야 한다.
        self.declare_parameter("board_surface_y_m", 0.810)
        self.board_surface_y_m = float(
            self.get_parameter("board_surface_y_m").value)

        try:    # ── HUD 계측 (제거: 이 4줄만 지우면 된다) ──
            import sys, os; sys.path.append(os.path.expanduser(os.environ.get(
                "HARVEST_HUD_DIR", "~/strawberry_grasp_environment/strawberry_harvest/scripts/hud")))
            import harvest_probe; harvest_probe.attach("vision", self)
        except Exception: pass

    def cell_state_cb(self, msg: String):
        cell_id, state = parse_cell_state(msg.data)
        if cell_id is None:
            return
        quad = quadrant_from_cell_id(cell_id)
        if state == "VIEWING" and self.subcell_view_enabled:
            bounds = cell_bounds(cell_id)            # 세부 칸이면 세부 칸, 분면이면 분면
        else:
            bounds = cell_bounds("root/%s" % quad) if quad else None   # 종전: 분면 단위
        if bounds == self.active_bounds and quad == self.active_quadrant:
            self.active_cell_id = cell_id
            return
        self.active_cell_id = cell_id
        self.active_quadrant = quad
        self.active_bounds = bounds
        self.get_logger().info(
            "SCAN_CELL %s (%s) -> 시야 = %s"
            % (cell_id, state, "x[%.3f,%.3f] z[%.3f,%.3f]" % bounds if bounds else "없음(전 분면)"))

    def _visible(self, poses):
        """지금 스캔 중인 분면에 속한 딸기만 남긴다."""
        if not self.quadrant_filter_enabled or self.active_bounds is None:
            return list(poses)
        return [p for p in poses if in_bounds(p.position.x, p.position.z, self.active_bounds)]

    def strawberry_cb(self, msg: PoseArray):
        current_time = time.time()

        # 너무 빈번한 발행을 막기 위해 0.5초 간격으로 발행
        if current_time - self.last_pub_time < 0.5:
            return
        self.last_pub_time = current_time

        if not msg.poses:
            # [T2 2026-09-10] 빈 입력 = "보이는 익은 과실이 없다". 종전엔 여기서 그냥 돌아가
            # scene_positions 발행이 끊겼고, 마지막 딸기가 부착되는 순간부터 플래너 하트비트와
            # HUD 비전 램프가 죽은 것처럼 보였다 (12:02 런 종료 전 23.7초 공백). 빈 배열을
            # 발행해 하류가 "직전 목록이 아직 유효하다"고 오해하지 않게 한다.
            self.scene_pub.publish(Float64MultiArray())
            return

        if not self._geom_logged:
            # [2026-09-09] 씬을 옮긴 뒤 Isaac 을 다시 열지 않으면 **옛 좌표가 계속
            # 발행되어** 로봇이 딸기에서 -y 로 빗나간 지점을 집는다 (실제 발생).
            # 코드로는 안 보이는 실패라 발행 좌표를 기동 시 한 번 찍어 둔다.
            ys = [p.position.y for p in msg.poses]
            expected = self.board_surface_y_m - 0.0272   # 과실 중심 = 보드면 - 27.2mm
            drift = abs(sum(ys) / len(ys) - expected)
            self.get_logger().warn(
                "BERRY_GEOMETRY: n=%d  y=%.1f~%.1fmm  (기대 %.1fmm, 편차 %.1fmm)%s"
                % (len(ys), min(ys) * 1000, max(ys) * 1000, expected * 1000,
                   drift * 1000,
                   "" if drift < 0.02 else
                   "  <-- 보드 이동이 반영 안 됐다. Isaac 씬을 다시 열 것"))
            self._geom_logged = True

        visible = self._visible(msg.poses)

        # 1. scene_positions 발행 (보이는 딸기의 좌표 배열)
        #    분면에 딸기가 없으면 **빈 배열을 발행한다.** 종전 값이 남아 "직전
        #    분면의 딸기가 아직 보인다" 고 오해되면 안 된다.
        scene_msg = Float64MultiArray()
        for pose in visible:
            scene_msg.data.extend([pose.position.x, pose.position.y, pose.position.z])
        self.scene_pub.publish(scene_msg)

        # 2. pick_pose 발행 (각 딸기마다 하나씩 순차적으로 발행)
        # scan_executor는 Dwell 시간 동안 이 토픽들을 수집하여 중복 제거 후 리스트를 만듭니다.
        for pose in visible:
            pick_msg = PoseStamped()
            pick_msg.header.stamp = self.get_clock().now().to_msg()
            pick_msg.header.frame_id = "base_link" # 로봇 베이스 기준이라고 가정

            # 카메라가 아닌 월드/베이스 기준의 좌표를 그대로 사용
            pick_msg.pose = pose

            self.pick_pub.publish(pick_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
