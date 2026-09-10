import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint, MoveLine
from std_msgs.msg import Float64MultiArray

class BridgeTestNode(Node):
    def __init__(self):
        super().__init__('bridge_test_node')
        self.cli_movej = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        self.cli_movel = self.create_client(MoveLine, '/dsr01/motion/move_line')

    def test_move_joint(self):
        if not self.cli_movej.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("MoveJoint service not available")
            return
        req = MoveJoint.Request()
        req.pos = [10.0, -10.0, 20.0, 0.0, 30.0, -10.0]
        req.time = 0.5
        future = self.cli_movej.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        self.get_logger().info(f"MoveJoint result: {res.success}")

    def test_move_line(self):
        if not self.cli_movel.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("MoveLine service not available")
            return
        req = MoveLine.Request()
        req.pos = [0.0, 0.0, -30.0, 0.0, 0.0, 0.0] # 30mm down
        req.time = 0.5
        future = self.cli_movel.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        self.get_logger().info(f"MoveLine result: {res.success}")

def main():
    rclpy.init()
    node = BridgeTestNode()
    node.get_logger().info("Testing MoveJoint...")
    node.test_move_joint()
    node.get_logger().info("Testing MoveLine...")
    node.test_move_line()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
