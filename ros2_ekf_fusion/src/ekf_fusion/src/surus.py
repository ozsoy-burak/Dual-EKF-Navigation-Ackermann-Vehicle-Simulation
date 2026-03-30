import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelController(Node):

    def __init__(self):
        super().__init__('cmd_vel_controller')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # timer: 0.1 saniyede bir publish
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.start_time = self.get_clock().now()
        self.get_logger().info("CmdVel controller basladi")

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9  # saniye

        msg = Twist()

        # 0 - 6 saniye
        if elapsed < 6.0:
            msg.linear.x = 3.0
            msg.angular.z = 0.0
            self.get_logger().info(f"FULL ileri | t={elapsed:.2f}")

        # 6 - 11 saniye
        elif elapsed < 17.0:
            msg.linear.x = 3.0
            msg.angular.z = 0.25
            self.get_logger().info(f"Ileri + Donus | t={elapsed:.2f}")

        # 11 - 14 saniye (fren)
        elif elapsed < 18.0:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info(f"FREN | t={elapsed:.2f}")

        else:
            self.get_logger().info("Gorev bitti, node kapanıyor")
            self.destroy_node()
            rclpy.shutdown()
            return

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelController()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
