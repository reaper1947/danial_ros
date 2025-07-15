import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import time
import threading

class CmdVelTestNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_test_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.odom_msg = None

        self.rate_10 = self.create_rate(0.1)  
        self.rate_1 = self.create_rate(1)

        # Spin in background
        self.executor_thread = threading.Thread(target=self.spin_odom, daemon=True)
        self.executor_thread.start()

        self.wait_for_odom()

        # === MOTION SEQUENCE ===
        self.get_logger().info("Initial position:")
        self.print_odom()

        self.send_velocity(0.5, 0.0)
        self.get_logger().info("Moving forward 10 sec...")
        self.rate_10.sleep()

        self.send_velocity(0.0, 0.0)
        self.get_logger().info("Stopped. Reading odom...")
        self.rate_1.sleep()
        self.print_odom()

        self.send_velocity(-0.5, 0.0)
        self.get_logger().info("Moving backward 10 sec...")
        self.rate_10.sleep()

        self.send_velocity(0.0, 0.0)
        self.get_logger().info("Stopped. Reading odom...")
        self.rate_1.sleep()
        self.print_odom()

        self.send_velocity(0.0, 0.5)
        self.get_logger().info("Rotating left 5 sec...")
        self.rate_10.sleep()

        self.send_velocity(0.0, 0.0)
        self.get_logger().info("Stopped. Reading odom...")
        self.rate_1.sleep()
        self.print_odom()

        self.send_velocity(0.0, -0.5)
        self.get_logger().info("Rotating right 5 sec...")
        self.rate_10.sleep()

        self.send_velocity(0.0, 0.0)
        self.get_logger().info("Final odom:")
        self.rate_1.sleep()
        self.print_odom()

        self.get_logger().info("Done.")
        rclpy.shutdown()

    def spin_odom(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_odom(self):
        self.get_logger().info("Waiting for first /odom message...")
        while rclpy.ok() and self.odom_msg is None:
            time.sleep(0.1)

    def send_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def odom_callback(self, msg):
        self.odom_msg = msg

    def print_odom(self):
        if self.odom_msg:
            pos = self.odom_msg.pose.pose.position
            ori = self.odom_msg.pose.pose.orientation
            quaternion = [ori.x, ori.y, ori.z, ori.w]
            roll, pitch, yaw = euler_from_quaternion(quaternion)

            self.get_logger().info(
                f"Position -> x: {pos.x:.2f}, y: {pos.y:.2f}, yaw: {yaw:.2f} rad"
            )
        else:
            self.get_logger().warn("No odom message received yet.")

def main(args=None):
    rclpy.init(args=args)
    CmdVelTestNode()

if __name__ == '__main__':
    main()
