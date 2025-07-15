import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import time
import threading

class VelocityPlotter(Node):
    def __init__(self):
        super().__init__('velocity_plot_saver')

        self.odom_times = []
        self.odom_data = []

        self.cmd_vel_times = []
        self.cmd_vel_data = []

        self.start_time = time.time()

        self.plot_thread = threading.Thread(target=self.plot_loop, daemon=True)
        self.plot_thread.start()

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def odom_callback(self, msg):
        t = time.time() - self.start_time
        self.odom_times.append(t)
        self.odom_data.append(msg.twist.twist.linear.x)

    def cmd_vel_callback(self, msg):
        t = time.time() - self.start_time
        self.cmd_vel_times.append(t)
        self.cmd_vel_data.append(msg.linear.x)

    def plot_loop(self):
        while rclpy.ok():
            time.sleep(2.0)
            self.save_plot()

    def save_plot(self):
        if not self.odom_times and not self.cmd_vel_times:
            return

        plt.clf()

        if self.odom_times:
            plt.plot(self.odom_times, self.odom_data, label='Odom Velocity X')
        if self.cmd_vel_times:
            plt.plot(self.cmd_vel_times, self.cmd_vel_data, label='Cmd_vel X')

        plt.xlabel('Time (s)')
        plt.ylabel('Linear Velocity X (m/s)')
        plt.title('Velocity Comparison Over Time (Full History)')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig('velocity_plot.png')

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
