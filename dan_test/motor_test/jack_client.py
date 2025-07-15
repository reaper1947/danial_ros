import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration

from next2_msgs.action import JackDaniel


class JackDanielClient(Node):
    def __init__(self):
        super().__init__('jack_control_client')
        self._action_client = ActionClient(self, JackDaniel, 'jack_control')

    def send_goal(self, operation, speed_rpm, timeout_sec):
        goal_msg = JackDaniel.Goal()
        goal_msg.operation = operation
        goal_msg.speed_rpm = speed_rpm
        goal_msg.timeout = Duration(sec=timeout_sec)

        self._action_client.wait_for_server()

        self._send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('❌ Goal rejected')
            rclpy.shutdown()
            return

        self.get_logger().info('✅ Goal accepted')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'[Feedback] {feedback.status_text}')

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'[Result] success: {result.success}')
        self.get_logger().info(f'[Result] text: {result.result_text}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    client = JackDanielClient()

    # Example: LOAD (go to floor) at 100 RPM for 10 seconds
    client.send_goal(operation=JackDaniel.Goal.LOAD, speed_rpm=100, timeout_sec=10)

    rclpy.spin(client)


if __name__ == '__main__':
    main()

"""
ros2 action send_goal /jack_control next2_msgs/action/JackDaniel \
"{operation: 0, speed_rpm: 100, timeout: {sec: 10, nanosec: 0}}" \
--feedback
"""