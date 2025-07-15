#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile

from next2_msgs.action import JackDaniel
from std_srvs.srv import Trigger  # Built-in
from rclpy.executors import MultiThreadedExecutor

import sys
import time

class JackGoalClient(Node):
    def __init__(self, operation_code: int, rpm: int, timeout: int):
        super().__init__('jack_goal_client')

        # Action client for jack control
        self._client = ActionClient(self, JackDaniel, '/jack_control')

        # Service to allow external cancel
        self._cancel_service = self.create_service(
            Trigger,
            'cancel_jack',
            self.cancel_jack_callback
        )

        self.goal_handle = None
        self.code = operation_code
        self.rpm = rpm
        self.timeout = timeout

    def send_goal(self):
        self._client.wait_for_server(10)

        goal_msg = JackDaniel.Goal()
        goal_msg.operation = self.code
        goal_msg.speed_rpm = self.rpm
        goal_msg.timeout.sec = self.timeout
        goal_msg.timeout.nanosec = 0

        self.get_logger().info('🚀 Sending goal to /jack_control...')

        self._send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected')
            rclpy.shutdown()
            return

        self.get_logger().info('✅ Goal accepted')

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'📡 Feedback: {feedback.status_text}')

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'🎯 Result: {result.result_text}')
        self.destroy_node()
        if rclpy.ok():  # ✅ Prevent shutdown crash
            rclpy.shutdown()
        # rclpy.shutdown()

    def cancel_goal(self):
        if self.goal_handle:
            self.get_logger().warn('🛑 Sending cancel request...')
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.goal_canceled_callback)
        else:
            self.get_logger().warn('⚠️ No goal to cancel.')

    def goal_canceled_callback(self, future):
        cancel_response = future.result()
        if cancel_response and cancel_response.goals_canceling:
            self.get_logger().info('✅ Goal cancel request accepted.')
            self.destroy_node()
            if rclpy.ok():  # ✅ Prevent shutdown crash
                rclpy.shutdown()
        else:
            self.get_logger().warn('❌ Cancel request failed or no active goal.')

    def cancel_jack_callback(self, request, response):
        if self.goal_handle:  # 1 = STATUS_ACCEPTED
            self.cancel_goal()
            response.success = True
            response.message = 'Cancel requested via service.'
        else:
            response.success = False
            response.message = 'No active goal to cancel.' + ' ' + str(self.goal_handle.status)
        return response


def main():
    if len(sys.argv) < 4:
        print("Usage: python3 send_goal.py <operation> <rpm> <timeout>")
        print("Example: python3 send_goal.py 1 100 1000")
        return

    try:
        operation_code = int(sys.argv[1])
        rpm = int(sys.argv[2])
        timeout = int(sys.argv[3])
    except ValueError:
        print("❌ Operation must be an integer.")
        return

    time.sleep(1)

    rclpy.init()
    node = JackGoalClient(operation_code, rpm, timeout)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    node.send_goal()
    executor.spin()
    node.destroy_node()
    if rclpy.ok():  # ✅ Prevent shutdown crash
        rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
cancel goal
ros2 service call /cancel_jack std_srvs/srv/Trigger
'''