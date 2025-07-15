import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from next2_msgs.action import JackDaniel  # your action type

class CancelJack(Node):
    def __init__(self):
        super().__init__('cancel_jack')
        self._client = ActionClient(self, JackDaniel, 'jack_control')

    def cancel(self):
        future = self._client

        self._client.wait_for_server()
        self.get_logger().info('Sending cancel request...')
        future = self._client.cancel_all_goals()
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info('Cancel request sent.')
        else:
            self.get_logger().error('Cancel failed.')

def main():
    rclpy.init()
    node = CancelJack()
    node.cancel()
    node.destroy_node()
    rclpy.shutdown()
