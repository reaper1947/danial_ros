
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import sys


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('set_mode_client')
        self.cli = self.create_client(SetBool, 'pause_mode')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, data):
        self.req.data = True if data == 'True' else False
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):

    if len(sys.argv) < 2:
        print("Usage: python3 client_mode.py True/False")
        return

    try:
        data = sys.argv[1]
    except ValueError:
        print("❌")
        return
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(data)
    minimal_client.get_logger().info(f'Result : {response.success} , {response.message}')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()