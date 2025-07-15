import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyDynamicPubNode(Node):
    def __init__(self):
        super().__init__('dynamic_pub_node')

        self.pub_topic = 'initial_topic'
        self.publisher = self.create_publisher(String, self.pub_topic, 10)
        self.get_logger().info(f"Publishing to: {self.pub_topic}")

        # Timer to simulate topic change after 5 seconds
        self.timer = self.create_timer(1.0, self.publish_message)
        self.change_timer = self.create_timer(5.0, self.change_publisher_topic)

    def publish_message(self):
        msg = String()
        msg.data = f"Hello from {self.pub_topic}"
        self.publisher.publish(msg)

    def change_publisher_topic(self):
        # Change to new topic name
        new_topic = 'updated_topic'

        if self.pub_topic != new_topic:
            self.get_logger().info(f"Changing publisher topic to: {new_topic}")

            # Destroy old publisher
            self.destroy_publisher(self.publisher)

            # Update topic name
            self.pub_topic = new_topic

            # Create new publisher
            self.publisher = self.create_publisher(String, self.pub_topic, 10)

def main(args=None):
    rclpy.init(args=args)
    node = MyDynamicPubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
