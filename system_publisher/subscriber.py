import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')  # Node name
        # Create a subscriber
        self.subscription = self.create_subscription(
            String,             # Message type
            'system_monitor',    # Topic name
            self.listener_callback,  # Callback function
            10                   # QoS history depth
        )

    # This is the callback function that gets called whenever a new message is received
    def listener_callback(self, msg):
        # Log the message data
        self.get_logger().info(f'Received message: {msg.data}')

def main(args=None):
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create the subscriber node
    node = SimpleSubscriber()

    # Keep the node running to listen for messages
    rclpy.spin(node)

    # Shutdown after the node stops
    rclpy.shutdown()

if __name__ == '__main__':
    main()
