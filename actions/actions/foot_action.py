import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import threading

class Send_action(Node):
    def __init__(self):
        super().__init__('send_action')
        qos_profile = QoSProfile(depth=10)
        self.action_publisher = self.create_publisher(String, 'foot_command', qos_profile)
        th = threading.Thread(target=self.key_input)
        th.start()

    def key_input(self):
        while True:
            input_value = input("fl fr bl br")
            msg = String()
            msg.data = input_value
            self.action_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Send_action()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()