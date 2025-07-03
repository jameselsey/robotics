import rclpy
from rclpy.node import Node

class Brain(Node):
    def __init__(self):
        super().__init__('brain')
        self.get_logger().info('Brain node has been started.')

def main(args=None):
    rclpy.init(args=args)
    node = Brain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
