import rclpy
from rclpy.node import Node

class Eyes(Node):
    def __init__(self):
        super().__init__('eyes')
        self.get_logger().info('Eyes node has been started.')

def main(args=None):
    rclpy.init(args=args)
    node = Eyes()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
