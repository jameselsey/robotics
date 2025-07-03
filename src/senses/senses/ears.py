import rclpy
from rclpy.node import Node

class Ears(Node):
    def __init__(self):
        super().__init__('ears')
        self.get_logger().info('Ears node has been started.')

def main(args=None):
    rclpy.init(args=args)
    node = Ears()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
