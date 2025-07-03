import rclpy
from rclpy.node import Node

class Mouth(Node):
    def __init__(self):
        super().__init__('mouth')
        self.get_logger().info('Mouth node has been started.')

def main(args=None):
    rclpy.init(args=args)
    node = Mouth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
