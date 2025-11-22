import rclpy
from rclpy.node import Node


class test(Node):
    def __init__(self):
        super().__init__('test')

        self.get_logger().info('Testing')

def main(args=None):
    rclpy.init()
    node = test()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()