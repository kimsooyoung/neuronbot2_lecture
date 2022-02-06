import rclpy
from rclpy.node import Node

class TestParams(Node):
    def __init__(self):
        super().__init__('test_params_rclpy')
        self.declare_parameter('my_str')
        self.declare_parameter('my_int')
        self.declare_parameter('my_double_array')

# The following is just to start the node
def main(args=None):
    rclpy.init(args=args)
    node = TestParams()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()