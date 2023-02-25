import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class BackgroundChanger(Node):
    def __init__(self):
        super().__init__('background_changer')
        self.background_service = self.create_client(SetBool, '/set_background')
        while not self.background_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.set_background_color(101, 0, 11)

    def set_background_color(self, r, g, b):
        req = SetBool.Request()
        req.data = True
        req.value = f"{r},{g},{b}"
        self.background_service.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = BackgroundChanger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

