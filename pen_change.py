import sys
import rclpy
from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from turtlesim.srv import SetPen

from rclpy.node import Node

class AMTurtle(Node):
    def __init__(self):
        super().__init__('am_turtle')

        # Creates a client that can be used to change the pen
        self.cli = self.create_client(SetPen,'/turtle1/set_pen')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.req = SetPen.Request()

    def send_request(self):
        # Setting up the parameters for the pen color to be white
        self.req.r = 255
        self.req.g = 255
        self.req.b = 255
        self.req.width = 3 # if we don't set this then the pen is really thin

        # Completing an async call to change the background color
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    am_turtle = AMTurtle()
    am_turtle.send_request()

    rclpy.spin(am_turtle)
    
    if am_turtle.future.done():
        try:
            response = am_turtle.future.result()
        except Exception as e:
            am_turtle.get_logger().info(
                'Service call failed %r' % (e,)
            )
    
    am_turtle.destroy_node()
    rclpy.shutdown()

# Used to run the script
if __name__ == '__main__':
    main()