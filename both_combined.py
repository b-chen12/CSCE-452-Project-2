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
        self.pen_cli = self.create_client(SetPen,'/turtle1/set_pen')

        # Handles waiting for the SetPen service
        while not self.pen_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # Creates a client that can be used to change the background color
        self.background_cli = self.create_client(SetParameters,'/turtlesim/set_parameters')
        
        # Handles waiting for the SetParameters service
        while not self.background_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.pen_req = SetPen.Request()
        self.background_req = SetParameters.Request()

    def send_pen_request(self):
        # Setting up the parameters for the pen color to be white
        self.pen_req.r = 255
        self.pen_req.g = 255
        self.pen_req.b = 255
        self.pen_req.width = 3 # if we don't set this then the pen is really thin

        # Completing an async call to change the background color
        self.future = self.pen_cli.call_async(self.pen_req)
    
    def send_background_request(self):
        # Setting up parameters for the maroon background color
        param_r = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=80)
        param_g = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=0)
        param_b = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=0)

        # Setting the parameters in the request
        self.background_req.parameters = [
            Parameter(name='background_r', value=param_r),
            Parameter(name='background_g', value=param_g),
            Parameter(name='background_b', value=param_b)
        ]

        # Completing an async call to change the background color
        self.future = self.background_cli.call_async(self.background_req)


def main(args=None):
    rclpy.init(args=args)

    am_turtle = AMTurtle()
    am_turtle.send_pen_request()
    am_turtle.send_background_request()

    rclpy.spin(am_turtle)

    # Might need to use while rclpy isok here
    
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