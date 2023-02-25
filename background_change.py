import sys
import rclpy
from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue

from rclpy.node import Node

class AMTurtle(Node):
    def __init__(self):
        super().__init__('am_turtle')

        # Creates a client that can be used to change the background color
        self.cli = self.create_client(SetParameters,'/turtlesim/set_parameters')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.req = SetParameters.Request()

    def send_request(self):
        # Setting up parameters for the maroon background color
        param_r = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=80)
        param_g = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=0)
        param_b = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=0)

        # Setting the parameters in the request
        self.req.parameters = [
            Parameter(name='background_r', value=param_r),
            Parameter(name='background_g', value=param_g),
            Parameter(name='background_b', value=param_b)
        ]

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