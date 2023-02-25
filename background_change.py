import sys
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue

from rclpy.node import Node

class AMTurtle(Node):
    def __init__(self):
        super().__init__('am_turtle')

        # Creates a client that can be used to change the background color
        self.cli = self.create_client(SetParameters,'turtlesim/set_parameters')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_loger().info('service not available, waiting again...')
        
        self.req = SetParameters.Request()

    def send_request(self):
        # Setting up parameters for the maroon background color
        param_r = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=80)
        param_g = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=0)
        param_b = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=0)

        # Setting the parameters in the request
        self.req.parameters = [
            Parameter(name='background_r', value=param_r).to_parameter_msg(),
            Parameter(name='background_g', value=param_g).to_parameter_msg(),
            Parameter(name='background_b', value=param_b).to_parameter_msg()
        ]

        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    am_turtle = AMTurtle()
    am_turtle.send_request()
    

# Used to run the script
if __name__ == '__main__':
    main()