import sys
import rclpy
from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from turtlesim.srv import SetPen
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
import time

from rclpy.node import Node
reached = False

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

        self.move_m = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = .5  # will move every .5 seconds
        self.timer = self.create_timer(timer_period, self.go_to_point)

        self.get_pos = self.create_subscription(Pose, '/turtle1/pose', self.pos_callback, 10)   

        self.pos = Pose()

    def send_pen_request(self):
        # Setting up the parameters for the pen color to be white
        self.pen_req.off = 0
        self.pen_req.r = 255
        self.pen_req.g = 255
        self.pen_req.b = 255
        self.pen_req.width = 3 # if we dorclpy.spin_once(am_turtle)n't set this then the pen is really thin
        
        # Completing an async call to change the background color
        self.future = self.pen_cli.call_async(self.pen_req)

    def transparent_pen_request(self):
        self.pen_req.off = 1
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

    def pos_callback(self, coords):
        msg = 'X: {:.2f}, Y: {:.2f}, Theta: {:.3f}'.format(coords.x, coords.y, coords.theta)
        self.get_logger().info(msg)
        self.pos.x = coords.x
        self.pos.y = coords.y
        self.pos.theta = coords.theta

    # Calculates distance using the euclidean distance formula
    def distance(self, goal):
        return math.sqrt(pow((goal.x - self.pos.x), 2) + pow((goal.y - self.pos.y), 2))
    
    # Calculates the linear velocity
    def linear_vel(self, goal):
        return 0.5 * self.distance(goal)
    
    # Calulates the angle between two points
    def angle(self, goal):
        return math.atan2(goal.y - self.pos.y, goal.x - self.pos.x)

    # Calculates angular velocity
    def angular_vel(self, goal):
        return 0.5 * (self.angle(goal) - self.pos.theta)

    # Moves turtle to goal point
    def go_to_point(self):
        new_point = Pose()
        new_point.x = 4.0
        new_point.y = 1.0
        new_point.theta = math.pi/2

        print(new_point.x, new_point.y, new_point.theta)

        vel_msg = Twist()
        distance = self.distance(new_point)

        if abs(self.angle(new_point) - self.pos.theta) > 0.01:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = self.angle(new_point) - self.pos.theta
            if abs(vel_msg.angular.z) < 0.1:
                vel_msg.linear.x = self.linear_vel(new_point)
        
        if abs(distance) < 2:
            vel_msg.angular.z = new_point.theta - self.pos.theta
        
        self.move_m.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    global current_x
    global current_y

    am_turtle = AMTurtle()

    am_turtle.send_background_request()
    am_turtle.transparent_pen_request()
    
    x_cords = [4.0, 4.0, 5.0, 5.0, 3.0, 3.0, 2.0, 2.0, 9.0, 9.0, 8.0, 8.0, 6.0, 6.0, 7.0, 7.0, 4.0]
    y_cords = [1.0, 2.0, 2.0, 6.0, 6.0, 4.0, 4.0, 8.0, 8.0, 4.0, 4.0, 6.0, 6.0, 2.0, 2.0, 1.0, 1.0]
    angles = [math.pi/2, 0.0, math.pi/2, math.pi, 3*math.pi/2, math.pi, math.pi/2, 0.0, 3*math.pi/2, math.pi, math.pi/2, math.pi, 3*math.pi/2, 0.0, 3*math.pi/2, math.pi]

    # for i in range(len(x_cords)):
    #     while reached == False:
    #         rclpy.spin_once(am_turtle)
    #     am_turtle.go_to_point(x_cords[i], y_cords[i], angles[i])
    #     am_turtle.send_pen_request()
    
    am_turtle.go_to_point()

    rclpy.spin(am_turtle)

    am_turtle.destroy_node()
    
    rclpy.shutdown()

# Used to run the script
if __name__ == '__main__':
    main()