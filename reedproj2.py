import sys
import rclpy
from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from turtlesim.srv import SetPen
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

from rclpy.node import Node

# global variables needed across the program
reached = False
current_x = 0.0
current_y = 0.0

class AMTurtle(Node):
    def __init__(self):
        super().__init__('am_turtle')

        # Initialize current position
        self.current_position = Pose()

        # Creates a client that can be used to change the pen
        self.pen_cli = self.create_client(SetPen,'/turtle1/set_pen')

        # Handles waiting for the SetPen service
        while not self.pen_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting')

        # Creates a client that can be used to change the background color
        self.background_cli = self.create_client(SetParameters,'/turtlesim/set_parameters')
        
        # Handles waiting for the SetParameters service
        while not self.background_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting')

        # Creates a client that can be used to clear the board
        self.clear_cli = self.create_client(Empty,'/clear')

        # Handles waiting for the Empty service
        while not self.clear_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting')
        
        self.pen_req = SetPen.Request()
        self.background_req = SetParameters.Request()
        self.clear_req = Empty.Request()

        # Publisher for moving the turtle
        self.move_m = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = .1  # will move every .1 seconds

        self.timer = self.create_timer(timer_period, self.move_turtle)

        # Subscribtion to get the turtle's location and direction using Pose
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.listener_callback, 10)   

    # Clears the board of any existing drawing
    def send_clear_request(self):
        self.future = self.clear_cli.call_async(self.clear_req)

    def send_pen_request(self):
        # Setting up the parameters for the pen color to be white
        self.pen_req.off = 0
        self.pen_req.r = 255
        self.pen_req.g = 255
        self.pen_req.b = 255
        self.pen_req.width = 3 # if we don't set this then the pen is really thin
        
        # Completing an async call to change the background color
        self.future = self.pen_cli.call_async(self.pen_req)

    # Makes it so that the pen does not draw (the turtle can move without a trail)
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

    # Gets the turtle's current location when subscription is notified
    def listener_callback(self, msg):
        self.current_position = msg   

    # Gets distance between two points
    def distance_formula(self, point):
        # Formula for distance between two points: d = sqrt( (x2-x1)^2 + (y2-y2)^2 )
        x = point.x - self.current_position.x
        y = point.y - self.current_position.y
        length = math.sqrt(x**2 + y**2)
        return length

    # Gets angle between two points and then determines how many radians are between the robot's current orientation and where it wants to go
    def turning_distance(self, point):
        # Formula for angle from one point to another: arctan(y2-y1 / x2-x1)
        y = point.y - self.current_position.y
        x = point.x - self.current_position.x
        angle = math.atan2(y, x)
        angle = angle - self.current_position.theta
        return angle

    def angular (self, point):
        # gain parameter for our PID controller calibration
        k1 = 8
        # The closer that we get tour point, the slower we want the robot to turn; can also turn back if it overshoots
        return k1*(self.turning_distance(point))
    
    def linear (self, point):
        # gain parameter for out PID controller calibration
        k2 = 6
        # The closer that we get to our point, the slower we want the robot to move so that it does not overshoot
        current_velocity = k2 * self.distance_formula(point)
        return current_velocity
    
    def move_turtle(self):
        global reached
        global current_x
        global current_y
        point = Pose()
        point.x = current_x        
        point.y = current_y
        margin = .01 # margin of error for the robot to get as close to at a point
        margin2 = .001 # margin of error for the robot's new angle to be before it moves forward
        msg = Twist()

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        # if the turtle is not yet within the margin of error we allow, then will continue to rotate
        if abs(self.turning_distance(point)) > margin2:
            msg.angular.z = self.angular(point)
        # if the turtle is within the margin of error we allow for the angle, will check if we keep moving forward
        elif abs(self.turning_distance(point)) <= margin2:
            # Have not yet made it to our end point within our margin of error
            if self.distance_formula(point)>=margin:
                msg.linear.x = self.linear(point)
            # Made it to our end point
            else:
                reached = True

        self.move_m.publish(msg)

def main(args=None):
    global current_x # Current x value the robot is going to
    global current_y # Current y value the robot is going to
    global reached # Boolean if the robot has successfully reached its goal
    rclpy.init(args=args)

    am_turtle = AMTurtle()
    am_turtle.send_clear_request() # Clears board
    am_turtle.send_background_request() # Makes background maroon
    am_turtle.transparent_pen_request() # Makes pen invisible

    c = 64.8 # ratio of the size of the image provided vs the size of our workspace
    # Coordinates for the A and the inside portion of the A
    x_cords_a = [72/c, 140/c, 140/c, 131/c, 138/c, 194/c, 203/c, 192/c, 192/c, 260/c, 260/c, 241/c, 195/c, 203/c, 203/c, 128/c, 128/c, 138/c, 91/c, 72/c, 72/c]
    y_cords_a = [217/c, 217/c, 252/c, 252/c, 270/c, 270/c, 252/c, 252/c, 217/c, 217/c, 252/c, 252/c, 360/c, 360/c, 395/c, 395/c, 360/c, 360/c, 252/c, 252/c, 217/c]

    x_cords_a_inside = [154/c, 179/c,166/c,154/c]
    y_cords_a_inside = [305/c,305/c,334/c,305/c]

    # T Coordinates
    x_cords_t = [242/c, 404/c, 404/c, 361/c, 361/c, 442/c, 441/c, 518/c, 518/c, 128/c, 128/c, 203/c, 203/c, 285/c, 285/c, 242/c, 242/c]
    y_cords_t = [117/c, 117/c, 193/c, 193/c, 461/c, 461/c, 418/c, 418/c, 530/c, 530/c, 418/c, 418/c, 461/c, 461/c, 193/c, 193/c, 117/c]

    # W Coordinates
    x_cords_w = [386/c, 450/c, 450/c, 436/c, 436/c, 480/c, 525/c, 525/c, 512/c, 512/c, 575/c, 575/c, 561/c, 561/c, 575/c, 575/c, 518/c, 480/c, 442/c, 387/c, 387/c, 400/c, 400/c, 386/c, 386/c]
    y_cords_w = [217/c, 217/c, 252/c, 252/c, 326/c, 236/c, 326/c, 252/c, 252/c, 217/c, 217/c, 252/c, 252/c, 360/c, 360/c, 395/c, 395/c, 318/c, 395/c, 395/c, 360/c, 360/c, 252/c, 252/c, 217/c]

    # Drawing A: Goes through each coordinate; for each point, keeps spinning until the turtle reaches the point; Picks up the pen to move to its next location
    for i in range(len(x_cords_a)):
        current_x = x_cords_a[i]
        current_y = y_cords_a[i]
        while reached == False:
            rclpy.spin_once(am_turtle)
        reached = False
        am_turtle.send_pen_request()
    am_turtle.transparent_pen_request()

    # Drawing inside A
    for i in range(len(x_cords_a_inside)):
        current_x = x_cords_a_inside[i]
        current_y = y_cords_a_inside[i]
        while reached == False:
            rclpy.spin_once(am_turtle)
        reached = False
        am_turtle.send_pen_request()
    am_turtle.transparent_pen_request()

    # Drawing T
    for i in range(len(x_cords_t)):
        current_x = x_cords_t[i]
        current_y = y_cords_t[i]
        while reached == False:
            rclpy.spin_once(am_turtle)
        reached = False
        am_turtle.send_pen_request()
    am_turtle.transparent_pen_request()

    # Drawing W
    for i in range(len(x_cords_w)):
        current_x = x_cords_w[i]
        current_y = y_cords_w[i]
        while reached == False:
            rclpy.spin_once(am_turtle)
        reached = False
        am_turtle.send_pen_request()
    am_turtle.transparent_pen_request()

    # Moves out of the way at the end
    current_x = 10.0
    current_y = 10.0
    while reached == False:
        rclpy.spin_once(am_turtle)
    reached = False
    am_turtle.send_pen_request()
    
    am_turtle.destroy_node()
    rclpy.shutdown()

# Used to run the script
if __name__ == '__main__':
    main()
