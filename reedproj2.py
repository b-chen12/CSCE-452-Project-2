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
reached = False
current_x = 0.0
current_y = 0.0
current_angle = 0.0
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

        # Creates a client that can be used to clear the board
        self.clear_cli = self.create_client(Empty,'/clear')

        # Handles waiting for the Empty service
        while not self.clear_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.pen_req = SetPen.Request()
        self.background_req = SetParameters.Request()
        self.clear_req = Empty.Request()

        self.move_m = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = .1  # will move every .1 seconds
        self.timer = self.create_timer(timer_period, self.move_turtle)

        self.get_pos = self.create_subscription(Pose, '/turtle1/pose', self.pos_callback, 10)   
        self.pos = Pose()

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

    #def move_request(self):
    def pos_callback(self, msg):
        self.pos = msg
        self.pos.x = msg.x
        self.pos.y = msg.y
        
        self.get_logger().info('"%s"' % self.pos.x )    

    def distance_formula(self, pos):
        length = math.sqrt((pos.x - self.pos.x)**2 + (pos.y - self.pos.y)**2)
        return length

    def angle_between_points(self, pos):
        angle = math.atan2(pos.y - self.pos.y, pos.x - self.pos.x)
        return angle

    def angular (self, pos):
        return 8*(self.angle_between_points(pos) - self.pos.theta)
    
    def linear (self, pos):
        return 6 * self.distance_formula(pos)
    
    def move_turtle(self):
        global reached
        global current_x
        global current_y
        global current_angle
        pos = Pose()
        pos.x = current_x        
        pos.y = current_y
        pos.theta = current_angle
        t = .01
        t_angle = .001
        msg = Twist()

        if abs(self.angle_between_points(pos) - self.pos.theta) > t_angle:
            msg.linear.x = 0.0
            msg.angular.z = self.angular(pos)
        else:
            msg.angular.z = 0.0
            if self.distance_formula(pos)>=t:
                msg.linear.x = self.linear(pos)
            else:
                msg.linear.x = 0.0
                reached = True

        if reached:
            msg.angular.z = pos.theta-self.pos.theta

        self.move_m.publish(msg)

def main(args=None):
    global current_x
    global current_y
    global reached
    rclpy.init(args=args)

    am_turtle = AMTurtle()
    am_turtle.send_clear_request()
    am_turtle.send_background_request()
    am_turtle.transparent_pen_request()
    c = 64.8
    x_cords_a = [72/c, 140/c, 140/c, 131/c, 138/c, 194/c, 203/c, 192/c, 192/c, 260/c, 260/c, 241/c, 195/c, 203/c, 203/c, 128/c, 128/c, 138/c, 91/c, 72/c, 72/c]
    y_cords_a = [217/c, 217/c, 252/c, 252/c, 270/c, 270/c, 252/c, 252/c, 217/c, 217/c, 252/c, 252/c, 360/c, 360/c, 395/c, 395/c, 360/c, 360/c, 252/c, 252/c, 217/c]
    angles = [math.pi/2, 0, math.pi/2, math.pi, 3*math.pi/2, math.pi, math.pi/2, 0, 3*math.pi/2, math.pi, math.pi/2, math.pi, 3*math.pi/2, 0, 3*math.pi/2, math.pi]

    x_cords_a_inside = [154/c, 179/c,166/c,154/c]
    y_cords_a_inside = [305/c,305/c,334/c,305/c]

    x_cords_t = [242/c, 404/c, 404/c, 361/c, 361/c, 442/c, 441/c, 518/c, 518/c, 128/c, 128/c, 203/c, 203/c, 285/c, 285/c, 242/c, 242/c]
    y_cords_t = [117/c, 117/c, 193/c, 193/c, 461/c, 461/c, 418/c, 418/c, 530/c, 530/c, 418/c, 418/c, 461/c, 461/c, 193/c, 193/c, 117/c]

    x_cords_w = [386/c, 450/c, 450/c, 436/c, 436/c, 480/c, 525/c, 525/c, 512/c, 512/c, 575/c, 575/c, 561/c, 561/c, 575/c, 575/c, 518/c, 480/c, 442/c, 387/c, 387/c, 400/c, 400/c, 386/c, 386/c]
    y_cords_w = [217/c, 217/c, 252/c, 252/c, 326/c, 236/c, 326/c, 252/c, 252/c, 217/c, 217/c, 252/c, 252/c, 360/c, 360/c, 395/c, 395/c, 318/c, 395/c, 395/c, 360/c, 360/c, 252/c, 252/c, 217/c]

    for i in range(len(x_cords_a)):
        current_x = x_cords_a[i]
        current_y = y_cords_a[i]
        while reached == False:
            rclpy.spin_once(am_turtle)
        reached = False
        am_turtle.send_pen_request()
    
    am_turtle.transparent_pen_request()
    for i in range(len(x_cords_a_inside)):
        current_x = x_cords_a_inside[i]
        current_y = y_cords_a_inside[i]
        while reached == False:
            rclpy.spin_once(am_turtle)
        reached = False
        am_turtle.send_pen_request()

    am_turtle.transparent_pen_request()
    for i in range(len(x_cords_t)):
        current_x = x_cords_t[i]
        current_y = y_cords_t[i]
        while reached == False:
            rclpy.spin_once(am_turtle)
        reached = False
        am_turtle.send_pen_request()

    am_turtle.transparent_pen_request()
    for i in range(len(x_cords_w)):
        current_x = x_cords_w[i]
        current_y = y_cords_w[i]
        while reached == False:
            rclpy.spin_once(am_turtle)
        reached = False
        am_turtle.send_pen_request()
    
    am_turtle.transparent_pen_request()
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
