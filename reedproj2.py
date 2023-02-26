import sys
import rclpy
from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from turtlesim.srv import SetPen
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import *

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
        timer_period = .1  # will move every 1 seconds
        self.timer = self.create_timer(timer_period, self.move_turtle)

        self.get_pos = self.create_subscription(Pose, '/turtle1/pose', self.pos_callback, 10)   
        #self.get_pos
       # self.get_logger().info('test')
        self.pos = Pose()
        self.rate = self.create_rate(1.0)
        

      #  self.move_turtle

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

    #def move_request(self):
    def pos_callback(self, msg):
        self.pos = msg
        self.pos.x = msg.x
        self.pos.y = msg.y
        
        self.get_logger().info('"%s"' % self.pos.x )    
        #self.get_logger().info('Message from /turtle1/pose: "%s"' % msg ) 

    def distance_formula(self, pos):
        length = sqrt((pos.x - self.pos.x)**2 + (pos.y - self.pos.y)**2)
        return length

    def angle_between_points(self, pos):
        angle = atan2(pos.y - self.pos.y, pos.x - self.pos.x)
        return angle

    def angular (self, pos):
        return 6*(self.angle_between_points(pos) - self.pos.theta)
    
    def linear (self, pos):
        return .5 * self.distance_formula(pos)
    
    def move_turtle(self):
        global reached
        pos = Pose()
        pos.x = 3.0        
        pos.y = 1.0
        t = .5
        msg = Twist()

       # while self.distance_formula(pos) >= t:
        self.get_logger().info('"%s"sss' % self.distance_formula(pos) )  
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular(pos)

        msg.linear.x = self.linear(pos)
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        self.move_m.publish(msg)
        #self.rate.sleep()
            
        self.get_logger().info('"%s"sdf' % self.distance_formula(pos) )  
        if(self.distance_formula(pos) <= 0.5):     
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            self.move_m.publish(msg)
            #sys.exit()
            reached =True
        # rclpy.spin(self)



def main(args=None):
    rclpy.init(args=args)

    am_turtle = AMTurtle()
    am_turtle.send_pen_request()
    am_turtle.send_background_request()
    
    while reached == False:
        rclpy.spin_once(am_turtle)
    #am_turtle.move_turtle()

    # Might need to use while rclpy isok here

    
    am_turtle.destroy_node()
    rclpy.shutdown()

# Used to run the script
if __name__ == '__main__':
    main()