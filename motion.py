import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from turtlesim.msg import Pose

from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from turtlesim.srv import SetPen
from std_srvs.srv import Empty

import sys

import math

i = 0

class TurtleGoToGoal(Node):
    def __init__(self):
        super().__init__("turtle_go_to_goal")
        self.cmdvel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.pose_subs = self.create_subscription(Pose,'/turtle1/pose', self.pose_callback, 10)

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
        
        # Timer stuff
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.move2goal)
        self.pose=Pose()
        self.flag=False

        # Clearing everything
        self.send_clear_request()

        # Setting the background up
        self.send_background_request()

        # Setting up the coordinates to be used
        self.x_cords = [4.0, 4.0, 5.0, 5.0, 3.0, 3.0, 2.0, 2.0, 9.0, 9.0, 8.0, 8.0, 6.0, 6.0, 7.0, 7.0, 4.0]
        self.y_cords = [1.0, 2.0, 2.0, 6.0, 6.0, 4.0, 4.0, 8.0, 8.0, 4.0, 4.0, 6.0, 6.0, 2.0, 2.0, 1.0, 1.0]
        self.angles = [math.pi/2, 0.0, math.pi/2, math.pi, 3*math.pi/2, math.pi, math.pi/2, 0.0, 3*math.pi/2, math.pi, math.pi/2, math.pi, 3*math.pi/2, 0.0, 3*math.pi/2, math.pi, math.pi/2]
    
    def send_clear_request(self):
        self.future = self.clear_cli.call_async(self.clear_req)

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

    def pose_callback(self, data):
        self.pose.x=data.x
        self.pose.y=data.y
        self.pose.theta=data.theta
        
    def eucliden_distance(self,goal_pose):
        return math.sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y),2))
    
    def linear_vel(self,goal_pose, constant=2):
        return constant*self.eucliden_distance(goal_pose)
    
    def steering_angle(self, goal_pose):
        return math.atan2(goal_pose.y-self.pose.y, goal_pose.x - self.pose.x)
    def angular_vel(self, goal_pose, constant=2):
        return constant*(self.steering_angle(goal_pose) - self.pose.theta)
    
    def move2goal(self):
        global i

        goal_pose = Pose()
        goal_pose.x = self.x_cords[i]
        goal_pose.y = self.y_cords[i]
        goal_pose.theta = self.angles[i]

        distance_tolerance = 0.1
        angular_tolerance = 0.01

        vel_msg = Twist()

        if i == 0:
            self.transparent_pen_request()
        else:
            self.send_pen_request()

        if abs(self.steering_angle(goal_pose) - self.pose.theta) > angular_tolerance:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = self.angular_vel(goal_pose)
        else:
            vel_msg.angular.z=0.0
            if self.eucliden_distance(goal_pose) >= distance_tolerance:
                vel_msg.linear.x = self.linear_vel(goal_pose)
            else:
                vel_msg.angular.z = 0.0
                if self.eucliden_distance(goal_pose) >= distance_tolerance:
                    vel_msg.linear.x = self.linear_vel(goal_pose)
                else:
                    vel_msg.linear.x = 0.0
                    self.flag = True
        if self.flag:
            vel_msg.angular.z=goal_pose.theta-self.pose.theta

            if abs(goal_pose.theta - self.pose.theta) <= angular_tolerance:
                quit()
            
            self.flag = False

            if i < len(self.x_cords):
                print("i: " + str(i))
                i += 1
            else:
                print("Here!")
                self.destroy_node()
        
        self.get_logger().info('"%s"' % "publishing vel message")    
        self.cmdvel_pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleGoToGoal()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
