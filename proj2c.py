import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from turtlesim.msg import Pose

import sys

from math import pow, atan2, sqrt
import json
reached = False
current_x = 0.0
current_y = 0.0
current_theta = 0.0
class TurtleGoToGoal(Node):
    def __init__(self):
        super().__init__("turtle_go_to_goal")
        self.cmdvel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.pose_subs = self.create_subscription(Pose,'/turtle1/pose', self.pose_callback, 10)

        timer_period = 0.5

        self.timer = self.create_timer(timer_period, self.move2goal)
        self.pose=Pose()
        self.flag=False

    def pose_callback(self, data):
        print("pose callback called")
        self.pose.x=data.x
        self.pose.y=data.y
        self.pose.theta=data.theta
        msg = 'X: {:.3f}, Y: {:.3f}, Theta: {:.3f}'.format(data.x,data.y, data.theta)
        # self.get_logger().info(msg)
        
    def eucliden_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y),2))
    
    def linear_vel(self,goal_pose, constant=2):
        return constant*self.eucliden_distance(goal_pose)
    
    def steering_angle(self, goal_pose):
        print("steering angle called")
        return atan2(goal_pose.y-self.pose.y, goal_pose.x - self.pose.x)
        
    def angular_vel(self, goal_pose, constant=2):
        return constant*(self.steering_angle(goal_pose) - self.pose.theta)
    

        
    def move2goal(self):
    
        while not self.flag:
            global reached
            global current_x
            global current_y
            global current_theta

            goal_pose = Pose()
            goal_pose.x = current_x
            goal_pose.y = current_y
            goal_pose.theta = current_theta

            distance_tolerance = 0.1
            angular_tolerance = 0.01

            vel_msg = Twist()

            if abs(self.steering_angle(goal_pose) - current_theta) > angular_tolerance:
                print("spinning")
                print(self.pose.theta, "self pose theta")
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = self.angular_vel(goal_pose)
            else:
                print("angular z = 0")
                vel_msg.angular.z=0.0
                if self.eucliden_distance(goal_pose) >= distance_tolerance:
                    vel_msg.linear.x = self.linear_vel(goal_pose)
                else:
                    vel_msg.linear.x = 0.0
                    self.flag = True
                    reached = True
                
            if self.flag:
                self.get_logger().info("point found")
                vel_msg.angular.z=goal_pose.theta-self.pose.theta
                if abs(goal_pose.theta - self.pose.theta) <= angular_tolerance:
                    print("breaking")
                    break
            
            self.cmdvel_pub.publish(vel_msg)
def two_point_angle_calc(x1, x2, y1, y2):
    return atan2(y2 - y1, x2 - x1)

def main(args=None):
    global reached
    global current_x
    global current_y
    rclpy.init(args=args)
    f = open('final_coords.json')
    
    data = json.load(f)
    x_cords = data['x_coords']
    y_cords = data['y_coords']
    angles = []
    for i in range(len(x_cords) - 1):
        angles.append(two_point_angle_calc(x_cords[i], x_cords[i+1], y_cords[i], y_cords[i+1]))
    node = TurtleGoToGoal()

    for i in range(len(x_cords) - 1):
        current_x = x_cords[i]
        current_y = y_cords[i]
        current_theta = angles[i]
        while reached == False:
            rclpy.spin_once(node)
        reached = False
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
