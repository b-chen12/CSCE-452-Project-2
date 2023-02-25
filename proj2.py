import rclpy
import sys
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
# set paramater service
class TurtleControl:

    def __init__(self):
    
        # creating the node, publisher, and subscriber
        self.node = rclpy.create_node('turtle_control')
        self.publisher = self.node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.node.create_timer(1, self.timer_callback)
        self.subscription = self.node.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        # self.subscription = self.node.create_subscription(Color, '/turtle1/color_sensor', self.color_callback, 10)
  
    # publish the message every second
    def timer_callback(self):
        msg = Twist()
        
        # print(type(msg))
        msg.linear.x = 2.0
        msg.angular.z = 2.0
        
        # print('starting publishing twist message with linear.x=', msg.linear.x, 'and angular.z=', msg.angular.z)
        self.publisher.publish(msg)
    
    def pose_callback(self, msg):
        print('pose message should print')
        self.timer = self.node.create_timer(1, self.timer_callback)
        x = msg.x
        y = msg.y
        theta = msg.theta
        print(x, y, theta)
        
    # print out the rgb every time the subscriber is called        
    def color_callback(self, msg):
        self.timer = self.node.create_timer(1, self.timer_callback)
        print("red:", msg.r, "green:",msg.g, "blue:", msg.b)
        
    def move2goal(self):
        # query user for goal coordinate
        goal_pose = Pose()
        goal_pose.x = float(input("Set your x goal: "))
        goal_pose.y = float(input("Set your y goal: "))
        distance_tolerance = float(input("Set your tolerance: "))
        
        
        # rotate the turtle to the desired angle
        # stop rotating
        
        # move the turtle linearly
        # stop moving

def main(args=None):
    print("program started")
    rclpy.init(args=args)

    turtle = TurtleControl()

    rclpy.spin(turtle.node)
    print('done spinning')
    rclpy.shutdown()

if __name__ == '__main__':
    main()

