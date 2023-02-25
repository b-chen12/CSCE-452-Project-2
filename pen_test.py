import rclpy
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist, Pose
from turtlesim.msg import Color

#ros2 run turtlesim turtlesim_node --ros-args -p background_r:=128 -p background_b:=0 -p  background_g:=0
import sys
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QApplication, QMainWindow
class TurtleSimNode():
    def __init__(self):
        self.node = rclpy.create_node('turtle_control')
        self.publisher = self.node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.node.create_timer(1, self.timer_callback)
        self.subscription = self.node.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)


        self.pen_client = self.node.create_client(SetPen, '/turtle1/set_pen')
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        self.set_pen_off()

    
    def set_pen_on(self):
        # Create a SetParameters request with pen_off=False
        request = SetParameters.Request()
        request.parameters.append(Parameter(name='pen_off', value=ParameterType(value=False)))

        # Send the request to the turtlesim
        client = self.create_client(SetParameters, '/turtlesim/set_parameters')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info('Pen on')

    def set_pen_off(self):
        # Create a SetParameters request with pen_off=True
        request = SetParameters.Request()
        request.parameters.append(Parameter(name='pen_off', value=ParameterType(value=True)))

        # Send the request to the turtlesim
        client = self.create_client(SetParameters, '/turtlesim/set_parameters')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info('Pen off')


    def pose_callback(self, msg):

        self.timer = self.node.create_timer(1, self.timer_callback)
        x = msg.x
        y = msg.y
        theta = msg.theta

    def timer_callback(self):
        cmd = Twist()

        # Move forward until the target x-coordinate is reached
        while abs(cmd.linear.x) < abs(5.0):
            cmd.linear.x = 0.5
            self.publisher.publish(cmd)

        # Stop moving
        cmd.linear.x = 0.0
        self.publisher.publish(cmd)

if __name__ == "__main__":
    rclpy.init()
    node = TurtleSimNode()

    # Turn the pen off after 5 seconds
    node.get_logger().info("Turning the turtlesim pen off in 5 seconds...")
    node.create_timer(5.0, lambda: node.set_pen_off())

    rclpy.spin(node)
    rclpy.shutdown()

