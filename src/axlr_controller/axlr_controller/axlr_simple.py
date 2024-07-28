#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np

class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")
        self.declare_parameter("wheel_radius",0.042)
        self.declare_parameter("wheel_separation",0.316)
        
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value
        self.get_logger().info(f"Using parameters wheel radius: {self.wheel_radius} and wheel separation: {self.wheel_separation}")
        self.wheel_cmd_pub = self.create_publisher(Float64MultiArray,"/axlr_controller/commands",10)
        self.axlr_vel_sub = self.create_subscription(TwistStamped,"axlr_controller/cmd_vel",self.axlr_vel_callback,10)
        
        self.conversion_matrix = np.array([[self.wheel_radius/2,self.wheel_radius/2],
                                           [self.wheel_radius/self.wheel_separation,-self.wheel_radius/self.wheel_separation]])
    def axlr_vel_callback(self, msg):
        axlr_speed = np.array([[msg.twist.linear.x],
                               [msg.twist.angular.z]])
        wheel_speeds = np.matmul(np.linalg.inv(self.conversion_matrix),axlr_speed)
        
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speeds[1,0],wheel_speeds[0,0]]
        self.wheel_cmd_pub.publish(wheel_speed_msg)
        
def main():
    rclpy.init()
    controller = SimpleController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()   


if __name__ == "__main__":
    main()