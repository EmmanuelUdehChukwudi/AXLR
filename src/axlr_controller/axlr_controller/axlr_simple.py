#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import numpy as np

class AXLRController(Node):
    def __init__(self):
        super().__init__("simple_controller")
        self.get_logger().info("Initializing AXLR Controller node")
        self.declare_parameter("wheel_radius", 0.042)
        self.declare_parameter("wheel_separation", 0.316)
        
        self.left_prev_pos = 0
        self.right_prev_pos = 0
        self.prev_time = self.get_clock().now()
        
        # Odometry
        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0
        
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value
        self.get_logger().info(f"Using parameters wheel radius: {self.wheel_radius} and wheel separation: {self.wheel_separation}")
        
        self.wheel_cmd_pub = self.create_publisher(Float64MultiArray, "/axlr_controller/commands", 10)
        self.odometry_pub = self.create_publisher(Odometry, "axlr_controller/odom", 10)
        self.axlr_vel_sub = self.create_subscription(TwistStamped, "axlr_controller/cmd_vel", self.axlr_vel_callback, 10)
        self.joint_subs = self.create_subscription(JointState, "joint_states", self.JointCallback, 10)
        
        self.conversion_matrix = np.array([[self.wheel_radius / 2, self.wheel_radius / 2],
                                           [self.wheel_radius / self.wheel_separation, -self.wheel_radius / self.wheel_separation]])
        
        self.odometry_msg = Odometry()
        self.odometry_msg.header.frame_id = "odom"
        self.odometry_msg.child_frame_id = "base_footprint"
        self.odometry_msg.pose.pose.orientation.x = 0.0
        self.odometry_msg.pose.pose.orientation.y = 0.0
        self.odometry_msg.pose.pose.orientation.z = 0.0
        self.odometry_msg.pose.pose.orientation.w = 1.0
        
    def JointCallback(self, msg):
        try:
            # self.get_logger().info(f"JointCallback: msg.position[0]={msg.position[0]}, msg.position[1]={msg.position[1]}")
            del_phi_left = msg.position[0] - self.left_prev_pos
            del_phi_right = msg.position[1] - self.right_prev_pos
            current_time = Time.from_msg(msg.header.stamp)
            dt = (current_time - self.prev_time).nanoseconds / S_TO_NS
            self.left_prev_pos = msg.position[0]
            self.right_prev_pos = msg.position[1]
            self.prev_time = current_time
            
            phi_left = del_phi_left / dt
            phi_right = del_phi_right / dt
            # self.get_logger().info(f"del_phi_left={del_phi_left}, del_phi_right={del_phi_right}, dt={dt}")
            
            V = (self.wheel_radius * phi_left + self.wheel_radius * phi_right) / 2
            W = (self.wheel_radius * phi_right - self.wheel_radius * phi_left) / self.wheel_separation
            
            robot_dis = (self.wheel_radius * del_phi_left + self.wheel_radius * del_phi_right) / 2
            robot_phi = (self.wheel_radius * del_phi_right - self.wheel_radius * del_phi_left) / self.wheel_separation
            # self.get_logger().info(f"robot_dis={robot_dis}, robot_phi={robot_phi}")
            
            self.phi += robot_phi
            self.x += robot_dis * np.cos(self.phi)
            self.y += robot_dis * np.sin(self.phi)
            # self.get_logger().info(f"x={self.x}, y={self.y}, phi={self.phi}")
            
            ori = quaternion_from_euler(0.0, 0.0, self.phi)
            self.odometry_msg.pose.pose.orientation.x = ori[0]
            self.odometry_msg.pose.pose.orientation.y = ori[1]
            self.odometry_msg.pose.pose.orientation.z = ori[2]
            self.odometry_msg.pose.pose.orientation.w = ori[3]
            self.odometry_msg.header.stamp = self.get_clock().now().to_msg()
            self.odometry_msg.pose.pose.position.x = self.x
            self.odometry_msg.pose.pose.position.y = self.y
            self.odometry_msg.twist.twist.linear.x = V
            self.odometry_msg.twist.twist.angular.z = W
            
            self.odometry_pub.publish(self.odometry_msg)
        except Exception as e:
            self.get_logger().error(f"Error in JointCallback: {e}")
    
    def axlr_vel_callback(self, msg):
        try:
            axlr_speed = np.array([[msg.twist.linear.x],
                                   [msg.twist.angular.z]])
            wheel_speeds = np.matmul(np.linalg.inv(self.conversion_matrix), axlr_speed)
            
            wheel_speed_msg = Float64MultiArray()
            wheel_speed_msg.data = [wheel_speeds[0, 0], wheel_speeds[1, 0]]
            self.wheel_cmd_pub.publish(wheel_speed_msg)
        except Exception as e:
            self.get_logger().error(f"Error in axlr_vel_callback: {e}")
        
def main():
    rclpy.init()
    controller = AXLRController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()   

if __name__ == "__main__":
    main()
