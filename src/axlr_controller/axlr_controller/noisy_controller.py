#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
import numpy as np


class NoisyAXLRController(Node):
    def __init__(self):
        super().__init__("noisy_controller")
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

        self.odometry_pub = self.create_publisher(Odometry, "axlr_controller/noisy_odom", 10)
        self.joint_subs = self.create_subscription(JointState, "joint_states", self.JointCallback, 10)
        
        
        self.odometry_msg = Odometry()
        self.odometry_msg.header.frame_id = "odom"
        self.odometry_msg.child_frame_id = "base_footprint_ekf"
        self.odometry_msg.pose.pose.orientation.x = 0.0
        self.odometry_msg.pose.pose.orientation.y = 0.0
        self.odometry_msg.pose.pose.orientation.z = 0.0
        self.odometry_msg.pose.pose.orientation.w = 1.0
        
        self.tf_broadcaster_ = TransformBroadcaster(self)
        self.stamped_transform_ = TransformStamped()
        self.stamped_transform_.header.frame_id = "odom"
        self.stamped_transform_.child_frame_id = "base_footprint_noisy"
        
    def JointCallback(self, msg):
        
        try:
            left_wheel_encoder = msg.position[0] + np.random.normal(0,0.005)
            right_wheel_encoder = msg.position[1] + np.random.normal(0,0.005)
            del_phi_left = left_wheel_encoder - self.left_prev_pos
            del_phi_right = right_wheel_encoder - self.right_prev_pos
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
            
            self.stamped_transform_.transform.translation.x = self.x
            self.stamped_transform_.transform.translation.y = self.y
            self.stamped_transform_.transform.rotation.x = ori[0]
            self.stamped_transform_.transform.rotation.y = ori[1]
            self.stamped_transform_.transform.rotation.z = ori[2]
            self.stamped_transform_.transform.rotation.w = ori[3]
            self.stamped_transform_.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster_.sendTransform(self.stamped_transform_)
            
            self.odometry_pub.publish(self.odometry_msg)
            self.tf_broadcaster_.sendTransform(self.stamped_transform_)
        except Exception as e:
            self.get_logger().error(f"Error in JointCallback: {e}")
    
def main():
    rclpy.init()
    controller_ = NoisyAXLRController()
    rclpy.spin(controller_)
    controller_.destroy_node()
    rclpy.shutdown()   

if __name__ == "__main__":
    main()
