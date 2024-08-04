#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class KalmanFilter(Node):
    def __init__(self):
        super().__init__("kalmanfilter")
        self.imu_sub = self.create_subscription(Imu,"/imu/out",self.ImuCallback,10)
        self.noisy_odom_sub = self.create_subscription(Odometry,"/axlr_controller/noisy_odom",self.OdomCallback,10)
        self.filtered_odom_pub = self.create_publisher(Odometry,"/axlr_controller/filtered_odom",10)
        
        self.mean_ = 0.0
        self.variance_ = 1000
        
        self.imu_angular_z = 0.0
        self.is_first_odom = True
        self.prev_angular_z = 0.0
        self.moved_angular = 0.0
        self.motion_variance = 4.0
        self.imu_variance = 0.5
        self.kalman_filter_odom = Odometry()
        
    def MeasurementUpdate(self):
        self.mean_ = (self.imu_variance * self.mean_ + self.variance_ * self.imu_angular_z) / (self.imu_variance + self.variance_)
        self.variance_ = (self.variance_ * self.imu_variance) / (self.imu_variance + self.variance_)
    
    def StatePrediction(self):
        self.mean_ = self.mean_ + self.moved_angular
        self.variance_ = self.variance_ + self.motion_variance
        
        
    def OdomCallback(self,odom:Odometry):
        self.kalman_filter_odom = odom
        if self.is_first_odom:
            self.initial_mean = odom.twist.twist.angular.z
            self.prev_angular_z = odom.twist.twist.angular.z
            self.is_first_odom = False
            return
        self.moved_angular = odom.twist.twist.angular.z - self.prev_angular_z
        
        self.StatePrediction()
        self.MeasurementUpdate()
        
        self.kalman_filter_odom.twist.twist.angular.z = self.mean_
        self.filtered_odom_pub.publish(self.kalman_filter_odom)
        
    
    def ImuCallback(self,imu_data:Imu):
        self.imu_angular_z = imu_data.angular_velocity.z
        
        
def main():
    rclpy.init()
    k_filter = KalmanFilter()
    rclpy.spin(k_filter)
    k_filter.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
    
    