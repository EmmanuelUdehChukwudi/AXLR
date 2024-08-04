#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import Imu
imu_pub = None

def main():
    global imu_pub
    rclpy.init()
    node = Node("imu_repulisher_node")
    time.sleep(1)
    imu_pub = node.create_publisher(Imu,"imu_ekf",10)
    imu_sub = node.create_subscription(Imu,"imu/out",ImuCallback,10)
    rclpy.spin(node)
    rclpy.shutdown()
    
def ImuCallback(msg:Imu):
    global imu_pub
    msg.header.frame_id = "base_footprint_ekf"
    imu_pub.publish(msg)

if __name__ == '__main__':
    main()