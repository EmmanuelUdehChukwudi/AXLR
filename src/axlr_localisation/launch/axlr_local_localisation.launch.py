from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    static_transfor_pub = Node(
        package="tf2_ros",
        executable= "static_transform_publisher",
        arguments=["--x", "0", "--y", "0","--z", "0",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_link_ekf",
                   "--child-frame-id", "imu_link_ekf"],
    )
    
    
    axlr_loacalisation = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("axlr_localisation"), "config","axlr_ekf.yaml")]
    )
    
    imu_republisher = Node(
        package="axlr_localisation",
        executable="republish_imu.py"
    )
    
    
    
    return LaunchDescription([
        static_transfor_pub,
        axlr_loacalisation,
        imu_republisher,  
    ])