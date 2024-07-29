import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,GroupAction,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition

def noisy_controller(context, *args, **kwargs):
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))
    noisy_controller_ = Node(
                package="axlr_controller",
                executable="noisy_controller.py",
                parameters=[
                    {"wheel_radius": wheel_radius + wheel_radius_error,
                     "wheel_separation": wheel_separation + wheel_separation_error}],
            )
    return [
        noisy_controller_
    ]
def generate_launch_description():
    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value = "0.005"
    )
    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value = "0.02"
    )
    use_custom_axlr_controller_arg = DeclareLaunchArgument(
        "use_custom_axlr_controller",
        default_value = "true"
    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value = "0.042"
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value = "0.316"
    )
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_custom_axlr_controller = LaunchConfiguration("use_custom_axlr_controller")
    
    
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    
    axlr_custom_controller = GroupAction(
        condition=IfCondition(use_custom_axlr_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["axlr_controller", 
                           "--controller-manager", 
                           "/controller_manager"
                ]
            ),
            Node(
                package="axlr_controller",
                executable="axlr_simple.py",
                parameters=[
                    {"wheel_radius": wheel_radius,
                     "wheel_separation": wheel_separation}],
            ),
        ]
    )
    
    noisy_controller_launch = OpaqueFunction(function=noisy_controller)

    simulated_axlr_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["axlr_ros_controller",
                "--controller-manager",
                "/controller_manager"
        ],
        condition=UnlessCondition(use_custom_axlr_controller),
    )

    return LaunchDescription(
        [
            wheel_radius_arg,
            wheel_separation_arg,
            wheel_radius_error_arg,
            wheel_separation_error_arg,
            use_custom_axlr_controller_arg,
            axlr_custom_controller,
            joint_state_broadcaster_spawner,
            simulated_axlr_controller,
            noisy_controller_launch
        ]
    )