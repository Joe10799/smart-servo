from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def launch_nodes(context, *args, **kwargs):
    mode = context.launch_configurations.get("mode", "simulated")
    host = context.launch_configurations.get("host", "127.0.0.1")
    port = int(context.launch_configurations.get("port", "3333"))
    keyboard_enabled = context.launch_configurations.get("keyboard", "true")
    rviz_enabled = context.launch_configurations.get("rviz", "true")

    package_share = get_package_share_directory("smart_servo_ros")
    rviz_config = os.path.join(package_share, "launch", "smart_servo.rviz")
    controller_config = os.path.join(package_share, "config", "controllers.yaml")
    description_path = os.path.join(
        package_share, "description", "smart_servo.urdf"
    )
    with open(description_path, encoding="utf-8") as description_file:
        robot_description = description_file.read()

    nodes = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description},
                controller_config,
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["servo_velocity_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="smart_servo_ros",
            executable="rviz_visualizer",
            name="rviz_visualizer",
            output="screen",
        ),
        Node(
            package="smart_servo_ros",
            executable="keyboard_node",
            name="keyboard_node",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(keyboard_enabled),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(rviz_enabled),
        ),
    ]

    if mode == "wokwi":
        nodes.insert(
            0,
            Node(
                package="smart_servo_ros",
                executable="tcp_bridge",
                name="tcp_bridge",
                output="screen",
                parameters=[{"host": host, "port": port}],
            ),
        )
    else:
        nodes.insert(
            0,
            Node(
                package="smart_servo_ros",
                executable="simulated_esp32_node",
                name="simulated_esp32_node",
                output="screen",
            ),
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="simulated"),
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="3333"),
        DeclareLaunchArgument("keyboard", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=launch_nodes),
    ])
