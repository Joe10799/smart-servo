from setuptools import setup

package_name = "smart_servo_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/smart_servo.launch.py", "launch/smart_servo.rviz"],
        ),
        ("share/" + package_name + "/config", ["config/controllers.yaml"]),
        ("share/" + package_name + "/description", ["description/smart_servo.urdf"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yousef",
    maintainer_email="yousef@example.com",
    description="Smart servo ROS bridge",
    license="MIT",
    entry_points={
        "console_scripts": [
            "keyboard_node = smart_servo_ros.keyboard_node:main",
            "simulated_esp32_node = smart_servo_ros.simulated_esp32_node:main",
            "rviz_visualizer = smart_servo_ros.rviz_visualizer:main",
            "tcp_bridge = smart_servo_ros.tcp_bridge:main",
            "plot_logs = smart_servo_ros.plot_logs:main",
        ],
    },
)
