from setuptools import find_packages, setup

package_name = "klevor_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/lidar_launch.py",
                "launch/wro_state_machine_launch.py",
                "launch/telemetry_bridge_launch.py",
                "launch/rpi5_nodes.launch.py",
                "launch/rpi_zero_nodes.launch.py",
                "launch/simulator.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ramón Álvarez",
    description="WRO 2026 Future Engineers — ROS2 navigation runtime",
    entry_points={
        "console_scripts": [
            "bno08x_i2c_node = klevor_robot.imu.bno08x.mcp2221.i2c_node:main",
            "bno08x_uart_rvc_node = klevor_robot.imu.bno08x.mcp2221.uart_rvc_node:main",
            "state_machine_node = klevor_robot.state_machine_node:main",
            "oled_display_node = klevor_robot.oled_display_node:main",
            "ackermann_motor_node = klevor_robot.motors.ackermann_motor_node:main",
            "telemetry_bridge_node = klevor_robot.telemetry_bridge_node:main",
        ],
    },
)
