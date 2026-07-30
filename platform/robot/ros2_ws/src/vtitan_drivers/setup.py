from setuptools import find_packages, setup

package_name = "vtitan_drivers"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ramón Álvarez",
    description="WRO 2026 Future Engineers — hardware driver nodes (IMU, motors, OLED display, button)",
    entry_points={
        "console_scripts": [
            "bno08x_i2c_node = vtitan_drivers.imu.bno08x.mcp2221.i2c_node:main",
            "bno08x_uart_rvc_node = vtitan_drivers.imu.bno08x.mcp2221.uart_rvc_node:main",
            "ackermann_motor_node = vtitan_drivers.motors.ackermann_motor_node:main",
            "oled_display_node = vtitan_drivers.oled_display_node:main",
            "button_node = vtitan_drivers.button_node:main",
            "challenge_mode_node = vtitan_drivers.challenge_mode_node:main",
            "pi_zero_peripherals_node = vtitan_drivers.pi_zero_peripherals_node:main",
            "joy_teleop_node = vtitan_drivers.teleop.joy_teleop_node:main",
        ],
    },
)
