from setuptools import setup

package_name = "voldemorbot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/lidar_launch.py",
                "launch/static_tfs.launch.py",
                "launch/wro_state_machine_launch.py",
                "launch/telemetry_bridge_launch.py",
                "launch/rpi5_nodes.launch.py",
                "launch/rpi_zero_nodes.launch.py",
                "launch/simulator.launch.py",
                "launch/race.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ramón Álvarez",
    description="WRO 2026 Future Engineers — launch files and Pi5/PiZero board topology (no source code)",
)
