from setuptools import find_packages, setup

package_name = "vtitan_state_machine"

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
    description="WRO 2026 Future Engineers — race state machine and telemetry bridge (orchestration)",
    entry_points={
        "console_scripts": [
            "state_machine_node = vtitan_state_machine.state_machine_node:main",
            "telemetry_bridge_node = vtitan_state_machine.telemetry_bridge_node:main",
            "bag_recorder_node = vtitan_state_machine.bag_recorder_node:main",
        ],
    },
)
