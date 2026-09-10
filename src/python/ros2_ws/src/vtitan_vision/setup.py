from setuptools import find_packages, setup

package_name = "vtitan_vision"

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
    description="WRO 2026 Future Engineers — YOLO/Hailo traffic-sign vision node",
    entry_points={
        "console_scripts": [
            "vision_node = vtitan_vision.node:main",
        ],
    },
)
