from setuptools import find_packages, setup

package_name = "vtitan_navigation"

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
    description="WRO 2026 Future Engineers — track navigator node (CoreNavigator ROS2 adapter)",
    entry_points={
        "console_scripts": [
            "track_navigator_node = vtitan_navigation.node:main",
        ],
    },
)
