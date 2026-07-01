import os
from glob import glob

from setuptools import setup

package_name = "skate_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Daniels Skots Lavs",
    maintainer_email="porche121004@gmail.com",
    description="ROS 2 bridge over the R.Botic Skate's native UDP protocol "
                "+ MuJoCo sim endpoint speaking the same protocol.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "driver = skate_ros2.driver_node:main",
            "sim_endpoint = skate_ros2.sim_endpoint:main",
            "moveit_bridge = skate_ros2.moveit_bridge_node:main",
        ],
    },
)
