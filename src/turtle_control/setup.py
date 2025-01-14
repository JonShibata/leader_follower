from setuptools import setup
import os
from glob import glob

package_name = "turtle_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools", "launch"],
    zip_safe=True,
    maintainer="Jon Shibata",
    maintainer_email="jonathan.shibata@gmail.com",
    description="ros2 learning",
    license="MIT",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "follower = turtle_control.follower:main",
            "leader = turtle_control.leader:main",
            "turtle1 = turtle_control.turtle1:main",
            "input_service = turtle_control.input_service:main",
        ],
    },
)
