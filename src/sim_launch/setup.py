from setuptools import setup
import os
from glob import glob

package_name = "sim_launch"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools", "launch"],
    zip_safe=True,
    maintainer="Jon Shibata",
    maintainer_email="jonathan.shibata@gmail.com",
    description="Launch file for ros2 learning package",
    license="Apache License 2.0",
    extras_require={
        "test": [],
    },
    entry_points={
        "console_scripts": [],
    },
)
