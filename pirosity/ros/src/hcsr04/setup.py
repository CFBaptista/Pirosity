import os

from setuptools import find_packages, setup

package_name = "hcsr04"
requirements_filepath = os.path.join(os.path.dirname(__file__), "requirements.txt")

with open(requirements_filepath) as requirements_file:
    requirements = requirements_file.read().splitlines()

setup(
    name=package_name,
    version="0.1.0.dev0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=requirements,
    zip_safe=True,
    maintainer="Carlos Fernando Baptista",
    maintainer_email="cfd.baptista@gmail.com",
    description="A ROS2 package for interfacing with the HC-SR04 ultrasonic sensor",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["ultrasonic_distance = hcsr04.ultrasonic_distance:main"],
    },
)
