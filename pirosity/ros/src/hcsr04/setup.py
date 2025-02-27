from setuptools import find_packages, setup

package_name = 'hcsr04'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Carlos Fernando Baptista',
    maintainer_email='cfd.baptista@gmail.com',
    description='A ROS2 pacakge for interfacing with the HC-SR04 ultrasonic sensor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultrasonic_distance = hcsr04.ultrasonic_distance:main'
        ],
    },
)
