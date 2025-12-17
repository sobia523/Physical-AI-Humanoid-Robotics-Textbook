from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'module2_digital_twin'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*launch.[pxy][yeml]'))),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='ROS 2 package for Module 2 Digital Twin examples',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_processor = module2_digital_twin.lidar_processor:main',
            'simple_navigator = module2_digital_twin.simple_navigator:main',
        ],
    },
)
