from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'arm_arduino_praser'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    install_requires=[
        'setuptools',
    ],
    extras_require={'test': ['pytest']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    zip_safe=True,
    maintainer='Steven Zhang Yancheng',
    maintainer_email='yanczhang8@gmail.com',
    description='Arm Arduino sensor parser: serial data parsing for arm-side sensors',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_arduino_node = arm_arduino_praser.arm_arduino_node:main',
        ],
    },
)
