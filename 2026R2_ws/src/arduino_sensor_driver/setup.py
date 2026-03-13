from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arduino_sensor_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EdUHK Robocon Team',
    maintainer_email='robocon@eduhk.hk',
    description='Arduino sensor driver for IMU + Encoder with CRC8 validation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_sensor_parser = arduino_sensor_driver.arduino_sensor_parser_node:main',
        ],
    },
)
