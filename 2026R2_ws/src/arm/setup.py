from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'arm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    install_requires=[
        'setuptools',
        'pyserial>=3.0,<4.0',
        'numpy',
    ],
    extras_require={'test': ['pytest']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    zip_safe=True,
    maintainer='Steven Zhang Yancheng',
    maintainer_email='yanczhang8@gmail.com',
    description='Damiao-motor-driven robotic arm control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_ctrl_node = arm.arm_ctrl_node:main',
            'arm_damiao_node = arm.arm_damiao_node:main',
        ],
    },
)
