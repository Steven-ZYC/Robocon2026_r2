from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'routes'), glob('routes/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Steven Zhang Yancheng',
    maintainer_email='s11766@s.eduhk.hk',
    description='Global navigation node for R2 omniwheel base.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'global_navigation_node = navigation.global_navigation_node:main'
        ],
    },
)
