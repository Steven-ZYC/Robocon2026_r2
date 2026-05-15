from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'base_omniwheel_r2_700'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=[
        'setuptools',
        'numpy'
    ],
    extras_require={'test': ['pytest']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    zip_safe=True,
    maintainer='steven',
    maintainer_email='yanczhang8@gmail.com',
    description='Omniwheel chassis kinematics and local navigation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'local_navigation_node = base_omniwheel_r2_700.local_navigation_node:main',
        ],
    },
)
