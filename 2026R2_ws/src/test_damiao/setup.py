from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'test_damiao'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=[
        'setuptools',
        'pyserial>=3.0,<4.0',
        'numpy',
    ],
    extras_require={'test': ['pytest']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    zip_safe=True,
    maintainer='Steven Zhang Yancheng',
    maintainer_email='yanczhang8@gmail.com',
    description='Damiao CAN motor test package for feedback and sensor-mode experiments.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'damiao_node = test_damiao.damiao_node:main',
        ],
    },
)
