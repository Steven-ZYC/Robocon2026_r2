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
        'pyserial>=3.0,<4.0',
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
    description='TODO: Package description',
    license='Apache-2.0',
#    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'damiao_node = base_omniwheel_r2_700.damiao_node:main',
            'local_navigation_node = base_omniwheel_r2_700.local_navigation_node:main',
        ],
    },
)
