from setuptools import setup

package_name = 'plot_debug'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Steven Zhang Yancheng',
    maintainer_email='yanczhang8@gmail.com',
    description='Real-time matplotlib visualisation for Robocon debugging',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plot_debug_node = plot_debug.plot_debug_node:main',
        ],
    },
)
