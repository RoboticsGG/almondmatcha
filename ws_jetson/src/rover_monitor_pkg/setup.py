import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_monitor_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yupi',
    maintainer_email='viewjirapat@gmail.com',
    description='Rover local monitoring for CSV/DB logging on Jetson (Domain 4)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_rover_local_monitoring = rover_monitor_pkg.node_rover_local_monitoring:main',
        ],
    },
)
