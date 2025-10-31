import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_launch_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 1. Standard requirement: Install the package.xml file
        ('share/' + package_name, ['package.xml']),
        
        # 2. Add the package index marker to silence the colcon warning
        (os.path.join('share', package_name, 'resource'), ['resource/' + package_name]),
        
        # 3. Install all files found in the 'launch' directory
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][y|m]'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='curry',
    maintainer_email='curry@todo.com',
    description='Centralized launch system for the rover.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # Since this package only contains launch files and no executables, this section is empty.
    entry_points={
        'console_scripts': [
        ],
    },
)
