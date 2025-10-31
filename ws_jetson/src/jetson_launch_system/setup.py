from setuptools import setup # 1. Make sure setuptools is imported
import os
from glob import glob

package_name = 'jetson_launch_system'

# 2. Ensure the setup function is called at the end of the file
setup(
    name=package_name,
    version='0.0.0',
    packages=[], # Must be present, even if empty
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install the launch directory
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Package for system-level launch configurations.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)