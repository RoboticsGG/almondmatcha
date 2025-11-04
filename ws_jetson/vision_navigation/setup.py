from setuptools import find_packages, setup

package_name = 'vision_navigation'

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
    description='Visual navigation system for autonomous rover',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_stream = vision_navigation_pkg.camera_stream_node:main',
            'lane_detection = vision_navigation_pkg.lane_detection_node:main',
            'steering_control = vision_navigation_pkg.steering_control_node:main',
            'demo_lane = vision_navigation_pkg.demo_lane:main',
        ],
    },
)
