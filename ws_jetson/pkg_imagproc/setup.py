from setuptools import find_packages, setup

package_name = 'pkg_imagproc'

setup(
    name=package_name,
    version='0.0.0',
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
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_roctl = pkg_imagproc.node_rover_ctl:main',
            'node_cam_stream = pkg_imagproc.node_cam1_d415_stream:main',
            'node_nav_process = pkg_imagproc.node_cam1_nav_process:main',
            'node_demo_lane = pkg_imagproc.demo_lane:main',
        ],
    },
)
