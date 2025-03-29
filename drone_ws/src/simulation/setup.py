import os
from glob import glob
from setuptools import setup

package_name = 'simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
        glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yifei',
    maintainer_email='yifei@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_publisher = simulation.follow_joint_path:main',
            'simple_waypoints = simulation.simple_path:main',
            'line_path = simulation.line_path:main',
            'loop_path = simulation.loop_path:main',
            'joint_to_pose = simulation.publish_position:main'
        ],
    },
)
