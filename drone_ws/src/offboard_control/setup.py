from setuptools import setup

package_name = 'offboard_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adilq',
    maintainer_email='adil.quettawala@mail.utoronto.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'skeleton = offboard_control.comm_node_skeleton:main',
            'flight2 = offboard_control.flight_test_2:main',
            'flight3 = offboard_control.flight_test_3:main',
            'waypoints = offboard_control.waypoint_node:main'
        ],
    },
)
