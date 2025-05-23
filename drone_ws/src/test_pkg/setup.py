from setuptools import setup

package_name = 'test_pkg'

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
    maintainer='yifei',
    maintainer_email='yifei@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stationary_pub = test_pkg.stationary_pub:main',
            'offboard_node = test_pkg.offboard:main',
            'odom_sub = test_pkg.odom_sub:main'
        ],
    },
)
