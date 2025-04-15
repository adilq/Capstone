from setuptools import setup

package_name = 'cv_basics'

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
            'img_publisher = cv_basics.webcam_pub:main',
            'img_subscriber = cv_basics.webcam_sub:main',
            'img_sub_compressed = cv_basics.webcam_sub_compressed:main',
            'img_pub_compressed = cv_basics.webcam_pub_compressed:main',
            'pipeline_yolov8 = cv_basics.pipeline_yolov8:main',
            'pipeline_aruco = cv_basics.pipeline_aruco:main',
            'sim_yolov8 = cv_basics.sim_yolov8:main'
        ],
    },
)
