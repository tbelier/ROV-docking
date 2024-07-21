from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'bluerov_ros_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
        glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Titouan',
    maintainer_email='titouan.belier@ensta-bretagne.org',
    description='This package allows us to use the vision and Aruco detection for our robot to plug into the mecanical system.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bluerov_ros_cameraNode = bluerov_ros_camera.bluerov_ros_camera:main'
        ],
    },
)
