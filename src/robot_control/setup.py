from setuptools import find_packages, setup
import os

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            ['launch/robot_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karolis',
    maintainer_email='karovitu@gmail.com',
    description='Robot control publisher and GUI',
    license='TODO: License declaration',

    entry_points={
        'console_scripts': [
            # executable name  =  package.module:function
            'robot_control_publisher = robot_control.control_publisher:main',
            'odom_to_tf = robot_control.odom_to_tf:main',
            'pure_pursuit = robot_control.pure_pursuit:main',
            'pfm = robot_control.PFM:main',


        ],
    },
)
