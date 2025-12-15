from setuptools import find_packages, setup

package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/navigation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karolis',
    maintainer_email='karovitu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_to_speed_control = navigation.cmd_vel_tospeed_control:main',
            'relative_goal_navigator = navigation.set_goal:main',
        ],
    },
)
