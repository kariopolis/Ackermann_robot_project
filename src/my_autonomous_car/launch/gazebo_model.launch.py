import os
import launch
import launch_ros
import xacro
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# Define xacro robot name
robotXacroName='robot'

# Define package name
namePackage='my_autonomous_car'

# Define model file path
modelFileRelativePath = 'description/robot.urdf.xacro'

# Define rviz config file path
rvizRelativePath = 'config/view_car.rviz'

# Define relative path of the gz_ros2_control configuration file with respect to the pkg folder
ros2ControlConfigRelativePath = 'config/gz_ros2_control.yaml'




def generate_launch_description():
    #absolute pkg path
    pkgPath = launch_ros.substitutions.FindPackageShare(package=namePackage).find(namePackage)

    #absolute xacro_model_path
    xacroModelPath = os.path.join(pkgPath, modelFileRelativePath)

    #absolute rviz config path
    rvizConfigPath = os.path.join(pkgPath, rvizRelativePath)

    #absolute ros2_control config path
    ros2ControlConfigPath =  os.path.join(pkgPath, ros2ControlConfigRelativePath)

    #get the robot description from the xacro file
    robotDesccriptionXML = xacro.process_file(xacroModelPath).toxml()

    #define parameters with the robot xacro description
    robotDesccription= {'robot_description': robotDesccriptionXML}

    #Declare arguments
    declared_arguments = []
    declared_arguments.append(launch.actions.DeclareLaunchArgument(name="gui", default_value="true", description="Start the RViz2 GUI"))

    #Initialize arguments
    gui = LaunchConfiguration("gui")

    #Start Gazebo headless if argumet gui is false
    gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_ros.substitutions.FindPackageShare(package='ros_gz_sim'), '/launch/gz_sim.launch.py']
        ),
        launch_arguments=[('gz_args', '-r -v 4 empty.sdf')], # -r to reset simulation, -v 4 for verbose output, empty world
        condition=launch.conditions.IfCondition(gui))
    

    #Start Gazebo headless if argumet gui is false
    gazebo_headless = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_ros.substitutions.FindPackageShare(package='ros_gz_sim'), '/launch/gz_sim.launch.py']
        ),
        launch_arguments=[('gz_args', ['-s -r -v 3 empty.sdf'])], # -s to start headless, -r to reset simulation, -v 3 for less verbose output, empty world
        condition=launch.conditions.UnlessCondition(gui))
    
    #Gazebo bridge 
    bridge_params = os.path.join(
    get_package_share_directory(namePackage),
    'config',
    'bridge_parameters.yaml'
    )

    # Start Gazebo ROS bridge node
    start_gazebo_ros_bridge_cmd = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', 
            f'config_file:={bridge_params}',
                ],
        output='screen',)

    # Start spawn model node in Gazebo
    spawnModelNodeGazebo = launch_ros.actions.Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-topic',
        'robot_description',
        '-name',
        'robot',
        '-allow_renaming',
        'true'
        ],
    output='screen',
    )

    # Start robot state publisher node
    nodeRobotStatePublisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='both',
    parameters=[robotDesccription]
    )

    # Start rviz node
    rvizNode = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizConfigPath]
    )

    #Start ros2_control node
    control_node = launch_ros.actions.Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ros2ControlConfigPath],
        output='both',
    )

    #Start joint state broadcaster node
    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # #Start forward velocity controller node
    # forward_velocity_controller_spawner = launch_ros.actions.Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['forward_velocity_controller', "--param-file", ros2ControlConfigPath],
    # )

    # #Start forward position controller node
    # forward_position_controller_spawner = launch_ros.actions.Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['forward_position_controller', "--param-file", ros2ControlConfigPath],
    # )
    ackermann_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller',
            '--param-file', ros2ControlConfigPath,
            '--controller-ros-args',
            # remap TF odometry -> /tf
            '-r', 'ackermann_steering_controller/tf_odometry:=/tf',],
        
    )

#     ackermann_controller_spawner = launch_ros.actions.Node(
#     package='controller_manager',
#     executable='spawner',
#     arguments=[
#         'ackermann_steering_controller',
#         '--param-file', ros2ControlConfigPath,
#         '--controller-ros-args',
#         # remap TF odometry -> /tf
#         '-r', 'ackermann_steering_controller/tf_odometry:=/tf',
#     ],
#     output='screen',
# )
    nodeList = [
        #gazebo,
        #gazebo_headless,
        #start_gazebo_ros_bridge_cmd,
        #spawnModelNodeGazebo,

        #control_node,                       # 1 start control
        #joint_state_broadcaster_spawner,    # 2 publish joint states
        #ackermann_controller_spawner,       # 3 publish odom->base_link TF

        nodeRobotStatePublisher,            # 4 robot_state_publisher LAST
        rvizNode
    ]

    return launch.LaunchDescription(declared_arguments + nodeList)



