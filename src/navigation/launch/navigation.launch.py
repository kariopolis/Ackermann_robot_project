#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    params_file = os.path.join(
        get_package_share_directory('navigation'),
        'config',
        'nav2_params.yaml'
    )

    lifecycle_nodes = [
        'planner_server',
        'controller_server',
        'bt_navigator',
        'behavior_server',
        'collision_monitor'
    ]

    return LaunchDescription([

        # ================= NAV2 CORE =================

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
            remappings=[('cmd_vel', 'cmd_vel_nav')]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),

        # ✅ THIS IS THE RECOVERY BEHAVIOR SERVER (JAZZY)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file]
        ),

        # ================= SAFETY =================

        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[params_file]
        ),

        # ================= LIFECYCLE =================

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': lifecycle_nodes
            }]
        ),

        # ================= MOTOR INTERFACE =================

        Node(
            package='navigation',
            executable='cmd_vel_to_speed_control',
            name='cmd_vel_to_speed_control',
            output='screen',
            remappings=[('cmd_vel', 'cmd_vel_smoothed')]
        ),

        # ================= GOAL SETTER =================

        Node(
            package='navigation',
            executable='relative_goal_navigator',
            name='relative_goal_navigator',
            output='screen'
        ),

    ])
