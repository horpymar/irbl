#!/usr/bin/env python3

import launch
import os

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = launch.LaunchDescription()

    uav_name = LaunchConfiguration('uav_name')
    standalone = LaunchConfiguration('standalone')
    custom_config = LaunchConfiguration('custom_config')
    platform_config = LaunchConfiguration('platform_config')
    world_config = LaunchConfiguration('world_config')
    network_config = LaunchConfiguration('network_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', 'uav1'),
        description='The uav name used for namespacing.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'standalone',
        default_value='false',
        description='Whether to start as a standalone or load into an existing container.',
    ))

    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value='',
        description='Path to the custom configuration file.',
    ))
    custom_config = IfElseSubstitution(
        condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
        if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
        else_value=custom_config,
    )

    ld.add_action(DeclareLaunchArgument(
        'platform_config',
        default_value='',
        description='Path to the platform configuration file.',
    ))
    platform_config = IfElseSubstitution(
        condition=PythonExpression(['"', platform_config, '" != "" and ', 'not "', platform_config, '".startswith("/")']),
        if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), platform_config]),
        else_value=platform_config,
    )

    ld.add_action(DeclareLaunchArgument(
        'world_config',
        default_value='',
        description='Path to the world configuration file.',
    ))
    world_config = IfElseSubstitution(
        condition=PythonExpression(['"', world_config, '" != "" and ', 'not "', world_config, '".startswith("/")']),
        if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), world_config]),
        else_value=world_config,
    )

    ld.add_action(DeclareLaunchArgument(
        'network_config',
        default_value='',
        description='Path to the network configuration file.',
    ))
    network_config = IfElseSubstitution(
        condition=PythonExpression(['"', network_config, '" != "" and ', 'not "', network_config, '".startswith("/")']),
        if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), network_config]),
        else_value=network_config,
    )

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', 'false'),
        description='Should the node subscribe to sim time?',
    ))

    container_name = ['/', uav_name, '/uav_core_container']

    ld.add_action(
        ComposableNodeContainer(
            namespace=uav_name,
            name='uav_core_container',
            package='rclcpp_components',
            executable='component_container_mt',
            output='screen',
            parameters=[
                {'use_intra_process_comms': True},
                {'thread_num': os.cpu_count()},
                {'use_sim_time': use_sim_time},
            ],
            condition=UnlessCondition(standalone),
        )
    )

    includes = [
        ('mrs_uav_managers', 'control_manager.launch.py'),
        ('mrs_uav_managers', 'safety_area_manager.launch.py'),
        ('mrs_uav_managers', 'uav_manager.launch.py'),
        ('mrs_uav_managers', 'transform_manager.launch.py'),
        ('mrs_uav_managers', 'constraint_manager.launch.py'),
        ('mrs_uav_managers', 'gain_manager.launch.py'),
        ('mrs_uav_managers', 'estimation_manager.launch.py'),
        ('mrs_uav_trajectory_generation', 'trajectory_generation.launch.py'),
    ]

    for package_name, launch_file in includes:
        launch_arguments = {
            'use_sim_time': use_sim_time,
            'custom_config': custom_config,
            'platform_config': platform_config,
            'standalone': standalone,
            'container_name': container_name,
        }
        if package_name == 'mrs_uav_managers':
            launch_arguments['world_config'] = world_config
            launch_arguments['network_config'] = network_config

        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare(package_name), '/launch/', launch_file,
                ]),
                launch_arguments=launch_arguments.items(),
            )
        )

    return ld