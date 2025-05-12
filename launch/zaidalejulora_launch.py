#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.events import Shutdown

def generate_launch_description():
    pkg = 'zaidalejulora_nav2_puzzlebot'
    pkg_share = FindPackageShare(pkg).find(pkg)

    # ─────── ARGUMENTOS DE ENTRADA ─────────────────────────────
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='all',
        description="Modo de lanzamiento: 'sim', 'map', 'nav', o 'all'"
    )

    mode = LaunchConfiguration('mode')

    # ─────── PATHS ─────────────────────────────────────────────
    world_path = os.path.join(pkg_share, 'worlds', 'world.world')
    urdf_path = os.path.join(pkg_share, 'urdf', 'puzzlebot.urdf')

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"[ERROR] No se encontró el URDF: {urdf_path}")
    
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()
        
    rviz_mapping_config = os.path.join(get_package_share_directory('zaidalejulora_nav2_puzzlebot'),'rviz','mapping.rviz')
    rviz_nav_config = os.path.join(get_package_share_directory('zaidalejulora_nav2_puzzlebot'),'rviz','navigation.rviz')

    # ─────── 0) SIMULACIÓN GAZEBO ─────────────────────────────
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.dirname(pkg_share)
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('ros_gz_sim').find('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_path}.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' in ['sim','all','map','nav']"]))
    )

    # ─────── 1) ROBOT ─────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}, {'use_sim_time': True}],
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' in ['sim','all','map','nav']"]))
    )

    joint_state_publisher = Node(
        package=pkg,
        executable='joint_state_publisher',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' in ['sim','all','map','nav']"]))
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'puzzlebot',
            '-x',     '0.3',
            '-y',     '2.5',
            '-z',     '0.0',
            '-roll',  '0.0',
            '-pitch', '0.0',
            '-yaw',   '0.0'
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' in ['sim','all','map','nav']"]))
    )

    # ─────── 2) MODULOS ESPECÍFICOS POR MODO ─────────────────
    # Modo 'sim' o 'all'
    puzzlebot_sim = Node(
        package=pkg,
        executable='puzzlebot_sim',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' in ['sim','all']"]))
    )

    localisation_node = Node(
        package=pkg,
        executable='localisation',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression(["'", mode, "' in ['sim','all']"]))
    )

    point_stab_node = Node(
        package=pkg,
        executable='point_stabilisation_controller',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' in ['sim','all']"]))
    )

    # Mapping
    rviz_mapping = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapping',
        arguments=['-d', rviz_mapping_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'map'"]))
    )

    # Navigation
    rviz_navigation = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_navigation',
        arguments=['-d', rviz_nav_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'nav'"]))
    )
    
    return LaunchDescription([
        mode_arg,
        LogInfo(msg=['[MAIN] Launch mode: ', mode]),
        set_ign_resource_path,
        gazebo_launch,

        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,

        puzzlebot_sim,
        localisation_node,
        point_stab_node,

        rviz_mapping,
        rviz_navigation,
    ])
