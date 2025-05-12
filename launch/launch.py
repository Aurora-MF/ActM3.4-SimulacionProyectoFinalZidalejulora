#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    pkg = 'zaidalejulora_nav2_puzzlebot'
    pkg_share = get_package_share_directory(pkg)

    # 1) Cargar URDF
    urdf_path = os.path.join(pkg_share, 'urdf', 'puzzlebot.urdf')
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    # 2) Ajustar variable de entorno para Ignition
    set_ign_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',  #se actualizó IGN → GZ
        value=os.path.dirname(pkg_share)
    )

    # 3) Ruta al mundo
    world_path = os.path.join(pkg_share, 'worlds', 'world.world')

    # 4) Lanzar Ignition Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # 5) Publicador del URDF (TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 6) Joint State Publisher (desde tu paquete)
    joint_state_publisher = Node(
        package=pkg,
        executable='joint_state_publisher',
        output='screen'
    )

    # 7) Spawnear el robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'puzzlebot',
            '-x',     '0.3',
            '-y',     '0.3',
            '-z',     '0.0',
            '-roll',  '0.0',
            '-pitch', '0.0',
            '-yaw',   '0.0'
        ]
    )

    # 8) Nodos del robot
    puzzlebot_sim = Node(
        package=pkg,
        executable='puzzlebot_sim',
        output='screen'
    )
    localisation_node = Node(
        package=pkg,
        executable='localisation',
        output='screen'
    )
    point_stab_node = Node(
        package=pkg,
        executable='point_stabilisation_controller',
        output='screen'
    )
    shape_drawer = Node(
        package=pkg,
        executable='shapeDrawer',
        parameters=[{'shape': 'square'}, {'size': 1.0}],
        output='screen'
    )


    # Argumentos para habilitar mapeo / navegación en RViz
    mapping_arg = DeclareLaunchArgument(
        'mapping',
        default_value='true',
        description='Habilita RViz para mapeo'
    )
    navigation_arg = DeclareLaunchArgument(
        'navigation',
        default_value='true',
        description='Habilita RViz para navegación'
    )

       # Configs de RViz
    rviz_mapping_config = PathJoinSubstitution([
        FindPackageShare(pkg),
        'rviz', 'mapping', 'mapping_config.rviz'
    ])
    rviz_nav_config = PathJoinSubstitution([
        FindPackageShare(pkg),
        'rviz', 'navigation', 'navigation_config.rviz'
    ])


    return LaunchDescription([
        set_ign_path,
        gazebo_launch,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        puzzlebot_sim,
        localisation_node,
        point_stab_node,
        shape_drawer,
         mapping_arg,
        navigation_arg,

        # Nodo RViz para mapeo (solo si mapping=='true')
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_mapping',
            output='screen',
            arguments=['-d', rviz_mapping_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('mapping'))
        ),

        # Nodo RViz para navegación (solo si navigation=='true')
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_navigation',
            output='screen',
            arguments=['-d', rviz_nav_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('navigation'))
        ),
    ])
