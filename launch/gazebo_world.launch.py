import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,         # Para permitir pasar argumentos desde la terminal
    SetEnvironmentVariable,        # Para modificar variables de entorno
    IncludeLaunchDescription,       # Para incluir otros launch files
    OpaqueFunction
)

from launch.substitutions import (
    LaunchConfiguration,           # Para referenciar los argumentos pasados al launch
    PathJoinSubstitution,          # Para construir rutas válidas
    Command                        # Para ejecutar comandos como xacro
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    map_name = LaunchConfiguration('map_name').perform(context)


    world_file = f"{map_name}_world.world"      # Mundo SDF que se cargará
    robot = 'puzzlebot_jetson_lidar_ed'      # Variante del robot Puzzlebot que se usará

    # Posición inicial del robot (coordenadas x, y, orientación yaw)
    if map_name == 'hexagonal':
        pos_x = '-1.2'
        pos_y = '1.2'
        pos_th = '0.0'
    elif map_name == 'puzzlebot':
        pos_x = '0.0'
        pos_y = '0.0'
        pos_th = '0.0'

    # Configuración del tiempo y pausa en la simulación
    sim_time = 'true'      # Indica si se debe usar tiempo simulado
    pause_gazebo = 'false' # Determina si Gazebo inicia en pausa

    # Nivel de verbosidad para mensajes de Gazebo
    gazebo_verbosity = 4   # Mayor número = más detalles


    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_resources = get_package_share_directory('zaidalejulora_nav2_puzzlebot')

    robot_path = os.path.join(gazebo_resources, 'urdf', 'mcr2_robots', f"{robot}.xacro")         # Path al archivo xacro
    world_path = os.path.join(gazebo_resources, 'worlds', world_file)                            # Path al mundo
    gazebo_models_path = os.path.join(gazebo_resources, 'models')                                # Carpeta de modelos
    gazebo_plugins_path = os.path.join(gazebo_resources, 'plugins')                              # Carpeta de plugins
    gazebo_media_path = os.path.join(gazebo_models_path, 'models', 'media', 'materials')         # Texturas y materiales
    default_ros_gz_bridge_config_file_path = os.path.join(gazebo_resources, 'config', f"{robot}.yaml") # Archivo de configuración del bridge

   
    declare_x_arg = DeclareLaunchArgument('x', default_value=pos_x, description='Posición X del robot')
    declare_y_arg = DeclareLaunchArgument('y', default_value=pos_y, description='Posición Y del robot')
    declare_th_arg = DeclareLaunchArgument('yaw', default_value=pos_th, description='Ángulo del robot (Yaw)')
    declare_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=sim_time, description='Usar tiempo simulado')
    declare_pause_arg = DeclareLaunchArgument('pause', default_value=pause_gazebo, description='Iniciar Gazebo pausado')

    # Se almacenan los argumentos para ser usados luego
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')
    pause = LaunchConfiguration('pause')


    # Incluye los directorios con modelos y materiales personalizados
    set_gazebo_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{gazebo_models_path}:{gazebo_media_path}"
    )

    # Incluye los plugins personalizados de Gazebo (si los hay)
    set_gazebo_plugins = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=f"{gazebo_plugins_path}"
    )

    robot_description = Command(['xacro ', str(robot_path)])
    # Esto ejecuta el archivo Xacro y devuelve un string con el contenido URDF del robot.

    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': ['-r', f'-v {gazebo_verbosity} ', world_path],
            'on_exit_shutdown': 'true',
            'pause': pause
        }.items()
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(robot_description, value_type=str),
            "use_sim_time": use_sim_time,
        }],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "puzzlebot",
            "-topic", "robot_description",
            "-x", x, "-y", y, "-Y", yaw,
        ],
        output="screen",
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': default_ros_gz_bridge_config_file_path,
        }],
        output='screen'
    )

    start_gazebo_ros_image_bridge_cmd = None
    if robot != "puzzlebot_hacker_ed":
        start_gazebo_ros_image_bridge_cmd = Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['camera']
        )


    l_d = [
        declare_x_arg, declare_y_arg, declare_th_arg, declare_sim_time_arg, declare_pause_arg,
        set_gazebo_resources, set_gazebo_plugins,
        robot_state_publisher_node,
        start_gazebo_server,
        spawn_robot,
        start_gazebo_ros_bridge_cmd
    ]

    # Si se usa cámara, se añade el bridge de imagen
    if start_gazebo_ros_image_bridge_cmd:
        l_d.append(start_gazebo_ros_image_bridge_cmd)

    # Launch path base
    launch_dir = os.path.join(gazebo_resources, 'launch')

    if mode == 'map':
        l_d.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'mapping.launch.py'))
            )
        )
    elif mode == 'nav':
        l_d.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation.launch.py')),
                launch_arguments={'map_name': LaunchConfiguration('map_name')}.items()
            )
        )
    else:
        raise RuntimeError(f"[gazebo_world.launch.py] Unknown mode '{mode}', expected 'map' or 'nav'.")

    return l_d

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='map',
            description='Execution mode: map or nav'
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='hexagonal',
            description='Map/world to use: hexagonal or puzzlebot'
        ),
        OpaqueFunction(function=launch_setup)
    ])
