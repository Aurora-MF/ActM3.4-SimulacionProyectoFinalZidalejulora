import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Obtener la ruta al directorio compartido del paquete
    pkg_share = get_package_share_directory('zaidalejulora_nav2_puzzlebot')
    
    # Crear la ruta completa al archivo SDF
    sdf_file = os.path.join(pkg_share, 'worlds', 'model.sdf')

    # Verificar si el archivo SDF existe
    if not os.path.exists(sdf_file):
        raise FileNotFoundError(f"El archivo SDF no se encuentra en: {sdf_file}")

    # Lanzar Gazebo con el archivo SDF
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', sdf_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Devolver la descripción de la ejecución
    return LaunchDescription([gazebo])