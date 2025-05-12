from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'zaidalejulora_nav2_puzzlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # Meshes
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl'))),
        # World files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        #rvizzz mapping y nvigatiom
        (os.path.join('share', package_name, 'rviz/mapping'), glob('rviz/mapping/*.rviz')),
        (os.path.join('share', package_name, 'rviz/navigation'), glob('rviz/navigation/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auromtf',
    maintainer_email='A01709293@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'puzzlebot_sim = zaidalejulora_nav2_puzzlebot.puzzlebot_sim:main',
            'localisation = zaidalejulora_nav2_puzzlebot.localisation:main',
            'point_stabilisation_controller = zaidalejulora_nav2_puzzlebot.point_stabilisation_controller:main',
            'shapeDrawer = zaidalejulora_nav2_puzzlebot.shapeDrawer:main',
            'joint_state_publisher = zaidalejulora_nav2_puzzlebot.joint_state_publisher:main',
        ],
    },
)
