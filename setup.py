from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'zaidalejulora_nav2_puzzlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl'))), 
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'rviz/mapping'), glob('rviz/mapping/*.rviz')),
        (os.path.join('share', package_name, 'rviz/navigation'), glob('rviz/navigation/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zaida Irais López Mendieta\nDaniela Aurora Matínez Fajardo\nFrancisco Alejandro Velázquez Ledesma\nJulio David Reséndiz Cruz',
    maintainer_email='A01708755@tec.mx\nA01708755@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher = zaidalejulora_nav2_puzzlebot.joint_state_publisher:main',
            'puzzlebot_sim = zaidalejulora_nav2_puzzlebot.puzzlebot_sim:main',
            'point_stabilisation_controller = zaidalejulora_nav2_puzzlebot.point_stabilisation_controller:main',
            'localisation = zaidalejulora_nav2_puzzlebot.localisation:main',
            'publish_wr_wl = zaidalejulora_nav2_puzzlebot.publish_wr_wl:main',
            'frame = zaidalejulora_nav2_puzzlebot.frame:main',
        ],
    },
)
