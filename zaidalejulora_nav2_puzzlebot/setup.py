from setuptools import find_packages, setup

package_name = 'zaidalejulora_nav2_puzzlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
<<<<<<< HEAD
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auromtf',
    maintainer_email='auromtf@todo.todo',
    description='TODO: Package description',
=======
        ('share/' + package_name + '/launch', ['launch/launch.py']),  # Actualizado el nombre del archivo de lanzamiento
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zaidal',
    maintainer_email='zaidal@todo.todo',
    description='Package for the PuzzleBot navigation and simulation in Gazebo and RViz',
>>>>>>> 4c5e3ac (Se agrega archivo launch con mapa)
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
