# TE3003B - zaidalejulora_nav2_puzzlebot (Team 3)

**Description**  
This project demonstrates the creation and simulation of a ROS 2 package focused on autonomous navigation using the PuzzleBot robot in a custom Gazebo Garden environment. 


**Authors** 
- Zaida Irais López Mendieta - A01708755
- Daniela Aurora Martínez Fajardo - A01709293
- Francisco Alejandro Velázquez Ledesma - A01709475
- Julio David REséndiz Cruz - A01709375

**License**  
cambiar license a la de MIT


**Requirements**
- Ubuntu 22.04
- ROS2 Humble
- Python3
- Gazebo Garden

**Commands To Run (Mapping)**
```
 En terminal 1:
 cd ros2_ws/
ros2 launch  zaidalejulora_nav2_puzzlebot gazebo_world.launch.py mode:=map use_sim_time:=True

En Terminal 2:
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

En Terminal 3 para mover el robot:
ros2 run turtlebot3_teleop teleop_keyboard



```

**Commands To Run (Navigation)**
```
 En Terminal 1:
 cd ros2_ws/
ros2 launch  zaidalejulora_nav2_puzzlebot gazebo_world.launch.py mode:=nav use_sim_time:=True

```
