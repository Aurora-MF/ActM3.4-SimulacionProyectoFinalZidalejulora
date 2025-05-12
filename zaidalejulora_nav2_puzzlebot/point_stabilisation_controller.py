import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np

class PuzzlebotController(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        self.namespace = self.get_namespace().rstrip('/')
        
        # Declare target position parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('x_goal', 1.0),
                ('y_goal', 1.0),
            ]
        )
        
        self.x_goal = self.get_parameter('x_goal').value
        self.y_goal = self.get_parameter('y_goal').value
        self.theta_goal = 0.0  # not used for now
        
        # Proportional control gains
        self.Kp_linear = 0.5
        self.Kp_angular = 3.0
        
        # Subscribe to odometry to get robot pose
        self.odom_sub = self.create_subscription(
            Odometry, 
            'odom', 
            self.odom_callback,  # callback to compute control
            10)
        
        # Publisher to send velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            'cmd_vel', 
            10)

        # Timer to periodically check for parameter updates
        self.create_timer(1.0, self.update_parameters)
        
        self.get_logger().info(f"Controlador iniciado en {self.namespace}")

    def update_parameters(self):
        # Reload goal position parameters
        self.x_goal = self.get_parameter('x_goal').value
        self.y_goal = self.get_parameter('y_goal').value

    def odom_callback(self, msg):
        # msg: nav_msgs/Odometry with current robot pose
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = 2 * np.arctan2(
            msg.pose.pose.orientation.z, 
            msg.pose.pose.orientation.w)  # convert quaternion to yaw
        
        # Compute errors in position and orientation
        error_x = self.x_goal - x
        error_y = self.y_goal - y
        error_distance = np.sqrt(error_x**2 + error_y**2)
        error_theta = np.arctan2(error_y, error_x) - theta
        error_theta = np.arctan2(np.sin(error_theta), np.cos(error_theta))  # normalize angle
        
        cmd_vel = Twist()
        if error_distance > 0.05:
            # Compute velocity commands using proportional control
            cmd_vel.linear.x = self.Kp_linear * error_distance
            cmd_vel.angular.z = self.Kp_angular * error_theta
        else:
            self.get_logger().info("Goal Reached!")
        
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotController()
    try:
        rclpy.spin(node)  # keep node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
