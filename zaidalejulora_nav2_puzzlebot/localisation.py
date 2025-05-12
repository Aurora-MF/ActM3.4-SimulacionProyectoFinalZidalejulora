import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np

class Localisation(Node):
    def __init__(self):
        super().__init__('dead_reckoning')
        
        self.namespace = self.get_namespace().rstrip('/')
        
        self.wheel_radius = 0.05
        self.wheel_separation = 0.19
        
        self.x = 0.0  # robot x position
        self.y = 0.0  # robot y position
        self.theta = 0.0  # robot orientation (yaw)
        self.wr = 0.0  # right wheel angular velocity
        self.wl = 0.0  # left wheel angular velocity
        
        # Subscribe to right wheel velocity
        self.create_subscription(
            Float32, 
            'wr', 
            self.wr_callback,  # callback to update wr
            10)
        
        # Subscribe to left wheel velocity
        self.create_subscription(
            Float32, 
            'wl', 
            self.wl_callback,  # callback to update wl
            10)
        
        # Publisher for odometry message
        self.odom_pub = self.create_publisher(
            Odometry, 
            'odom', 
            10)
        
        # TF broadcaster for transform between /odom and base_footprint
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.sample_time = 0.02  # seconds
        # Timer to periodically update odometry
        self.create_timer(
            self.sample_time, 
            self.update_odometry)
        
        self.get_logger().info(f"Nodo de odometría iniciado en {self.namespace}")

    def wr_callback(self, msg):
        # msg: std_msgs/Float32 with right wheel velocity
        self.wr = msg.data

    def wl_callback(self, msg):
        # msg: std_msgs/Float32 with left wheel velocity
        self.wl = msg.data

    def update_odometry(self):
        # Estimate pose using differential drive dead reckoning
        
        v = (self.wheel_radius * (self.wr + self.wl)) / 2.0  # linear velocity
        omega = (self.wheel_radius * (self.wr - self.wl)) / self.wheel_separation  # angular velocity
        
        dt = self.sample_time
        # Update robot position and orientation
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += omega * dt
        
        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = '/odom'
        odom_msg.child_frame_id = f'{self.namespace}/base_footprint'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = np.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = np.cos(self.theta / 2.0)
        self.odom_pub.publish(odom_msg)
        
        # Broadcast the transform for TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = odom_msg.header.stamp
        tf_msg.header.frame_id = '/odom'
        tf_msg.child_frame_id = f'{self.namespace}/base_footprint'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Localisation()
    try:
        rclpy.spin(node)  # keep node alive
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
