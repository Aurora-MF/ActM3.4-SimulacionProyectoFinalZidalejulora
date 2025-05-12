import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

class PuzzlebotPhysicalSimulator(Node):
    def __init__(self):
        super().__init__('puzzlebot_sim')
        
        self.namespace = self.get_namespace().rstrip('/')
        
        self.wheel_radius = 0.05
        self.wheel_separation = 0.19
        
        # Subscribe to velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            'cmd_vel', 
            self.cmd_vel_callback,  # callback for velocity
            10)
        
        # Publishers for wheel angular velocities
        self.wr_pub = self.create_publisher(Float32, 'wr', 10)
        self.wl_pub = self.create_publisher(Float32, 'wl', 10)
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(
            JointState, 
            'joint_states', 
            10)
        
        self.wr = 0.0  # right wheel angular velocity
        self.wl = 0.0  # left wheel angular velocity
        self.start_time = self.get_clock().now()
        
        self.sample_time = 0.1  # seconds
        # Timer to update joint states periodically
        self.timer = self.create_timer(
            self.sample_time, 
            self.update_joints)
        
        self.get_logger().info(f"Simulador cinemático iniciado en {self.namespace}")

    def cmd_vel_callback(self, msg):
        # msg: geometry_msgs/Twist with linear and angular velocity
        
        v = msg.linear.x   # linear velocity
        w = msg.angular.z  # angular velocity
        
        # Compute angular velocities for each wheel
        self.wr = (2 * v + w * self.wheel_separation) / (2 * self.wheel_radius)
        self.wl = (2 * v - w * self.wheel_separation) / (2 * self.wheel_radius)
        
        # Publish right wheel velocity
        wr_msg = Float32()
        wr_msg.data = self.wr
        self.wr_pub.publish(wr_msg)
        
        # Publish left wheel velocity
        wl_msg = Float32()
        wl_msg.data = self.wl
        self.wl_pub.publish(wl_msg)

    def update_joints(self):
        # Publishes accumulated joint positions based on velocities
        
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [
            'wheel_left_joint', 
            'wheel_right_joint'
        ]
        
        # Time since node started
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Approximate joint position as velocity * time
        joint_msg.position = [
            self.wl * elapsed_time,
            self.wr * elapsed_time
        ]
        self.joint_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotPhysicalSimulator()
    try:
        rclpy.spin(node)  # keep node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
