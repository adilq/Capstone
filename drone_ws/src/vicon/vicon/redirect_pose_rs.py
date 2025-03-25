import rclpy
from rclpy.node import Node
import rclpy.qos
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class Redirector_rs(Node):
    def __init__(self):
        super().__init__('redirector_node_rs')
        self.publisher_ = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        self.subscriber_ = self.create_subscription(Odometry, '/camera/pose/sample', self.pose_callback, rclpy.qos.qos_profile_system_default)
        self.subscriber_
        
        self.get_logger().info("node up")

    def pose_callback(self, msg):
        self.get_logger().info(f"received: {msg.pose.pose}")
        out_msg = PoseStamped()
        out_msg.header.frame_id = 'map'
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.pose = msg.pose.pose
        self.publisher_.publish(out_msg)
    
def main(args=None):
    rclpy.init(args=args)
    redirector = Redirector_rs()
    rclpy.spin(redirector)

if  __name__ == '__main__':
    main()
