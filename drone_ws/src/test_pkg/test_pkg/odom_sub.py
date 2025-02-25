import rclpy 
import rclpy.qos
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import threading 

class ReaderNode(Node):
    def __init__(self):
        super().__init__('odom_subscriber')

        self.rate = self.create_rate(50)

        self.odom_sub = self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.odom_callback,
                                                    rclpy.qos.qos_profile_system_default)

    def odom_callback(self, msg):
        self.get_logger().info(f"Received {msg}")

def main(args=None):
    rclpy.init(args=args)
    node = ReaderNode()

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    while rclpy.ok():
        node.rate.sleep()

    rclpy.shutdown()
    thread.join()