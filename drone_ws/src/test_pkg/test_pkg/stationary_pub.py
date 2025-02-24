import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion

class PosePub(Node):
    def __init__(self):
        super().__init__('stationary_pub')

        self.publisher_ = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        timer_period = 1/20
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 0.
        pose.pose.position.y = 0.
        pose.pose.position.z = 1.
        pose.pose.orientation = Quaternion(x=0., y=0., z=0., w=1.)

        self.pose = pose

    def timer_callback(self):
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.pose)
        self.get_logger().info(f"Publishing: {self.pose}")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = PosePub()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
