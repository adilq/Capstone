import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import rclpy.time
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
import rclpy.qos
from geometry_msgs.msg import PoseStamped, Twist

import threading 

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        self.rate = self.create_rate(30)

        # subscriber for mavros/state
        self.state_sub = self.create_subscription(State, 'mavros/state', callback = self.state_callback, qos_profile=10)
        self.state = State()

        # create client for set mode, use with wait for service
        self.set_mode_cli = self.create_client(SetMode, 'mavros/set_mode')
        while not self.set_mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_mode service not available')

        # create client for arming
        self.arm_cli = self.create_client(CommandBool, 'mavros/cmd/arming')
        while not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arming service not available')

        # publisher
        self.vel_pub = self.create_publisher(Twist, 'mavros/setpoint_velocity/cmd_vel', 10)

        # odom: in map frame
        self.odom_sub = self.create_subscription(PoseStamped, 'mavros/local_position/pose', callback = self.odom_callback, qos_profile=rclpy.qos.qos_profile_system_default)
        # self.odom_pose = PoseStamped()
        self.odom_pose = None   # init to None to check in main()

    # state callback
    def state_callback(self, msg):
        self.state = msg
        self.get_logger().debug(f"Received {msg}")

    # odom callback
    def odom_callback(self, msg):
        self.odom_pose = msg
        self.get_logger().debug(f"Received {msg}")


def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    while rclpy.ok():
        node.rate.sleep()

    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
