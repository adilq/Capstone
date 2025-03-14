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
        self.pose_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)

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

    node.get_logger().info("Node online.")

    # wait for odom message
    while rclpy.ok() and not node.odom_pose:
        node.rate.sleep()

    # publish poses for offboard
    cmd = PoseStamped()
    cmd.pose.position = node.odom_pose.pose.position
    cmd.header.frame_id = "map"
    cmd.header.stamp = node.get_clock().now().to_msg()

    vel_cmd = Twist()

    # wait to connect
    while rclpy.ok() and not node.state.connected:
        node.rate.sleep()
    node.get_logger().info("Node connected.")

    # for arm and offboard
    offb_set_mode = SetMode.Request()
    offb_set_mode.custom_mode = "OFFBOARD"
    arm_cmd = CommandBool.Request()
    arm_cmd.value = True 

    # logic variables
    prev_request = node.get_clock().now()
    counter = 0
    counter_total = 100
    
    node.get_logger().info("Starting loop.")

    while rclpy.ok():
        # check if armed and in offboard mode
        if node.state.armed and node.state.mode == "OFFBOARD":
            if(node.odom_pose.pose.position.z > 1.0):
                vel_cmd.linear.z = 0.0
            else:
                vel_cmd.linear.z = 0.4
            print(vel_cmd)
            node.vel_pub.publish(vel_cmd)
        else:
            if counter >= counter_total and node.get_clock().now() - prev_request > Duration(seconds=2.0):
                # arm and set mode (try every 5 seconds)
                node.get_logger().debug(f"current mode: {node.state.mode}")
                if not node.state.armed:
                # if not node.state.armed and node.state.mode == "OFFBOARD":
                    node.get_logger().debug("attempting to arm")
                    if node.arm_cli.call(arm_cmd).success:
                        node.get_logger().info("Vehicle armed")
                if node.state.armed and node.state.mode != "OFFBOARD":
                # if node.state.mode != "OFFBOARD":
                    node.get_logger().debug("attempting to offboard")
                    if node.set_mode_cli.call(offb_set_mode).mode_sent:
                        node.get_logger().info("OFFBOARD enabled")   
                prev_request = node.get_clock().now()
            
            # publish to setpoint_local until counter == counter_total                
            counter += 1

            # publish setpoint
            cmd.header.stamp = node.get_clock().now().to_msg()
            node.pose_pub.publish(cmd)
        node.rate.sleep()

    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
