import rclpy
import rclpy.duration
from rclpy.node import Node
import rclpy.time
from std_srvs.srv import Empty, Trigger
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
import numpy as np

COMMAND = 'ground'
goal_pos = Pose()

# Callback handlers
def handle_launch():
    # publish to some topic that tells the offb_node to do some predetermined launch sequence
    COMMAND = 'launch'
    print('Launch Requested. Your drone should take off.')

def handle_test():
    # publish to some topic that tells the offb_node to hover in place
    COMMAND = 'test'
    print('Test Requested. Your drone should perform the required tasks. Recording starts now.')

def handle_land():
    # publish to some topic that tells the offb_node to do some predetermined land sequence
    COMMAND = 'land'
    print('Land Requested. Your drone should land.')

def handle_abort():
    # publish to some topic that tells the offb_node to kill the motors? land? switch to manual control?
    COMMAND = 'abort'
    print('Abort Requested. Your drone should land immediately due to safety considerations')

# Service callbacks
def callback_launch(request, response):
    handle_launch()
    response.success = True
    return response

def callback_test(request, response):
    handle_test()
    response.success = True
    return response

def callback_land(request, response):
    handle_land()
    response.success = True
    return response

def callback_abort(request, response):
    handle_abort()
    response.success = True
    return response

class CommNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_XX')
        self.srv_launch = self.create_service(Trigger, 'rob498_drone_XX/comm/launch', callback_launch)
        self.srv_test = self.create_service(Trigger, 'rob498_drone_XX/comm/test', callback_test)
        self.srv_land = self.create_service(Trigger, 'rob498_drone_XX/comm/land', callback_land)
        self.srv_abort = self.create_service(Trigger, 'rob498_drone_XX/comm/abort', callback_abort)

        self.rate = self.create_rate(20)

        # subscriber for mavros/state
        self.state_sub = self.create_subscription(State, 'mavros/state', callback = self.state_callback)
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
        self.pose_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)

        # odom: in map frame
        self.odom_sub = self.create_subscription(PoseStamped, 'mavros/local_position/pose', callback = self.odom_callback)
        self.odom_pose = PoseStamped()

    # state callback
    def state_callback(self, msg):
        self.state = msg

    # odom callback
    def odom_callback(self, msg):
        self.odom_pose = msg
    

def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    goal_pos.position = node.odom_pose.pose.position
    goal_pos.position.z = 1.5

    # publish poses so we can switch to offboard later
    cmd = PoseStamped()
    cmd.pose.position = node.odom_pose.pose.position
    cmd.header.stamp = node.get_clock().now()
    cmd.header.frame_id = 'map'
    

    # wait to connect
    while rclpy.ok() and not node.state.connected:
        node.rate.sleep()

    for i in range(100):   
        if(not rclpy.ok()):
            break
        node.pose_pub.publish(cmd)
        node.rate.sleep()

    prev_request = node.get_clock().now()
    offb_set_mode = SetMode.Request()
    arm_cmd = CommandBool.Request()

    while rclpy.ok():
        node.rate.sleep()

        # check flags
        if COMMAND == 'launch':
            # Set mode to offboard, arm
            offb_set_mode.custom_mode = 'OFFBOARD'
            arm_cmd.value = True
            if(node.state.mode != "OFFBOARD" and (node.get_clock().now() - prev_request) > rclpy.duration.Duration(5.0)):
                if(node.set_mode_cli.call(offb_set_mode).mode_sent == True):
                    node.get_logger.info("OFFBOARD enabled")
            
                prev_request = node.get_clock().now()
            else:
                if(not node.state.armed and (node.get_clock().now() - prev_request) > rclpy.duration.Duration(5.0)):
                    if(node.arm_cli.call(arm_cmd).success == True):
                        node.get_logger.info("Vehicle armed")
                
                prev_request = node.get_clock().now()

            if np.abs(cmd.pose.position.z - node.odom_pose.pose.position.z) < 0.02:
                cmd.pose.position.z += 0.1

            if np.abs(goal_pos.position.z - node.odom_pose.pose.position.z) < 0.05:
                COMMAND = 'hover'
                continue

            cmd.header.stamp = node.get_clock().now()
            node.pose_pub.publish(cmd)
        elif COMMAND == 'hover':
            cmd.pose = goal_pos
            cmd.header.stamp = node.get_clock().now()
            node.pose_pub.publish(cmd)
        elif COMMAND == 'test':
            COMMAND = 'hover'
            continue
        elif COMMAND == 'land':
            if np.abs(cmd.pose.position.z - node.odom_pose.pose.position.z) < 0.02:
                cmd.pose.position.z -= 0.1

            if np.abs(0 - node.odom_pose.pose.position.z) < 0.2:
                offb_set_mode.custom_mode = 'AUTO.LAND'
                if(node.state.mode != "AUTO.LAND" and (node.get_clock().now() - prev_request) > rclpy.duration.Duration(5.0)):
                    if(node.set_mode_cli.call(offb_set_mode).mode_sent == True):
                        node.get_logger.info("Landing mode enabled")
                    prev_request = node.get_clock().now()
                continue
        elif COMMAND == 'abort':
            if np.abs(0 - node.odom_pose.pose.position.z) < 0.2:
                offb_set_mode.custom_mode = 'AUTO.LAND'
                if(node.state.mode != "AUTO.LAND" and (node.get_clock().now() - prev_request) > rclpy.duration.Duration(5.0)):
                    if(node.set_mode_cli.call(offb_set_mode).mode_sent == True):
                        node.get_logger.info("Landing mode enabled")
                    prev_request = node.get_clock().now()
                continue
            

    rclpy.shutdown()

if __name__ == "__main__":
    main()
