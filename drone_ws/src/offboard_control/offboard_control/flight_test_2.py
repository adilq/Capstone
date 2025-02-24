import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import rclpy.time
from std_srvs.srv import Trigger
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
import threading

GROUND = 'GROUND'
CONNECT = 'CONNECT'
TAKEOFF = 'TAKEOFF'
HOVER = 'HOVER'
LAND = 'LAND'

COMMAND = 'ground'
MODE = GROUND

# Callback handlers
def handle_launch():
    # publish to some topic that tells the offb_node to do some predetermined launch sequence
    global COMMAND
    COMMAND = 'launch'
    print('Launch Requested. Your drone should take off.')

def handle_test():
    # publish to some topic that tells the offb_node to hover in place
    global COMMAND
    COMMAND = 'test'
    print('Test Requested. Your drone should perform the required tasks. Recording starts now.')

def handle_land():
    # publish to some topic that tells the offb_node to do some predetermined land sequence
    global COMMAND 
    COMMAND = 'land'
    print('Land Requested. Your drone should land.')

def handle_abort():
    # publish to some topic that tells the offb_node to kill the motors? land? switch to manual control?
    global COMMAND 
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
        super().__init__('rob498_drone_08')
        self.srv_launch = self.create_service(Trigger, 'rob498_drone_08/comm/launch', callback_launch)
        self.srv_test = self.create_service(Trigger, 'rob498_drone_08/comm/test', callback_test)
        self.srv_land = self.create_service(Trigger, 'rob498_drone_08/comm/land', callback_land)
        self.srv_abort = self.create_service(Trigger, 'rob498_drone_08/comm/abort', callback_abort)

        self.rate = self.create_rate(20)

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
        self.pose_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)

        # odom: in map frame
        self.odom_sub = self.create_subscription(PoseStamped, 'mavros/local_position/pose', callback = self.odom_callback, qos_profile=10)
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
    global COMMAND, MODE 

    # node init
    rclpy.init(args=args)
    node = CommNode()

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    # hover goal
    goal_pos = PoseStamped()
    # wait for odom message
    while rclpy.ok() and not node.odom_pose:
        node.rate.sleep()
    goal_pos.position = node.odom_pose.pose.position
    goal_pos.position.z = 1.5
    node.get_logger.info("Initial pose received. Goal position set.")

    # publish poses for offboard
    cmd = PoseStamped()
    cmd.pose.position = node.odom_pose.pose.position
    cmd.header.frame_id = "map"
    cmd.header.stamp = node.get_clock().now().to_msg()

    # wait to connect
    while rclpy.ok() and not node.state.connected:
        node.rate.sleep()
    node.get_logger.info("Node connected.")

    # for arm and offboard
    offb_set_mode = SetMode.Request()
    offb_set_mode.custom_mode = "OFFBOARD"
    arm_cmd = CommandBool.request()
    arm_cmd.value = True 

    # logic variables
    prev_request = node.get_clock().now()
    counter = 0
    counter_total = 100
    
    node.get_logger().info("Starting loop.")

    while rclpy.ok():
        # state machine
        if COMMAND == 'abort':
            MODE = LAND
        elif COMMAND == 'launch' and MODE == GROUND:
            MODE = CONNECT
        elif COMMAND == 'test':
            MODE = HOVER
        elif COMMAND == 'land':
            MODE = LAND

        # behaviour
        node.get_logger().debug(f"Mode: {MODE}")
        if MODE == CONNECT:
            # check if armed and in offboard mode


            # publish to setpoint_local until counter == counter_total

            # arm and set mode

            # once set, initialize positions and proceed to TAKEOFF mode
            pass
        elif MODE == TAKEOFF:
            # check distance from goal
            # if far, check distance from local goal
            # update local goal as needed in increments to ascend
            # if close to goal, proceed to HOVER mode
            pass
        elif MODE == HOVER:
            # nothing?
            pass 
        elif MODE == LAND:
            # initiate auto land
            pass 
        elif MODE == GROUND:
            # nothing?
            pass

        # publish setpoint
        node.pose_pub.publish(cmd)
        node.rate.sleep()

    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()
