import numpy as np
import rclpy
import rclpy.time
import rclpy.qos
from rclpy.node import Node
from rclpy.duration import Duration
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseArray, PoseStamped
from std_srvs.srv import Trigger
import threading

GROUND = 'GROUND'
CONNECT = 'CONNECT'
TAKEOFF = 'TAKEOFF'
HOVER = 'HOVER'
NAVIGATE = 'NAVIGATE'
LAND = 'LAND'
ABORT = 'ABORT'

COMMAND = 'ground'
MODE = GROUND
COMPLETE = False

LOCAL_GOAL_TOLERANCE = 0.15 # [m]: height tolerance of "reached local goal"
GOAL_TOLERANCE = 0.05
TAKEOFF_INCREMENT = 0.2     # [m]: how much to increase takeoff goal
LANDING_INCREMENT = 0.3
NAV_THRESHOLD = 0.5

GOAL_HEIGHT = 0.5

WAYPOINTS = None
WAYPOINTS_RECEIVED = False

# Callback handlers
def handle_launch():
    global COMMAND
    COMMAND = 'launch'
    print('Launch Requested. Your drone should take off.')

def handle_test():
    global COMMAND
    COMMAND = 'test'
    print('Test Requested. Your drone should perform the required tasks. Recording starts now.')

def handle_land():
    global COMMAND 
    COMMAND = 'land'
    print('Land Requested. Your drone should land.')

def handle_abort():
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

def callback_waypoints(msg):
    global WAYPOINTS_RECEIVED, WAYPOINTS
    if WAYPOINTS_RECEIVED:
        return
    print('Waypoints Received')
    WAYPOINTS_RECEIVED = True
    # WAYPOINTS = np.empty((0,3))
    
    goals = WAYPOINTS
    
    for pose in msg.poses:
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        goals = np.vstack((goals, pos))
    
    for i, pos in enumerate(goals[1:]):
        distance = pos - goals[i-1]
        length = np.linalg.norm(distance)
        
        if length > NAV_THRESHOLD:
            num_steps = round(length / NAV_THRESHOLD)
            steps = np.linspace(goals[i], pos, num_steps+1)
            
            WAYPOINTS = np.vstack([WAYPOINTS, steps[1:]])

class CommNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_8')
        self.srv_launch = self.create_service(Trigger, 'rob498_drone_8/comm/launch', callback_launch)
        self.srv_test = self.create_service(Trigger, 'rob498_drone_8/comm/test', callback_test)
        self.srv_land = self.create_service(Trigger, 'rob498_drone_8/comm/land', callback_land)
        self.srv_abort = self.create_service(Trigger, 'rob498_drone_8/comm/abort', callback_abort)
        self.sub_waypoints = self.create_subscription(PoseArray, 'rob498_drone_8/comm/waypoints', callback_waypoints, 10)

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
    global COMMAND, MODE, WAYPOINTS, WAYPOINTS_RECEIVED, COMPLETE

    rclpy.init(args=args)
    node = CommNode()
    print('This is a dummy drone node to test communication with the ground control')
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    node.get_logger().info("Node online.")

    # hover goal
    goal_pos = PoseStamped()
    # wait for odom message
    while rclpy.ok() and not node.odom_pose:
        node.rate.sleep()
    goal_pos.pose = node.odom_pose.pose
    goal_pos.pose.position.z = GOAL_HEIGHT
    
    # TODO: set orientation
    node.get_logger().info("Initial pose received. Hover position set.")

    # publish poses for offboard
    cmd = PoseStamped()
    cmd.pose.position = node.odom_pose.pose.position
    cmd.header.frame_id = "map"
    cmd.header.stamp = node.get_clock().now().to_msg()

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
    waypoint_counter = 0
    WAYPOINTS = np.array([goal_pos.pose.position.x, goal_pos.pose.position.y, goal_pos.pose.position.z])
    
    node.get_logger().info("Starting loop.")

    while rclpy.ok():
        if WAYPOINTS_RECEIVED:
            print('Waypoints:\n', WAYPOINTS)

        # state machine
        if COMMAND == 'abort' and MODE != GROUND:
            MODE = ABORT
        elif COMMAND == 'launch' and MODE == GROUND:
            MODE = CONNECT
        elif COMMAND == 'test':
            if WAYPOINTS_RECEIVED and not COMPLETE:
                MODE = NAVIGATE
            else:
                MODE = HOVER
        elif COMMAND == 'land' and MODE != GROUND:
            MODE = LAND

        # behaviour
        node.get_logger().info(f"Mode: {MODE}")
        if MODE == CONNECT:
            # check if armed and in offboard mode
            if node.state.armed and node.state.mode == "OFFBOARD":
                # once set, initialize positions and proceed to TAKEOFF mode
                MODE = TAKEOFF
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
            cmd.pose.position = node.odom_pose.pose.position
                
        elif MODE == TAKEOFF:
            # check distance from goal
            node.get_logger().info(f"local goal distance: {np.abs(cmd.pose.position.z - node.odom_pose.pose.position.z)}")
            if np.abs(goal_pos.pose.position.z - node.odom_pose.pose.position.z) < GOAL_TOLERANCE:
                # if close to goal, proceed to HOVER mode
                cmd.pose.position.z = goal_pos.pose.position.z
                MODE = HOVER
            # if far, check distance from local goal
            elif np.abs(cmd.pose.position.z - node.odom_pose.pose.position.z) < LOCAL_GOAL_TOLERANCE:
                # update local goal as needed in increments to ascend
                cmd.pose.position.z = min(cmd.pose.position.z + TAKEOFF_INCREMENT, goal_pos.pose.position.z)
            
        elif MODE == HOVER:
            # nothing?
            pass
         
        elif MODE == NAVIGATE:
            # check if we are at the goal point
            goal_pos_pos = goal_pos.pose.position
            goal_point = np.array([goal_pos_pos.x, goal_pos_pos.y, goal_pos_pos.z])
            curr_pos_pos = node.odom_pose.pose.position
            curr_point = np.array([curr_pos_pos.x, curr_pos_pos.y, curr_pos_pos.z])
            
            if np.linalg.norm(goal_point - curr_point) < 0.4:
                node.get_logger().info(f"Reached waypoint {waypoint_counter}")
                # we are there, set the goal to the next waypoint
                # check if we are at the last waypoint
                print(WAYPOINTS.shape)
                if waypoint_counter == (WAYPOINTS.shape[0]-1):
                    # that was the last waypoint, go back to hover mode
                    COMPLETE = True
                    MODE = HOVER
                    node.get_logger().info(f"Final waypoint reached, switched back to hover mode")
                else:
                    # set to the next waypoint
                    waypoint_counter += 1
                    new_goal_point = WAYPOINTS[waypoint_counter, :]
                    goal_pos.pose.position.x = new_goal_point[0]
                    goal_pos.pose.position.y = new_goal_point[1]
                    goal_pos.pose.position.z = new_goal_point[2]
                    node.get_logger().info(f"Next waypoint scheduled: {new_goal_point}")
                
            # make sure we are in the correct state, set command
            if WAYPOINTS_RECEIVED and node.state.armed and node.state.mode == 'OFFBOARD':
                cmd.pose.position = goal_pos.pose.position

            else:
                # something is wrong, we should abort?
                MODE = ABORT
            
        elif MODE == LAND:
            # initiate auto land
            # offb_set_mode.custom_mode = "AUTO.LAND"
            # if node.state.mode != "AUTO.LAND" and node.get_clock().now() - prev_request > Duration(seconds=0.5):
            #     if node.set_mode_cli.call(offb_set_mode).mode_sent == True:
            #         node.get_logger().info("Landing mode enabled")
            #     prev_request = node.get_clock().now()

            # when landing begins, set local setpoint relative to current altitude
            if cmd.pose.position.z == goal_pos.pose.position.z:
                cmd.pose.position.z = node.odom_pose.pose.position.z - LANDING_INCREMENT

            # check distance from local goal
            if np.abs(cmd.pose.position.z - node.odom_pose.pose.position.z) < LOCAL_GOAL_TOLERANCE:
                # update local goal
                cmd.pose.position.z -= LANDING_INCREMENT
            
            # set to GROUND mode when landed
            if not node.state.armed and node.get_clock().now() - prev_request > Duration(seconds=5.0):
                # set mode to AUTO.LOITER
                offb_set_mode.custom_mode = "AUTO.LOITER"
                if node.set_mode_cli.call(offb_set_mode).mode_sent == True:
                    node.get_logger().info("Landed")
                    MODE = GROUND 
                prev_request = node.get_clock().now()
                
        elif MODE == ABORT:
            # initiate auto land
            offb_set_mode.custom_mode = "AUTO.LAND"
            if node.state.mode != "AUTO.LAND" and node.get_clock().now() - prev_request > Duration(seconds=0.5):
                if node.set_mode_cli.call(offb_set_mode).mode_sent == True:
                    node.get_logger().info("Landing mode enabled")
                prev_request = node.get_clock().now()
            
            # set to GROUND mode when landed
            if node.state.mode == "AUTO.LAND" and not node.state.armed and node.get_clock().now() - prev_request > Duration(seconds=5.0):
                # set mode to AUTO.LOITER
                offb_set_mode.custom_mode = "STABILIZED"
                if node.set_mode_cli.call(offb_set_mode).mode_sent == True:
                    node.get_logger().info("Landed")
                    MODE = GROUND 
                prev_request = node.get_clock().now()

        elif MODE == GROUND:
            # nothing?
            cmd.pose.position = node.odom_pose.pose.position

        # publish setpoint
        cmd.header.stamp = node.get_clock().now().to_msg()
        node.pose_pub.publish(cmd)
        node.rate.sleep()

    rclpy.shutdown()
    thread.join()

if __name__ == "__main__":
    main()