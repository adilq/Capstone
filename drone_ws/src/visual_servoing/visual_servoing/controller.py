import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import rclpy.time
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
import rclpy.qos
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
from std_srvs.srv import Trigger
import threading 
import tf_transformations

# To-do:
# Delta pose estimate function
# Test on random square input (four points)
# plot in rviz: error, state (pose), 
# Incorporate saturation
# Add depth estimate
GROUND = 'GROUND'
CONNECT = 'CONNECT'
TAKEOFF = 'TAKEOFF'
HOVER = 'HOVER'
SWEEP = 'SWEEP'
TRACK = 'TRACK'
LAND = 'LAND'
ABORT = 'ABORT'

LOCAL_GOAL_TOLERANCE = 0.15 # [m]: height tolerance of "reached local goal"
GOAL_TOLERANCE = 0.05
TAKEOFF_INCREMENT = 0.2     # [m]: how much to increase takeoff goal
LANDING_INCREMENT = 0.3

OFFSET = 0.0
GOAL_HEIGHT = 1.5 + OFFSET

COMMAND = 'ground'
MODE = GROUND
FOLLOW = True

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

class Controller(Node):
    def __init__(self):
        super().__init__('rob498_drone_8')
        self.srv_launch = self.create_service(Trigger, 'rob498_drone_8/comm/launch', callback_launch)
        self.srv_test = self.create_service(Trigger, 'rob498_drone_8/comm/test', callback_test)
        self.srv_land = self.create_service(Trigger, 'rob498_drone_8/comm/land', callback_land)
        self.srv_abort = self.create_service(Trigger, 'rob498_drone_8/comm/abort', callback_abort)
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
        self.odom_pose = None

        # controller
        self.K = np.eye(3)
        self.gain = 1 # tune later
        self.prev_ev = np.zeros((8,1))
        self.prev_v = np.zeros((4,1))
        self.delta_t = 1 # change to time since last camera message?
        self.depth = None


    # state callback
    def state_callback(self, msg):
        self.state = msg
        self.get_logger().debug(f"Received {msg}")

    # odom callback
    def odom_callback(self, msg):
        self.odom_pose = msg
        self.depth = msg.pose.position.z
        self.get_logger().debug(f"Received {msg}")

    def get_virtual_image_coordinates(self, u, v, Z):
        '''
        args:
            u, v: regular image coordinates
            Z: depth
        return:
            u_v, v_v: virtual image coordinates
            Z_v: depth of virtual image plane
        '''
        X = u*Z
        Y = v*Z

        cam_coords = np.array([[X], [Y], [Z]])

        odom_quat = self.odom_pose.pose.orientation

        # r, p, y = tf_transformations.euler_from_quaternion(self.odom_pose.pose.orientation)
        r, p, y = tf_transformations.euler_from_quaternion([odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w])
        cy = np.cos(y)
        sy = np.sin(y)
        cp = np.cos(p)
        sp = np.sin(p)
        cr = np.cos(r)
        sr = np.sin(r)
        C = np.array([[cy, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])
        
        virtual_coords = C@cam_coords

        X_v = virtual_coords[0]
        Y_v = virtual_coords[1]
        Z_v = virtual_coords[2]
        u_v = X_v/Z_v
        v_v = Y_v/Z_v

        return u_v.item(), v_v.item(), Z_v.item()

    def jacobian(self, u_v, v_v, Z_v):
        '''obs: 2x1 np array'''
        J = np.array([[-1/Z_v, 0, u_v/Z_v, v_v],
                    [0, -1/Z_v, v_v/Z_v, -u_v]], dtype=np.float64)
        return J
    
    def get_control_velocity(self, obs, des, depths):
        '''
        obs: 2xn array of the pt coords in camera frame
        des: 2xn array of desired pt coords in image coordinates
        depths: nx0
        v: 4x1
        '''
        # velocity control input
        v = np.zeros((4, 1))

        # number of points
        n = depths.size

        # TO-DO: vectorize this part?
        # Calculate error
        f = self.K[0,0] # assuming for now xy focal lengths are the same
        cx = self.K[0,2]
        cy = self.K[1,2]

        # Stack Jacobians
        # self.get_logger().info(f"f: {f}, {type(f)}")
        u = (obs[0, 0] - cx) / f
        v = (obs[1, 0] - cy) / f
        u_v, v_v, Z_v = self.get_virtual_image_coordinates(u, v, depths[0])
        # self.get_logger().info(f"array shapes: {u_v.shape}, {v_v.shape}, {Z_v.shape}")
        J = self.jacobian(u_v, v_v, Z_v)
        ev = np.array([[u_v],[v_v]]) - des[:, 0]
        for i in range(1, n):
            u = (obs[0, i] - cx) / f
            v = (obs[1, i] - cy) / f
            u_v, v_v, Z_v = self.get_virtual_image_coordinates(u, v, depths[i])
            J = np.vstack((J, self.jacobian(u_v, v_v, Z_v)))
            ev = np.vstack((ev, (np.array([[u_v],[v_v]]) - des[:, i])))

        # Estimate motion error
        delta_ev = (ev - self.prev_ev) / (self.delta_t -  np.dot(J, self.prev_v))

        # Get pseudoinverse
        J_inv = np.dot(np.linalg.inv(np.dot(np.transpose(J),J)),np.transpose(J))
        
        # Compute v
        v = -self.gain * np.dot(J_inv, ev) - np.dot(J_inv, delta_ev)

        # Save values for next iteration
        self.prev_ev = ev
        self.prev_v = v

        return v

def main(args=None):
    global COMMAND, MODE, FOLLOW

    # node init
    rclpy.init(args=args)
    node = Controller()

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
    node.get_logger().info("Initial pose received. Goal height set.")

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
    
    node.get_logger().info("Starting loop.")
    node.get_logger().info(f"Mode: {MODE}")

    while rclpy.ok():
        # state machine
        if COMMAND == 'abort' and MODE != GROUND:
            MODE = ABORT
            node.get_logger().info(f"Mode: {MODE}")
        elif COMMAND == 'launch' and MODE == GROUND:
            MODE = CONNECT
            node.get_logger().info(f"Mode: {MODE}")
        elif COMMAND == 'test':
            if FOLLOW:
                MODE = TRACK
            else:
                MODE = SWEEP
            node.get_logger().info(f"Mode: {MODE}")
        elif COMMAND == 'land' and MODE != GROUND:
            MODE = LAND
            node.get_logger().info(f"Mode: {MODE}")

        # behaviour
        node.get_logger().debug(f"Mode: {MODE}")
        if MODE == CONNECT:
            # check if armed and in offboard mode
            if node.state.armed and node.state.mode == "OFFBOARD":
                # once set, initialize positions and proceed to TAKEOFF mode
                MODE = TAKEOFF
                node.get_logger().info(f"Mode: {MODE}")
            else:
                if counter >= counter_total and node.get_clock().now() - prev_request > Duration(seconds=2.0):
                    # arm and set mode (try every 2 seconds)
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
            cmd.pose.position = goal_pos.pose.position
                
        elif MODE == TAKEOFF:
            # check distance from goal
            node.get_logger().debug(f"local goal distance: {np.abs(cmd.pose.position.z - node.odom_pose.pose.position.z)}")
            if np.abs(goal_pos.pose.position.z - node.odom_pose.pose.position.z) < GOAL_TOLERANCE:
                # if close to goal, proceed to HOVER mode
                cmd.pose.position.z = goal_pos.pose.position.z
                MODE = HOVER
                node.get_logger().info(f"Mode: {MODE}")
            # if far, check distance from local goal
            elif np.abs(cmd.pose.position.z - node.odom_pose.pose.position.z) < LOCAL_GOAL_TOLERANCE:
                # update local goal as needed in increments to ascend
                cmd.pose.position.z = min(cmd.pose.position.z + TAKEOFF_INCREMENT, goal_pos.pose.position.z)
                #cmd.pose.position = goal_pos.pose.position
        elif MODE == HOVER:
            pass 
        elif MODE == SWEEP:
            # TO-DO: make the drone move around until it detects something to follow?
            # if something is detected and we deem it worth following, set FOLLOW = True
            pass
        elif MODE == TRACK:
            # do visual servoing
            # if we want to stop following (maybe all the animals left the frame), set FOLLOW = False
            # TO-DO: setpoint = current pose + self.delta_t * self.get_velocity_input
            pts_des = np.array([[1, -1], [-1, 1], [-1, -1], [1, 1]]).T
            pts_obs = np.array([[1, -1], [-1, 1], [-1, -1], [1, 1]]).T
            depths = np.array([node.depth, node.depth, node.depth, node.depth])
            velocity = node.get_control_velocity(pts_des, pts_obs, depths)
            odom_quat = node.odom_pose.pose.orientation
            # roll, pitch, yaw = tf_transformations.euler_from_quaternion(node.odom_pose.pose.orientation)
            roll, pitch, yaw = tf_transformations.euler_from_quaternion([odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w])
            cur_pose = np.array([node.odom_pose.pose.position.x, 
                                 node.odom_pose.pose.position.y, 
                                 node.odom_pose.pose.position.z,
                                 yaw])
            
            print(f"cur_pose: {cur_pose.shape}")
            print(f"delta_t: {node.delta_t} ({type(node.delta_t)})")
            print(f"velocity: {velocity} ({type(velocity)})")
            setpoint = cur_pose + node.delta_t * velocity
            cmd.pose.position.x = setpoint[0]
            cmd.pose.position.y = setpoint[1]
            cmd.pose.position.z = setpoint[2]
            # cmd.pose.orientation = setpoint[2] how to give a yaw set point?
            
        elif MODE == LAND:
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
                    node.get_logger().info(f"Mode: {MODE}")
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
                    node.get_logger().info(f"Mode: {MODE}")
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

if __name__ == '__main__':
    main()
