import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import rclpy.time
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
import rclpy.qos
from geometry_msgs.msg import PoseStamped, Twist, Point
import numpy as np
from std_srvs.srv import Trigger
import threading 
# import transforms3d as tf_transformations
from scipy.spatial.transform import Rotation as R

import yaml

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
LOCAL_NAV_TOLERANCE = 0.2  # [m]: distance to setpoint considered "reached"
GOAL_TOLERANCE = 0.05
TAKEOFF_INCREMENT = 0.2     # [m]: how much to increase takeoff goal
LANDING_INCREMENT = 0.3
MAXIMUM_INCREMENT = 0.3

TRACK_TIMEOUT = 20          # [s]: how much time since last zebra pose received to return to sweeping

OFFSET = 0.0
GOAL_HEIGHT = 2. # 1.5 + OFFSET

HOME = np.array([0., 1., GOAL_HEIGHT])

COMMAND = 'ground'
MODE = GROUND
# FOLLOW = True
SEARCH = True      # if True, sweep when no zebras. if False, hover when no zebras

def euler_from_quaternion(x, y, z, w):
    t0 = 2. * (w * x + y * z)
    t1 = 1. - 2 * (x*x + y*y)
    roll_x = np.arctan2(t0, t1)

    t2 = 2. * (w*y - z*x)
    t2 = 1. if t2 > 1. else t2
    t2 = -1. if t2 < -1. else t2
    pitch_y = np.arcsin(t2)

    t3 = 2. * (w*z + x*y)
    t4 = 1. - 2. * (y*y + z*z)
    yaw_z = np.arctan2(t3, t4)

    # print(type(roll_x), type(pitch_y), type(yaw_z))
    return roll_x, pitch_y, yaw_z

def dcm_from_rpy(r, p, y):
    cr = np.cos(r)
    sr = np.sin(r)
    cp = np.cos(p)
    sp = np.sin(p)
    cy = np.cos(y)
    sy = np.sin(y)

    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                    [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                    [  -sp,            cp*sr,            cp*cr]])

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
    
def handle_hold():
    global COMMAND
    COMMAND = 'hold'
    print('Hold Requested. Hover in place.')

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

def callback_hold(request, response):
    handle_hold()
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

        self.declare_parameter("camera_k", rclpy.Parameter.Type.STRING)
        camera_k_file = str(self.get_parameter("camera_k").value)
        self.K = np.load(camera_k_file)
        self.get_logger().info(f"Read camera calibration file at {camera_k_file}")

        self.declare_parameter("camera_rotation", rclpy.Parameter.Type.STRING)
        camera_rotation_file = str(self.get_parameter("camera_rotation").value)
        self.C_dc = np.load(camera_rotation_file)
        self.get_logger().info(f"Read camera rotation file at {camera_rotation_file}")
        
        self.declare_parameter("bounds", rclpy.Parameter.Type.STRING)
        bounds_file = str(self.get_parameter("bounds").value)
        with open(bounds_file, 'r') as f:
            self.bounds = yaml.safe_load(f)
        self.x_min = self.bounds['min_x']
        self.x_max = self.bounds['max_x']
        self.y_min = self.bounds['min_y']
        self.y_max = self.bounds['max_y']
        self.get_logger().info(f"Read bounds file at {bounds_file}")

        self.srv_launch = self.create_service(Trigger, 'rob498_drone_8/comm/launch', callback_launch)
        self.srv_test = self.create_service(Trigger, 'rob498_drone_8/comm/test', callback_test)
        self.srv_hold = self.create_service(Trigger, 'rob498_drone_8/comm/hold', callback_hold)
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

        # observed points
        # self.point_sub = self.create_subscription(Point, 'zebra_pose/camera', callback = self.zebra_point_callback, qos_profile=10)
        self.zebra_sub = self.create_subscription(PoseStamped, 'zebra_pose/image', callback = self.zebra_pose_callback, qos_profile=10)
        self.obs = np.zeros((2,1))  # zebra centroid in pixels
        # self.last_zebra_detection = self.get_clock().now().to_msg().sec
        self.last_zebra_detection = self.get_clock().now()
        self.new_zebra_pose_received = False    # if a new zebra pose has been received

        # controller
        # self.K = np.load('drone_ws/src/visual_servoing/visual_servoing/cameraK.npy')
        
        # self.gain = 1 # tune later
        # self.prev_ev = np.zeros((2,1))
        # self.prev_v = np.zeros((2,1))
        # self.delta_t = 1 # change to time since last camera message?
        self.depth = None
        
        self.steps = None   # np array of setpoints to go through
        self.step_counter = None    # index of self.steps that is the next setpoint

    # state callback
    def state_callback(self, msg):
        self.state = msg
        self.get_logger().debug(f"Received {msg}")

    # odom callback
    def odom_callback(self, msg):
        self.odom_pose = msg
        self.depth = msg.pose.position.z
        self.get_logger().debug(f"Received {msg}")

    # zebra point callback
    def zebra_point_callback(self, msg):
        self.obs = np.array([[msg.x],[msg.y]])

    def zebra_pose_callback(self, msg):
        self.obs = np.array([[msg.pose.position.x], [msg.pose.position.y]])
        # self.last_zebra_detection = msg.header.stamp.sec
        self.last_zebra_detection = self.get_clock().now()
        self.new_zebra_pose_received = True
        
    def zebra_camera_to_world(self):
        '''
        returns: np.array([[x], [y], [z]]) of zebra centroid in world frame; z should be 0
        '''
        
        # rotation from drone frame to world frame
        odom_quat = self.odom_pose.pose.orientation
        rotation = R.from_quat([odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w])
        C_wd = rotation.as_matrix()
        
        odom_pos = self.odom_pose.pose.position
        t = np.array([[odom_pos.x, odom_pos.y, odom_pos.z]]).T
        
        T_wd = np.vstack([np.hstack([C_wd, t]), [0, 0, 0, 1]])
        
        # rotation from camera frame to drone frame
        T_dc = np.vstack([np.hstack([self.C_dc, np.zeros((3, 1))]), [0, 0, 0, 1]])
        
        # multiply zebra camera coords by depth
        x_zebra = np.vstack([self.obs * self.depth, self.depth])
        self.get_logger().info(f"obs: {self.obs}")
        self.get_logger().info(f"x_zebra: {x_zebra} (expect 3x1 in pixels)")
        
        # multiply by inverse camera intrinsics to get zebra in camera frame
        p_camera = np.linalg.inv(self.K) @ x_zebra 
        self.get_logger().info(f"p_camera: {p_camera} (expect 3x1 in m)")
        
        # multiply by T matrices to get zebra in world frame
        self.get_logger().info(f"{T_wd.shape}, {T_dc.shape}, {p_camera.shape}")
        p_world = T_wd @ T_dc @ np.vstack([p_camera, 1])
        self.get_logger().info(f"p_world: {p_world} (expect 4x1 in m (homogeneous), z is 0)")
        
        return p_world[:3]
    
    def plan_local_path(self, p_world):
        '''
        p_world: position of zebra in world frame (3x1 np array)
        
        returns: N x 3 array of setpoints from drone's current position to p_world + z
            N determined by distance and MAXIMUM_INCREMENT
        '''
        # current position
        odom_pos = self.odom_pose.pose.position
        cur_pos = np.array([odom_pos.x, odom_pos.y, odom_pos.z])
        self.get_logger().info(f"cur_pos: {cur_pos} (expect 1x3 in m)")
        
        # goal position
        goal_pos = np.hstack([p_world[0], p_world[1], GOAL_HEIGHT])
        self.get_logger().info(f"goal_pos: {goal_pos} (expect 1x3 in m)")
        
        # build incremental setpoints
        distance = np.linalg.norm(goal_pos - cur_pos)
        self.get_logger().info(f"distance: {distance} (expect float in m)")
        
        if distance > MAXIMUM_INCREMENT:
            num_steps = int(round(distance / MAXIMUM_INCREMENT))
            steps = np.linspace(cur_pos, goal_pos, num_steps+1)[1:]
        else:
            steps = goal_pos.reshape((1, 3))
            
        self.get_logger().info(f"steps shape: {steps.shape} (expect N by 3, N = distance/MAXIMUM_INCREMENT)")
            
        return steps
    
    def plan_sweep_path(self):
        '''
        update self.steps
        '''
        
        x_steps = int(np.floor((self.x_max - self.x_min) / MAXIMUM_INCREMENT)) + 1
        y_steps = int(np.floor((self.y_max - self.y_min) / MAXIMUM_INCREMENT)) + 1
        
        x_vals = np.linspace(self.x_min, self.x_min + MAXIMUM_INCREMENT*(x_steps -1), x_steps)
        y_vals = np.linspace(self.y_min, self.y_min + MAXIMUM_INCREMENT*(y_steps - 1), y_steps)
        
        coords = []

        for idx, y in enumerate(y_vals):
            if idx % 2 == 0:
                x_row = x_vals
            else:
                x_row = x_vals[::-1]
                
            y_row = np.full_like(x_row, y)
            row_coords = np.column_stack((x_row, y_row))
            coords.append(row_coords)
            
        coords = np.vstack(coords)
        reverse_coords = coords[-2::-1]
        full_coords = np.vstack((coords, reverse_coords))
        
        sweep_path = np.hstack([full_coords, np.ones((full_coords.shape[0], 1)) * GOAL_HEIGHT])

        cur_pos = self.odom_pose.pose.position
        cur_pos = np.array([cur_pos.x, cur_pos.y, cur_pos.z])

        distances = np.linalg.norm(sweep_path - cur_pos, axis=1)
        self.step_counter = np.argmin(distances)

        self.steps = sweep_path
        
        self.get_logger().info(f"self.steps shape: {self.steps.shape} (expect Nx3)")
        self.get_logger().info(f"Sweep path calculated. Total {self.steps.shape[0]} steps. Starting at step {self.step_counter}.")


    # def get_virtual_image_coordinates(self, u, v, Z):
    #     '''
    #     args:
    #         u, v: regular image coordinates
    #         Z: depth
    #     return:
    #         u_v, v_v: virtual image coordinates
    #         Z_v: depth of virtual image plane
    #     '''
    #     X = u*Z
    #     Y = v*Z

    #     cam_coords = np.array([[X], [Y], [Z]])

    #     odom_quat = self.odom_pose.pose.orientation

    #     rotation = R.from_quat([odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w])
    #     C = rotation.as_matrix()

    #     virtual_coords = C@cam_coords

    #     X_v = virtual_coords[0]
    #     Y_v = virtual_coords[1]
    #     Z_v = virtual_coords[2]
    #     u_v = X_v/Z_v
    #     v_v = Y_v/Z_v

    #     return u_v.item(), v_v.item(), Z_v.item()

    # def jacobian(self, u_v, v_v, Z_v):
    #     J = np.array([[-1/Z_v, 0],
    #                 [0, -1/Z_v]], dtype=np.float64)
    #     return J

    
    # def get_control_velocity(self, obs, des):
    #     '''
    #     obs: 2xn array of the pt coords in camera frame
    #     des: 2xn array of desired pt coords in image coordinates
    #     depths: nx0
    #     v: 4x1
    #     '''
    #     # velocity control input
    #     v = np.zeros((2, 1))

    #     # TO-DO: vectorize this part?
    #     # Calculate error
    #     f = self.K[0,0] # assuming for now xy focal lengths are the same
    #     cx = self.K[0,2]
    #     cy = self.K[1,2]

    #     # Stack Jacobians
    #     u_i = (obs[0, 0] - cx) / f
    #     v_i = (obs[1, 0] - cy) / f
    #     # u_v, v_v, Z_v = self.get_virtual_image_coordinates(u_i, v_i, self.depth)
    #     u_v, v_v, Z_v = u_i, v_i, self.depth    # assume drone is always level, skip virtual image coords step
    #     J = self.jacobian(u_v, v_v, Z_v)
    #     u_di = (des[0, 0] - cx) / f
    #     v_di = (des[1, 0] - cy) / f
    #     # u_dv, v_dv, Z_dv = self.get_virtual_image_coordinates(u_di, v_di, self.depth)
    #     u_dv, v_dv, Z_dv = u_di, v_di, self.depth        
    #     ev = np.array([[u_v],[v_v]]) - np.array([[u_dv],[v_dv]])

    #     self.get_logger().info(f"zebra at: ({u_v}, {v_v})")
    #     self.get_logger().info(f"desired at: ({u_dv}, {v_dv}")

    #     # Estimate motion error
    #     # delta_ev = (ev - self.prev_ev) / (self.delta_t -  np.dot(J, self.prev_v))
    #     delta_ev = np.zeros((2, 1))

    #     # Get pseudoinverse
    #     J_inv = np.dot(np.linalg.inv(np.dot(np.transpose(J),J)),np.transpose(J))
        
    #     # Compute v
    #     # v = -self.gain * np.dot(J_inv, ev) - np.dot(J_inv, delta_ev)
    #     v = -self.gain * np.dot(J_inv, ev)
    #     # self.get_logger().info(f"{self.C_cd.shape}, {J_inv.shape}, {ev.shape}")
    #     # v = -self.gain * self.C_cd @ J_inv @ ev

    #     v = self.C_cd @ np.vstack([v, 0])
    #     v = v[:2]

    #     # Save values for next iteration
    #     self.prev_ev = ev
    #     self.prev_v = v

    #     return v

def main(args=None):
    global COMMAND, MODE, FOLLOW

    # print(tf_transformations)

    # node init
    rclpy.init(args=args)
    node = Controller()

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    node.get_logger().info("Node online.")

    # hover goal
    launch_goal_pos = PoseStamped()
    # wait for odom message
    node.get_logger().info("Waiting for initial pose.")
    while rclpy.ok() and not node.odom_pose:
        node.rate.sleep()
    launch_goal_pos.pose = node.odom_pose.pose
    launch_goal_pos.pose.position.z = GOAL_HEIGHT
    # TODO: set orientation
    node.get_logger().info("Initial pose received. Goal height set.")

    # publish poses for offboard
    cmd = PoseStamped()
    cmd.pose.position = node.odom_pose.pose.position
    cmd.header.frame_id = "map"
    cmd.header.stamp = node.get_clock().now().to_msg()
    
    # next setpoint as np array
    setpoint = None

    # wait to connect
    node.get_logger().info("Waiting to connect.")
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

    # for logging
    msg_sent = False

    while rclpy.ok():
        # state machine
        if COMMAND == 'abort' and MODE != GROUND:
            MODE = ABORT
            node.get_logger().info(f"Mode: {MODE}")
        elif COMMAND == 'launch' and MODE == GROUND:
            MODE = CONNECT
            node.get_logger().info(f"Mode: {MODE}")
        elif COMMAND == 'hold' and MODE != HOVER:
            MODE = HOVER
            node.get_logger().info(f"Mode: {MODE}")
        # elif COMMAND == 'test':
            # TODO: fix this
            # if FOLLOW:
            #     if MODE != TRACK:
            #         MODE = TRACK
            #         node.get_logger().info(f"Mode: {MODE}")
            # else:
            #     if MODE != SWEEP:
            #         MODE = SWEEP
            #         node.get_logger().info(f"Mode: {MODE}")
        elif COMMAND == 'test' and MODE != SWEEP and MODE != TRACK:
            MODE = SWEEP
            node.get_logger().info(f"Mode: {MODE}")            
        elif COMMAND == 'land' and MODE != GROUND:
            if MODE != LAND:
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
            cmd.pose.position = launch_goal_pos.pose.position
                
        elif MODE == TAKEOFF:
            # check distance from goal
            node.get_logger().debug(f"local goal distance: {np.abs(cmd.pose.position.z - node.odom_pose.pose.position.z)}")
            if np.abs(launch_goal_pos.pose.position.z - node.odom_pose.pose.position.z) < GOAL_TOLERANCE:
                # if close to goal, proceed to HOVER mode
                cmd.pose.position.z = launch_goal_pos.pose.position.z
                MODE = HOVER
                node.get_logger().info(f"Mode: {MODE}")
            # if far, check distance from local goal
            elif np.abs(cmd.pose.position.z - node.odom_pose.pose.position.z) < LOCAL_GOAL_TOLERANCE:
                # update local goal as needed in increments to ascend
                cmd.pose.position.z = min(cmd.pose.position.z + TAKEOFF_INCREMENT, launch_goal_pos.pose.position.z)
        elif MODE == HOVER:
            pass 
            # for debug: (TODO: remove)
            # velocity = node.get_control_velocity(node.obs, np.zeros((2, 1)))
        elif MODE == SWEEP:
            # TO-DO: make the drone move around until it detects something to follow?
            # if something is detected and we deem it worth following, set FOLLOW = True
            
            # if zebra detected, switch to tracking mode
            if node.new_zebra_pose_received:
                MODE = TRACK
                node.get_logger().info(f"Zebra detected. Switching to Track.")
                node.get_logger().info(f"MODE: {MODE}")
            else:
                # init sweep
                if node.steps is None:
                    # build sweep pattern
                    node.plan_sweep_path()
                    
                    setpoint = node.steps[node.step_counter]
                    
                # cmd_pose is next setpoint
                cur_p = node.odom_pose.pose.position
                cur_pos = np.array([cur_p.x, cur_p.y, cur_p.z]) 
                
                # proceed to next setpoint if close enough, otherwise continue to current setpoint
                if np.linalg.norm(setpoint - cur_pos) < LOCAL_NAV_TOLERANCE and SEARCH:
                    # last waypoint: stay at current position
                    if node.step_counter >= node.steps.shape[0] - 1:
                        # node.get_logger().info(f"Final setpoint reached. Waiting for next destination.")
                        # restart sweep
                        node.steps = None
                        node.get_logger().info(f"Final setpoint reached. Restarting sweep.")
                    else:
                        node.step_counter += 1
                        setpoint = node.steps[node.step_counter, :]
                        node.get_logger().info(f"Next setpoint scheduled: {setpoint}")
                        
                if not SEARCH:
                    setpoint = np.array([cmd.pose.position.x, cmd.pose.position.y, GOAL_HEIGHT])
                
                # assign cmd pose to setpoint
                cmd.pose.position.x = setpoint[0]
                cmd.pose.position.y = setpoint[1]
                cmd.pose.position.z = setpoint[2]
            
        elif MODE == TRACK:
            # # do visual servoing
            # # if we want to stop following (maybe all the animals left the frame), set FOLLOW = False
            # # TO-DO: setpoint = current pose + self.delta_t * self.get_velocity_input
            # des = np.zeros((2,1))
            # velocity = node.get_control_velocity(node.obs, des)
            # velocity = np.clip(velocity, a_min=-MAXIMUM_INCREMENT, a_max=MAXIMUM_INCREMENT)

            # # setpoint = np.array([[node.pose.position.x],[node.pose.position.y]]) + node.delta_t * velocity
            # setpoint = np.array([[node.odom_pose.pose.position.x], [node.odom_pose.pose.position.y]]) + node.delta_t * velocity
            # cmd.pose.position.x = float(setpoint[0])
            # cmd.pose.position.y = float(setpoint[1])
            # cmd.pose.position.z = GOAL_HEIGHT
            
            # if no new zebra pose in last TRACK_TIMEOUT s, return to sweeping
            if node.get_clock().now() - node.last_zebra_detection > Duration(seconds=TRACK_TIMEOUT):
                node.new_zebra_pose_received = False
                MODE = SWEEP 
                node.steps = None
                node.get_logger().info(f"No image received in last {TRACK_TIMEOUT} seconds.")
                node.get_logger().info(f"MODE: {MODE}")
            else:
            # if True:
                # tracking
                # if a new image has been received, recalculate setpoints
                if node.new_zebra_pose_received:
                    p_world = node.zebra_camera_to_world()
                    node.steps = node.plan_local_path(p_world)
                    node.step_counter = 0
                    
                    node.new_zebra_pose_received = False 
                    
                    setpoint = node.steps[node.step_counter, :]
                    node.get_logger().info(f"setpoint: {setpoint} (expect 1x3 array in m (map))")

                    msg_sent = False
                
                if setpoint is None:
                    node.get_logger().error(f"Setpoint not set. Continue.")
                    cmd.header.stamp = node.get_clock().now().to_msg()
                    node.pose_pub.publish(cmd)
                    node.rate.sleep()
                    continue
                
                # cmd_pose is next setpoint
                cur_p = node.odom_pose.pose.position
                cur_pos = np.array([cur_p.x, cur_p.y, cur_p.z]) 
                
                # proceed to next setpoint if close enough, otherwise continue to current setpoint
                if np.linalg.norm(setpoint - cur_pos) < LOCAL_NAV_TOLERANCE:
                    # last waypoint: stay at current position
                    if node.step_counter >= node.steps.shape[0] - 1:
                        if not msg_sent:
                            node.get_logger().info(f"Final setpoint reached. Waiting for next destination.")
                            msg_sent = True
                    else:
                        node.step_counter += 1
                        setpoint = node.steps[node.step_counter, :]
                        node.get_logger().info(f"Next setpoint scheduled: {setpoint}")
                        
                # assign cmd pose to setpoint
                cmd.pose.position.x = setpoint[0]
                cmd.pose.position.y = setpoint[1]
                cmd.pose.position.z = setpoint[2]
            
        elif MODE == LAND:
            # when landing begins, set local setpoint relative to current altitude
            if cmd.pose.position.z == launch_goal_pos.pose.position.z:
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
