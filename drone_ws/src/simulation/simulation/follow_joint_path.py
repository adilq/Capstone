import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import numpy as np
import threading
import yaml
from enum import Enum

NAMESPACE = "/box"

PATH_FILE = "/home/yifei/capstone/Capstone/drone_ws/src/simulation/simulation/square_path.yaml"
# PATH_FILE = "square_path.yaml"

# modes
class PublishMode(Enum):
    CONSTANT_SPEED = 1

class JointPublisher(Node):
    def __init__(self):
        super().__init__(f"joint_publisher")

        # load and process path
        self.get_logger().info(f"Loading trajectory from {PATH_FILE}")
        self.load_joint_path()
        # self.path_length = self.path.shape[0]

        # subscribe to joint states
        self.state_sub = self.create_subscription(JointState, f"{NAMESPACE}/joint_states", callback=self.joint_state_cb, qos_profile=10) 
        self.joint_state = JointState()

        # make publisher
        self.joint_pub = self.create_publisher(JointTrajectory, f"{NAMESPACE}/set_joint_trajectory", qos_profile=10)

        self.path_idx = 0

        self.rate = self.create_rate(self.pub_freq)

    def joint_state_cb(self, msg):
        # save data
        if len(msg.name) != len(self.joint_names):
            self.get_logger().warning(f"mismatch in number of joints (received: {len(msg.name)}; expected: {len(self.joint_names)}")

        self.joint_state = msg

        # increment joint path index
        self.path_idx += 1
        # if self.path_idx >= self.path_length:
        if self.path_idx >= self.num_waypoints:
            self.path_idx = 0

        # if rate == 0, publish next position immediately
        if self.pub_freq == 0:
            self.publish_next_pose()

    def publish_next_pose(self):
        # publish next pose in self.path
        pose = JointTrajectory()

        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id

        pose.joint_names = self.joint_names

        pose.points = []

        if self.mode == PublishMode.CONSTANT_SPEED:
            for i in range(self.num_joints):
                joint_state = JointTrajectoryPoint()
                
                joint_state.positions = self.waypoints[:, i].tolist()
                joint_state.velocities = [self.speed[i]] * self.num_waypoints

                pose.points.append(joint_state)

        self.joint_pub.publish(pose)

    def load_joint_path(self):
        with open(PATH_FILE, 'r') as f:
            path_data = yaml.safe_load(f)
    
        self.joint_names = path_data['joints']
        self.loop = path_data['loop']
        self.frame_id = path_data['frame_id']
        self.pub_freq = path_data['rate']

        self.num_joints = len(self.joint_names)

        self.mode = PublishMode[path_data['mode'].upper()]

        if self.mode == PublishMode.CONSTANT_SPEED:
            self.waypoints = np.vstack([path_data['path']])
            self.num_waypoints = self.waypoints.shape[0]
            self.speed = np.array(path_data['speed'])

#         num_joints = len(self.joint_names)
#     
#         steps = path_data['step']
#         waypoints = np.vstack(path_data['path'])
#         if self.loop:
#             waypoints = np.vstack([waypoints, waypoints[0]])
#     
#         path = waypoints[0]
#     
#         for i, pose in enumerate(waypoints[1:]):
#             num_increments = int(round(np.max(np.abs(pose - waypoints[i]) / steps)))
#     
#             increments = np.linspace(waypoints[i], pose, num_increments+1)
#     
#             path = np.vstack([path, increments[1:]])
#     
#         self.num_joints = num_joints
#         self.waypoints = waypoints
#         self.path = path

def main(args=None):
    rclpy.init(args=args)

    # init node
    node = JointPublisher()
    
    # start thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    node.get_logger().info("Node online.")

    # loop
    while rclpy.ok():
        # publish next joint state
        node.publish_next_pose()

        # reset index when necessary if node.loop

        node.rate.sleep()

    # shutdown
    rclpy.shutdown()

    # join thread
    thread.join()

if __name__ == '__main__':
    main()
