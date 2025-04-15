import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point, Quaternion

import numpy as np
import threading

NAMESPACE= "/box"

class PoseBroadcaster(Node):
    def __init__(self):
        super().__init__(f"sim_pose_broadcaster")

        self.state_sub = self.create_subscription(JointState, f"{NAMESPACE}/joint_states", callback=self.joint_state_cb, qos_profile=10)

        self.joint_state = JointState()
        self.x_joint = 0.
        self.y_joint = 0.
        self.theta_joint = 0.

        self.pose_pub = self.create_publisher(PoseStamped, f"{NAMESPACE}/position/global", qos_profile=10)
        # self.point_pub = self.create_publisher(Point, f"zebra_point", qos_profile=10)

        self.rate = self.create_rate(20)

    def joint_state_cb(self, msg):
        self.joint_state = msg
        self.x_joint = msg.position[msg.name.index('x_joint')]
        self.y_joint = msg.position[msg.name.index('y_joint')]
        self.theta_joint = msg.position[msg.name.index('theta_joint')]

def rpy_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
    return {'x': qx, 'y': qy, 'z': qz, 'w': qw}

def main(args=None):
    rclpy.init(args=args)

    node = PoseBroadcaster()

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    node.get_logger().info("Node online.")

    pose = PoseStamped()


    while rclpy.ok():
        pose.pose.position = Point(x=node.x_joint, y=node.y_joint, z=0.)

        orientation = rpy_to_quaternion(0., 0., node.theta_joint)
        pose.pose.orientation = Quaternion(**orientation)


        pose.header.stamp = node.get_clock().now().to_msg()
        pose.header.frame_id = "map"

        # node.get_logger().info(f"publishing message of type {type(pose)}")
        node.pose_pub.publish(pose)
        # node.point_pub.publish(pose.pose.position)

        node.rate.sleep()

        


    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()
