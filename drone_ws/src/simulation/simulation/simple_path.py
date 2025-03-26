import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import numpy as np
import threading

class JointPublisher(Node):
    def __init__(self):
        super().__init__(f"simple_joint_publisher")

        # publisher
        self.joint_pub = self.create_publisher(JointTrajectory, "/box/set_joint_trajectory", qos_profile=10)

        # rate
        self.rate = self.create_rate(1)


def main(args=None):
    rclpy.init(args=args)

    # init node
    node = JointPublisher()

    # start thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    node.get_logger().info("Node online.")

    # pose
    positions = np.zeros((1, 3))
    increment = np.ones((1, 3)) * 0.01

    start_time = 0.0
    time_delta = 0.1

    pos_counter = 0
    time_counter = 0

    # trajectory
    pose = JointTrajectory()

    pose.header.stamp = node.get_clock().now().to_msg()
    pose.header.frame_id = "world"

    pose.joint_names = ["x_joint", "y_joint", "theta_joint"]
    pose.points = []
       
    steps = 3

    for i in range(steps):
        point = JointTrajectoryPoint()

        target_pose = positions + increment*i
        target_pose = target_pose.reshape((3,)).tolist()
        point.positions = target_pose
        point.time_from_start = Duration(seconds=start_time+i*time_delta).to_msg()

        pose.points.append(point)

    last_time = node.get_clock().now()
    # loop
    while rclpy.ok():
        now = node.get_clock().now()
        if now - last_time > Duration(seconds=time_delta*steps):
            pose.header.stamp = now.to_msg()
            node.joint_pub.publish(pose)

            node.get_logger().info("publishing trajectory")

            last_time = now

        node.rate.sleep()

    # shutdown
    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()
