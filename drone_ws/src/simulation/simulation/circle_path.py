import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import numpy as np
import threading

SCALE = 1
n = 5000
delta_time = 0.03 # 0.01

total_time = 2 * n * delta_time + 5 

class JointPublisher(Node):
    def __init__(self):
        super().__init__(f"simple_joint_publisher")

        # publisher
        self.joint_pub = self.create_publisher(JointTrajectory, "/box/set_joint_trajectory", qos_profile=10)

        # rate
        self.rate = self.create_rate(1/total_time)


def main(args=None):
    rclpy.init(args=args)

    # init node
    node = JointPublisher()

    # start thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    node.get_logger().info("Node online.")

    # path
    start = 0.
    end = 2 * np.pi

    dt = (end - start)/n

    def f(t):
        r = 1
        return SCALE * (r * np.sin(t))

    def g(t):
        r = 1
        return SCALE * (r * np.cos(t))

    def h(t):
        return np.arctan2(g(t+1)-g(t), f(t+1)-f(t)) + np.pi/4

    jt = JointTrajectory()

    jt.header.frame_id = "world"
    jt.joint_names = ["x_joint", "y_joint", "theta_joint"]

    for i in range(n):
        t = i * dt

        jtp = JointTrajectoryPoint()

        jtp.positions = [f(t), g(t), h(t)]
        jtp.time_from_start = Duration(seconds=delta_time).to_msg()

        jt.points.append(jtp)

    while rclpy.ok():
        node.joint_pub.publish(jt)

        node.get_logger().info("publishing trajectory")

        node.rate.sleep()

    # shutdown
    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()
