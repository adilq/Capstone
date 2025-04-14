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
        # self.rate = self.create_rate(1/11)



def main(args=None):
    rclpy.init(args=args)

    # init node
    node = JointPublisher()

    # start thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    node.get_logger().info("Node online.")

    # path
    dt = 0.01

    start = -5.
    end = 20.
    step_size = 0.005 # 0.01

    n = int((end - start)/step_size)

    jt = JointTrajectory()

    jt.header.frame_id = "world"
    jt.joint_names = ["x_joint"]

    for i in range(n):
        jtp = JointTrajectoryPoint()

        jtp.positions = [start + step_size*i]
        jtp.time_from_start = Duration(seconds=dt).to_msg()

        jt.points.append(jtp)

    node.rate = node.create_rate(1/(n*dt))

    while rclpy.ok():
        # jt = JointTrajectory()

        # jt.header.stamp = node.get_clock().now().to_msg()
        # jt.header.frame_id = "world"

        # jt.joint_names = ["x_joint"]

        # n = 3
        # for i in range(n):
        #     jtp = JointTrajectoryPoint()
        #     jtp.positions = [0. + 1.*i]
        #     jtp.velocities = [0.2]
        #     jtp.accelerations = [0.01]
        #     jtp.time_from_start = Duration(seconds=1.).to_msg()

        #     jt.points.append(jtp)

        node.joint_pub.publish(jt)

        node.get_logger().info("publishing trajectory")

        node.rate.sleep()

    # shutdown
    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()
