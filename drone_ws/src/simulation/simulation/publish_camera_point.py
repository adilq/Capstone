import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

import numpy as np
import threading

def rpy_from_quaternion(x, y, z, w):
    t0 = +2. * (w * x + y * z)
    t1 = +1. - 2. * (x*x + y*y)
    roll_x = np.arctan2(t0, t1)

    t2 = +2. * (w*y - z*x)
    t2 = +1. if t2 > +1. else t2
    t2 = -1. if t2 < -1. else t2
    pitch_y = np.arcsin(t2)

    t3 = +2. * (w*z + x*y)
    t4 = +1. - 2. * (y*y + z*z)
    yaw_z = np.arctan2(t3, t4)

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

class PoseConverter(Node):
    def __init__(self):
        super().__init__(f"camera_pose_broadcaster")

        self.declare_parameter("camera_k", rclpy.Parameter.Type.STRING)
        camera_k_file = str(self.get_parameter("camera_k").value)
        self.get_logger().info(f"Read camera calibration file at {camera_k_file}")
        self.K = np.load(camera_k_file)

        #  self.C_cd = np.array([[-1, 0, 0],
        #                        [0, 1, 0],
        #                        [0, 0, -1]], dtype=float)
        self.C_cd = np.array([[0, -1, 0],
                             [-1, 0, 0],
                             [0, 0, -1]], dtype=float)

        self.box_pose_sub = self.create_subscription(PoseStamped, "box/position/global", callback=self.box_pose_cb, qos_profile=10)

        self.box_pose = Pose()

        self.drone_pose_sub = self.create_subscription(PoseStamped, "mavros/local_position/pose", callback = self.drone_pose_cb, qos_profile=rclpy.qos.qos_profile_system_default)
        self.drone_pose = Pose()

        self.cam_pose_pub = self.create_publisher(Point, "zebra_pose/camera", qos_profile=10)

        self.cam_pose = Point()

        self.rate = self.create_rate(20)

    def box_pose_cb(self, msg):
        self.box_pose = msg.pose

    def drone_pose_cb(self, msg):
        self.drone_pose = msg.pose


def main(args=None):
    rclpy.init(args=args)

    node = PoseConverter()

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    node.get_logger().info("Node online.")

    poses_x = []
    poses_y = []
    actual_x = []
    actual_y = []
    drone_x = []
    drone_y = []

    while rclpy.ok():
        r, p, y = rpy_from_quaternion(*[node.drone_pose.orientation.x, node.drone_pose.orientation.y, node.drone_pose.orientation.z, node.drone_pose.orientation.w])
        # p += np.pi/2
        y += np.pi
        C = dcm_from_rpy(r, p, y)

        t = np.array([[node.drone_pose.position.x, node.drone_pose.position.y, node.drone_pose.position.z]]).T 

        Twc = np.eye(4)
        Twc[0:3, :] = np.hstack((C, t))
        node.get_logger().info(str((r, p, y)))
        node.get_logger().info(str(list(Twc)))

        # apply transformation to zebra
        zebra_point = np.array([[node.box_pose.position.x], [node.box_pose.position.y], [node.box_pose.position.z], [1]])
        zebra_point_cam = np.linalg.inv(Twc) @ zebra_point
        zebra_point_cam = zebra_point_cam[0:3, :]
        # node.get_logger().info(str(list(zebra_point_cam)))
        zebra_point_cam = node.C_cd @ zebra_point_cam
        zebra_point_cam = node.K @ zebra_point_cam 
        zebra_point_cam = zebra_point_cam / zebra_point_cam[2]

        node.cam_pose.x = float(zebra_point_cam[0])
        node.cam_pose.y = float(zebra_point_cam[1])

        poses_x.append(node.cam_pose.x)
        poses_y.append(node.cam_pose.y)
        actual_x.append(node.box_pose.position.x)
        actual_y.append(node.box_pose.position.y)
        drone_x.append(node.drone_pose.position.x)
        drone_y.append(node.drone_pose.position.y)
        if len(poses_x) == 1000:
            np.save("zebra_path.npy", np.vstack([poses_x, poses_y, actual_x, actual_y, drone_x, drone_y]))
            node.get_logger().info("path saved to \"zebra_path.npy\"")

        node.cam_pose_pub.publish(node.cam_pose)
        node.rate.sleep()

    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
