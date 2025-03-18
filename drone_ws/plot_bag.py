import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
import time
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting 
import threading

last_msg_time = 0    
playing_started = False   
local_x = []
local_y = []
local_z = []
local_t = []
vision_x = []
vision_y = []
vision_z = []
vision_t = []

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('plotter')
        self.local_pose = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_pose_callback,
            qos_profile=rclpy.qos.qos_profile_system_default)
        # self.local_pose = self.create_subscription(
        #     PoseStamped,
        #     '/mavros/local_position/pose',
        #     self.local_pose_callback,
        #     10)
        self.vision_pose = self.create_subscription(
            PoseStamped,
            '/mavros/vision_pose/pose',
            self.vision_pose_callback,
            10)
        self.local_pose  # prevent unused variable warning
        self.vision_pose  # prevent unused variable warning
        self.rate = self.create_rate(1)

    def local_pose_callback(self, msg):
        global playing_started
        playing_started = True
        global local_x; global local_y; global local_z; global last_msg_time
        global local_t
        last_msg_time = time.time()
        local_x.append(msg.pose.position.x)
        local_y.append(msg.pose.position.y)
        local_z.append(msg.pose.position.z)
        local_t.append(msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9)
        print(local_x[-1])
        
    def vision_pose_callback(self, msg):
        global playing_started
        playing_started = True
        global vision_x; global vision_y; global vision_z; global last_msg_time
        global vision_t
        last_msg_time = time.time()
        vision_x.append(msg.pose.position.x)
        vision_y.append(msg.pose.position.y)
        vision_z.append(msg.pose.position.z)
        vision_t.append(msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9)
        print(vision_x[-1])
        


def main(args=None):
    global last_msg_time
    global playing_started
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    thread = threading.Thread(target=rclpy.spin, args=(minimal_subscriber,), daemon=True)
    thread.start()
    
    while rclpy.ok() and not playing_started:
        minimal_subscriber.rate.sleep()
        minimal_subscriber.get_logger().info('not started playing yet')

    while rclpy.ok() and time.time() - last_msg_time < 1:
        minimal_subscriber.rate.sleep()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
    print(f'local_x length: {len(local_x)}, vision_x length: {len(vision_x)}')
    ax = plt.figure().add_subplot(projection='3d')
    # axv = plt.figure().add_subplot(projection='3d')
    ax.plot(local_x, local_y, [t - local_t[0] for t in local_t], zdir='z', label='local_pose', c='b')
    ax.plot(vision_x, vision_y, [t - vision_t[0] for t in vision_t], zdir='z', label='vision_pose', c='r')
    plt.show()