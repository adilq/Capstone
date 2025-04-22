import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
import time
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting 
import threading
import json

last_msg_time = 0    
playing_started = False   
local_x = []
local_y = []
local_z = []
local_t = []
setpoint_x = []
setpoint_y = []
setpoint_z = []
setpoint_t = []
zebra_x = []
zebra_y = []
zebra_z = []
zebra_t = []
box_x = []
box_y = []
box_z = []
box_t = []

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('plotter')
        self.local_pose = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_pose_callback,
            qos_profile=rclpy.qos.qos_profile_system_default)

        self.setpoint_position = self.create_subscription(
            PoseStamped,
            '/mavros/setpoint_position/local',
            self.setpoint_pose_callback,
            10)
        
        self.zebra_pose = self.create_subscription(
            PoseStamped,
            '/zebra_pose/image',
            self.zebra_pose_callback,
            10)

        self.box_position = self.create_subscription(
            PoseStamped,
            '/box/position/global',
            self.box_pose_callback,
            10)
        
        self.local_pose  # prevent unused variable warning
        self.setpoint_position  # prevent unused variable warning
        self.zebra_pose  # prevent unused variable warning
        self.box_position  # prevent unused variable warning
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
        
    def setpoint_pose_callback(self, msg):
        global playing_started
        playing_started = True
        global setpoint_x; global setpoint_y; global setpoint_z; global last_msg_time
        global setpoint_t
        last_msg_time = time.time()
        setpoint_x.append(msg.pose.position.x)
        setpoint_y.append(msg.pose.position.y)
        setpoint_z.append(msg.pose.position.z)
        setpoint_t.append(msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9)
        print(setpoint_x[-1])
        
    def zebra_pose_callback(self, msg):
        global playing_started
        playing_started = True
        global zebra_x; global zebra_y; global zebra_z; global last_msg_time
        global zebra_t
        last_msg_time = time.time()
        zebra_x.append(msg.pose.position.x)
        zebra_y.append(msg.pose.position.y)
        zebra_z.append(msg.pose.position.z)
        zebra_t.append(msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9)
        print(zebra_x[-1])
        
    def box_pose_callback(self, msg):
        global playing_started
        playing_started = True
        global box_x; global box_y; global box_z; global last_msg_time
        global box_t
        last_msg_time = time.time()
        box_x.append(msg.pose.position.x)
        box_y.append(msg.pose.position.y)
        box_z.append(msg.pose.position.z)
        box_t.append(msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9)
        print(box_x[-1])
        


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
    print(f'local_x length: {len(local_x)}, setpoint_x length: {len(setpoint_x)}, zebra_x length: {len(zebra_x)}, box_x length: {len(box_x)}')
    local = {'x': local_x, 'y': local_y, 'z': local_z, 't': local_t}
    setpoint = {'x': setpoint_x, 'y': setpoint_y, 'z': setpoint_z, 't': setpoint_t}
    zebra = {'x': zebra_x, 'y': zebra_y, 'z': zebra_z, 't': zebra_t}
    box = {'x': box_x, 'y': box_y, 'z': box_z, 't': box_t}
    
    with open('poses_eval9', 'w+') as f:
        json_ = json.dumps({'local': local, 'setpoint': setpoint, 'zebra': zebra, 'box': box})
        print(json_, file=f)
        
    ax = plt.figure().add_subplot(projection='3d')
    # axv = plt.figure().add_subplot(projection='3d')
    ax.plot(local_x, local_y, [t - local_t[0] for t in local_t], zdir='z', label='local_pose', c='b')
    ax.plot(setpoint_x, setpoint_y, [t - setpoint_t[0] for t in setpoint_t], zdir='z', label='setpoint_pose', c='r')
    # ax.plot(zebra_x, zebra_y, [t -zebra_t[0] for t in zebra_t], zdir='z', label='zebra_pose', c='g')
    ax.plot(box_x, box_y, [t - box_t[0] for t in box_t], zdir='z', label='box_pose', c='k')
    plt.show()