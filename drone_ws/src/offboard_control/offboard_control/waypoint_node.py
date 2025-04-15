import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point

# TO-DO: set up waypoints as a parameter we can modify from the command line

class WaypointNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_8')
        self.waypoints_pub = self.create_publisher(PoseArray, 'rob498_drone_8/comm/waypoints', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.waypoints = PoseArray()
        p1 = Pose()
        # p1.position.x = 5
        # p1.position.y =5
        # p1.pose.position.z = 5
        p1.position = Point(x=1.8, y=1.9, z=1.0)
        # p1.pose.orientation = Quaternion(x=0., y=0., z=0., w=1.)
        # 1781 1884 893

        p2 = Pose()
        p2.position = Point(x=1.8, y=-1.8, z=1.0)
        # p2.pose.orientation = Quaternion(x=0., y=0., z=0., w=1.)
        # 1765 -1778 897

        p3 = Pose()
        p3.position = Point(x=-1.8, y=-1.6, z=1.0)
        # p3.pose.orientation = Quaternion(x=0., y=0., z=0., w=1.)
        # -1785 -1607 941

        p4 = Pose()
        p4.position = Point(x=-1.7, y=2.0, z=1.0)
        # p3.pose.orientation = Quaternion(x=0., y=0., z=0., w=1.)
        # -1679 1961 931
        for pose in [p1, p2, p3, p4]:
            self.waypoints.poses.append(pose)
        
    def timer_callback(self):
        self.waypoints_pub.publish(self.waypoints)
	
def main(args=None):
	rclpy.init(args=args)
	node = WaypointNode()
	rclpy.spin(node)
	rclpy.shutdown()

if  __name__ == '__main__':
	main()
