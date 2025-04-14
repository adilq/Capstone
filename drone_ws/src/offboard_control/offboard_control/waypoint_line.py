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
        p1.position = Point(x=0., y=0.5, z=2.)
        p2 = Pose()
        p2.position = Point(x=0., y=0., z=2.)
        p3 = Pose()
        p3.position = Point(x=20., y=0., z=2.)
        
        for pose in [p1, p2, p3]:
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
