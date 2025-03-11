import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose

# TO-DO: set up waypoints as a parameter we can modify from the command line
p1 = Pose()
p2 = Pose()
p3 = Pose()
p4 = Pose()
waypoints = PoseArray()
waypoints.poses = [p1, p2, p3, p4]

class WaypointNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_8')
        self.waypoints_pub = self.create_publisher(PoseArray, 'rob498_drone_8/comm/waypoints', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        
    def timer_callback(self):
        self.waypoints_pub.publish(waypoints)
	
def main(args=None):
	rclpy.init(args=args)
	node = WaypointNode()
	rclpy.spin(node)
	rclpy.shutdown()

if  __name__ == '__main__':
	main()
