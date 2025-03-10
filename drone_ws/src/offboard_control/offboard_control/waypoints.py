import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose

# TO-DO: set up waypoints as a parameter we can modify from the command line

class WaypointNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_8')
        self.waypoints_pub = self.create_publisher(PoseArray, 'rob498_drone_8/comm/waypoints', 10)

        self.rate = self.create_rate(30)
	
def main(args=None):
	rclpy.init(args=args)
	node = WaypointNode()
	node.waypoints_pub.publish(WAYPOINTS)
	rclpy.spin(node)

if  __name__ == '__main__':
	main()
