import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class Redirector(Node):
	def __init__(self):
		super().__init__('redirector_node')
		self.publisher_ = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
		self.subscriber_ = self.create_subscription(PoseStamped, '/vicon/ROB498_Drone/ROB498_Drone', self.pose_callback, 10)
		self.subscriber_

	def pose_callback(self, msg):
		out_msg = msg
		out_msg.header.frame_id = 'map'
		self.publisher_.publish(out_msg)
	
def main(args=None):
	rclpy.init(args=args)
	redirector = Redirector()
	rclpy.spin(redirector)

if  __name__ == '__main__':
	main()