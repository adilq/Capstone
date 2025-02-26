import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import rclpy.time
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool

import threading

class OffboardNode(Node):
    def __init__(self):
        super().__init__('offboard_node')

        self.state_sub = self.create_subscription(State, 'mavros/state', callback = self.state_callback, qos_profile=10)
        self.state = State()

        # create client for set mode, use with wait for service
        self.set_mode_cli = self.create_client(SetMode, 'mavros/set_mode')
        while not self.set_mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_mode service not available')

        # create client for arming
        self.arm_cli = self.create_client(CommandBool, 'mavros/cmd/arming')
        while not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arming service not available')

        self.rate = self.create_rate(1, clock=self.get_clock())

        self.prev_request = self.get_clock().now()
        self.offb_set_mode = SetMode.Request()
        self.offb_set_mode.custom_mode = "OFFBOARD"
        self.arm_cmd = CommandBool.Request()
        self.arm_cmd.value = True 

    def state_callback(self, msg):
        self.state = msg

        self.get_logger().debug(f"Received: {msg}")

def main(args=None):
    rclpy.init(args=args)
    node = OffboardNode()

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    while rclpy.ok():
        if node.get_clock().now() - node.prev_request > Duration(seconds=5.0):
            node.get_logger().debug(f"current mode: {node.state.mode}")
            if not node.state.armed:
                node.get_logger().debug("attempting to arm")
                if node.arm_cli.call(node.arm_cmd).success:
                    node.get_logger().info("Vehicle armed")
            elif node.state.mode != "OFFBOARD":
                node.get_logger().debug("attempting to offboard")
                if node.set_mode_cli.call(node.offb_set_mode).mode_sent:
                    node.get_logger().info("OFFBOARD enabled")                

            node.prev_request = node.get_clock().now()

        node.rate.sleep()


    node.destroy_node()
    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()