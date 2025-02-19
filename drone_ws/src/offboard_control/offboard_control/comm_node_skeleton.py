import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger

# Callback handlers
def handle_launch():
    # publish to some topic that tells the offb_node to do some predetermined launch sequence
    print('Launch Requested. Your drone should take off.')

def handle_test():
    # publish to some topic that tells the offb_node to hover in place
    print('Test Requested. Your drone should perform the required tasks. Recording starts now.')

def handle_land():
    # publish to some topic that tells the offb_node to do some predetermined land sequence
    print('Land Requested. Your drone should land.')

def handle_abort():
    # publish to some topic that tells the offb_node to kill the motors? land? switch to manual control?
    print('Abort Requested. Your drone should land immediately due to safety considerations')

# Service callbacks
def callback_launch(request, response):
    handle_launch()
    return response

def callback_test(request, response):
    handle_test()
    return response

def callback_land(request, response):
    handle_land()
    return response

def callback_abort(request, response):
    handle_abort()
    return response

class CommNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_XX')
        self.srv_launch = self.create_service(Trigger, 'rob498_drone_XX/comm/launch', callback_launch)
        self.srv_test = self.create_service(Trigger, 'rob498_drone_XX/comm/test', callback_test)
        self.srv_land = self.create_service(Trigger, 'rob498_drone_XX/comm/land', callback_land)
        self.srv_abort = self.create_service(Trigger, 'rob498_drone_XX/comm/abort', callback_abort)

def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
