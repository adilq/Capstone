import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import matplotlib.pyplot as plt
import numpy as np

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'image/compressed',
            self.image_callback,
            10
        )
        self.get_logger().info('Subscribed to image/compressed topic')

    def image_callback(self, msg):
        try:
            # Convert the compressed image data back to an OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if image is None:
                self.get_logger().error('Failed to decode compressed image')
                return

            # Display the image using OpenCV
            cv2.imshow('Compressed Image', image)
            cv2.waitKey(1)  # Required for OpenCV to update the window
            # plt.imshow(image)
            # plt.pause(1)
        except Exception as e:
            self.get_logger().error(f'Error processing the image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node...')
    finally:
        cv2.destroyAllWindows()  # Close OpenCV windows
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
