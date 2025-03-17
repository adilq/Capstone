import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import matplotlib.pyplot as plt

class CompressedImagePublisher(Node):
    def __init__(self):
        super().__init__('compressed_image_publisher')
        self.publisher = self.create_publisher(CompressedImage, 'image/compressed', 10)
        self.timer = self.create_timer(5, self.publish_image)  # Publish at 5 Hz
        self.cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink")  # Open video device

    def publish_image(self):
        ret, frame = self.cap.read()
        plt.imshow(frame)
        self.get_logger().info("frame captured")
        plt.pause(1)
        self.get_logger().info(f"frame shape: {frame.shape}")
        if not ret:
            self.get_logger().error('Failed to capture image from /dev/video0')
            return
        
        # Compress the image to JPEG format
        ret, jpeg_data = cv2.imencode('.jpg', frame)
        if not ret:
            self.get_logger().error('Failed to encode image as JPEG')
            return
        
        # Create a CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = jpeg_data.tobytes()
        self.image_callback(msg)

        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info('Published compressed image')
    
    def image_callback(self, msg):
        try:
            # Convert the compressed image data back to an OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if image is None:
                self.get_logger().error('Failed to decode compressed image')
                return

            # Display the image using OpenCV
            # cv2.imwrite('Compressed Image', image)
            # cv2.waitKey(1)  # Required for OpenCV to update the window
            plt.imshow(image)
            plt.pause(1)
        except Exception as e:
            self.get_logger().error(f'Error processing the image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CompressedImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node...')
    finally:
        node.cap.release()  # Release the video capture device
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
