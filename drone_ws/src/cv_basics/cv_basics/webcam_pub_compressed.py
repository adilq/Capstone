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
        self.cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink")  # Open video device

    def publish_image(self):
        ret, frame = self.cap.read()
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

        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info('Published compressed image')

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
