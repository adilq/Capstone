# version of pipeline code that works for sim

# this node should
# - subscribe to cam
# - run model on received frame
# - publish out the number of zebras and bounding boxes via some datastructure
# take in image and run inference on it and return bounding boxes. 


"""
AFRIN TODO LIST: 
- label/annotate adil's camera lense corrected images so we can test on them. 
    - speaking of which, that's gonna need a testing script
- finish this node up - how does one load yolov8.pt into here

# - NEXT STEPS: 2 different models, other datasets, new training resolution, only if this script is done

"""
#ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point
from custom_messages.msg import BoundingBoxes # Import BBox message (custom)
#FasteR-CNN
from cv_bridge import CvBridge
# import torch
# from torchvision import models, transforms
import cv2
# from PIL import Image as PILImage
#YOLO
from ultralytics import YOLO
# other
import os
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node') # class constructor+name
                
        #MODEL LOAD
        cwd = os.getcwd().split('/')
        model_name = 'best_nano_augdata.pt'
        if cwd[-1] == 'drone_ws':
            model_path = f'src/cv_basics/cv_basics/{model_name}'
        elif cwd[-1] == 'src':
            model_path = f'cv_basics/cv_basics/{model_name}'
        elif cwd[-2:] == ['src', 'cv_basics']:
            model_path = f'cv_basics/{model_name}'
        else:
            model_path = model_name
            
        self.model = YOLO(model_path, task='detect') 
        self.model.eval()  #EVAL

        #PUB
        self.bbox_publisher = self.create_publisher(BoundingBoxes, '/bbox_out', 10) #topic bbox_out is using BoundingBox.msg type

        self.centroid_pub = self.create_publisher(PoseStamped, "/zebra_pose/image", 10)
        self.centroid = PoseStamped()
        self.centroid.header.frame_id = "camera"
        
        # # subscriber
        # self.image_sub = self.create_subscription(
        #     Image,
        #     "/camera/image_raw",
        #     self.camera_cb,
        #     10
        # )
        # self.br = CvBridge()
        # self.image = None


        #timer to determine how fast we do the callback
        self.timer = self.create_timer(0.2, self.image_callback)
         
        # capture object to keep streaming data
        self.cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360,format=(string)NV12, framerate=(fraction)10/1 ! \
            nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! appsink drop=true sync=false", cv2.CAP_GSTREAMER)

    def image_callback(self):
        # get a frame
        ret, cv_image = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture image from /dev/video0')
            return

        # if self.image is None:
        #     return

        # cv_image = self.image
        cv2.imwrite("sim_capture.png", cv_image)

        result = self.model(cv_image, conf=0.2, half=True)
        
        output = BoundingBoxes()
        if len(result[0].boxes.xyxy) > 0:
            # we detected something, fill message
            boxes_xyxy = (result[0].boxes.xyxy)
            bb = np.array(boxes_xyxy)
            output.x1 = bb[:, 0].tolist()
            output.y1 = bb[:, 1].tolist()
            output.x2 = bb[:, 2].tolist()
            output.y2 = bb[:, 3].tolist()

            output.cls = result[0].boxes.cls.to(int).tolist()
            output.conf = result[0].boxes.conf.tolist()

            self.get_logger().info("zebras detected!")

            # publish zebra pose whenever zebras are spotted in the new image
            zebra_x, zebra_y = self.calc_centroid(boxes_xyxy)
            self.centroid.pose.position = Point(x=float(zebra_x), y=float(zebra_y))
            self.centroid.header.stamp = self.get_clock().now().to_msg()

            self.centroid_pub.publish(self.centroid)

        #publish
        self.bbox_publisher.publish(output)
        self.get_logger().info("Published bboxes")

        

    def camera_cb(self, msg):
        self.get_logger().info("Receiving image.")
        self.image = self.br.imgmsg_to_cv2(msg)

    def calc_centroid(self, boxes):
        # boxes: in xyxy Tensor form
        bbox = np.array(boxes)
        x = np.mean([bbox[:, 0], bbox[:, 2]])
        y = np.mean([bbox[:, 1], bbox[:, 3]])

        return x, y

    
def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
 main()
