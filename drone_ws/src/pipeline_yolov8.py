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
from std_msgs.msg import Header
from custom_msgs.msg import BoundingBoxes # Import BBox message (custom)
#FasteR-CNN
# from cv_bridge import CvBridge
import torch
from torchvision import models, transforms
import cv2
from PIL import Image as PILImage
#YOLO
from ultralytics import YOLO


class ObjectDetectionNode(Node):
    """
    init bbox publisher node subclass of Node class.
    """
    def __init__(self):
        super().__init__('object_detection_node') # class constructor+name
        self.bridge = CvBridge()
        
        #MODEL LOAD
        self.model = model = YOLO('best.pt') 
        # self.model = models.detection.fasterrcnn_resnet50_fpn(pretrained=True)
        self.model.eval()  #EVAL

        #SUB/PUB
        self.subscription = self.create_subscription(Image, '/camera/video_frames', self.image_callback, 10) #pretty sure cam sends me ROSImage RN, but can send .jpg probably
        self.bbox_publisher = self.create_publisher(BoundingBoxes, '/bbox_out', 10) #topic bbox_out is using BoundingBox.msg type

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") #ROSImage to CV2IMAGE
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return
        
        #Image Preprocess - get PILImage, make tensor
        pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)) #CV2 image to PILImage
        transform = transforms.Compose([transforms.ToTensor()])
        input_image = transform(pil_image).unsqueeze(0)

        results = self.model(cv_image)
        #results = model(input_image)
        boxes = results.pred[0].boxes  
        labels = results.pred[0].cls  
        scores = results.pred[0].conf

        # Displaying the results:
        print(f"Got {len(boxes)} objects")
        print(f"BBoxes: {boxes}")
        print(f"Labels: {labels}")
        print(f"Scores: {scores}")

        # OPTIONAL: Tried to filter low-confidence predictions
        # threshold = 0.5
        # boxes = boxes[scores > threshold]
        # labels = labels[scores > threshold]
        # scores = scores[scores > threshold]        

        #FOR TEST: look at the boxes in the image
        # for i, box in enumerate(boxes):
        #     x1, y1, x2, y2 = box
        #     label = labels[i]
        #     score = scores[i]

        #     #DRAW Bbox
        #     cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

        #     #LABEL + SCORE text
        #     cv2.putText(cv_image, f'{label}: {score:.2f}', (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # #DISP Image with Bboxes
        # cv2.imshow("Detected Objects", cv_image)
        # cv2.waitKey(1)    
        
    
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