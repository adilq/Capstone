import cv2

from ultralytics import YOLO
import torch

from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument(
    "image",
    type=str,
    help="input image file"
)

args = parser.parse_args()

# cv_image = cv2.imread('aerial_zebras.jpg')
cv_image = cv2.imread(args.image)
model = YOLO('best_nano_augdata.pt')
model.eval()
result = model(cv_image)
boxes_xyxy =  result[0].boxes.xyxy
# print(boxes_xyxy[:, 0])

# boxes_cls =  torch.Tensor.numpy(result[0].boxes.cls)
# boxes_conf = torch.Tensor.numpy(result[0].boxes.conf)
# boxes_id =  torch.Tensor.numpy(result[0].boxes.id)

# masks = result.masks
# keypoints = result.keypoints  
# probs = result.probs 
# obb = result.obb  
result[0].show()  
# result[0].save(filename="result.jpg")  
