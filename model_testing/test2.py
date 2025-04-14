import cv2

from ultralytics import YOLO
import torch

cv_image = cv2.imread('aerial_zebras.jpg')
model = YOLO('best.pt')
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
