import cv2

from ultralytics import YOLO
# import torch

# cv_image = cv2.imread('goodpic.jpg')
q = False # flag to stop execution

disp = input("Do you want to display the most recent frame? y/n:")

cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)720, height=(int)480,format=(string)NV12, framerate=(fraction)30/1 ! \
    nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink drop=true sync=false", cv2.CAP_GSTREAMER)

while not q:
    try:
        ret, cv_image = cap.read()
        if not ret:
            print('Failed to capture image from /dev/video0')
        else:
            print('Frame captured')
            
        model = YOLO('best_nano.pt')
        model.eval()
        result = model(cv_image)

        boxes_xyxy =  result[0].boxes.xyxy
        print(list(boxes_xyxy[:, 0]))
        
        if disp.lower() == 'y':
            # print(result[0])
            # if result[0].probs is not None:
                # boxes = result[0].boxes
                # labels = result[0].boxes.cls  
                # scores = result[0].boxes.conf
                # for i, box in enumerate(boxes):
                #     x1, y1, x2, y2 = box
                #     label = labels[i]
                #     score = scores[i]

                #     #DRAW Bbox
                #     cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

                #     #LABEL + SCORE text
                #     cv2.putText(cv_image, f'{label}: {score:.2f}', (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # result[0].show()

            # #DISP Image with Bboxes
            cv2.imshow("Detected Objects", cv_image)
            cv2.waitKey(1)  
        
    except KeyboardInterrupt:
        print("End test")
        q = True

# boxes_cls =  torch.Tensor.numpy(result[0].boxes.cls)
# boxes_conf = torch.Tensor.numpy(result[0].boxes.conf)
# boxes_id =  torch.Tensor.numpy(result[0].boxes.id)

# masks = result.masks
# keypoints = result.keypoints  
# probs = result.probs 
# obb = result.obb  
# result[0].show()  
# result[0].save(filename="result.jpg")  

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

# # Displaying the results:
# print(f"Got {len(boxes)} objects")
# print(f"BBoxes: {boxes}")
# print(f"Labels: {labels}")
# print(f"Scores: {scores}")