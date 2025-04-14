import glob
from roboflow import Roboflow
import os

apikey = open("API.txt", 'r').read()

rf = Roboflow(api_key=apikey)

imagedir_PATH = "2.Capstone\\datasets\\zebras_WAID_images\\valid"
file_extension_type = ".jpg"

annotation_PATH = "2.Capstone\\datasets\\zebras_WAID_labels\\valid"

project = rf.workspace().project("od1-7yvbk")
labelmap_PATH = "2.Capstone\\datasets\\labelmap_zebraonly.txt"

image_glob = glob.glob(imagedir_PATH + '/*' + file_extension_type)

# for image_path in image_glob:
#     # print(image_path)
#     annotation_filename = image_path.split('\\')[-1][:-4]+".txt"
#     print(annotation_filename)

#     print(project.single_upload(
#         image_path=image_path,
#         annotation_path=annotation_PATH+'\\'+annotation_filename,
#         # -- optional parameters: --
#         annotation_labelmap=labelmap_PATH,
#         split='train'
#         # num_retry_uploads=0,
#         # batch_name='batch_name',
#         # tag_names=['tag1', 'tag2'],
#         # is_prediction=False,
#     ))