import cv2
import numpy as np
import glob
from tqdm import tqdm

# You should replace these 3 lines with the output in calibration step
DIM=(640, 480)
K=np.load('cameraK.npy')
D=np.load('cameraD.npy')

def undistort(img_path):
    img = cv2.imread(img_path)
    h,w = img.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    # both = np.concatenate((img, undistorted_img), axis=0)
    # cv2.imshow("compare", both)
    # cv2.waitKey(1000)
    # cv2.destroyAllWindows()
    return undistorted_img

files = glob.glob('aruco_markers\\*.png')
# test_set = np.array(files)[np.random.randint(0, len(files), (20,))]

for fname in tqdm(files):
    ud_img = undistort(fname)
    n = fname.split('\\')[-1]
    cv2.imwrite(f'aruco_markers_undistorted\\{n}', ud_img)
    
