import cv2
import numpy as np
import os
import glob
import pickle
from tqdm import tqdm

use_precomputed_points = False

CHECKERBOARD = (6,8)
subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW
objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
_img_shape = None
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('checkerboard\*.jpg')

if not use_precomputed_points:
    for fname in tqdm(images):
        img = cv2.imread(fname)
        if _img_shape is None:
            _img_shape = img.shape[:2]
        else:
            try:
                assert _img_shape == img.shape[:2], f"All images must share the same size. {fname}"
            except Exception as e:
                print(e)
                print(fname)
                raise e
                
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)
            cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),subpix_criteria)
            imgpoints.append(corners)

    with open('objpoints.pkl', 'wb') as f:
        pickle.dump(objpoints, f)
    with open('imgpoints.pkl', 'wb') as f:
        pickle.dump(imgpoints, f)
else:
    with open('objpoints.pkl', 'rb') as f:
        objpoints = pickle.load(f)
    with open('imgpoints.pkl', 'rb') as f:
        imgpoints = pickle.load(f)
    
N_OK = len(objpoints)
K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]

print("Found " + str(N_OK) + " valid images for calibration")
print("Found " + str(len(imgpoints)) + "imgpts")

rms, _, _, _, _ = \
    cv2.fisheye.calibrate(
        objpoints,
        imgpoints,
        gray.shape[::-1],
        K,
        D,
        rvecs,
        tvecs,
        calibration_flags,
        (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )
print("K=np.array(" + str(K.tolist()) + ")")
print("D=np.array(" + str(D.tolist()) + ")")

np.save('cameraK.npy', K)
np.save('cameraD.npy', D)