import numpy as np
import cv2 as cv
import glob
import os

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((18*13,3), np.float32)
objp[:,:2] = np.mgrid[0:13,0:18].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints_left = [] # 2d points in left image plane.
imgpoints_right = [] # 2d points in right image plane.

image_dir = "images"
if not os.path.exists(image_dir):
    print(f"Error: {image_dir} directory does not exist")
    exit()

# Read the left and right images from the "images" folder
gray = None
for fname in os.listdir(image_dir):
    if "left" in fname:
        img = cv.imread(os.path.join(image_dir, fname))
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (18,13), None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints_left.append(corners2)

            # Draw and display the corners
            cv.drawChessboardCorners(img, (18,13), corners2, ret)
            cv.imshow('Left Image', img)
            cv.waitKey(500)

    elif "right" in fname:
        img = cv.imread(os.path.join(image_dir, fname))
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (18,13), None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints_right.append(corners2)

            # Draw and display the corners
            cv.drawChessboardCorners(img, (18,13), corners2, ret)
            cv.imshow('Right Image', img)
            cv.waitKey(500)

if gray is None:
    print("Error: no images found")
    exit()

# Calibrate the camera using the left and right image points
ret, mtx_left, dist_left, mtx_right, dist_right, R, T, E, F = cv.stereoCalibrate(
    objpoints, imgpoints_left, imgpoints_right, None, None, None, None, gray.shape[::-1])

# Print the reprojection error
mean_error = cv.stereoCalibrate(objpoints, imgpoints_left, imgpoints_right, mtx_left, dist_left, mtx_right, dist_right, gray.shape[::-1])[1]
print("Reprojection error: {}".format(mean_error))

R_left, R_right, P_left, P_right, Q, _, _ = cv.stereoRectify(mtx_left, dist_left, mtx_right, dist_right, gray.shape[::-1], R, T)

map_left_x, map_left_y = cv.initUndistortRectifyMap(mtx_left, dist_left, R_left, P_left, gray.shape[::-1], cv.CV_32FC1)
map_right_x, map_right_y = cv.initUndistortRectifyMap(mtx_right, dist_right, R_right, P_right, gray.shape[::-1], cv.CV_32FC1)

cv.destroyAllWindows()