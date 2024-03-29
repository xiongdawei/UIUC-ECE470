#!/usr/bin/env python3

import cv2
import numpy as np
import math

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = 0.0394
beta = 750.01
ty = (320 - 259)/beta
tx = (240 - 53)/beta
# Function that converts image coord to world coord
def IMG2W(col, row):
    # coordinates of image pixel
    # print("xw and xy" + str((col, row))) 
    row -= 240
    col -= 320
    xw = (row*np.cos(-theta) - col*np.sin(-theta))/beta + tx
    xy = (row*np.sin(-theta) + col*np.cos(-theta))/beta + ty
    return [xw, xy, 3.2/100]
# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 50
    # Filter by Circularity
    params.filterByCircularity = False
    params.minCircularity = 0.5
    params.maxCircularity = 0.6

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.7

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)
    hsv_image[:150, :640] = np.zeros([150, 640, 3])


    # ========================= Student's code starts here =========================
    mask_image = None
    if color == "blue":
        lower = (110,50,50)     # blue lower
        upper = (130,255,255)  

    elif color == 'green':
        lower = (35, 60, 70)
        upper = (95, 255, 200)

    elif color == 'red':
        lower_red = np.array([0,50,50])
        upper_red = np.array([10,255,255])
        lower_red2 = np.array([160,50,50])
        upper_red2 = np.array([180,255,255])
        mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask_image = cv2.bitwise_or(mask1, mask2)
    if color != 'red':
        mask_image = cv2.inRange(hsv_image, lower, upper)

    # Define a mask using the lower and upper bounds of the target color
  

    # ========================= Student's code ends here ===========================
    mask_image[:150, :640] = np.zeros([150, 640])
    # mask_image[300:, :640] = np.zeros([120, 640])
    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = 0
    for i in range(len(keypoints)):
        if (keypoints[i].pt[0] > 200 and keypoints[i].pt[0] < 456 and keypoints[i].pt[1] > 150 and keypoints[i].pt[1] < 370):
            blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))
            num_blobs += 1

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, None, color=(0,0,255), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    im_with_keypoints[:150, :640] = np.zeros([150,640, 3])
    # im_with_keypoints[300:, :640] = np.zeros([120,640, 3])
    # ========================= Student's code ends here ===========================

    xw_yw = []

    # if(num_blobs == 0):
        # print("No block found!")
        
    #else:
    if (num_blobs != 0):
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))
            #print("append xw yw" + str(xw_yw))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
