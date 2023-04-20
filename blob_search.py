#!/usr/bin/env python3

import cv2
import numpy as np
import math

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = 0.0656
beta = 770.584
ty = (320 - 267)/beta
tx = (240 - 52)/beta
# Function that converts image coord to world coord
def IMG2W(col, row):
    # coordinates of image pixel
    # print("xw and xy" + str((col, row))) 
    row -= 240
    col -= 320
    xw = (row*np.cos(-theta) - col*np.sin(-theta))/beta + tx
    xy = (row*np.sin(-theta) + col*np.cos(-theta))/beta + ty
    return [xw, xy, 3.4/100]
# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 200
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.4

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================
    mask_image = None
    if color == "blue":
        lower = (110,50,50)     # blue lower
        upper = (130,255,255)   # blue upper

    elif color == 'green':
        lower = (50, 50, 50)
        upper = (70, 255, 255)

    elif color == 'yellow':
        lower = (20, 50, 50)
        upper = (30, 255, 255)
    
    elif color == 'red':
        lower_red = [0,50,50]
        upper_red = [10,255,255]
        lower_red2 = [160,50,50]
        upper_red2 = [180,255,255]
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_image = cv2.bitwise_or(mask1, mask2)
    if color != 'red':
        mask_image = cv2.inRange(hsv_image, lower, upper)

    # Define a mask using the lower and upper bounds of the target color
  

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, None, color=(0,0,255), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # ========================= Student's code ends here ===========================

    xw_yw = []

    # if(num_blobs == 0):
        #print("No block found!")
        
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
