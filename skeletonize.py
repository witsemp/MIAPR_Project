# Import the necessary libraries
import cv2
import numpy as np

def skeletonize(image):
    # img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # ret, img = cv2.threshold(img, 127, 255, 0)
    img = image
    wh_pixels_up = []
    wh_pixels_left = []
    wh_pixels_right = []
    thresh_erode = img.copy()
    size = np.size(img)
    skel = np.zeros(img.shape, np.uint8)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    while True:
        open = cv2.morphologyEx(img, cv2.MORPH_OPEN, element)
        temp = cv2.subtract(img, open)
        eroded = cv2.erode(img, element)
        skel = cv2.bitwise_or(skel, temp)
        img = eroded.copy()
        if cv2.countNonZero(img) == 0:
            break
    thresh_erode = cv2.erode(thresh_erode, np.ones((7, 7), dtype=np.uint8), iterations=20)
    out = cv2.bitwise_and(skel, thresh_erode)
    out1 = cv2.threshold(out, 127, 255, cv2.THRESH_BINARY_INV)
    for col in range(np.shape(out)[1]):
        if out[0][col] != 0:
            wh_pixels_up.append((0, col))
    for row in range(np.shape(out)[0]):
        if out[row][0] != 0:
            wh_pixels_left.append((row, 0))
    for row in range(np.shape(out)[0]):
        if out[row][np.shape(out)[0]-1] != 0:
            wh_pixels_right.append((row, 0))

    # find goal point
    if len(wh_pixels_up) > 0:
        goal_point = wh_pixels_up[0]
        goal_point = list(goal_point)
        goal_point[0] = goal_point[0] + np.shape(image)[0]/2
        goal_point = tuple(goal_point)
        # print("Goal point" + str(goal_point))

    if len(wh_pixels_left) > 0:
        goal_point = wh_pixels_left[0]
        goal_point = list(goal_point)
        goal_point[1] = goal_point[1] + np.shape(image)[1]/35
        goal_point = tuple(goal_point)
        # print("Goal point" + str(goal_point))

    if len(wh_pixels_right) > 0:
        goal_point = wh_pixels_right[0]
        goal_point = list(goal_point)
        goal_point[1] = goal_point[1] + np.shape(image)[1]/35
        goal_point = tuple(goal_point)
        # print("Goal point" + str(goal_point))

    cv2.imshow('skel', out)
    # cv2.imshow('ihohio', out1)



    wh_pixels_left.clear()
    wh_pixels_right.clear()
    wh_pixels_up.clear()
    return goal_point
