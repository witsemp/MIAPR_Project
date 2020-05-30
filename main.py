import gym
import numpy as np
import cv2 as cv
import math
from PIL import Image
import rrt
def process_obs(obs):
    goal_row = 0
    goal_col = 0
    goal_point = (goal_row, goal_col)
    wh_pixels_up = []
    wh_pixels_left = []
    wh_pixels_right = []
    start_point = (72, 48)
    # create and save observation image
    img = Image.fromarray(obs, 'RGB')
    img.save('obs.png')
    # load observation image in greyscale
    obs_greyscale = cv.imread('obs.png', 0)
    # threshold image to create occupancy map
    ret, obs_thresh = cv.threshold(obs_greyscale, 127, 255, cv.THRESH_BINARY_INV)
    # crop the status bar
    obs_thresh = obs_thresh[0:84, 0:96]
    # convert thresholded image back to BGR
    obs_thresh = cv.cvtColor(obs_thresh, cv.COLOR_GRAY2RGB)
    # draw starting point
    obs_thresh[67:77, 46:50] = (0, 0, 255)
    # resize to match game window size
    obs_thresh = cv.resize(obs_thresh, (1000, 700))
    # count white pixels on image borders
    for col in range(np.shape(obs_thresh)[1]):
        if obs_thresh[0][col][0] == 255 and obs_thresh[0][col][1] == 255 and obs_thresh[0][col][2] == 255:
            wh_pixels_up.append(col)
    for row in range(np.shape(obs_thresh)[0]):
        if obs_thresh[row][0][0] == 255 and obs_thresh[row][0][1] == 255 and obs_thresh[row][0][2] == 255:
            wh_pixels_left.append(row)
    for row in range(np.shape(obs_thresh)[0]):
        if obs_thresh[row][np.shape(obs_thresh)[0]-1][0] == 255 and obs_thresh[row][np.shape(obs_thresh)[0]-1][1] == 255 and obs_thresh[row][np.shape(obs_thresh)[0]-1][2] == 255:
            wh_pixels_right.append(row)

    # find goal point
    if len(wh_pixels_left) > 0:
        goal_row = (wh_pixels_left[-1] + wh_pixels_left[0])/2
        goal_point = (round(goal_row), 0)
        print("Goal point" + str(goal_point))

    if len(wh_pixels_right) > 0:
        goal_row = (wh_pixels_right[-1] + wh_pixels_right[0])/2
        goal_point = (round(goal_row), np.shape(obs_thresh)[0]-1)
        print("Goal point" + str(goal_point))

    if len(wh_pixels_up) > 0:
        goal_col = (wh_pixels_up[-1] + wh_pixels_up[0])/2
        goal_point = (round(goal_row), round(goal_col))
        print("Goal point" + str(goal_point))


    obs_thresh[goal_point[0]:goal_point[0]+10, goal_point[1]:goal_point[1]+10, 0] = 255
    obs_thresh[goal_point[0]:goal_point[0]+10, goal_point[1]:goal_point[1]+10, 1] = 0
    obs_thresh[goal_point[0]:goal_point[0]+10, goal_point[1]:goal_point[1]+10, 2] = 0

    wh_pixels_left.clear()
    wh_pixels_right.clear()
    wh_pixels_up.clear()
    cv.imshow('thresh', obs_thresh)
    return obs_thresh, start_point, goal_point
env = gym.make('CarRacing-v0')
observation = env.reset()
# process_obs(observation)
render = 0
while render < 1000:

    render = render + 1
    env.render()
    action = [0, 0.01, 0]
    print("Render no: " + str(render))
    print("Action taken: " + str(action))
    print("--------------------------------")
    observation, reward, done, info = env.step(action)
    if render > 30:
        thresh, start_point, goal_point = process_obs(observation)
        RRT = rrt.RRT(thresh, start_point, goal_point)
        RRT.search()






env.close()





