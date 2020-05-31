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
    start_point = (500, 600)

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
    # obs_thresh = cv.cvtColor(obs_thresh, cv.COLOR_GRAY2RGB)

    # draw starting point
    # obs_thresh[67:77, 46:50] = (0, 0, 255)

    # resize to match game window size
    obs_thresh = cv.resize(obs_thresh, (1000, 700))

    # count white pixels on image borders
    for col in range(np.shape(obs_thresh)[1]):
        if obs_thresh[0][col] == 255:
            wh_pixels_up.append(col)
    for row in range(np.shape(obs_thresh)[0]):
        if obs_thresh[row][0] == 255:
            wh_pixels_left.append(row)
    for row in range(np.shape(obs_thresh)[0]):
        if obs_thresh[row][np.shape(obs_thresh)[0]-1] == 255:
            wh_pixels_right.append(row)

    # find goal point
    if len(wh_pixels_left) > 0:
        goal_row = (wh_pixels_left[-1] + wh_pixels_left[0])/2
        goal_point = (round(goal_row), 0)
        # print("Goal point" + str(goal_point))

    if len(wh_pixels_right) > 0:
        goal_row = (wh_pixels_right[-1] + wh_pixels_right[0])/2
        goal_point = (round(goal_row), np.shape(obs_thresh)[0]-1)
        # print("Goal point" + str(goal_point))

    if len(wh_pixels_up) > 0:
        goal_col = (wh_pixels_up[-1] + wh_pixels_up[0])/2
        goal_point = (round(goal_row), round(goal_col))
        # print("Goal point" + str(goal_point))

    # obs_thresh[goal_point[0]:goal_point[0]+10, goal_point[1]:goal_point[1]+10, 0] = 255
    # obs_thresh[goal_point[0]:goal_point[0]+10, goal_point[1]:goal_point[1]+10, 1] = 0
    # obs_thresh[goal_point[0]:goal_point[0]+10, goal_point[1]:goal_point[1]+10, 2] = 0

    # goal_point = (goal_point[1], goal_point[0])
    end_point = (goal_point[1], goal_point[0])
    wh_pixels_left.clear()
    wh_pixels_right.clear()
    wh_pixels_up.clear()
    # cv.imshow('thresh', obs_thresh)
    return obs_thresh, start_point, end_point

def draw_path(points, graph):
    obs_rgb = cv.imread('obs.png', 1)
    obs_rgb = obs_rgb[0:84, 0:96, :]
    obs_rgb = cv.resize(obs_rgb, (1000, 700))
    # cv.imwrite('rgb_resized.png', obs_rgb)
    for key, value in graph.items():
        obs_rgb[key[1]:key[1]+5, key[0]:key[0]+5, 0] = 0
        obs_rgb[key[1]:key[1]+5, key[0]:key[0]+5, 1] = 0
        obs_rgb[key[1]:key[1]+5, key[0]:key[0]+5, 2] = 255
    # for point in points:
    #     obs_rgb[point[1]:point[1]+5, point[0]:point[0]+5, 0] = 255
    #     obs_rgb[point[1]:point[1]+5, point[0]:point[0]+5, 1] = 0
    #     obs_rgb[point[1]:point[1]+5, point[0]:point[0]+5, 2] = 0
    a = np.array(points)
    # cv.drawContours(obs_rgb, [a], 0, (255, 0, 0), 2)
    for point1, point2 in zip(a, a[1:]):
        cv.line(obs_rgb, tuple(point1), tuple(point2), [255, 0, 0], 2)
    cv.imshow('hehe', obs_rgb)


env = gym.make('CarRacing-v0')
observation = env.reset()
# process_obs(observation)
render = 0
while render < 5000:
    render = render + 1
    env.render()
    action = [0, 0.01, 0]
    print("Render no: " + str(render))
    print("--------------------------------")
    observation, reward, done, info = env.step(action)
    if render > 50:
        thresh, start_point, goal_point = process_obs(observation)
        RRT = rrt.RRT(thresh, start_point, goal_point)
        path, graph = RRT.search()
        draw_path(path, graph)
        print("-----------------------------------------------------")
env.close()





