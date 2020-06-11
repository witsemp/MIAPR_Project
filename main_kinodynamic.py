import gym
import numpy as np
import cv2 as cv
import math
from PIL import Image
from new_kinodynamic_rrt import RRT
def process_obs(obs):
    goal_row = 0
    goal_col = 0
    goal_point = (goal_row, goal_col)
    wh_pixels_up = []
    wh_pixels_left = []
    wh_pixels_right = []
    start_point = (500, 600)
    start_state = (500, 600, 0)

    # create and save observation image
    img = Image.fromarray(obs, 'RGB')
    img.save('obs.png')

    # load observation image in greyscale
    obs_greyscale = cv.imread('obs.png', 0)

    # threshold image to create occupancy map
    ret, obs_thresh = cv.threshold(obs_greyscale, 127, 255, cv.THRESH_BINARY_INV)

    # crop the status bar
    obs_thresh = obs_thresh[0:84, 0:96]

    # resize to match game window size
    obs_thresh = cv.resize(obs_thresh, (1000, 700))
    cv.imwrite('thresh.png', obs_thresh)

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

    end_point = (goal_point[1], goal_point[0])
    goal_state = (goal_point[1], round(goal_point[0]), 0)
    wh_pixels_left.clear()
    wh_pixels_right.clear()
    wh_pixels_up.clear()
    print(goal_state)
    return obs_thresh, start_state, goal_state

def draw_path(points, graph):
    obs_rgb = cv.imread('obs.png', 1)
    obs_rgb = obs_rgb[0:84, 0:96, :]
    obs_rgb = cv.resize(obs_rgb, (1000, 700))
    for key, value in graph.items():
        obs_rgb[key[1]:key[1]+5, key[0]:key[0]+5, 0] = 0
        obs_rgb[key[1]:key[1]+5, key[0]:key[0]+5, 1] = 0
        obs_rgb[key[1]:key[1]+5, key[0]:key[0]+5, 2] = 255
    a = np.array(points)
    for point1, point2 in zip(a, a[1:]):
        cv.line(obs_rgb, tuple(point1), tuple(point2), [255, 0, 0], 2)
    cv.imshow('hehe', obs_rgb)
def draw_points(points, goal_state, all_points):
    obs_rgb = cv.imread('obs.png', 1)
    obs_rgb = obs_rgb[0:84, 0:96, :]
    obs_rgb = cv.resize(obs_rgb, (1000, 700))
    if len(points) > 2:
        a = np.array(points)
        for point1, point2 in zip(a, a[1:]):
            cv.line(obs_rgb, tuple(point1), tuple(point2), [255, 0, 0], 2)
        for point in all_points:
            obs_rgb[point[1] - 5:point[1] + 5, point[0] - 5:point[0] + 5] = (0, 255, 0)
    obs_rgb[goal_state[1]:goal_state[1]+5, goal_state[0]-5:goal_state[0]+5] = (0, 0, 255)
    cv.imshow('hehe', obs_rgb)

def draw_goal_state(goal_state):
    obs_rgb = cv.imread('obs.png', 1)
    obs_rgb = obs_rgb[0:84, 0:96, :]
    obs_rgb = cv.resize(obs_rgb, (1000, 700))
    obs_rgb[goal_state[1]:goal_state[1]+5, goal_state[0]-5:goal_state[0]+5] = (0, 0, 255)
    cv.imshow('hehe', obs_rgb)

env = gym.make('CarRacing-v0')
env.reset()
render = 0
while render < 5000:
    render = render + 1
    env.render()
    action = [0, 0.25, 0.0]
    print("Render no: " + str(render))
    observation, reward, done, info = env.step(action)
    if render >= 50:
        thresh, start_state, goal_state = process_obs(observation)
        rrt = RRT(thresh, start_state, goal_state, (action[1], action[0]))
        path, points = rrt.search()
        draw_points(path, goal_state, points)
        print("-----------------------------------------------------")
env.close()





