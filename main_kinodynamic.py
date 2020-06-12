import gym
import numpy as np
import cv2 as cv
import math
from PIL import Image
from new_kinodynamic_rrt import RRT
from skeletonize import skeletonize

def process_obs(obs):
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
    # cv.imwrite('thresh.png', obs_thresh)
    goal_point = skeletonize(obs_thresh.copy())
    # obs_thresh = cv.dilate(obs_thresh, np.zeros((5, 5), dtype=np.uint8), iterations=40)
    goal_state = (round(goal_point[1]), round(goal_point[0]), 0)
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
observation = env.reset()
render = 0
action = [0, 0.1, 0.0]
while render < 5000:
    render = render + 1
    print("Render no: " + str(render))
    env.render()
    if render < 50:
        observation, reward, done, info = env.step(action)
    if render >= 50:
        thresh, start_state, goal_state = process_obs(observation)
        rrt = RRT(thresh, start_state, goal_state, (action[1], action[0]))
        path, points, controls = rrt.search()
        if len(controls) > 1:
            print(len(controls))
            print(controls)
            action = [controls[1][1]/(np.pi/4), controls[1][0], 0]
            if action[1] > 0.5:
                action[1] = 0.3
            if action[0] > 0.5:
                action[1] = 0
                action[2] = 0.1
            if action[0] < -0.5:
                action[1] = 0
                action[2] = 0.1
        if len(controls) == 1:
            print(len(controls))
            print(controls)
            action = [controls[0][1]/(np.pi/4), controls[0][0], 0]
            if action[1] > 0.5:
                action[1] = 0.3
            if action[0] > 0.5:
                action[1] = 0
                action[2] = 0.1
            if action[0] < -0.5:
                action[1] = 0
                action[2] = 0.1
        if len(controls) == 0:
            action = [0, 0, 0.05]



        print(action)
        draw_points(path, goal_state, points)
        observation, reward, done, info = env.step(action)
        print("-----------------------------------------------------")
env.close()





