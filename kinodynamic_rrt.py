import numpy as np
import math
np.random.seed(444)
class RRT():
    def __init__(self, map, start_state, goal_state, initial_control):
        self.map = map
        self.height = np.shape(map)[0]
        self.width = np.shape(map)[1]
        self.start_state = start_state
        self.goal_state = goal_state
        self.vertices = {self.start_state: {'Parent' : None, 'Control': initial_control}}
        self.delta_t = 10
        self.T = 1000
        self.L = 50
    def check_if_valid(self, pos):
        if self.map[pos[1], pos[0]] == 0:
            return False
        else:
            return True

    def random_state(self):
        x = int(self.width * np.random.random_sample())
        y = int(self.height * np.random.random_sample())
        theta = (2 * np.random.random_sample() - 1)*np.pi/4
        random_state = (x, y, theta)
        return random_state

    def find_closest_state(self, pos):
        distance_value_min = math.sqrt(self.width ** 2 + self.height ** 2)
        angle_value_min = 2
        value_min = distance_value_min + angle_value_min
        for key in self.vertices.keys():
            distance_value = math.sqrt((pos[0] - key[0]) ** 2 + (pos[1] - key[1]) ** 2)
            angle_value = abs(pos[2] - key[2])
            value = angle_value + distance_value
            if value < value_min:
                closest = key
                value_min = value
        return closest

    def random_control(self):
        v_lin = np.random.random_sample()
        turn_angle = (2 * np.random.random_sample() - 1)*np.pi/4
        return (v_lin, turn_angle)



    def new_state(self, v_lin, turn_angle, current_state):
        x = v_lin*np.cos(current_state[2])*self.delta_t+current_state[0]
        y = v_lin * np.sin(current_state[2]) * self.delta_t + current_state[1]
        theta = (v_lin / self.L) * np.tan(turn_angle) * self.delta_t + current_state[2]
        new_state = (int(x), int(y), theta)
        print("Current state", current_state)
        print("New state", new_state)
        print("Control", v_lin, turn_angle)
        print('--------------------------------')
        return new_state

    def search(self):
        endReached = False
        path = []
        actions = []
        while(endReached != True):
            x_rand = self.random_state()
            if not self.check_if_valid(x_rand):
                continue
            else:
                x_near = self.find_closest_state(x_rand)
                u = self.random_control()
                new_state = self.new_state(u[0], u[1], x_near)
                if not self.check_if_valid(new_state):
                    continue
                else:
                    self.vertices[new_state] = {}
                    self.vertices[new_state]['Parent'] = x_near
                    self.vertices[new_state]['Control'] = u
                #print(self.vertices)
                if self.goal_state[0] + 300 > new_state[0] > self.goal_state[0] - 200 and self.goal_state[1] + 200 > new_state[1] > self.goal_state[1] - 200:
                    self.vertices[self.goal_state] = {}
                    self.vertices[self.goal_state]['Parent'] = new_state
                    self.vertices[self.goal_state]['Control'] = u
                    considered_node = self.goal_state
                    path.append(considered_node)
                    actions.append(u)
                    startReached = False
                    while not startReached:
                        considered_node = self.vertices[considered_node]['Parent']
                        considered_action = self.vertices[considered_node]['Control']
                        path.append(considered_node)
                        actions.append(considered_action)
                        if considered_node[0] == self.start_state[0] and considered_node[1] == self.start_state[1]:
                            startReached = True
                    endReached = True
        actions.reverse()
        print("Path", path)
        print("Actions", actions)















