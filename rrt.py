import numpy as np
import math
np.random.seed(444)
class RRT():
    def __init__(self, map, start_point, goal_point):
        self.step = 5
        self.map = map
        self.width = np.shape(map)[1]
        self.height = np.shape(map)[0]
        self.start = start_point
        self.goal = goal_point
        self.parent = {}
        self.parent[self.start] = None

    def random_point(self):
        x = int(self.width * np.random.random_sample())
        y = int(self.height * np.random.random_sample())
        return np.array([x, y])

    def find_closest(self, pos):
        value_min = math.sqrt(self.width ** 2 + self.height ** 2)
        for key in self.parent.keys():
            value = math.sqrt((pos[0] - key[0]) ** 2 + (pos[1] - key[1]) ** 2)
            if value < value_min:
                closest = key
                value_min = value
        return closest

    def new_pt(self, pt, closest):

        a = (pt[1] - closest[1]) / (pt[0] - closest[0])
        b = closest[1] - a * closest[0]
        eukl = math.sqrt((pt[0] - closest[0]) ** 2 + (pt[1] - closest[1]) ** 2)

        cos = abs(pt[0] - closest[0])/eukl

        if closest[0] != pt[0]:
            if closest[0] > pt[0]:
                x = closest[0] - cos * self.step

            elif closest[0] < pt[0]:
                x = closest[0] + cos * self.step

            y = a * x + b

        else:
            x = closest[0]
            if closest[1] < pt[1]:
                y = closest[1] + self.step
            else:
                y = closest[1] - self.step

        pt[0] = int(x)
        pt[1] = int(y)

        return pt

    def check_if_valid(self, a, b):
        in_free_space = True
        div_x = 1
        div_y = 1
        if (a[0] != b[0]):
            a_v = (a[1]-b[1])/(a[0]-b[0])
            b_v = a[1] - a_v * a[0]
            step_x = np.abs((b[0] - a[0]) / 10)
            x = min(a[0], b[0])
            for i in range(1, 10):
                y = a_v * x + b_v
                x_int = int(x * div_x)
                y_int = int(y * div_y)

                if self.map[y_int, x_int] == 0:
                    in_free_space = False
                    break
                x = x + step_x
        else:
            step_y = np.abs((b[1] - a[1]) / 10)
            y = min(a[1], b[1])
            for i in range(1, 10):
                if self.map[int(y*div_y), int(a[0] * div_x)] == 0:
                    in_free_space = False
                    break
                y = y + step_y
        return in_free_space

    def search(self):
        path = []
        new_path = []
        endReached = False
        while (endReached != True):
            newPoint = self.random_point()
            closestNeigh = self.find_closest(newPoint)
            newPointSegment = self.new_pt(newPoint, closestNeigh)
            if self.check_if_valid(newPointSegment, closestNeigh):
                self.parent.update({tuple(newPointSegment) : tuple(closestNeigh)})
            # else: print('Connection not found')

            if self.check_if_valid(newPointSegment, self.goal):
                self.parent.update({self.goal : newPointSegment})
                considered_node = self.goal
                start_reached = False
                path.append(considered_node)
                while not start_reached:
                    if considered_node in self.parent.keys():
                        considered_node = tuple(self.parent[considered_node])
                        path.append(considered_node)
                        if considered_node[0] == self.start[0] and considered_node[1] == self.start[1]:
                            start_reached = True
                    else: continue

                endReached = True
                for object in path:
                    object = list(object)
                    object[0] = int(object[0])
                    object[1] = int(object[1])
                    object = tuple(object)
                    new_path.append(object)
                print("Path found")
                print("New path: " + str(new_path))
            # else:
            #     print("Connection not found")
        return new_path, self.parent
