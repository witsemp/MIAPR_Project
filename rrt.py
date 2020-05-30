import numpy as np
import math
class RRT():
    def __init__(self, map, start_point, goal_point):
        self.step = 10
        self.map = map
        self.width = np.shape(map)[1]
        self.height = np.shape(map)[0]
        self.start = start_point
        self.goal = goal_point
        self.parent = {}
        self.parent[self.start] = None

    def random_point(self):
        """
        Draws random point in 2D
        :return: point in 2D
        """

        x = int(self.width * np.random.random_sample())
        y = int(self.height * np.random.random_sample())
        print(np.array([x, y]))
        return np.array([x, y])

    def find_closest(self, pos):
        """
        Finds the closest vertex in the graph to the pos argument

        :param pos: point id 2D
        :return: vertex from graph in 2D closest to the pos
        """
        value_min = math.sqrt(self.width ** 2 + self.height ** 2)
        for key in self.parent.keys():
            value = math.sqrt((pos[0] - key[0]) ** 2 + (pos[1] - key[1]) ** 2)
            if value < value_min:
                closest = key
                value_min = value
        return closest

    def new_pt(self, pt, closest):
        """
        Finds the point on the segment connecting closest with pt, which lies self.step from the closest (vertex in graph)
        :param pt: point in 2D
        :param closest: vertex in the tree (point in 2D)
        :return: point in 2D
        """
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

        pt[0] = x
        pt[1] = y

        return pt

    def check_if_valid(self, a, b):
        """
        Checks if the segment connecting a and b lies in the free space.
        :param a: point in 2D
        :param b: point in 2D
        :return: boolean
        """
        in_free_space = True
        div_x = 1
        div_y = 1
        if (a[0] != b[0]):
            a_v = (a[1]-b[1])/(a[0]-b[0])
            b_v = a[1] - a_v * a[0]
            step_x = np.abs((b[0] - a[0]) / 100)
            x = min(a[0], b[0])
            for i in range(1, 100):
                y = a_v * x + b_v
                x_int = int(x * div_x)
                y_int = int(y * div_y)

                if self.map[y_int, x_int, 0] < 255 and self.map[y_int, x_int, 1] < 255 and self.map[y_int, x_int, 2] < 255:
                    in_free_space = False
                    break
                x = x + step_x
        else:
            step_y = np.abs((b[1] - a[1]) / 100)
            y = min(a[1], b[1])
            for i in range(1, 100):
                if self.map[int(a[0] * div_x), int(y*div_y), 0] < 255 and self.map[int(a[0] * div_x), int(y*div_y), 1] < 255 and self.map[int(a[0] * div_x), int(y*div_y), 2] < 255 :
                    in_free_space = False
                    break
                y = y + step_y
        return in_free_space

    def search(self):
        """
        RRT search algorithm for start point self.start and desired state self.end.
        Saves the search tree in the self.parent dictionary, with key value pairs representing segments
        (key is the child vertex, and value is its parent vertex).
        Uses self.publish_search() and self.publish_path(path) to publish the search tree and the final path respectively.
        """
        path = []
        new_path = []
        endReached = False
        while (endReached != True):
            newPoint = self.random_point()
            closestNeigh = self.find_closest(newPoint)
            newPointSegment = self.new_pt(newPoint, closestNeigh)
            if self.check_if_valid(newPointSegment, closestNeigh):
                self.parent.update({tuple(newPointSegment) : tuple(closestNeigh)})
            else: print('Connection not found')

            if self.check_if_valid(newPointSegment, self.goal):
                self.parent.update( {self.goal : newPointSegment} )
                considered_node = self.goal
                start_reached = False
                path.append(considered_node)
                while not start_reached:
                    considered_node = tuple(self.parent[considered_node])
                    path.append(considered_node)
                    if considered_node[0] == self.start[0] and considered_node[1] == self.start[1]:
                        start_reached = True

                endReached = True
                for object in path:
                    object = list(object)
                    object[0] = int(object[0])
                    object[1] = int(object[1])
                    object = tuple(object)
                    new_path.append(object)
                print("Path found")
                print(new_path)
            else:
                print("Connection not found")
