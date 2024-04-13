import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import numpy as np
import time
import ros_talker

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None

class Map:
    def __init__(self):
        self.x_range = (0, 30)
        self.y_range = (0, 20)
        self.map_boundary = self.map_boundary()
        self.rectangle = self.rectangle()

    def map_boundary(self):
        map_boundary = [
            [0, 0, 0.1, 20],
            [0, 20, 30, 0.1],
            [0.1, 0, 30, 0.1],
            [30, 0.1, 0.1, 20]
        ]
        return map_boundary

    def rectangle(self):
        # Map-1
        rectangle = [
            [7, 14.75, 1.5, 1.5],
            [7, 9.25, 1.5, 1.5],
            [7, 3.75, 1.5, 1.5],
            [14.5, 18.5, 1.5, 1.5],
            [14.5, 12.5, 1.5, 1.5],
            [14.5, 6, 1.5, 1.5],
            [14.5, 0, 1.5, 1.5],
            [22, 14.75, 1.5, 1.5],
            [22, 9.25, 1.5, 1.5],
            [22, 3.75, 1.5, 1.5]
        ]

        # Map -2
        # rectangle = [
        #     [7, 14.75, 1.5, 1.5],
        #     [7, 9.25, 1.5, 1.5],
        #     [7, 3.75, 1.5, 1.5],
        #     [14.5, 18.5, 1.5, 1.5],
        #     [14.5, 12.5, 1.5, 1.5],
        #     [14.5, 6, 1.5, 1.5],
        #     [14.5, 0, 1.5, 1.5],
        #     [22, 14.75, 1.5, 1.5],
        #     [22, 9.25, 1.5, 1.5],
        #     [22, 3.75, 1.5, 1.5],
        #     [5, 16.75, 1.5, 1.5],
        #     [5, 11.25, 1.5, 1.5],
        #     [5, 5.75, 1.5, 1.5],
        #     [12.5, 17, 1.5, 1.5],
        #     [12.5, 10.5, 1.5, 1.5],
        #     [12.5, 4, 1.5, 1.5],
        #     [20, 16.75, 1.5, 1.5],
        #     [20, 11.25, 1.5, 1.5],
        #     [20, 5.75, 1.5, 1.5],
        #     [26, 12, 1.5, 1.5],
        #     [26, 16.5, 1.5, 1.5],
        #     [26, 7.5, 1.5, 1.5],
        #     [18, 3.25, 1.5, 1.5],
        #     [18, 8.75, 1.5, 1.5],
        #     [18, 14.25, 1.5, 1.5],
        #     [12.5, 1.5, 1.5, 1.5],
        #     [5, 2, 1.5, 1.5],
        #     [24, 18, 1.5, 1.5],
        #     [8, 18, 1.5, 1.5],
        #     [4, 18.5, 1.5, 1.5],
        #     [4, 12.5, 1.5, 1.5],
        #     [4, 6, 1.5, 1.5],
        #     [4, 0, 1.5, 1.5],
        #     [11.5, 14.75, 1.5, 1.5],
        #     [11.5, 9.25, 1.5, 1.5],
        #     [11.5, 3.75, 1.5, 1.5],
        #     [19, 18.5, 1.5, 1.5],
        #     [19, 12.5, 1.5, 1.5],
        #     [19, 6, 1.5, 1.5]
        # ]
        return rectangle


class Plotting:
    def __init__(self, x_start, x_goal):
        self.xS = x_start
        self.xG = x_goal
        self.map = Map()
        self.map_bound = self.map.map_boundary
        self.rectangle = self.map.rectangle

    def animation(self, nodelist, path, name, animation=False):
        self.grid_plot(name)
        self.visited_plot(nodelist, animation)
        self.plot_path(path)

    def animation_connect(self, V1, V2, path, name):
        self.grid_plot(name)
        self.plot_visited_connect(V1, V2)
        self.plot_path(path)

    def grid_plot(self, name):
        plt.style.use('dark_background')
        fig, ax = plt.subplots()

        for rect in self.map_bound:
            ax.add_patch(
                patches.Rectangle(
                    rect[:2], rect[2], rect[3],
                    edgecolor='white',
                    facecolor='white',
                    fill=True
                )
            )

        for rect in self.rectangle:
            ax.add_patch(
                patches.Rectangle(
                    rect[:2], rect[2], rect[3],
                    edgecolor='black',
                    facecolor='white',
                    fill=True
                )
            )

        plt.plot(self.xS[0], self.xS[1], "bs", linewidth=3)
        plt.plot(self.xG[0], self.xG[1], "gs", linewidth=3)

        plt.title(name)
        plt.axis("equal")

    def visited_plot(self, nodelist, animation):
        if animation:
            count = 0
            for i, node in enumerate(nodelist):
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event: exit(0) if event.key == 'escape' else None)
                    if count % 10 == 0:
                        plt.pause(0.001)
        else:
            for node in nodelist:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    def plot_visited_connect(self, V1, V2):
        len1, len2 = len(V1), len(V2)

        for k in range(max(len1, len2)):
            if k < len1 and V1[k].parent:
                plt.plot([V1[k].x, V1[k].parent.x], [V1[k].y, V1[k].parent.y], "-g")
            if k < len2 and V2[k].parent:
                plt.plot([V2[k].x, V2[k].parent.x], [V2[k].y, V2[k].parent.y], "-g")

            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: exit(0) if event.key == 'escape' else None)

            if k % 2 == 0:
                plt.pause(0.001)

        plt.pause(0.01)

    def plot_path(self, path):
        if path:
            plt.plot([x[0] for x in path], [x[1] for x in path], '-r', linewidth=2)
            plt.pause(0.01)
        plt.show()

class Helpers:
    def __init__(self):
        self.map = Map()
        self.delta = 0.5
        self.rectangle = self.map.rectangle
        self.map_boundary = self.map.map_boundary

    def update_obstacles(self, obs_bound, obs_rec):
        self.map_boundary = obs_bound
        self.rectangle = obs_rec

    def get_obstacle_vertex(self):
        delta = self.delta
        obstacle_list = []

        for rect in self.rectangle:
            x, y, w, h = rect
            vertex_list = [[x - delta, y - delta], [x + w + delta, y - delta],
                           [x + w + delta, y + h + delta], [x - delta, y + h + delta]]
            obstacle_list.append(vertex_list)

        return obstacle_list

    def is_intersecting_rectangle(self, start, end, o, d, a, b):
        v1 = np.subtract(o, a)
        v2 = np.subtract(b, a)
        v3 = np.array([-d[1], d[0]])

        div = np.dot(v2, v3)

        if div == 0:
            return False

        t1 = np.linalg.norm(np.cross(v2, v1)) / div
        t2 = np.dot(v1, v3) / div

        if 0 <= t1 and 0 <= t2 <= 1:
            shot = Node((o[0] + t1 * d[0], o[1] + t1 * d[1]))
            dist_obs = self.get_distance(start, shot)
            dist_seg = self.get_distance(start, end)
            if dist_obs <= dist_seg:
                return True

        return False

    def is_collision(self, start, end):
        if self.is_in_obstacle(start) or self.is_in_obstacle(end):
            return True

        o, d = self.get_ray(start, end)
        obs_vertex = self.get_obstacle_vertex()

        for vertices in obs_vertex:
            for i in range(len(vertices)):
                if i == len(vertices) - 1:
                    a, b = vertices[i], vertices[0]
                else:
                    a, b = vertices[i], vertices[i+1]
                if self.is_intersecting_rectangle(start, end, o, d, a, b):
                    return True

        return False

    def is_in_obstacle(self, node):
        delta = self.delta

        for rect in self.rectangle + self.map_boundary:
            x, y, w, h = rect
            if 0 <= node.x - (x - delta) <= w + 2 * delta \
                    and 0 <= node.y - (y - delta) <= h + 2 * delta:
                return True

        return False

    def get_ray(self, start, end):
        orig = [start.x, start.y]
        direc = [end.x - start.x, end.y - start.y]
        return orig, direc

    def get_distance(self, start, end):
        return math.hypot(end.x - start.x, end.y - start.y)

class RRT:
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, maximum_iterations):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.maximum_iterations = maximum_iterations
        self.vertex = [self.s_start]

        self.map = Map()
        self.plotting = Plotting(s_start, s_goal)
        self.helpers = Helpers()

        self.x_range = self.map.x_range
        self.y_range = self.map.y_range
        self.rectangle = self.map.rectangle
        self.map_boundary = self.map.map_boundary

    def planning(self):
        iter_count = 0
        node_count = 1  # start node
        for i in range(self.maximum_iterations):
            iter_count += 1
            q_rand = self.generate_random_node(self.goal_sample_rate)
            q_near = self.nearest_neighbor(self.vertex, q_rand)
            q_new = self.new_state(q_near, q_rand)

            if q_new and not self.helpers.is_collision(q_near, q_new):
                self.vertex.append(q_new)
                node_count += 1
                dist, _ = self.get_distance_and_angle(q_new, self.s_goal)

                if dist <= self.step_len and not self.helpers.is_collision(q_new, self.s_goal):
                    self.new_state(q_new, self.s_goal)
                    return self.extract_path(q_new), iter_count, node_count

        return None

    def generate_random_node(self, goal_sample_rate):
        delta = self.helpers.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal

    def nearest_neighbor(self, node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        q_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        q_new.parent = node_start

        return q_new

    def extract_path(self, node_end):
        path = [(self.s_goal.x, self.s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))
        return path

    def get_distance_and_angle(self, node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

#Start and Goal Nodes
x_start = (2, 13)
x_goal = (23, 17)

start_time = time.time()
rrt = RRT(x_start, x_goal, 0.5, 0.05, 10000)
path, iterations, nodes = rrt.planning()
end_time = time.time()
path_reversed = path[::-1]
print(path_reversed)
print(f"Total time taken: {end_time-start_time}")
print(f"Total iterations taken: {iterations}")
print(f"Total Nodes Visited: {nodes}")
print(f"Total Nodes in Optimised Path: {len(path)}")

if path:
    rrt.plotting.animation(rrt.vertex, path, "RRT", True)
    ros_talker.send_vel(path_reversed)
else:
    print("No Path Found!")
