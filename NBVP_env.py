# from scipy import spatial
# from skimage import io
# import numpy as np
# import time
# import sys
# from scipy import ndimage
# import matplotlib.pyplot as plt
# import os
# import copy
# from skimage.measure import block_reduce

# from sensor import *
# from parameter import *


# class Env():
#     def __init__(self, map_index, plot=False, test=False):
#         self.test = test
#         if self.test:
#             self.map_dir = f'DungeonMaps/test'
#         else:
#             self.map_dir = f'DungeonMaps/train'
#         self.map_list = os.listdir(self.map_dir)
#         self.map_list.sort(reverse=True)
#         self.map_index = map_index % np.size(self.map_list)
#         self.ground_truth, self.robot_position = self.import_ground_truth(self.map_dir + '/' + self.map_list[self.map_index])
#         self.ground_truth_size = np.shape(self.ground_truth) # (480, 640)
#         self.robot_belief = np.ones(self.ground_truth_size) * 127 # unexplored 127
        
#         self.finish_percent = 0.985
#         self.resolution = 4
#         self.sensor_range = 80
#         self.old_robot_belief = copy.deepcopy(self.robot_belief)

#         self.plot = plot
#         self.frame_files = []
#         if self.plot:
#             # initialize the route
#             self.xPoints = [self.robot_position[0]]
#             self.yPoints = [self.robot_position[1]]

#         self.travel_dist = 0
#         self.explored_rate = 0
#         self.route_node = [self.robot_position]
#         self.frontiers = None
#         self.downsampled_belief = None

#     def begin(self):
#         self.robot_belief = self.update_robot_belief(self.robot_position, self.sensor_range, self.robot_belief, self.ground_truth)
#         self.downsampled_belief = block_reduce(self.robot_belief.copy(), block_size=(self.resolution, self.resolution), func=np.min)

#         self.frontiers = self.find_frontier()
        
#         self.old_robot_belief = copy.deepcopy(self.robot_belief)


#     def step(self, next_node_coords):
#         dist = np.linalg.norm(next_node_coords - self.robot_position)

#         self.travel_dist += dist
#         self.robot_position = next_node_coords
#         self.route_node.append(self.robot_position)
#         self.robot_belief = self.update_robot_belief(self.robot_position, self.sensor_range, self.robot_belief,
#                                                      self.ground_truth)
#         self.downsampled_belief = block_reduce(self.robot_belief.copy(), block_size=(self.resolution, self.resolution),
#                                                func=np.min)

#         frontiers = self.find_frontier()
#         self.explored_rate = self.evaluate_exploration_rate()

#         if self.plot:
#             self.xPoints.append(self.robot_position[0])
#             self.yPoints.append(self.robot_position[1])

#         self.frontiers = frontiers
#         #self.frontier_centers = self.get_frontier_centers()

#         done = self.check_done()

#         return done

#     def import_ground_truth(self, map_index):
#         # occupied 1, free 255, unexplored 127

#         ground_truth = (io.imread(map_index, 1) * 255).astype(int)
#         robot_location = np.nonzero(ground_truth == 208)
#         robot_location = np.array([np.array(robot_location)[1, 127], np.array(robot_location)[0, 127]])
#         ground_truth = (ground_truth > 150)
#         ground_truth = ground_truth * 254 + 1
#         return ground_truth, robot_location

#     def free_cells(self):
#         index = np.where(self.ground_truth == 255)
#         free = np.asarray([index[1], index[0]]).T
#         return free

#     def update_robot_belief(self, robot_position, sensor_range, robot_belief, ground_truth):
#         robot_belief = sensor_work(robot_position, sensor_range, robot_belief, ground_truth)
#         return robot_belief

#     def check_done(self):
#         done = False
#         #if self.node_utility.sum() == 0:
#         #if self.explored_rate >= 0.99 or self.node_utility.sum() == 0:
#         if np.sum(self.ground_truth == 255) - np.sum(self.robot_belief == 255) <= 250:
#             done = True
#         return done

#     def evaluate_exploration_rate(self):
#         rate = np.sum(self.robot_belief == 255) / np.sum(self.ground_truth == 255)
#         return rate

#     def calculate_new_free_area(self):
#         old_free_area = self.old_robot_belief == 255
#         current_free_area = self.robot_belief == 255

#         new_free_area = (current_free_area.astype(np.int) - old_free_area.astype(np.int)) * 255

#         return new_free_area, np.sum(old_free_area)

#     def calculate_utility_along_path(self, path, nodes_list):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

#         return unique_frontiers.shape[0]

#     def calculate_node_gain_over_path(self, node_index, path, nodes_list):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         pre_unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)
#         observable_frontiers += nodes_list[node_index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

#         return unique_frontiers.shape[0] - pre_unique_frontiers.shape[0]

#     def calculate_dist_path(self, path, node_list):
#         dist = 0
#         start = path[0]
#         end = path[-1]
#         for index in path:
#             if index == end:
#                 break
#             dist += np.linalg.norm(node_list[start].coords - node_list[index].coords)
#             start = index
#         return dist

#     def find_frontier(self):
#         y_len = self.downsampled_belief.shape[0]
#         x_len = self.downsampled_belief.shape[1]
#         mapping = self.downsampled_belief.copy()
#         belief = self.downsampled_belief.copy()
#         # 0-1 unknown area map
#         mapping = (mapping == 127) * 1
#         mapping = np.lib.pad(mapping, ((1, 1), (1, 1)), 'constant', constant_values=0)
#         fro_map = mapping[2:][:, 1:x_len + 1] + mapping[:y_len][:, 1:x_len + 1] + mapping[1:y_len + 1][:, 2:] + \
#                   mapping[1:y_len + 1][:, :x_len] + mapping[:y_len][:, 2:] + mapping[2:][:, :x_len] + mapping[2:][:,
#                                                                                                       2:] + \
#                   mapping[:y_len][:, :x_len]
#         ind_free = np.where(belief.ravel(order='F') == 255)[0]
#         ind_fron_1 = np.where(1 < fro_map.ravel(order='F'))[0]
#         ind_fron_2 = np.where(fro_map.ravel(order='F') < 8)[0]
#         ind_fron = np.intersect1d(ind_fron_1, ind_fron_2)
#         ind_to = np.intersect1d(ind_free, ind_fron)

#         map_x = x_len
#         map_y = y_len
#         x = np.linspace(0, map_x - 1, map_x)
#         y = np.linspace(0, map_y - 1, map_y)
#         t1, t2 = np.meshgrid(x, y)
#         points = np.vstack([t1.T.ravel(), t2.T.ravel()]).T

#         f = points[ind_to]
#         f = f.astype(int)

#         f = f * self.resolution

#         return f


#     def plot_env(self, n, path, step, planned_route=None):
#         plt.switch_backend('agg')
#         # plt.ion()
#         plt.cla()
#         plt.imshow(self.robot_belief, cmap='gray')
#         plt.axis((0, self.ground_truth_size[1], self.ground_truth_size[0], 0))
#         if planned_route:
#             for p in planned_route:
#                 planned_x, planned_y = [], []
#                 for coords in p:
#                     planned_x.append(coords[0])
#                     planned_y.append(coords[1])
#                 plt.plot(planned_x, planned_y, c='r', linewidth=2, zorder=2)
#         plt.plot(self.xPoints, self.yPoints, 'b', linewidth=2)
#         plt.plot(self.robot_position[0], self.robot_position[1], 'mo', markersize=8)
#         plt.plot(self.xPoints[0], self.yPoints[0], 'co', markersize=8)
#         plt.scatter(self.frontiers[:, 0], self.frontiers[:, 1], c='r', s=2, zorder=3)
#         # plt.pause(0.1)
#         plt.suptitle('Explored ratio: {:.4g}  Travel distance: {:.4g}'.format(self.explored_rate, self.travel_dist))
#         plt.tight_layout()
#         plt.savefig('{}/{}_{}_samples.png'.format(path, n, step, dpi=150))
#         # plt.show()
#         frame = '{}/{}_{}_samples.png'.format(path, n, step)
#         self.frame_files.append(frame)


# from scipy import spatial
# from skimage import io
# import numpy as np
# import time
# import sys
# from scipy import ndimage
# import matplotlib.pyplot as plt
# import os
# import copy
# from skimage.measure import block_reduce

# from sensor import *
# from parameter import *


# class Env():
#     def __init__(self, map_index, plot=False, test=False):
#         self.test = test
#         if self.test:
#             self.map_dir = f'DungeonMaps/test'
#         else:
#             self.map_dir = f'DungeonMaps/train'
#         self.map_list = os.listdir(self.map_dir)
#         self.map_list.sort(reverse=True)
#         self.map_index = map_index % np.size(self.map_list)
#         self.ground_truth, self.robot_position = self.import_ground_truth(self.map_dir + '/' + self.map_list[self.map_index])
#         self.ground_truth_size = np.shape(self.ground_truth) # (480, 640)
#         self.robot_belief = np.ones(self.ground_truth_size) * 127 # unexplored 127
        
#         self.finish_percent = 0.985
#         self.resolution = 4
#         self.sensor_range = 80
#         self.old_robot_belief = copy.deepcopy(self.robot_belief)

#         self.plot = plot
#         self.frame_files = []
#         if self.plot:
#             # initialize the route
#             self.xPoints = [self.robot_position[0]]
#             self.yPoints = [self.robot_position[1]]

#         self.travel_dist = 0
#         self.explored_rate = 0
#         self.route_node = [self.robot_position]
#         self.frontiers = None
#         self.downsampled_belief = None

#         # FoV settings
#         self.alpha_h = np.deg2rad(60)  # Horizontal FoV in radians
#         self.alpha_v = np.deg2rad(45)  # Vertical FoV in radians

#     def begin(self):
#         self.robot_belief = self.update_robot_belief(self.robot_position, self.sensor_range, self.robot_belief, self.ground_truth)
#         self.downsampled_belief = block_reduce(self.robot_belief.copy(), block_size=(self.resolution, self.resolution), func=np.min)
#         self.frontiers = self.find_frontier()
#         self.old_robot_belief = copy.deepcopy(self.robot_belief)

#     def step(self, next_node_coords):
#         dist = np.linalg.norm(next_node_coords - self.robot_position)
#         self.travel_dist += dist
#         self.robot_position = next_node_coords
#         self.route_node.append(self.robot_position)
#         self.robot_belief = self.update_robot_belief(self.robot_position, self.sensor_range, self.robot_belief,
#                                                      self.ground_truth)
#         self.downsampled_belief = block_reduce(self.robot_belief.copy(), block_size=(self.resolution, self.resolution),
#                                                func=np.min)
#         frontiers = self.find_frontier()
#         self.explored_rate = self.evaluate_exploration_rate()

#         if self.plot:
#             self.xPoints.append(self.robot_position[0])
#             self.yPoints.append(self.robot_position[1])

#         self.frontiers = frontiers
#         done = self.check_done()
#         return done

#     def import_ground_truth(self, map_index):
#         ground_truth = (io.imread(map_index, 1) * 255).astype(int)
#         robot_location = np.nonzero(ground_truth == 208)
#         robot_location = np.array([np.array(robot_location)[1, 127], np.array(robot_location)[0, 127]])
#         ground_truth = (ground_truth > 150)
#         ground_truth = ground_truth * 254 + 1
#         return ground_truth, robot_location

#     def free_cells(self):
#         index = np.where(self.ground_truth == 255)
#         free = np.asarray([index[1], index[0]]).T
#         return free

#     def update_robot_belief(self, robot_position, sensor_range, robot_belief, ground_truth):
#         # Implement sensor model with FoV constraints
#         robot_belief = sensor_work(robot_position, sensor_range, robot_belief, ground_truth, self.alpha_h)
#         return robot_belief

#     def check_done(self):
#         done = False
#         if np.sum(self.ground_truth == 255) - np.sum(self.robot_belief == 255) <= 250:
#             done = True
#         return done

#     def evaluate_exploration_rate(self):
#         rate = np.sum(self.robot_belief == 255) / np.sum(self.ground_truth == 255)
#         return rate

#     def calculate_new_free_area(self):
#         old_free_area = self.old_robot_belief == 255
#         current_free_area = self.robot_belief == 255

#         new_free_area = (current_free_area.astype(np.int) - old_free_area.astype(np.int)) * 255
#         return new_free_area, np.sum(old_free_area)

#     def calculate_utility_along_path(self, path, nodes_list):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)
#         return unique_frontiers.shape[0]

#     def calculate_node_gain_over_path(self, node_index, path, nodes_list):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         pre_unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)
#         observable_frontiers += nodes_list[node_index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)
#         return unique_frontiers.shape[0] - pre_unique_frontiers.shape[0]

#     def calculate_dist_path(self, path, node_list):
#         dist = 0
#         start = path[0]
#         end = path[-1]
#         for index in path:
#             if index == end:
#                 break
#             dist += np.linalg.norm(node_list[start].coords - node_list[index].coords)
#             start = index
#         return dist

#     def find_frontier(self):
#         y_len = self.downsampled_belief.shape[0]
#         x_len = self.downsampled_belief.shape[1]
#         mapping = self.downsampled_belief.copy()
#         belief = self.downsampled_belief.copy()
#         # 0-1 unknown area map
#         mapping = (mapping == 127) * 1
#         mapping = np.lib.pad(mapping, ((1, 1), (1, 1)), 'constant', constant_values=0)
#         fro_map = mapping[2:][:, 1:x_len + 1] + mapping[:y_len][:, 1:x_len + 1] + mapping[1:y_len + 1][:, 2:] + \
#                   mapping[1:y_len + 1][:, :x_len] + mapping[:y_len][:, 2:] + mapping[2:][:, :x_len] + mapping[2:][:,
#                                                                                                       2:] + \
#                   mapping[:y_len][:, :x_len]
#         ind_free = np.where(belief.ravel(order='F') == 255)[0]
#         ind_fron_1 = np.where(1 < fro_map.ravel(order='F'))[0]
#         ind_fron_2 = np.where(fro_map.ravel(order='F') < 8)[0]
#         ind_fron = np.intersect1d(ind_fron_1, ind_fron_2)
#         ind_to = np.intersect1d(ind_free, ind_fron)

#         map_x = x_len
#         map_y = y_len
#         x = np.linspace(0, map_x - 1, map_x)
#         y = np.linspace(0, map_y - 1, map_y)
#         t1, t2 = np.meshgrid(x, y)
#         points = np.vstack([t1.T.ravel(), t2.T.ravel()]).T

#         f = points[ind_to]
#         f = f.astype(int)

#         # Filter frontiers based on FoV
#         f = f * self.resolution
#         f = self.filter_frontiers_by_fov(f, self.robot_position)

#         return f

#     def filter_frontiers_by_fov(self, frontiers, robot_position):
#         # Convert the frontiers into vectors relative to the robot's position
#         relative_vectors = frontiers - robot_position
#         distances = np.linalg.norm(relative_vectors, axis=1)

#         # Normalize vectors
#         normalized_vectors = relative_vectors / distances[:, None]

#         # FoV constraints
#         horizontal_angle = np.arctan2(normalized_vectors[:, 1], normalized_vectors[:, 0])
#         within_fov = (np.abs(horizontal_angle) <= self.alpha_h / 2)

#         return frontiers[within_fov]

#     def plot_env(self, n, path, step, planned_route=None):
#         plt.switch_backend('agg')
#         plt.cla()
#         plt.imshow(self.robot_belief, cmap='gray')
#         plt.axis((0, self.ground_truth_size[1], self.ground_truth_size[0], 0))
#         if planned_route:
#             for p in planned_route:
#                 planned_x, planned_y = [], []
#                 for coords in p:
#                     planned_x.append(coords[0])
#                     planned_y.append(coords[1])
#                 plt.plot(planned_x, planned_y, c='r', linewidth=2, zorder=2)
#         plt.plot(self.xPoints, self.yPoints, 'b', linewidth=2)
#         plt.plot(self.robot_position[0], self.robot_position[1], 'mo', markersize=8)
#         plt.plot(self.xPoints[0], self.yPoints[0], 'co', markersize=8)
#         plt.scatter(self.frontiers[:, 0], self.frontiers[:, 1], c='r', s=2, zorder=3)
#         plt.suptitle('Explored ratio: {:.4g}  Travel distance: {:.4g}'.format(self.explored_rate, self.travel_dist))
#         plt.tight_layout()
#         plt.savefig('{}/{}_{}_samples.png'.format(path, n, step, dpi=150))
#         frame = '{}/{}_{}_samples.png'.format(path, n, step)
#         self.frame_files.append(frame)


# from scipy import spatial
# from skimage import io
# import numpy as np
# import time
# import sys
# from scipy import ndimage
# import matplotlib.pyplot as plt
# import os
# import copy
# from skimage.measure import block_reduce

# from sensor import *
# from parameter import *


# class Env():
#     def __init__(self, map_index, plot=False, test=False):
#         self.test = test
#         if self.test:
#             self.map_dir = f'DungeonMaps/test'
#         else:
#             self.map_dir = f'DungeonMaps/train'
#         self.map_list = os.listdir(self.map_dir)
#         self.map_list.sort(reverse=True)
#         self.map_index = map_index % np.size(self.map_list)
#         self.ground_truth, self.robot_position = self.import_ground_truth(self.map_dir + '/' + self.map_list[self.map_index])
#         self.ground_truth_size = np.shape(self.ground_truth) # (480, 640)
#         self.robot_belief = np.ones(self.ground_truth_size) * 127 # unexplored 127

#         self.finish_percent = 0.985
#         self.resolution = 4
#         self.sensor_range = 80
#         self.sensor_max_dist = 300  # Max distance for planning
#         self.horizontal_fov_angle = np.deg2rad(120)  # Horizontal FoV angle in radians
#         self.old_robot_belief = copy.deepcopy(self.robot_belief)

#         self.plot = plot
#         self.frame_files = []
#         if self.plot:
#             # initialize the route
#             self.xPoints = [self.robot_position[0]]
#             self.yPoints = [self.robot_position[1]]

#         self.travel_dist = 0
#         self.explored_rate = 0
#         self.route_node = [self.robot_position]
#         self.frontiers = None
#         self.downsampled_belief = None

#     def begin(self):
#         self.robot_belief = self.update_robot_belief(self.robot_position, self.sensor_range, self.robot_belief, self.ground_truth)
#         self.downsampled_belief = block_reduce(self.robot_belief.copy(), block_size=(self.resolution, self.resolution), func=np.min)

#         self.frontiers = self.find_frontier()

#         self.old_robot_belief = copy.deepcopy(self.robot_belief)

#     def step(self, next_node_coords):
#         dist = np.linalg.norm(next_node_coords - self.robot_position)

#         self.travel_dist += dist
#         self.robot_position = next_node_coords
#         self.route_node.append(self.robot_position)
#         self.robot_belief = self.update_robot_belief(self.robot_position, self.sensor_range, self.robot_belief,
#                                                      self.ground_truth)
#         self.downsampled_belief = block_reduce(self.robot_belief.copy(), block_size=(self.resolution, self.resolution),
#                                                func=np.min)

#         frontiers = self.find_frontier()
#         self.explored_rate = self.evaluate_exploration_rate()

#         if self.plot:
#             self.xPoints.append(self.robot_position[0])
#             self.yPoints.append(self.robot_position[1])

#         self.frontiers = frontiers

#         done = self.check_done()

#         return done

#     def import_ground_truth(self, map_index):
#         # occupied 1, free 255, unexplored 127

#         ground_truth = (io.imread(map_index, 1) * 255).astype(int)
#         robot_location = np.nonzero(ground_truth == 208)
#         robot_location = np.array([np.array(robot_location)[1, 127], np.array(robot_location)[0, 127]])
#         ground_truth = (ground_truth > 150)
#         ground_truth = ground_truth * 254 + 1
#         return ground_truth, robot_location

#     def free_cells(self):
#         index = np.where(self.ground_truth == 255)
#         free = np.asarray([index[1], index[0]]).T
#         return free

#     def update_robot_belief(self, robot_position, sensor_range, robot_belief, ground_truth):
#         # Modify this function to consider the horizontal FoV and max sensor distance
#         robot_belief = sensor_work(robot_position, sensor_range, robot_belief, ground_truth, self.horizontal_fov_angle, self.sensor_max_dist)
#         return robot_belief

#     def check_done(self):
#         done = False
#         if np.sum(self.ground_truth == 255) - np.sum(self.robot_belief == 255) <= (self.finish_percent * np.sum(self.ground_truth == 255)):
#             done = True
#         return done

#     def evaluate_exploration_rate(self):
#         rate = np.sum(self.robot_belief == 255) / np.sum(self.ground_truth == 255)
#         return rate

#     def calculate_new_free_area(self):
#         old_free_area = self.old_robot_belief == 255
#         current_free_area = self.robot_belief == 255

#         new_free_area = (current_free_area.astype(np.int) - old_free_area.astype(np.int)) * 255

#         return new_free_area, np.sum(old_free_area)

#     def calculate_utility_along_path(self, path, nodes_list):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

#         return unique_frontiers.shape[0]

#     def calculate_node_gain_over_path(self, node_index, path, nodes_list):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         pre_unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)
#         observable_frontiers += nodes_list[node_index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

#         return unique_frontiers.shape[0] - pre_unique_frontiers.shape[0]

#     def calculate_dist_path(self, path, node_list):
#         dist = 0
#         start = path[0]
#         end = path[-1]
#         for index in path:
#             if index == end:
#                 break
#             dist += np.linalg.norm(node_list[start].coords - node_list[index].coords)
#             start = index
#         return dist

#     def find_frontier(self):
#         y_len = self.downsampled_belief.shape[0]
#         x_len = self.downsampled_belief.shape[1]
#         mapping = self.downsampled_belief.copy()
#         belief = self.downsampled_belief.copy()
#         # 0-1 unknown area map
#         mapping = (mapping == 127) * 1
#         mapping = np.lib.pad(mapping, ((1, 1), (1, 1)), 'constant', constant_values=0)
#         fro_map = mapping[2:][:, 1:x_len + 1] + mapping[:y_len][:, 1:x_len + 1] + mapping[1:y_len + 1][:, 2:] + \
#                   mapping[1:y_len + 1][:, :x_len] + mapping[:y_len][:, 2:] + mapping[2:][:, :x_len] + mapping[2:][:,
#                                                                                                       2:] + \
#                   mapping[:y_len][:, :x_len]
#         ind_free = np.where(belief.ravel(order='F') == 255)[0]
#         ind_fron_1 = np.where(1 < fro_map.ravel(order='F'))[0]
#         ind_fron_2 = np.where(fro_map.ravel(order='F') < 8)[0]
#         ind_fron = np.intersect1d(ind_fron_1, ind_fron_2)
#         ind_to = np.intersect1d(ind_free, ind_fron)

#         map_x = x_len
#         map_y = y_len
#         x = np.linspace(0, map_x - 1, map_x)
#         y = np.linspace(0, map_y - 1, map_y)
#         t1, t2 = np.meshgrid(x, y)
#         points = np.vstack([t1.T.ravel(), t2.T.ravel()]).T

#         f = points[ind_to]
#         f = f.astype(int)

#         f = f * self.resolution

#         return f

#     def plot_env(self, n, path, step, planned_route=None):
#         plt.switch_backend('agg')
#         plt.cla()
#         plt.imshow(self.robot_belief, cmap='gray')
#         plt.axis((0, self.ground_truth_size[1], self.ground_truth_size[0], 0))
#         if planned_route:
#             for p in planned_route:
#                 planned_x, planned_y = [], []
#                 for coords in p:
#                     planned_x.append(coords[0])
#                     planned_y.append(coords[1])
#                 plt.plot(planned_x, planned_y, c='r', linewidth=2, zorder=2)
#         plt.plot(self.xPoints, self.yPoints, 'b', linewidth=2)
#         plt.plot(self.robot_position[0], self.robot_position[1], 'mo', markersize=8)
#         plt.plot(self.xPoints[0], self.yPoints[0], 'co', markersize=8)
#         plt.scatter(self.frontiers[:, 0], self.frontiers[:, 1], c='r', s=2, zorder=3)
#         plt.suptitle('Explored ratio: {:.4g}  Travel distance: {:.4g}'.format(self.explored_rate, self.travel_dist))
#         plt.tight_layout()
#         plt.savefig('{}/{}_{}_samples.png'.format(path, n, step, dpi=150))
#         frame = '{}/{}_{}_samples.png'.format(path, n, step)
#         self.frame_files.append(frame)



# import numpy as np
# from scipy import ndimage
# from skimage import io
# from skimage.measure import block_reduce
# import matplotlib.pyplot as plt
# import os
# import copy

# class Env():
#     def __init__(self, map_index, plot=False, test=False):
#         self.test = test
#         self.map_dir = f'DungeonMaps/test' if self.test else f'DungeonMaps/train'
#         self.map_list = sorted(os.listdir(self.map_dir), reverse=True)
#         self.map_index = map_index % len(self.map_list)
#         self.ground_truth, self.robot_position = self.import_ground_truth(f'{self.map_dir}/{self.map_list[self.map_index]}')
#         self.ground_truth_size = np.shape(self.ground_truth)
#         self.robot_belief = np.ones(self.ground_truth_size) * 127

#         self.finish_percent = 0.985
#         self.resolution = 4
#         self.sensor_range = 80
#         self.old_robot_belief = copy.deepcopy(self.robot_belief)

#         self.plot = plot
#         self.frame_files = []
#         if self.plot:
#             self.xPoints = [self.robot_position[0]]
#             self.yPoints = [self.robot_position[1]]

#         self.travel_dist = 0
#         self.explored_rate = 0
#         self.route_node = [self.robot_position]
#         self.frontiers = None
#         self.downsampled_belief = None

#     def begin(self):
#         self.robot_belief = self.update_robot_belief(self.robot_position, self.sensor_range, self.robot_belief, self.ground_truth)
#         self.downsampled_belief = block_reduce(self.robot_belief.copy(), block_size=(self.resolution, self.resolution), func=np.min)
#         self.frontiers = self.find_frontier()
#         self.old_robot_belief = copy.deepcopy(self.robot_belief)

#     def step(self, next_node_coords):
#         dist = np.linalg.norm(next_node_coords - self.robot_position)
#         self.travel_dist += dist
#         self.robot_position = next_node_coords
#         self.route_node.append(self.robot_position)
#         self.robot_belief = self.update_robot_belief(self.robot_position, self.sensor_range, self.robot_belief, self.ground_truth)
#         self.downsampled_belief = block_reduce(self.robot_belief.copy(), block_size=(self.resolution, self.resolution), func=np.min)
#         self.frontiers = self.find_frontier()
#         self.explored_rate = self.evaluate_exploration_rate()

#         if self.plot:
#             self.xPoints.append(self.robot_position[0])
#             self.yPoints.append(self.robot_position[1])

#         done = self.check_done()
#         return done

#     def import_ground_truth(self, map_path):
#         ground_truth = (io.imread(map_path, 1) * 255).astype(int)
#         robot_location = np.array(np.nonzero(ground_truth == 208))[:, 127][::-1]
#         ground_truth = (ground_truth > 150) * 254 + 1
#         return ground_truth, robot_location

#     def update_robot_belief(self, robot_position, sensor_range, robot_belief, ground_truth):
#         # Implement sensor_work function here
#         pass

#     def check_done(self):
#         return np.sum(self.ground_truth == 255) - np.sum(self.robot_belief == 255) <= 250

#     def evaluate_exploration_rate(self):
#         return np.sum(self.robot_belief == 255) / np.sum(self.ground_truth == 255)

#     def find_frontier(self):
#         y_len, x_len = self.downsampled_belief.shape
#         mapping = (self.downsampled_belief == 127) * 1
#         mapping = np.pad(mapping, ((1, 1), (1, 1)), 'constant', constant_values=0)
#         fro_map = np.sum([mapping[i:i+y_len, j:j+x_len] for i in range(3) for j in range(3)], axis=0)
#         ind_free = np.where(self.downsampled_belief.ravel(order='F') == 255)[0]
#         ind_fron = np.intersect1d(np.where((1 < fro_map.ravel(order='F')) & (fro_map.ravel(order='F') < 8))[0], ind_free)
        
#         points = np.vstack([np.repeat(np.arange(x_len), y_len), np.tile(np.arange(y_len), x_len)]).T
#         f = points[ind_fron] * self.resolution
#         return f

#     def calculate_utility_along_path(self, path, nodes_list):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         unique_frontiers = np.unique(np.array(observable_frontiers).reshape(-1,2).view(complex))
#         return unique_frontiers.shape[0]

#     def calculate_dist_path(self, path, node_list):
#         dist = 0
#         for start, end in zip(path[:-1], path[1:]):
#             dist += np.linalg.norm(node_list[start].coords - node_list[end].coords)
#         return dist

#     def plot_env(self, n, path, step, planned_route=None):
#         plt.switch_backend('agg')
#         plt.cla()
#         plt.imshow(self.robot_belief, cmap='gray')
#         plt.axis((0, self.ground_truth_size[1], self.ground_truth_size[0], 0))
#         if planned_route:
#             for p in planned_route:
#                 planned_x, planned_y = zip(*p)
#                 plt.plot(planned_x, planned_y, c='r', linewidth=2, zorder=2)
#         plt.plot(self.xPoints, self.yPoints, 'b', linewidth=2)
#         plt.plot(self.robot_position[0], self.robot_position[1], 'mo', markersize=8)
#         plt.plot(self.xPoints[0], self.yPoints[0], 'co', markersize=8)
#         plt.scatter(self.frontiers[:, 0], self.frontiers[:, 1], c='r', s=2, zorder=3)
#         plt.suptitle(f'Explored ratio: {self.explored_rate:.4g}  Travel distance: {self.travel_dist:.4g}')
#         plt.tight_layout()
#         plt.savefig(f'{path}/{n}_{step}_samples.png', dpi=150)
#         self.frame_files.append(f'{path}/{n}_{step}_samples.png')




 
# working with 3

# from scipy import spatial
# from skimage import io
# import numpy as np
# import time
# import sys
# from scipy import ndimage
# import matplotlib.pyplot as plt
# import os
# import copy
# from skimage.measure import block_reduce
# import math
# from sensor import *
# from parameter import *


# class Env():
#     def __init__(self, map_index, plot=False, test=False):
#         self.test = test
#         if self.test:
#             self.map_dir = f'DungeonMaps/test'
#         else:
#             self.map_dir = f'DungeonMaps/train'
#         self.map_list = os.listdir(self.map_dir)
#         self.map_list.sort(reverse=True)
#         self.map_index = map_index % np.size(self.map_list)
#         self.ground_truth, self.robot_position = self.import_ground_truth(self.map_dir + '/' + self.map_list[self.map_index])
#         self.ground_truth_size = np.shape(self.ground_truth) # (480, 640)
#         self.robot_belief = np.ones(self.ground_truth_size) * 127 # unexplored 127
        
#         self.finish_percent = 0.985
#         self.resolution = 4
#         self.old_robot_belief = copy.deepcopy(self.robot_belief)

#         # New parameters
#         self.horizontal_fov = math.radians(260)  # 60 degree horizontal FoV
#         self.sensor_max_dist = 100  # Maximum sensing distance
#         self.sensor_range = 80  # Planning distance, less than sensor_max_dist

#         self.plot = plot
#         self.frame_files = []
#         if self.plot:
#             # initialize the route
#             self.xPoints = [self.robot_position[0]]
#             self.yPoints = [self.robot_position[1]]

#         self.travel_dist = 0
#         self.explored_rate = 0
#         self.route_node = [self.robot_position]
#         self.frontiers = None
#         self.downsampled_belief = None
#     def begin(self):
#         self.robot_belief = self.update_robot_belief(self.robot_position, self.robot_belief, self.ground_truth)
#         self.downsampled_belief = block_reduce(self.robot_belief.copy(), block_size=(self.resolution, self.resolution), func=np.min)

#         self.frontiers = self.find_frontier()
        
#         self.old_robot_belief = copy.deepcopy(self.robot_belief)


#     def step(self, next_node_coords):
#         dist = np.linalg.norm(next_node_coords - self.robot_position)

#         self.travel_dist += dist
#         self.robot_position = next_node_coords
#         self.route_node.append(self.robot_position)
#         self.robot_belief = self.update_robot_belief(self.robot_position, self.robot_belief, self.ground_truth)
#         self.downsampled_belief = block_reduce(self.robot_belief.copy(), block_size=(self.resolution, self.resolution),
#                                                func=np.min)

#         frontiers = self.find_frontier()
#         self.explored_rate = self.evaluate_exploration_rate()

#         if self.plot:
#             self.xPoints.append(self.robot_position[0])
#             self.yPoints.append(self.robot_position[1])

#         self.frontiers = frontiers

#         done = self.check_done()

#         return done

#     def import_ground_truth(self, map_index):
#         # occupied 1, free 255, unexplored 127

#         ground_truth = (io.imread(map_index, 1) * 255).astype(int)
#         robot_location = np.nonzero(ground_truth == 208)
#         robot_location = np.array([np.array(robot_location)[1, 127], np.array(robot_location)[0, 127]])
#         ground_truth = (ground_truth > 150)
#         ground_truth = ground_truth * 254 + 1
#         return ground_truth, robot_location

#     def free_cells(self):
#         index = np.where(self.ground_truth == 255)
#         free = np.asarray([index[1], index[0]]).T
#         return free

#     def update_robot_belief(self, robot_position, robot_belief, ground_truth):
#         robot_belief = sensor_work(robot_position, self.sensor_range, robot_belief, ground_truth, 
#                                    self.horizontal_fov, self.sensor_max_dist)
#         return robot_belief


#     def check_done(self):
#         done = False
#         #if self.node_utility.sum() == 0:
#         #if self.explored_rate >= 0.99 or self.node_utility.sum() == 0:
#         if np.sum(self.ground_truth == 255) - np.sum(self.robot_belief == 255) <= 250:
#             done = True
#         return done

#     def evaluate_exploration_rate(self):
#         rate = np.sum(self.robot_belief == 255) / np.sum(self.ground_truth == 255)
#         return rate

#     def calculate_new_free_area(self):
#         old_free_area = self.old_robot_belief == 255
#         current_free_area = self.robot_belief == 255

#         new_free_area = (current_free_area.astype(np.int) - old_free_area.astype(np.int)) * 255

#         return new_free_area, np.sum(old_free_area)

#     def calculate_utility_along_path(self, path, nodes_list):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

#         return unique_frontiers.shape[0]

#     def calculate_node_gain_over_path(self, node_index, path, nodes_list):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         pre_unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)
#         observable_frontiers += nodes_list[node_index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

#         return unique_frontiers.shape[0] - pre_unique_frontiers.shape[0]

#     def calculate_dist_path(self, path, node_list):
#         dist = 0
#         start = path[0]
#         end = path[-1]
#         for index in path:
#             if index == end:
#                 break
#             dist += np.linalg.norm(node_list[start].coords - node_list[index].coords)
#             start = index
#         return dist

#     def find_frontier(self):
#         y_len = self.downsampled_belief.shape[0]
#         x_len = self.downsampled_belief.shape[1]
#         mapping = self.downsampled_belief.copy()
#         belief = self.downsampled_belief.copy()
#         # 0-1 unknown area map
#         mapping = (mapping == 127) * 1
#         mapping = np.lib.pad(mapping, ((1, 1), (1, 1)), 'constant', constant_values=0)
#         fro_map = mapping[2:][:, 1:x_len + 1] + mapping[:y_len][:, 1:x_len + 1] + mapping[1:y_len + 1][:, 2:] + \
#                   mapping[1:y_len + 1][:, :x_len] + mapping[:y_len][:, 2:] + mapping[2:][:, :x_len] + mapping[2:][:,
#                                                                                                       2:] + \
#                   mapping[:y_len][:, :x_len]
#         ind_free = np.where(belief.ravel(order='F') == 255)[0]
#         ind_fron_1 = np.where(1 < fro_map.ravel(order='F'))[0]
#         ind_fron_2 = np.where(fro_map.ravel(order='F') < 8)[0]
#         ind_fron = np.intersect1d(ind_fron_1, ind_fron_2)
#         ind_to = np.intersect1d(ind_free, ind_fron)

#         map_x = x_len
#         map_y = y_len
#         x = np.linspace(0, map_x - 1, map_x)
#         y = np.linspace(0, map_y - 1, map_y)
#         t1, t2 = np.meshgrid(x, y)
#         points = np.vstack([t1.T.ravel(), t2.T.ravel()]).T

#         f = points[ind_to]
#         f = f.astype(int)

#         f = f * self.resolution

#         return f


#     def plot_env(self, n, path, step, planned_route=None):
#         plt.switch_backend('agg')
#         # plt.ion()
#         plt.cla()
#         plt.imshow(self.robot_belief, cmap='gray')
#         plt.axis((0, self.ground_truth_size[1], self.ground_truth_size[0], 0))
#         if planned_route:
#             for p in planned_route:
#                 planned_x, planned_y = [], []
#                 for coords in p:
#                     planned_x.append(coords[0])
#                     planned_y.append(coords[1])
#                 plt.plot(planned_x, planned_y, c='r', linewidth=2, zorder=2)
#         plt.plot(self.xPoints, self.yPoints, 'b', linewidth=2)
#         plt.plot(self.robot_position[0], self.robot_position[1], 'mo', markersize=8)
#         plt.plot(self.xPoints[0], self.yPoints[0], 'co', markersize=8)
#         plt.scatter(self.frontiers[:, 0], self.frontiers[:, 1], c='r', s=2, zorder=3)
#         # plt.pause(0.1)
#         plt.suptitle('Explored ratio: {:.4g}  Travel distance: {:.4g}'.format(self.explored_rate, self.travel_dist))
#         plt.tight_layout()
#         plt.savefig('{}/{}_{}_samples.png'.format(path, n, step, dpi=150))
#         # plt.show()
#         frame = '{}/{}_{}_samples.png'.format(path, n, step)
#         self.frame_files.append(frame)


#working with 1 

# from scipy import spatial
# from skimage import io
# import numpy as np
# import time
# import sys
# from scipy import ndimage
# import matplotlib.pyplot as plt
# import os
# import copy
# from skimage.measure import block_reduce

# from sensor import *
# from parameter import *


# class Env():
#     def __init__(self, map_index, plot=False, test=False):
#         self.test = test
#         if self.test:
#             self.map_dir = f'DungeonMaps/test'
#         else:
#             self.map_dir = f'DungeonMaps/train'
#         self.map_list = os.listdir(self.map_dir)
#         self.map_list.sort(reverse=True)
#         self.map_index = map_index % np.size(self.map_list)
#         self.ground_truth, self.robot_position = self.import_ground_truth(self.map_dir + '/' + self.map_list[self.map_index])
#         self.ground_truth_size = np.shape(self.ground_truth)  # (480, 640)
#         self.robot_belief = np.ones(self.ground_truth_size) * 127  # unexplored 127

#         self.finish_percent = 0.985
#         self.resolution = 4
#         self.sensor_range = 80
#         self.horizontal_fov_angle = 360  # Horizontal Field of View in degrees
#         self.old_robot_belief = copy.deepcopy(self.robot_belief)

#         self.plot = plot
#         self.frame_files = []
#         if self.plot:
#             # initialize the route
#             self.xPoints = [self.robot_position[0]]
#             self.yPoints = [self.robot_position[1]]

#         self.travel_dist = 0
#         self.explored_rate = 0
#         self.route_node = [self.robot_position]
#         self.frontiers = None
#         self.downsampled_belief = None

#     def begin(self):
#         self.robot_belief = self.update_robot_belief(self.robot_position, self.sensor_range, self.robot_belief, self.ground_truth)
#         self.downsampled_belief = block_reduce(self.robot_belief.copy(), block_size=(self.resolution, self.resolution), func=np.min)

#         self.frontiers = self.find_frontier()

#         self.old_robot_belief = copy.deepcopy(self.robot_belief)

#     def step(self, next_node_coords):
#         dist = np.linalg.norm(next_node_coords - self.robot_position)

#         self.travel_dist += dist
#         self.robot_position = next_node_coords
#         self.route_node.append(self.robot_position)
#         self.robot_belief = self.update_robot_belief(self.robot_position, self.sensor_range, self.robot_belief,
#                                                      self.ground_truth)
#         self.downsampled_belief = block_reduce(self.robot_belief.copy(), block_size=(self.resolution, self.resolution),
#                                                func=np.min)

#         frontiers = self.find_frontier()
#         self.explored_rate = self.evaluate_exploration_rate()

#         if self.plot:
#             self.xPoints.append(self.robot_position[0])
#             self.yPoints.append(self.robot_position[1])

#         self.frontiers = frontiers

#         done = self.check_done()

#         return done

#     def import_ground_truth(self, map_index):
#         # occupied 1, free 255, unexplored 127

#         ground_truth = (io.imread(map_index, 1) * 255).astype(int)
#         robot_location = np.nonzero(ground_truth == 208)
#         robot_location = np.array([np.array(robot_location)[1, 127], np.array(robot_location)[0, 127]])
#         ground_truth = (ground_truth > 150)
#         ground_truth = ground_truth * 254 + 1
#         return ground_truth, robot_location

#     def free_cells(self):
#         index = np.where(self.ground_truth == 255)
#         free = np.asarray([index[1], index[0]]).T
#         return free

#     def update_robot_belief(self, robot_position, sensor_range, robot_belief, ground_truth):
#      robot_belief = sensor_work(robot_position, sensor_range, robot_belief, ground_truth, self.horizontal_fov_angle)
#      return robot_belief


#     def check_done(self):
#         done = False
#         if np.sum(self.ground_truth == 255) - np.sum(self.robot_belief == 255) <= 250:
#             done = True
#         return done

#     def evaluate_exploration_rate(self):
#         rate = np.sum(self.robot_belief == 255) / np.sum(self.ground_truth == 255)
#         return rate

#     def calculate_new_free_area(self):
#         old_free_area = self.old_robot_belief == 255
#         current_free_area = self.robot_belief == 255

#         new_free_area = (current_free_area.astype(np.int) - old_free_area.astype(np.int)) * 255

#         return new_free_area, np.sum(old_free_area)

#     def calculate_utility_along_path(self, path, nodes_list):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1, 2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1] * 1j)

#         return unique_frontiers.shape[0]

#     def calculate_node_gain_over_path(self, node_index, path, nodes_list):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1, 2)
#         pre_unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1] * 1j)
#         observable_frontiers += nodes_list[node_index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1, 2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1] * 1j)

#         return unique_frontiers.shape[0] - pre_unique_frontiers.shape[0]

#     def calculate_dist_path(self, path, node_list):
#         dist = 0
#         start = path[0]
#         end = path[-1]
#         for index in path:
#             if index == end:
#                 break
#             dist += np.linalg.norm(node_list[start].coords - node_list[index].coords)
#             start = index
#         return dist

#     def find_frontier(self):
#         y_len = self.downsampled_belief.shape[0]
#         x_len = self.downsampled_belief.shape[1]
#         mapping = self.downsampled_belief.copy()
#         belief = self.downsampled_belief.copy()
#         # 0-1 unknown area map
#         mapping = (mapping == 127) * 1
#         mapping = np.lib.pad(mapping, ((1, 1), (1, 1)), 'constant', constant_values=0)
#         fro_map = mapping[2:][:, 1:x_len + 1] + mapping[:y_len][:, 1:x_len + 1] + mapping[1:y_len + 1][:, 2:] + \
#                   mapping[1:y_len + 1][:, :x_len] + mapping[:y_len][:, 2:] + mapping[2:][:, :x_len] + mapping[2:][:,
#                                                                                                       2:] + \
#                   mapping[:y_len][:, :x_len]
#         ind_free = np.where(belief.ravel(order='F') == 255)[0]
#         ind_fron_1 = np.where(1 < fro_map.ravel(order='F'))[0]
#         ind_fron_2 = np.where(fro_map.ravel(order='F') < 8)[0]
#         ind_fron = np.intersect1d(ind_fron_1, ind_fron_2)
#         ind_to = np.intersect1d(ind_free, ind_fron)

#         map_x = x_len
#         map_y = y_len
#         x = np.linspace(0, map_x - 1, map_x)
#         y = np.linspace(0, map_y - 1, map_y)
#         t1, t2 = np.meshgrid(x, y)
#         points = np.vstack([t1.T.ravel(), t2.T.ravel()]).T

#         f = points[ind_to]
#         f = f.astype(int)

#         f = f * self.resolution

#         return f

#     def is_in_horizontal_fov(self, point):
#         direction_vector = point - self.robot_position
#         angle = np.arctan2(direction_vector[1], direction_vector[0])  # Angle in radians

#         # Assuming robot's forward direction is along the positive x-axis
#         robot_direction_angle = 0  # Adjust if robot's default direction is different

#         relative_angle = np.abs(angle - robot_direction_angle)

#         # Normalize angle to the range [-π, π]
#         relative_angle = (relative_angle + np.pi) % (2 * np.pi) - np.pi

#         # Check if the relative angle is within the horizontal FoV
#         horizontal_fov_rad = np.deg2rad(self.horizontal_fov_angle / 2.0)
#         return np.abs(relative_angle) <= horizontal_fov_rad

#     def plot_env(self, n, path, step, planned_route=None):
#         plt.switch_backend('agg')
#         plt.cla()
#         plt.imshow(self.robot_belief, cmap='gray')
#         plt.axis((0, self.ground_truth_size[1], self.ground_truth_size[0], 0))
#         if planned_route:
#             for p in planned_route:
#                 planned_x, planned_y = [], []
#                 for coords in p:
#                     planned_x.append(coords[0])
#                     planned_y.append(coords[1])
#                 plt.plot(planned_x, planned_y, c='r', linewidth=2, zorder=2)
#         plt.plot(self.xPoints, self.yPoints, 'b', linewidth=2)
#         plt.plot(self.robot_position[0], self.robot_position[1], 'mo', markersize=8)
#         plt.plot(self.xPoints[0], self.yPoints[0], 'co', markersize=8)
#         plt.scatter(self.frontiers[:, 0], self.frontiers[:, 1], c='r', s=2, zorder=3)
#         plt.suptitle('Explored ratio: {:.4g}  Travel distance: {:.4g}'.format(self.explored_rate, self.travel_dist))
#         plt.tight_layout()
#         plt.savefig('{}/{}_{}_samples.png'.format(path, n, step, dpi=150))
#         frame = '{}/{}_{}_samples.png'.format(path, n, step)
#         self.frame_files.append(frame)





# from scipy import spatial
# from skimage import io
# import numpy as np
# import time
# import sys
# from scipy import ndimage
# import matplotlib.pyplot as plt
# import os
# import copy
# from skimage.measure import block_reduce
# import math
# from sensor import *
# from parameter import *

# class Env():
#     def __init__(self, map_index, num_agents=1, plot=False, test=False):
#         self.test = test
#         self.num_agents = num_agents
#         if self.test:
#             self.map_dir = f'DungeonMaps/test'
#         else:
#             self.map_dir = f'DungeonMaps/train'
#         self.map_list = os.listdir(self.map_dir)
#         self.map_list.sort(reverse=True)
#         self.map_index = map_index % np.size(self.map_list)
        
#         # Fetch the ground truth and initial position
#         self.ground_truth, initial_robot_position = self.import_ground_truth(self.map_dir + '/' + self.map_list[self.map_index])
        
#         self.ground_truth_size = np.shape(self.ground_truth)  # (480, 640)

#         # Initialize each agent's state
#         self.robot_positions = [initial_robot_position for _ in range(num_agents)]
#         self.robot_beliefs = [np.ones(self.ground_truth_size) * 127 for _ in range(num_agents)]  # unexplored 127
#         self.old_robot_beliefs = [copy.deepcopy(self.robot_beliefs[i]) for i in range(num_agents)]
#         self.travel_dists = [0 for _ in range(num_agents)]
#         self.routes = [[initial_robot_position] for _ in range(num_agents)]
#         self.explored_rates = [0 for _ in range(num_agents)]
#         self.frontiers = None
#         self.downsampled_beliefs = [None for _ in range(num_agents)]

#         self.finish_percent = 0.985
#         self.resolution = 4

#         # New parameters
#         self.horizontal_fov = math.radians(260)  # 260 degree horizontal FoV
#         self.sensor_max_dist = 100  # Maximum sensing distance
#         self.sensor_range = 80  # Planning distance, less than sensor_max_dist

#         self.plot = plot
#         self.frame_files = []
#         if self.plot:
#             # Initialize routes for all agents
#             self.xPoints = [[initial_robot_position[0]] for _ in range(num_agents)]
#             self.yPoints = [[initial_robot_position[1]] for _ in range(num_agents)]

#     def begin(self):
#         for i in range(self.num_agents):
#             self.robot_beliefs[i] = self.update_robot_belief(self.robot_positions[i], self.robot_beliefs[i], self.ground_truth)
#         self.downsampled_beliefs = [block_reduce(belief.copy(), block_size=(self.resolution, self.resolution), func=np.min)
#                                     for belief in self.robot_beliefs]
#         self.frontiers = self.find_frontier()
#         self.old_robot_beliefs = [copy.deepcopy(belief) for belief in self.robot_beliefs]

#     def step(self, agent_index, next_node_coords):
#         dist = np.linalg.norm(next_node_coords - self.robot_positions[agent_index])
#         self.travel_dists[agent_index] += dist
#         self.robot_positions[agent_index] = next_node_coords
#         self.routes[agent_index].append(self.robot_positions[agent_index])
#         self.robot_beliefs[agent_index] = self.update_robot_belief(self.robot_positions[agent_index],
#                                                                   self.robot_beliefs[agent_index], self.ground_truth)
#         self.downsampled_beliefs[agent_index] = block_reduce(self.robot_beliefs[agent_index].copy(),
#                                                              block_size=(self.resolution, self.resolution),
#                                                              func=np.min)
#         self.frontiers = self.find_frontier()
#         self.explored_rates[agent_index] = self.evaluate_exploration_rate()

#         if self.plot:
#             self.xPoints[agent_index].append(self.robot_positions[agent_index][0])
#             self.yPoints[agent_index].append(self.robot_positions[agent_index][1])

#         done = self.check_done(agent_index)

#         return done
    
#     def import_ground_truth(self, map_path):
#         # occupied 1, free 255, unexplored 127

#         ground_truth = (io.imread(map_path, 1) * 255).astype(int)
#         robot_location = np.nonzero(ground_truth == 208)
#         robot_location = np.array([np.array(robot_location)[1, 127], np.array(robot_location)[0, 127]])
#         ground_truth = (ground_truth > 150)
#         ground_truth = ground_truth * 254 + 1
#         return ground_truth, robot_location

#     def update_robot_belief(self, robot_position, robot_belief, ground_truth):
#         robot_belief = sensor_work(robot_position, self.sensor_range, robot_belief, ground_truth, 
#                                    self.horizontal_fov, self.sensor_max_dist)
#         return robot_belief

#     def find_frontier(self):
#         # Example implementation; you may need to adapt it
#         frontiers = []
#         for y in range(1, self.ground_truth_size[0] - 1):
#             for x in range(1, self.ground_truth_size[1] - 1):
#                 if self.ground_truth[y, x] == 127:  # unexplored
#                     # Check if it borders a known area
#                     if np.any(self.ground_truth[y-1:y+2, x-1:x+2] == 255):  # free space
#                         frontiers.append((x, y))
#         return np.array(frontiers)

#     def evaluate_exploration_rate(self):
#         explored_area = np.sum([belief == 255 for belief in self.robot_beliefs])  # Count free space cells
#         total_area = self.ground_truth_size[0] * self.ground_truth_size[1]
#         return explored_area / total_area

#     def check_done(self, agent_index):
#         return self.explored_rates[agent_index] >= self.finish_percent

#     def plot_env(self, n, path, step, planned_route=None):
#         plt.switch_backend('agg')
#         plt.cla()
#         plt.imshow(self.robot_beliefs[0], cmap='gray')  # Assuming you want to plot the first agent's belief
#         plt.axis((0, self.ground_truth_size[1], self.ground_truth_size[0], 0))
#         if planned_route:
#             for p in planned_route:
#                 planned_x, planned_y = [], []
#                 for coords in p:
#                     planned_x.append(coords[0])
#                     planned_y.append(coords[1])
#                 plt.plot(planned_x, planned_y, c='r', linewidth=2, zorder=2)
#         for i in range(self.num_agents):
#             plt.plot(self.xPoints[i], self.yPoints[i], 'b', linewidth=2)
#             plt.plot(self.robot_positions[i][0], self.robot_positions[i][1], 'mo', markersize=8)
#             plt.plot(self.xPoints[i][0], self.yPoints[i][0], 'co', markersize=8)
#         plt.scatter(self.frontiers[:, 0], self.frontiers[:, 1], c='r', s=2, zorder=3)
#         plt.suptitle('Explored ratio: {:.4g}  Travel distance: {:.4g}'.format(self.explored_rates[0], sum(self.travel_dists)))
#         plt.tight_layout()
#         plt.savefig('{}/{}_{}_samples.png'.format(path, n, step, dpi=150))
#         frame = '{}/{}_{}_samples.png'.format(path, n, step)
#         self.frame_files.append(frame)


#claude

# from scipy import spatial
# from skimage import io
# import numpy as np
# import time
# import sys
# from scipy import ndimage
# import matplotlib.pyplot as plt
# import os
# import copy
# from skimage.measure import block_reduce
# import math
# from sensor import *
# from parameter import *

# class Env():
#     def __init__(self, map_index, plot=False, test=False, num_agents=3):
#         self.test = test
#         self.plot = plot
#         self.frame_files = []
#         if self.test:
#             self.map_dir = f'DungeonMaps/test'
#         else:
#             self.map_dir = f'DungeonMaps/train'
#         self.map_list = os.listdir(self.map_dir)
#         self.map_list.sort(reverse=True)
#         self.map_index = map_index % np.size(self.map_list)
#         self.ground_truth, self.robot_positions = self.import_ground_truth(self.map_dir + '/' + self.map_list[self.map_index], num_agents)
#         self.ground_truth_size = np.shape(self.ground_truth) # (480, 640)
#         self.robot_beliefs = [np.ones(self.ground_truth_size) * 127 for _ in range(num_agents)] # unexplored 127
        
#         self.finish_percent = 0.985
#         self.resolution = 4
#         self.old_robot_beliefs = copy.deepcopy(self.robot_beliefs)

#         self.horizontal_fov = math.radians(260)  # 60 degree horizontal FoV
#         self.sensor_max_dist = 100  # Maximum sensing distance
#         self.sensor_range = 80  # Planning distance, less than sensor_max_dist

#         self.plot = plot
#         self.frame_files = []
#         if self.plot:
#             # initialize the routes
#             self.xPoints = [[] for _ in range(num_agents)]
#             self.yPoints = [[] for _ in range(num_agents)]
#             for i in range(num_agents):
#                 self.xPoints[i].append(self.robot_positions[i][0])
#                 self.yPoints[i].append(self.robot_positions[i][1])

#         self.travel_dists = [0 for _ in range(num_agents)]
#         self.explored_rate = 0
#         self.route_nodes = [[pos] for pos in self.robot_positions]
#         self.frontiers = None
#         self.downsampled_belief = None
#         self.num_agents = num_agents
#         self.agent_colors = plt.cm.rainbow(np.linspace(0, 1, num_agents))

#     def import_ground_truth(self, map_index, num_agents):
#         ground_truth = (io.imread(map_index, 1) * 255).astype(int)
#         robot_locations = np.nonzero(ground_truth == 208)
#         robot_locations = np.array([np.array(robot_locations)[1, :num_agents], np.array(robot_locations)[0, :num_agents]]).T
#         ground_truth = (ground_truth > 150)
#         ground_truth = ground_truth * 254 + 1
#         return ground_truth, robot_locations

#     def begin(self):
#         for i in range(self.num_agents):
#             self.robot_beliefs[i] = self.update_robot_belief(self.robot_positions[i], self.robot_beliefs[i], self.ground_truth)
#         self.downsampled_belief = block_reduce(self.get_combined_belief(), block_size=(self.resolution, self.resolution), func=np.min)
#         self.frontiers = self.find_frontier()
#         self.old_robot_beliefs = copy.deepcopy(self.robot_beliefs)

#     def step(self, next_node_coords_list):
#         for i, next_node_coords in enumerate(next_node_coords_list):
#             dist = np.linalg.norm(next_node_coords - self.robot_positions[i])
#             self.travel_dists[i] += dist
#             self.robot_positions[i] = next_node_coords
#             self.route_nodes[i].append(self.robot_positions[i])
#             self.robot_beliefs[i] = self.update_robot_belief(self.robot_positions[i], self.robot_beliefs[i], self.ground_truth)

#             if self.plot:
#                 self.xPoints[i].append(self.robot_positions[i][0])
#                 self.yPoints[i].append(self.robot_positions[i][1])

#         self.downsampled_belief = block_reduce(self.get_combined_belief(), block_size=(self.resolution, self.resolution), func=np.min)
#         self.frontiers = self.find_frontier()
#         self.explored_rate = self.evaluate_exploration_rate()

#         done = self.check_done()
#         return done
#     def get_agent_position(self, agent_id):
#         return self.robot_positions[agent_id]

#     def get_agent_belief(self, agent_id):
#         return self.robot_beliefs[agent_id]

#     def get_combined_belief(self):
#         return np.min(self.robot_beliefs, axis=0)

#     def update_agent_belief(self, agent_id, belief):
#         self.robot_beliefs[agent_id] = belief

#     def get_total_travel_dist(self):
#         return sum(self.travel_dists)

#     def get_other_agents_planned_paths(self, agent_id):
#         # This method should be implemented to return the planned paths of other agents
#         # For now, we'll return an empty list
#         return []

#     def update_robot_belief(self, robot_position, robot_belief, ground_truth):
#         robot_belief = sensor_work(robot_position, self.sensor_range, robot_belief, ground_truth, 
#                                    self.horizontal_fov, self.sensor_max_dist)
#         return robot_belief

#     def check_done(self):
#         done = False
#         if np.sum(self.ground_truth == 255) - np.sum(self.get_combined_belief() == 255) <= 250:
#             done = True
#         return done

#     def evaluate_exploration_rate(self):
#         rate = np.sum(self.get_combined_belief() == 255) / np.sum(self.ground_truth == 255)
#         return rate

#     def calculate_new_free_area(self):
#         old_free_area = np.min([belief == 255 for belief in self.old_robot_beliefs], axis=0)
#         current_free_area = self.get_combined_belief() == 255

#         new_free_area = (current_free_area.astype(np.int) - old_free_area.astype(np.int)) * 255

#         return new_free_area, np.sum(old_free_area)

#     def calculate_utility_along_path(self, path, nodes_list, agent_id):
#      observable_frontiers = []
#      for index in path:
#         observable_frontiers += nodes_list[index].observable_frontiers
#      np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#      unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

#     # Filter out frontiers that are already observed by other agents
#      combined_belief = self.get_combined_belief()
#      valid_frontiers = [f for f in unique_frontiers if combined_belief[int(f.imag), int(f.real)] == 127]

#      return len(valid_frontiers)

#     def calculate_node_gain_over_path(self, node_index, path, nodes_list):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         pre_unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)
#         observable_frontiers += nodes_list[node_index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

#         return unique_frontiers.shape[0] - pre_unique_frontiers.shape[0]

#     def calculate_dist_path(self, path, node_list):
#         dist = 0
#         start = path[0]
#         end = path[-1]
#         for index in path:
#             if index == end:
#                 break
#             dist += np.linalg.norm(node_list[start].coords - node_list[index].coords)
#             start = index
#         return dist

#     def find_frontier(self):
#         y_len = self.downsampled_belief.shape[0]
#         x_len = self.downsampled_belief.shape[1]
#         mapping = self.downsampled_belief.copy()
#         belief = self.downsampled_belief.copy()
#         # 0-1 unknown area map
#         mapping = (mapping == 127) * 1
#         mapping = np.lib.pad(mapping, ((1, 1), (1, 1)), 'constant', constant_values=0)
#         fro_map = mapping[2:][:, 1:x_len + 1] + mapping[:y_len][:, 1:x_len + 1] + mapping[1:y_len + 1][:, 2:] + \
#                   mapping[1:y_len + 1][:, :x_len] + mapping[:y_len][:, 2:] + mapping[2:][:, :x_len] + mapping[2:][:,
#                                                                                                       2:] + \
#                   mapping[:y_len][:, :x_len]
#         ind_free = np.where(belief.ravel(order='F') == 255)[0]
#         ind_fron_1 = np.where(1 < fro_map.ravel(order='F'))[0]
#         ind_fron_2 = np.where(fro_map.ravel(order='F') < 8)[0]
#         ind_fron = np.intersect1d(ind_fron_1, ind_fron_2)
#         ind_to = np.intersect1d(ind_free, ind_fron)

#         map_x = x_len
#         map_y = y_len
#         x = np.linspace(0, map_x - 1, map_x)
#         y = np.linspace(0, map_y - 1, map_y)
#         t1, t2 = np.meshgrid(x, y)
#         points = np.vstack([t1.T.ravel(), t2.T.ravel()]).T

#         f = points[ind_to]
#         f = f.astype(int)

#         f = f * self.resolution

#         return f

#     def plot_env(self, n, path, step, planned_routes=None):
#         plt.switch_backend('agg')
#         plt.figure(figsize=(10, 8))
#         plt.cla()
#         plt.imshow(self.get_combined_belief(), cmap='gray')
#         plt.axis((0, self.ground_truth_size[1], self.ground_truth_size[0], 0))
        
#         if planned_routes:
#             for agent_id, routes in enumerate(planned_routes):
#                 for p in routes:
#                     planned_x, planned_y = [], []
#                     for coords in p:
#                         planned_x.append(coords[0])
#                         planned_y.append(coords[1])
#                     plt.plot(planned_x, planned_y, c=self.agent_colors[agent_id], linewidth=1, zorder=2, alpha=0.5)

#         for i in range(self.num_agents):
#             plt.plot(self.xPoints[i], self.yPoints[i], c=self.agent_colors[i], linewidth=2)
#             plt.plot(self.robot_positions[i][0], self.robot_positions[i][1], 'o', c=self.agent_colors[i], markersize=8)
#             plt.plot(self.xPoints[i][0], self.yPoints[i][0], 'co', markersize=8)

#         plt.scatter(self.frontiers[:, 0], self.frontiers[:, 1], c='r', s=2, zorder=3)
#         plt.suptitle('Explored ratio: {:.4g}  Total travel distance: {:.4g}'.format(self.explored_rate, self.get_total_travel_dist()))
#         plt.tight_layout()
#         filename = '{}/{}_{}_samples.png'.format(path, n, step)
#         plt.savefig(filename, dpi=150)
#         plt.close()
#         self.frame_files.append(filename)
#         print(f"Saved image: {filename}")



# from scipy import spatial
# from skimage import io
# import numpy as np
# import time
# import sys
# from scipy import ndimage
# import matplotlib.pyplot as plt
# import os
# import copy
# from skimage.measure import block_reduce
# import math
# from sensor import *
# from parameter import *

# class Env():
#     def __init__(self, map_index, plot=False, test=False):
#         self.test = test
#         if self.test:
#             self.map_dir = f'DungeonMaps/test'
#         else:
#             self.map_dir = f'DungeonMaps/train'
#         self.map_list = os.listdir(self.map_dir)
#         self.map_list.sort(reverse=True)
#         self.map_index = map_index % len(self.map_list)
#         self.ground_truth, self.robot_position = self.import_ground_truth(self.map_dir + '/' + self.map_list[self.map_index])
#         self.ground_truth_size = np.shape(self.ground_truth)  # (480, 640)
#         self.robot_belief = np.ones(self.ground_truth_size) * 127  # unexplored 127
        
#         self.finish_percent = 0.985
#         self.resolution = 4
#         self.old_robot_belief = np.copy(self.robot_belief)

#         # New parameters
#         self.horizontal_fov = math.radians(260)  # 260 degree horizontal FoV
#         self.sensor_max_dist = 100  # Maximum sensing distance
#         self.sensor_range = 80  # Planning distance, less than sensor_max_dist

#         self.plot = plot
#         self.frame_files = []
#         if self.plot:
#             # initialize the route
#             self.xPoints = [self.robot_position[0]]
#             self.yPoints = [self.robot_position[1]]

#         self.travel_dist = 0
#         self.explored_rate = 0
#         self.route_node = [self.robot_position]
#         self.frontiers = None
#         self.downsampled_belief = None

#     def begin(self, agent_id=None):
#         if self.robot_position is None:
#             raise ValueError("Environment not properly initialized with robot_position.")

#         self.robot_belief = self.update_robot_belief(self.robot_position, self.robot_belief, self.ground_truth)
#         self.downsampled_belief = block_reduce(self.robot_belief.copy(), block_size=(self.resolution, self.resolution), func=np.min)
#         self.frontiers = self.find_frontier()
#         self.old_robot_belief = np.copy(self.robot_belief)

#         if agent_id is not None:
#             # Custom initialization per agent if required
#             print(f"Initializing environment for agent {agent_id}")

#     def step(self, next_node_coords):
#         dist = np.linalg.norm(next_node_coords - self.robot_position)

#         self.travel_dist += dist
#         self.robot_position = next_node_coords
#         self.route_node.append(self.robot_position)
#         self.robot_belief = self.update_robot_belief(self.robot_position, self.robot_belief, self.ground_truth)
#         self.downsampled_belief = block_reduce(self.robot_belief.copy(), block_size=(self.resolution, self.resolution),
#                                                func=np.min)

#         frontiers = self.find_frontier()
#         self.explored_rate = self.evaluate_exploration_rate()

#         if self.plot:
#             self.xPoints.append(self.robot_position[0])
#             self.yPoints.append(self.robot_position[1])

#         self.frontiers = frontiers

#         done = self.check_done()

#         return done

#     def import_ground_truth(self, map_index):
#         # occupied 1, free 255, unexplored 127

#         ground_truth = (io.imread(map_index, 1) * 255).astype(int)
#         robot_location = np.nonzero(ground_truth == 208)
#         if len(robot_location[0]) == 0 or len(robot_location[1]) == 0:
#             raise ValueError("Robot location not found in ground truth map.")
#         robot_location = np.array([robot_location[1][0], robot_location[0][0]])
#         ground_truth = (ground_truth > 150)
#         ground_truth = ground_truth * 254 + 1
#         return ground_truth, robot_location

#     def free_cells(self):
#         index = np.where(self.ground_truth == 255)
#         free = np.asarray([index[1], index[0]]).T
#         return free

#     def update_robot_belief(self, robot_position, robot_belief, ground_truth):
#         robot_belief = sensor_work(robot_position, self.sensor_range, robot_belief, ground_truth, 
#                                    self.horizontal_fov, self.sensor_max_dist)
#         return robot_belief

#     def check_done(self):
#         done = False
#         if np.sum(self.ground_truth == 255) - np.sum(self.robot_belief == 255) <= 250:
#             done = True
#         return done

#     def evaluate_exploration_rate(self):
#         rate = np.sum(self.robot_belief == 255) / np.sum(self.ground_truth == 255)
#         return rate

#     def calculate_new_free_area(self):
#         old_free_area = self.old_robot_belief == 255
#         current_free_area = self.robot_belief == 255

#         new_free_area = (current_free_area.astype(np.int) - old_free_area.astype(np.int)) * 255

#         return new_free_area, np.sum(old_free_area)

#     def calculate_utility_along_path(self, path, nodes_list):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

#         return unique_frontiers.shape[0]

#     def calculate_node_gain_over_path(self, node_index, path, nodes_list):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         pre_unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)
#         observable_frontiers += nodes_list[node_index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

#         return unique_frontiers.shape[0] - pre_unique_frontiers.shape[0]

#     def calculate_dist_path(self, path, node_list):
#         dist = 0
#         start = path[0]
#         end = path[-1]
#         for index in path:
#             if index == end:
#                 break
#             dist += np.linalg.norm(node_list[start].coords - node_list[index].coords)
#             start = index
#         return dist

#     def find_frontier(self):
#         y_len = self.downsampled_belief.shape[0]
#         x_len = self.downsampled_belief.shape[1]
#         mapping = self.downsampled_belief.copy()
#         belief = self.downsampled_belief.copy()
#         # 0-1 unknown area map
#         mapping = (mapping == 127) * 1
#         mapping = np.lib.pad(mapping, ((1, 1), (1, 1)), 'constant', constant_values=0)
#         fro_map = mapping[2:][:, 1:x_len + 1] + mapping[:y_len][:, 1:x_len + 1] + mapping[1:y_len + 1][:, 2:] + \
#                   mapping[1:y_len + 1][:, :x_len] + mapping[:y_len][:, 2:] + mapping[2:][:, :x_len] + mapping[2:][:,
#                                                                                                       2:] + \
#                   mapping[:y_len][:, :x_len]
#         ind_free = np.where(belief.ravel(order='F') == 255)[0]
#         ind_fron_1 = np.where(1 < fro_map.ravel(order='F'))[0]
#         ind_fron_2 = np.where(fro_map.ravel(order='F') < 8)[0]
#         ind_fron = np.intersect1d(ind_fron_1, ind_fron_2)
#         ind_to = np.intersect1d(ind_free, ind_fron)

#         map_x = x_len
#         map_y = y_len
#         x = np.linspace(0, map_x - 1, map_x)
#         y = np.linspace(0, map_y - 1, map_y)
#         t1, t2 = np.meshgrid(x, y)
#         points = np.vstack([t1.T.ravel(), t2.T.ravel()]).T

#         f = points[ind_to]
#         f = f.astype(int)

#         f = f * self.resolution

#         return f

#     def plot_env(self, n, path, step, planned_route=None):
#         plt.switch_backend('agg')
#         plt.cla()
#         plt.imshow(self.robot_belief, cmap='gray')
#         plt.axis((0, self.ground_truth_size[1], self.ground_truth_size[0], 0))
#         if planned_route:
#             for p in planned_route:
#                 planned_x, planned_y = [], []
#                 for coords in p:
#                     planned_x.append(coords[0])
#                     planned_y.append(coords[1])
#                 plt.plot(planned_y, planned_x, 'g-', markersize=7)
#         if path:
#             path_x, path_y = [], []
#             for coords in path:
#                 path_x.append(coords[0])
#                 path_y.append(coords[1])
#             plt.plot(path_y, path_x, 'r-', markersize=7)
#         plt.plot(self.robot_position[1], self.robot_position[0], 'ro', markersize=7)
#         plt.title(f'Episode {n} - Step {step} Travel Distance {self.travel_dist:.1f}')
#         plt.savefig(f'./frames/{n:02d}_{step:02d}.png')
#         plt.close()



# from scipy import spatial
# from skimage import io
# import numpy as np
# import time
# import sys
# from scipy import ndimage
# import matplotlib.pyplot as plt
# import os
# import copy
# from skimage.measure import block_reduce
# import math
# from sensor import *
# from parameter import *
# class Env():
#     def __init__(self, map_index, plot=False, test=False, num_agents=3):
#         self.test = test
#         self.plot = plot
#         self.frame_files = []
#         if self.test:
#             self.map_dir = f'DungeonMaps/test'
#         else:
#             self.map_dir = f'DungeonMaps/train'
#         self.map_list = os.listdir(self.map_dir)
#         self.map_list.sort(reverse=True)
#         self.map_index = map_index % np.size(self.map_list)
#         self.ground_truth, self.robot_positions = self.import_ground_truth(self.map_dir + '/' + self.map_list[self.map_index], num_agents)
#         self.ground_truth_size = np.shape(self.ground_truth)
#         self.robot_beliefs = [np.ones(self.ground_truth_size) * 127 for _ in range(num_agents)]
        
#         self.finish_percent = 0.985
#         self.resolution = 4
#         self.old_robot_beliefs = copy.deepcopy(self.robot_beliefs)

#         self.horizontal_fov = math.radians(260)
#         self.sensor_max_dist = 100
#         self.sensor_range = 80

#         self.plot = plot
#         self.frame_files = []
#         if self.plot:
#             self.xPoints = [[] for _ in range(num_agents)]
#             self.yPoints = [[] for _ in range(num_agents)]
#             for i in range(num_agents):
#                 self.xPoints[i].append(self.robot_positions[i][0])
#                 self.yPoints[i].append(self.robot_positions[i][1])

#         self.travel_dists = [0 for _ in range(num_agents)]
#         self.explored_rate = 0
#         self.route_nodes = [[pos] for pos in self.robot_positions]
#         self.frontiers = None
#         self.downsampled_belief = None
#         self.num_agents = num_agents
#         self.agent_colors = plt.cm.rainbow(np.linspace(0, 1, num_agents))
#         self.bounding_box = np.array([1, 1, 1])  # Adjust as needed

#     def import_ground_truth(self, map_index, num_agents):
#         ground_truth = (io.imread(map_index, 1) * 255).astype(int)
#         robot_locations = np.nonzero(ground_truth == 208)
#         robot_locations = np.array([np.array(robot_locations)[1, :num_agents], np.array(robot_locations)[0, :num_agents]]).T
#         ground_truth = (ground_truth > 150)
#         ground_truth = ground_truth * 254 + 1
#         return ground_truth, robot_locations

#     def begin(self):
#         for i in range(self.num_agents):
#             self.robot_beliefs[i] = self.update_robot_belief(self.robot_positions[i], self.robot_beliefs[i], self.ground_truth)
#         self.downsampled_belief = block_reduce(self.get_combined_belief(), block_size=(self.resolution, self.resolution), func=np.min)
#         self.frontiers = self.find_frontier()
#         self.old_robot_beliefs = copy.deepcopy(self.robot_beliefs)

#     def step(self, next_node_coords_list):
#         for i, next_node_coords in enumerate(next_node_coords_list):
#             dist = np.linalg.norm(next_node_coords - self.robot_positions[i])
#             self.travel_dists[i] += dist
#             self.robot_positions[i] = next_node_coords
#             self.route_nodes[i].append(self.robot_positions[i])
#             self.robot_beliefs[i] = self.update_robot_belief(self.robot_positions[i], self.robot_beliefs[i], self.ground_truth)

#             if self.plot:
#                 self.xPoints[i].append(self.robot_positions[i][0])
#                 self.yPoints[i].append(self.robot_positions[i][1])

#         self.downsampled_belief = block_reduce(self.get_combined_belief(), block_size=(self.resolution, self.resolution), func=np.min)
#         self.frontiers = self.find_frontier()
#         self.explored_rate = self.evaluate_exploration_rate()

#         done = self.check_done()
#         return done

#     def get_agent_position(self, agent_id):
#         return self.robot_positions[agent_id]

#     def get_agent_belief(self, agent_id):
#         return self.robot_beliefs[agent_id]

#     def get_combined_belief(self):
#         return np.min(self.robot_beliefs, axis=0)

#     def update_agent_belief(self, agent_id, belief):
#         self.robot_beliefs[agent_id] = belief

#     def get_total_travel_dist(self):
#         return sum(self.travel_dists)

#     def get_other_agents_positions(self, agent_id):
#         return [pos for i, pos in enumerate(self.robot_positions) if i != agent_id]

#     def update_robot_belief(self, robot_position, robot_belief, ground_truth):
#         robot_belief = sensor_work(robot_position, self.sensor_range, robot_belief, ground_truth, 
#                                    self.horizontal_fov, self.sensor_max_dist)
#         return robot_belief

#     def check_done(self):
#         done = False
#         if np.sum(self.ground_truth == 255) - np.sum(self.get_combined_belief() == 255) <= 250:
#             done = True
#         return done

#     def evaluate_exploration_rate(self):
#         rate = np.sum(self.get_combined_belief() == 255) / np.sum(self.ground_truth == 255)
#         return rate

#     def calculate_new_free_area(self):
#         old_free_area = np.min([belief == 255 for belief in self.old_robot_beliefs], axis=0)
#         current_free_area = self.get_combined_belief() == 255

#         new_free_area = (current_free_area.astype(np.int) - old_free_area.astype(np.int)) * 255

#         return new_free_area, np.sum(old_free_area)

#     def calculate_utility_along_path(self, path, nodes_list, agent_id):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

#         # Filter out frontiers that are already observed by other agents
#         combined_belief = self.get_combined_belief()
#         valid_frontiers = [f for f in unique_frontiers if combined_belief[int(f.imag), int(f.real)] == 127]

#         return len(valid_frontiers)

#     def calculate_dist_path(self, path, node_list):
#         dist = 0
#         start = path[0]
#         end = path[-1]
#         for index in path:
#             if index == end:
#                 break
#             dist += np.linalg.norm(node_list[start].coords - node_list[index].coords)
#             start = index
#         return dist

#     def find_frontier(self):
#         y_len = self.downsampled_belief.shape[0]
#         x_len = self.downsampled_belief.shape[1]
#         mapping = self.downsampled_belief.copy()
#         belief = self.downsampled_belief.copy()
#         mapping = (mapping == 127) * 1
#         mapping = np.lib.pad(mapping, ((1, 1), (1, 1)), 'constant', constant_values=0)
#         fro_map = mapping[2:][:, 1:x_len + 1] + mapping[:y_len][:, 1:x_len + 1] + mapping[1:y_len + 1][:, 2:] + \
#                   mapping[1:y_len + 1][:, :x_len] + mapping[:y_len][:, 2:] + mapping[2:][:, :x_len] + mapping[2:][:,
#                                                                                                       2:] + \
#                   mapping[:y_len][:, :x_len]
#         ind_free = np.where(belief.ravel(order='F') == 255)[0]
#         ind_fron_1 = np.where(1 < fro_map.ravel(order='F'))[0]
#         ind_fron_2 = np.where(fro_map.ravel(order='F') < 8)[0]
#         ind_fron = np.intersect1d(ind_fron_1, ind_fron_2)
#         ind_to = np.intersect1d(ind_free, ind_fron)

#         map_x = x_len
#         map_y = y_len
#         x = np.linspace(0, map_x - 1, map_x)
#         y = np.linspace(0, map_y - 1, map_y)
#         t1, t2 = np.meshgrid(x, y)
#         points = np.vstack([t1.T.ravel(), t2.T.ravel()]).T

#         f = points[ind_to]
#         f = f.astype(int)

#         f = f * self.resolution

#         return f

#     def plot_env(self, n, path, step, planned_routes=None):
#         plt.switch_backend('agg')
#         plt.figure(figsize=(10, 8))
#         plt.cla()
#         plt.imshow(self.get_combined_belief(), cmap='gray')
#         plt.axis((0, self.ground_truth_size[1], self.ground_truth_size[0], 0))
        
#         if planned_routes:
#             for agent_id, routes in enumerate(planned_routes):
#                 for p in routes:
#                     planned_x, planned_y = [], []
#                     for coords in p:
#                         planned_x.append(coords[0])
#                         planned_y.append(coords[1])
#                     plt.plot(planned_x, planned_y, c=self.agent_colors[agent_id], linewidth=1, zorder=2, alpha=0.5)

#         for i in range(self.num_agents):
#             plt.plot(self.xPoints[i], self.yPoints[i], c=self.agent_colors[i], linewidth=2)
#             plt.plot(self.robot_positions[i][0], self.robot_positions[i][1], 'o', c=self.agent_colors[i], markersize=8)
#             plt.plot(self.xPoints[i][0], self.yPoints[i][0], 'co', markersize=8)

#         plt.scatter(self.frontiers[:, 0], self.frontiers[:, 1], c='r', s=2, zorder=3)
#         plt.suptitle('Explored ratio: {:.4g}  Total travel distance: {:.4g}'.format(self.explored_rate, self.get_total_travel_dist()))
#         plt.tight_layout()
#         filename = '{}/{}_{}_samples.png'.format(path, n, step)
#         plt.savefig(filename, dpi=150)
#         plt.close()
#         self.frame_files.append(filename)
#         print(f"Saved image: {filename}")



# iiteration error
# from scipy import spatial
# from skimage import io
# import numpy as np
# import time
# import sys
# from scipy import ndimage
# import matplotlib.pyplot as plt
# import os
# import copy
# from skimage.measure import block_reduce

# from sensor import *
# from parameter import *

# class Env():
#     def __init__(self, map_index, plot=False, test=False, num_agents=2):
#         self.test = test
#         self.num_agents = num_agents
#         if self.test:
#             self.map_dir = f'DungeonMaps/test'
#         else:
#             self.map_dir = f'DungeonMaps/train'
#         self.map_list = os.listdir(self.map_dir)
#         self.map_list.sort(reverse=True)
#         self.map_index = map_index % np.size(self.map_list)
#         self.ground_truth, self.robot_positions = self.import_ground_truth(self.map_dir + '/' + self.map_list[self.map_index])
#         self.ground_truth_size = np.shape(self.ground_truth) # (480, 640)
#         self.robot_beliefs = [np.ones(self.ground_truth_size) * 127 for _ in range(self.num_agents)] # unexplored 127
        
#         self.finish_percent = 0.985
#         self.resolution = 4
#         self.sensor_range = 180
#         self.old_robot_beliefs = copy.deepcopy(self.robot_beliefs)

#         self.plot = plot
#         self.frame_files = []
#         if self.plot:
#             # initialize the routes
#             self.xPoints = [[] for _ in range(self.num_agents)]
#             self.yPoints = [[] for _ in range(self.num_agents)]
#             for i in range(self.num_agents):
#                 self.xPoints[i].append(self.robot_positions[i][0])
#                 self.yPoints[i].append(self.robot_positions[i][1])

#         self.travel_dists = [0 for _ in range(self.num_agents)]
#         self.explored_rate = 0
#         self.route_nodes = [[] for _ in range(self.num_agents)]
#         for i in range(self.num_agents):
#             self.route_nodes[i] = [self.robot_positions[i]]
#         self.frontiers = None
#         self.downsampled_belief = None

#     def begin(self):
#         for i in range(self.num_agents):
#             self.robot_beliefs[i] = self.update_robot_belief(self.robot_positions[i], self.sensor_range, self.robot_beliefs[i], self.ground_truth)
#         self.merge_beliefs()
#         self.downsampled_belief = block_reduce(self.robot_beliefs[0].copy(), block_size=(self.resolution, self.resolution), func=np.min)

#         self.frontiers = self.find_frontier()
        
#         self.old_robot_beliefs = copy.deepcopy(self.robot_beliefs)

#     def step(self, next_node_coords):
#         for i in range(self.num_agents):
#             dist = np.linalg.norm(next_node_coords[i] - self.robot_positions[i])
#             self.travel_dists[i] += dist
#             self.robot_positions[i] = next_node_coords[i]
#             self.route_nodes[i].append(self.robot_positions[i])
#             self.robot_beliefs[i] = self.update_robot_belief(self.robot_positions[i], self.sensor_range, self.robot_beliefs[i], self.ground_truth)

#         self.merge_beliefs()
#         self.downsampled_belief = block_reduce(self.robot_beliefs[0].copy(), block_size=(self.resolution, self.resolution), func=np.min)

#         frontiers = self.find_frontier()
#         self.explored_rate = self.evaluate_exploration_rate()

#         if self.plot:
#             for i in range(self.num_agents):
#                 self.xPoints[i].append(self.robot_positions[i][0])
#                 self.yPoints[i].append(self.robot_positions[i][1])

#         self.frontiers = frontiers

#         done = self.check_done()

#         return done

#     def merge_beliefs(self):
#         merged_belief = np.ones(self.ground_truth_size) * 127
#         for belief in self.robot_beliefs:
#             merged_belief = np.minimum(merged_belief, belief)
#         for i in range(self.num_agents):
#             self.robot_beliefs[i] = merged_belief

#     def import_ground_truth(self, map_index):
#         ground_truth = (io.imread(map_index, 1) * 255).astype(int)
#         robot_locations = np.nonzero(ground_truth == 208)
#         robot_locations = np.array([np.array(robot_locations)[1, :self.num_agents], np.array(robot_locations)[0, :self.num_agents]]).T
#         ground_truth = (ground_truth > 150)
#         ground_truth = ground_truth * 254 + 1
#         return ground_truth, robot_locations

#     # ... (other methods remain the same) ...


#     def free_cells(self):
#         index = np.where(self.ground_truth == 255)
#         free = np.asarray([index[1], index[0]]).T
#         return free

#     def update_robot_belief(self, robot_position, sensor_range, robot_belief, ground_truth):
#         robot_belief = sensor_work(robot_position, sensor_range, robot_belief, ground_truth)
#         return robot_belief

#     def check_done(self):
#         done = False
#         #if self.node_utility.sum() == 0:
#         #if self.explored_rate >= 0.99 or self.node_utility.sum() == 0:
#         if np.sum(self.ground_truth == 255) - np.sum(self.robot_belief == 255) <= 250:
#             done = True
#         return done

#     def evaluate_exploration_rate(self):
#         rate = np.sum(self.robot_belief == 255) / np.sum(self.ground_truth == 255)
#         return rate

#     def calculate_new_free_area(self):
#         old_free_area = self.old_robot_belief == 255
#         current_free_area = self.robot_belief == 255

#         new_free_area = (current_free_area.astype(np.int) - old_free_area.astype(np.int)) * 255

#         return new_free_area, np.sum(old_free_area)

#     def calculate_utility_along_path(self, path, nodes_list):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

#         return unique_frontiers.shape[0]

#     def calculate_node_gain_over_path(self, node_index, path, nodes_list):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         pre_unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)
#         observable_frontiers += nodes_list[node_index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

#         return unique_frontiers.shape[0] - pre_unique_frontiers.shape[0]

#     def calculate_dist_path(self, path, node_list):
#         dist = 0
#         start = path[0]
#         end = path[-1]
#         for index in path:
#             if index == end:
#                 break
#             dist += np.linalg.norm(node_list[start].coords - node_list[index].coords)
#             start = index
#         return dist

#     def find_frontier(self):
#         y_len = self.downsampled_belief.shape[0]
#         x_len = self.downsampled_belief.shape[1]
#         mapping = self.downsampled_belief.copy()
#         belief = self.downsampled_belief.copy()
#         # 0-1 unknown area map
#         mapping = (mapping == 127) * 1
#         mapping = np.lib.pad(mapping, ((1, 1), (1, 1)), 'constant', constant_values=0)
#         fro_map = mapping[2:][:, 1:x_len + 1] + mapping[:y_len][:, 1:x_len + 1] + mapping[1:y_len + 1][:, 2:] + \
#                   mapping[1:y_len + 1][:, :x_len] + mapping[:y_len][:, 2:] + mapping[2:][:, :x_len] + mapping[2:][:,
#                                                                                                       2:] + \
#                   mapping[:y_len][:, :x_len]
#         ind_free = np.where(belief.ravel(order='F') == 255)[0]
#         ind_fron_1 = np.where(1 < fro_map.ravel(order='F'))[0]
#         ind_fron_2 = np.where(fro_map.ravel(order='F') < 8)[0]
#         ind_fron = np.intersect1d(ind_fron_1, ind_fron_2)
#         ind_to = np.intersect1d(ind_free, ind_fron)

#         map_x = x_len
#         map_y = y_len
#         x = np.linspace(0, map_x - 1, map_x)
#         y = np.linspace(0, map_y - 1, map_y)
#         t1, t2 = np.meshgrid(x, y)
#         points = np.vstack([t1.T.ravel(), t2.T.ravel()]).T

#         f = points[ind_to]
#         f = f.astype(int)

#         f = f * self.resolution

#         return f    

#     def plot_env(self, n, path, step, planned_routes=None):
#         plt.switch_backend('agg')
#         plt.cla()
#         plt.imshow(self.robot_beliefs[0], cmap='gray')
#         plt.axis((0, self.ground_truth_size[1], self.ground_truth_size[0], 0))
#         if planned_routes:
#             colors = ['r', 'g', 'b', 'y', 'c', 'm']  # Add more colors if needed
#             for i, routes in enumerate(planned_routes):
#                 for p in routes:
#                     planned_x, planned_y = [], []
#                     for coords in p:
#                         planned_x.append(coords[0])
#                         planned_y.append(coords[1])
#                     plt.plot(planned_x, planned_y, c=colors[i % len(colors)], linewidth=2, zorder=2)
        
#         for i in range(self.num_agents):
#             plt.plot(self.xPoints[i], self.yPoints[i], c=colors[i % len(colors)], linewidth=2)
#             plt.plot(self.robot_positions[i][0], self.robot_positions[i][1], 'o', color=colors[i % len(colors)], markersize=8)
#             plt.plot(self.xPoints[i][0], self.yPoints[i][0], 'co', markersize=8)
        
#         plt.scatter(self.frontiers[:, 0], self.frontiers[:, 1], c='r', s=2, zorder=3)
#         plt.suptitle('Explored ratio: {:.4g}  Total Travel distance: {:.4g}'.format(self.explored_rate, sum(self.travel_dists)))
#         plt.tight_layout()
#         plt.savefig('{}/{}_{}_samples.png'.format(path, n, step, dpi=150))
#         frame = '{}/{}_{}_samples.png'.format(path, n, step)
#         self.frame_files.append(frame)

# some error
# from scipy import spatial
# from skimage import io
# import numpy as np
# import time
# import sys
# from scipy import ndimage
# import matplotlib.pyplot as plt
# import os
# import copy
# from skimage.measure import block_reduce

# from sensor import *
# from parameter import *


# class Env:
#     def __init__(self, map_index, plot=False, test=False, num_agents=1):
#         self.test = test
#         self.num_agents = num_agents
#         if self.test:
#             self.map_dir = 'DungeonMaps/test'
#         else:
#             self.map_dir = 'DungeonMaps/train'
#         self.map_list = os.listdir(self.map_dir)
#         self.map_list.sort(reverse=True)
#         self.map_index = map_index % len(self.map_list)
#         self.ground_truth, self.robot_positions = self.import_ground_truth(self.map_dir + '/' + self.map_list[self.map_index])
#         self.ground_truth_size = np.shape(self.ground_truth)
        
#         # Debugging statements
#         print(f"Initial Robot Positions: {self.robot_positions}")
#         print(f"Type of Robot Positions: {type(self.robot_positions)}")
#         print(f"First Element Type: {type(self.robot_positions[0]) if len(self.robot_positions) > 0 else 'N/A'}")
        
#         # Ensure robot_positions is a list of tuples/lists
#         if not isinstance(self.robot_positions, list) or any(not isinstance(pos, (tuple, list)) or len(pos) != 2 for pos in self.robot_positions):
#             raise ValueError("robot_positions must be a list of coordinate tuples/lists with length 2.")
        
#         self.robot_beliefs = [np.ones(self.ground_truth_size) * 127 for _ in range(num_agents)]  # List for each agent's belief
#         self.finish_percent = 0.985
#         self.resolution = 4
#         self.sensor_range = 80
#         self.old_robot_beliefs = copy.deepcopy(self.robot_beliefs)

#         self.plot = plot
#         self.frame_files = []
#         if self.plot:
#             self.xPoints = [pos[0] for pos in self.robot_positions]
#             self.yPoints = [pos[1] for pos in self.robot_positions]

#         self.travel_dist = 0
#         self.explored_rate = 0
#         self.route_nodes = [self.robot_positions]
#         self.frontiers = None
#         self.downsampled_beliefs = [None for _ in range(num_agents)]
#     def begin(self):
#         for i in range(self.num_agents):
#             self.robot_beliefs[i] = self.update_robot_belief(self.robot_positions[i], self.sensor_range, self.robot_beliefs[i], self.ground_truth)
#             self.downsampled_beliefs[i] = block_reduce(self.robot_beliefs[i].copy(), block_size=(self.resolution, self.resolution), func=np.min)
#         self.frontiers = self.find_frontier()
#         self.old_robot_beliefs = copy.deepcopy(self.robot_beliefs)

#     def step(self, next_node_coords, agent_id):
#         dist = np.linalg.norm(next_node_coords - self.robot_positions[agent_id])

#         self.travel_dist += dist
#         self.robot_positions[agent_id] = next_node_coords
#         self.route_nodes[agent_id].append(self.robot_positions[agent_id])
#         self.robot_beliefs[agent_id] = self.update_robot_belief(self.robot_positions[agent_id], self.sensor_range, self.robot_beliefs[agent_id], self.ground_truth)
#         self.downsampled_beliefs[agent_id] = block_reduce(self.robot_beliefs[agent_id].copy(), block_size=(self.resolution, self.resolution), func=np.min)

#         self.frontiers = self.find_frontier()
#         self.explored_rate = self.evaluate_exploration_rate()

#         if self.plot:
#             self.xPoints.append(self.robot_positions[agent_id][0])
#             self.yPoints.append(self.robot_positions[agent_id][1])

#         done = self.check_done()

#         return done


#     def import_ground_truth(self, map_index):
#         ground_truth = (io.imread(map_index, 1) * 255).astype(int)
#         robot_location = np.nonzero(ground_truth == 208)
#         robot_location = np.array([np.array(robot_location)[1, 127], np.array(robot_location)[0, 127]])
#         ground_truth = (ground_truth > 150)
#         ground_truth = ground_truth * 254 + 1
#         return ground_truth, robot_location

#     def free_cells(self):
#         index = np.where(self.ground_truth == 255)
#         free = np.asarray([index[1], index[0]]).T
#         return free

#     def update_robot_belief(self, robot_position, sensor_range, robot_belief, ground_truth):
#         # Removed horizontal_fov_angle parameter
#         robot_belief = sensor_work(robot_position, sensor_range, robot_belief, ground_truth)
#         return robot_belief

#     def check_done(self, agent_id=0):
#         done = False
#         if np.sum(self.ground_truth == 255) - np.sum(self.robot_beliefs[agent_id] == 255) <= 250:
#             done = True
#         return done

#     def evaluate_exploration_rate(self, agent_id=0):
#         rate = np.sum(self.robot_beliefs[agent_id] == 255) / np.sum(self.ground_truth == 255)
#         return rate

#     def calculate_new_free_area(self, agent_id=0):
#         old_free_area = self.old_robot_beliefs[agent_id] == 255
#         current_free_area = self.robot_beliefs[agent_id] == 255

#         new_free_area = (current_free_area.astype(np.int) - old_free_area.astype(np.int)) * 255

#         return new_free_area, np.sum(old_free_area)

#     def calculate_utility_along_path(self, path, nodes_list, agent_id=0):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1, 2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

#         return unique_frontiers.shape[0]

#     def calculate_node_gain_over_path(self, node_index, path, nodes_list, agent_id=0):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1, 2)
#         pre_unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)
#         observable_frontiers += nodes_list[node_index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1, 2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

#         return unique_frontiers.shape[0] - pre_unique_frontiers.shape[0]

#     def calculate_dist_path(self, path, node_list, agent_id=0):
#         dist = 0
#         start = path[0]
#         end = path[-1]
#         for index in path:
#             if index == end:
#                 break
#             dist += np.linalg.norm(node_list[start].coords - node_list[index].coords)
#             start = index
#         return dist

#     def find_frontier(self, agent_id=0):
#         y_len = self.downsampled_beliefs[agent_id].shape[0]
#         x_len = self.downsampled_beliefs[agent_id].shape[1]
#         mapping = self.downsampled_beliefs[agent_id].copy()
#         belief = self.downsampled_beliefs[agent_id].copy()
#         # 0-1 unknown area map
#         mapping = (mapping == 127) * 1
#         mapping = np.lib.pad(mapping, ((1, 1), (1, 1)), 'constant', constant_values=0)
#         fro_map = mapping[2:][:, 1:x_len + 1] + mapping[:y_len][:, 1:x_len + 1] + mapping[1:y_len + 1][:, 2:] + \
#                   mapping[1:y_len + 1][:, :x_len] + mapping[:y_len][:, 2:] + mapping[2:][:, :x_len] + mapping[2:][:,
#                                                                                                       2:] + \
#                   mapping[:y_len][:, :x_len]
#         ind_free = np.where(belief.ravel(order='F') == 255)[0]
#         ind_fron_1 = np.where(1 < fro_map.ravel(order='F'))[0]
#         ind_fron_2 = np.where(fro_map.ravel(order='F') < 8)[0]
#         ind_fron = np.intersect1d(ind_fron_1, ind_fron_2)
#         ind_to = np.intersect1d(ind_free, ind_fron)

#         map_x = x_len
#         map_y = y_len
#         x = np.linspace(0, map_x - 1, map_x)
#         y = np.linspace(0, map_y - 1, map_y)
#         t1, t2 = np.meshgrid(x, y)
#         points = np.vstack([t1.T.ravel(), t2.T.ravel()]).T

#         f = points[ind_to]
#         f = f.astype(int)

#         f = f * self.resolution

#         return f

#     def plot_env(self, n, path, step, planned_route=None, agent_id=0):
#         plt.switch_backend('agg')
#         plt.cla()
#         plt.imshow(self.robot_beliefs[agent_id], cmap='gray')
#         plt.axis((0, self.ground_truth_size[1], self.ground_truth_size[0], 0))
#         if planned_route:
#             for p in planned_route:
#                 planned_x, planned_y = [], []
#                 for coords in p:
#                     planned_x.append(coords[0])
#                     planned_y.append(coords[1])
#                 plt.plot(planned_x, planned_y, c='r', linewidth=2, zorder=2)
#         plt.plot(self.xPoints[agent_id], self.yPoints[agent_id], 'b', linewidth=2)
#         plt.plot(self.robot_positions[agent_id][0], self.robot_positions[agent_id][1], 'mo', markersize=8)
#         plt.plot(self.xPoints[agent_id][0], self.yPoints[agent_id][0], 'co', markersize=8)
#         plt.scatter(self.frontiers[agent_id][:, 0], self.frontiers[agent_id][:, 1], c='r', s=2, zorder=3)
#         plt.suptitle('Explored ratio: {:.4g}  Travel distance: {:.4g}'.format(self.explored_rates[agent_id], self.travel_dists[agent_id]))
#         plt.tight_layout()
#         plt.savefig('{}/{}_{}_samples.png'.format(path, n, step, dpi=150))
#         frame = '{}/{}_{}_samples.png'.format(path, n, step)
#         self.frame_files.append(frame)





# # from scipy import spatial
# # from skimage import io
# # import numpy as np
# # import time
# # import sys
# # from scipy import ndimage
# import matplotlib.pyplot as plt
# import os
# import copy
# from skimage.measure import block_reduce

# from sensor import *
# from parameter import *

# class Env():
#     def __init__(self, map_index, plot=False, test=False, num_agents=3):
#         self.test = test
#         if self.test:
#             self.map_dir = f'DungeonMaps/test'
#         else:
#             self.map_dir = f'DungeonMaps/train'
#         self.map_list = os.listdir(self.map_dir)
#         self.map_list.sort(reverse=True)
#         self.map_index = map_index % np.size(self.map_list)
#         self.ground_truth, self.robot_positions = self.import_ground_truth(self.map_dir + '/' + self.map_list[self.map_index], num_agents)
#         self.ground_truth_size = np.shape(self.ground_truth) # (480, 640)
#         self.robot_beliefs = [np.ones(self.ground_truth_size) * 127 for _ in range(num_agents)] # unexplored 127
        
#         self.finish_percent = 0.985
#         self.resolution = 4
#         self.sensor_range = 80
#         self.old_robot_beliefs = copy.deepcopy(self.robot_beliefs)

#         self.plot = plot
#         self.frame_files = []
#         if self.plot:
#             # initialize the route
#             self.xPoints = [[] for _ in range(num_agents)]
#             self.yPoints = [[] for _ in range(num_agents)]
#             for i in range(num_agents):
#                 self.xPoints[i].append(self.robot_positions[i][0])
#                 self.yPoints[i].append(self.robot_positions[i][1])

#         self.travel_dists = [0 for _ in range(num_agents)]
#         self.explored_rate = 0
#         self.route_nodes = [[pos] for pos in self.robot_positions]
#         self.frontiers = None
#         self.downsampled_belief = None
#         self.num_agents = num_agents

#     def import_ground_truth(self, map_index, num_agents):
#         ground_truth = (io.imread(map_index, 1) * 255).astype(int)
#         robot_locations = np.nonzero(ground_truth == 208)
#         robot_locations = np.array([np.array(robot_locations)[1, :num_agents], np.array(robot_locations)[0, :num_agents]]).T
#         ground_truth = (ground_truth > 150)
#         ground_truth = ground_truth * 254 + 1
#         return ground_truth, robot_locations

#     def begin(self):
#         for i in range(self.num_agents):
#             self.robot_beliefs[i] = self.update_robot_belief(self.robot_positions[i], self.sensor_range, self.robot_beliefs[i], self.ground_truth)
#         self.downsampled_belief = block_reduce(self.merge_beliefs(), block_size=(self.resolution, self.resolution), func=np.min)

#         self.frontiers = self.find_frontier()
        
#         self.old_robot_beliefs = copy.deepcopy(self.robot_beliefs)

#     def step(self, next_node_coords_list):
#         for i in range(self.num_agents):
#             dist = np.linalg.norm(next_node_coords_list[i] - self.robot_positions[i])
#             self.travel_dists[i] += dist
#             self.robot_positions[i] = next_node_coords_list[i]
#             self.route_nodes[i].append(self.robot_positions[i])
#             self.robot_beliefs[i] = self.update_robot_belief(self.robot_positions[i], self.sensor_range, self.robot_beliefs[i], self.ground_truth)

#         self.downsampled_belief = block_reduce(self.merge_beliefs(), block_size=(self.resolution, self.resolution), func=np.min)

#         frontiers = self.find_frontier()
#         self.explored_rate = self.evaluate_exploration_rate()

#         if self.plot:
#             for i in range(self.num_agents):
#                 self.xPoints[i].append(self.robot_positions[i][0])
#                 self.yPoints[i].append(self.robot_positions[i][1])

#         self.frontiers = frontiers

#         done = self.check_done()

#         return done

#     def merge_beliefs(self):
#         merged_belief = np.ones(self.ground_truth_size) * 127
#         for belief in self.robot_beliefs:
#             merged_belief = np.minimum(merged_belief, belief)
#         return merged_belief

#     def update_robot_belief(self, robot_position, sensor_range, robot_belief, ground_truth):
#         robot_belief = sensor_work(robot_position, sensor_range, robot_belief, ground_truth)
#         return robot_belief

#     def check_done(self):
#         done = False
#         if np.sum(self.ground_truth == 255) - np.sum(self.merge_beliefs() == 255) <= 250:
#             done = True
#         return done

#     def evaluate_exploration_rate(self):
#         rate = np.sum(self.merge_beliefs() == 255) / np.sum(self.ground_truth == 255)
#         return rate

#     def calculate_new_free_area(self):
#         old_free_area = np.any([belief == 255 for belief in self.old_robot_beliefs], axis=0)
#         current_free_area = np.any([belief == 255 for belief in self.robot_beliefs], axis=0)

#         new_free_area = (current_free_area.astype(np.int) - old_free_area.astype(np.int)) * 255

#         return new_free_area, np.sum(old_free_area)

#     def calculate_utility_along_path(self, path, nodes_list, agent_id):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

#         return unique_frontiers.shape[0]

#     def calculate_node_gain_over_path(self, node_index, path, nodes_list, agent_id):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         pre_unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)
#         observable_frontiers += nodes_list[node_index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

#         return unique_frontiers.shape[0] - pre_unique_frontiers.shape[0]

#     def calculate_dist_path(self, path, node_list):
#         dist = 0
#         start = path[0]
#         end = path[-1]
#         for index in path:
#             if index == end:
#                 break
#             dist += np.linalg.norm(node_list[start].coords - node_list[index].coords)
#             start = index
#         return dist

#     def find_frontier(self):
#         y_len = self.downsampled_belief.shape[0]
#         x_len = self.downsampled_belief.shape[1]
#         mapping = self.downsampled_belief.copy()
#         belief = self.downsampled_belief.copy()
#         # 0-1 unknown area map
#         mapping = (mapping == 127) * 1
#         mapping = np.lib.pad(mapping, ((1, 1), (1, 1)), 'constant', constant_values=0)
#         fro_map = mapping[2:][:, 1:x_len + 1] + mapping[:y_len][:, 1:x_len + 1] + mapping[1:y_len + 1][:, 2:] + \
#                   mapping[1:y_len + 1][:, :x_len] + mapping[:y_len][:, 2:] + mapping[2:][:, :x_len] + mapping[2:][:,
#                                                                                                       2:] + \
#                   mapping[:y_len][:, :x_len]
#         ind_free = np.where(belief.ravel(order='F') == 255)[0]
#         ind_fron_1 = np.where(1 < fro_map.ravel(order='F'))[0]
#         ind_fron_2 = np.where(fro_map.ravel(order='F') < 8)[0]
#         ind_fron = np.intersect1d(ind_fron_1, ind_fron_2)
#         ind_to = np.intersect1d(ind_free, ind_fron)

#         map_x = x_len
#         map_y = y_len
#         x = np.linspace(0, map_x - 1, map_x)
#         y = np.linspace(0, map_y - 1, map_y)
#         t1, t2 = np.meshgrid(x, y)
#         points = np.vstack([t1.T.ravel(), t2.T.ravel()]).T

#         f = points[ind_to]
#         f = f.astype(int)

#         f = f * self.resolution

#         return f

#     def plot_env(self, n, path, step, planned_routes=None):
#         plt.switch_backend('agg')
#         plt.cla()
#         plt.imshow(self.merge_beliefs(), cmap='gray')
#         plt.axis((0, self.ground_truth_size[1], self.ground_truth_size[0], 0))
#         if planned_routes:
#             colors = ['r', 'g', 'b', 'c', 'm', 'y']
#             for i, routes in enumerate(planned_routes):
#                 for p in routes:
#                     planned_x, planned_y = [], []
#                     for coords in p:
#                         planned_x.append(coords[0])
#                         planned_y.append(coords[1])
#                     plt.plot(planned_x, planned_y, c=colors[i % len(colors)], linewidth=2, zorder=2)
        
#         for i in range(self.num_agents):
#             plt.plot(self.xPoints[i], self.yPoints[i], c=colors[i % len(colors)], linewidth=2)
#             plt.plot(self.robot_positions[i][0], self.robot_positions[i][1], 'o', c=colors[i % len(colors)], markersize=8)
#             plt.plot(self.xPoints[i][0], self.yPoints[i][0], 'co', markersize=8)
        
#         plt.scatter(self.frontiers[:, 0], self.frontiers[:, 1], c='r', s=2, zorder=3)
#         plt.suptitle('Explored ratio: {:.4g}  Travel distances: {}'.format(self.explored_rate, self.travel_dists))
#         plt.tight_layout()
#         plt.savefig('{}/{}_{}_samples.png'.format(path, n, step), dpi=150)
#         frame = '{}/{}_{}_samples.png'.format(path, n, step)
#         self.frame_files.append(frame)




# from scipy import spatial
# from skimage import io
# import numpy as np
# import time
# import sys
# from scipy import ndimage
# import matplotlib.pyplot as plt
# import os
# import copy
# from skimage.measure import block_reduce

# from sensor import *
# from parameter import *

# class Env():
#     def __init__(self, map_index, num_agents=2, plot=False, test=False):
#         self.test = test
#         if self.test:
#             self.map_dir = f'DungeonMaps/test'
#         else:
#             self.map_dir = f'DungeonMaps/train'
#         self.map_list = os.listdir(self.map_dir)
#         self.map_list.sort(reverse=True)
#         self.map_index = map_index % np.size(self.map_list)
#         self.ground_truth, self.robot_positions = self.import_ground_truth(self.map_dir + '/' + self.map_list[self.map_index], num_agents)
#         self.ground_truth_size = np.shape(self.ground_truth) # (480, 640)
        
#         # Initialize belief maps, routes, travel distances, and exploration rates for each agent
#         self.robot_beliefs = [np.ones(self.ground_truth_size) * 127 for _ in range(num_agents)] # unexplored 127
#         self.old_robot_beliefs = [copy.deepcopy(belief) for belief in self.robot_beliefs]
        
#         self.finish_percent = 0.985
#         self.resolution = 4
#         self.sensor_range = 80

#         self.num_agents = num_agents
#         self.plot = plot
#         self.frame_files = []
        
#         # Initialize plotting variables for each agent
#         if self.plot:
#             self.xPoints = [[] for _ in range(num_agents)]
#             self.yPoints = [[] for _ in range(num_agents)]
#             for i in range(num_agents):
#                 self.xPoints[i].append(self.robot_positions[i][0])
#                 self.yPoints[i].append(self.robot_positions[i][1])

#         self.travel_dists = [0 for _ in range(num_agents)]
#         self.explored_rates = [0 for _ in range(num_agents)]
#         self.route_nodes = [[self.robot_positions[i]] for i in range(num_agents)]
#         self.frontiers = None
#         self.downsampled_beliefs = None

#     def begin(self):
#         for i in range(self.num_agents):
#             self.robot_beliefs[i] = self.update_robot_belief(self.robot_positions[i], self.sensor_range, self.robot_beliefs[i], self.ground_truth)
#         self.downsampled_beliefs = [block_reduce(belief.copy(), block_size=(self.resolution, self.resolution), func=np.min) for belief in self.robot_beliefs]

#         self.frontiers = self.find_frontier()
        
#         self.old_robot_beliefs = [copy.deepcopy(belief) for belief in self.robot_beliefs]

#     def step(self, agent_index, next_node_coords):
#         dist = np.linalg.norm(next_node_coords - self.robot_positions[agent_index])

#         self.travel_dists[agent_index] += dist
#         self.robot_positions[agent_index] = next_node_coords
#         self.route_nodes[agent_index].append(self.robot_positions[agent_index])
#         self.robot_beliefs[agent_index] = self.update_robot_belief(self.robot_positions[agent_index], self.sensor_range, self.robot_beliefs[agent_index],
#                                                      self.ground_truth)
#         self.downsampled_beliefs[agent_index] = block_reduce(self.robot_beliefs[agent_index].copy(), block_size=(self.resolution, self.resolution),
#                                                func=np.min)

#         self.frontiers = self.find_frontier()
#         self.explored_rates[agent_index] = self.evaluate_exploration_rate(agent_index)

#         if self.plot:
#             self.xPoints[agent_index].append(self.robot_positions[agent_index][0])
#             self.yPoints[agent_index].append(self.robot_positions[agent_index][1])

#         done = self.check_done()

#         return done

#     def import_ground_truth(self, map_index, num_agents):
#         ground_truth = (io.imread(map_index, 1) * 255).astype(int)
#         robot_locations = np.nonzero(ground_truth == 208)
#         robot_positions = []
#         for i in range(num_agents):
#             x_idx = 127 + i
#             y_idx = 127 + i
#             robot_positions.append(np.array([np.array(robot_locations)[1, x_idx], np.array(robot_locations)[0, y_idx]]))
#         ground_truth = (ground_truth > 150)
#         ground_truth = ground_truth * 254 + 1
#         return ground_truth, robot_positions

#     def update_robot_belief(self, robot_position, sensor_range, robot_belief, ground_truth):
#         robot_belief = sensor_work(robot_position, sensor_range, robot_belief, ground_truth)
#         return robot_belief

#     def check_done(self):
#         done = False
#         # Check if all agents have finished exploring
#         if all(np.sum(self.ground_truth == 255) - np.sum(belief == 255) <= 250 for belief in self.robot_beliefs):
#             done = True
#         return done

#     def evaluate_exploration_rate(self, agent_index):
#         rate = np.sum(self.robot_beliefs[agent_index] == 255) / np.sum(self.ground_truth == 255)
#         return rate

#     def calculate_new_free_area(self, agent_index):
#         old_free_area = self.old_robot_beliefs[agent_index] == 255
#         current_free_area = self.robot_beliefs[agent_index] == 255

#         new_free_area = (current_free_area.astype(np.int) - old_free_area.astype(np.int)) * 255

#         return new_free_area, np.sum(old_free_area)

#     def find_frontier(self):
#         y_len = self.downsampled_beliefs[0].shape[0]
#         x_len = self.downsampled_beliefs[0].shape[1]
#         mapping = self.downsampled_beliefs[0].copy()
#         belief = self.downsampled_beliefs[0].copy()
#         # 0-1 unknown area map
#         mapping = (mapping == 127) * 1
#         mapping = np.lib.pad(mapping, ((1, 1), (1, 1)), 'constant', constant_values=0)
#         fro_map = mapping[2:][:, 1:x_len + 1] + mapping[:y_len][:, 1:x_len + 1] + mapping[1:y_len + 1][:, 2:] + \
#                   mapping[1:y_len + 1][:, :x_len] + mapping[:y_len][:, 2:] + mapping[2:][:, :x_len] + mapping[2:][:,
#                                                                                                       2:] + \
#                   mapping[:y_len][:, :x_len]
#         ind_free = np.where(belief.ravel(order='F') == 255)[0]
#         ind_fron_1 = np.where(1 < fro_map.ravel(order='F'))[0]
#         ind_fron_2 = np.where(fro_map.ravel(order='F') < 8)[0]
#         ind_fron = np.intersect1d(ind_fron_1, ind_fron_2)
#         ind_to = np.intersect1d(ind_free, ind_fron)

#         map_x = x_len
#         map_y = y_len
#         x = np.linspace(0, map_x - 1, map_x)
#         y = np.linspace(0, map_y - 1, map_y)
#         t1, t2 = np.meshgrid(x, y)
#         points = np.vstack([t1.T.ravel(), t2.T.ravel()]).T

#         f = points[ind_to]
#         f = f.astype(int)

#         f = f * self.resolution

#         return f

#     def plot_env(self, n, path, step, planned_route=None):
#         plt.switch_backend('agg')
#         plt.cla()
#         plt.imshow(self.robot_beliefs[0], cmap='gray')  # Assuming plotting for the first agent
#         plt.axis((0, self.ground_truth_size[1], self.ground_truth_size[0], 0))
#         if planned_route:
#             for p in planned_route:
#                 planned_x, planned_y = [], []
#                 for coords in p:
#                     planned_x.append(coords[0])
#                     planned_y.append(coords[1])
#                 plt.plot(planned_x, planned_y, c='r', linewidth=2, zorder=2)
#         for i in range(self.num_agents):
#             plt.plot(self.xPoints[i], self.yPoints[i], 'b', linewidth=2)
#             plt.plot(self.robot_positions[i][0], self.robot_positions[i][1], 'mo', markersize=8)
#         plt.plot(self.xPoints[0][0], self.yPoints[0][0], 'co', markersize=8)
#         plt.scatter(self.frontiers[:, 0], self.frontiers[:, 1], c='r', s=2, zorder=3)
#         plt.suptitle('Explored ratio: {:.4g}  Travel distance: {:.4g}'.format(self.explored_rates[0], self.travel_dists[0]))
#         plt.tight_layout()
#         plt.savefig('{}/{}_{}_samples.png'.format(path, n, step, dpi=150))

#     def save_env(self, path):
#         for i in range(self.num_agents):
#             self.frame_files.append(f"{path}_agent{i}_belief.png")
#             io.imsave(self.frame_files[-1], self.robot_beliefs[i].astype(np.uint8))

#         return self.frame_files



# from scipy import spatial
# from skimage import io
# import numpy as np
# import time
# import sys
# from scipy import ndimage
# import matplotlib.pyplot as plt
# import os
# import copy
# from skimage.measure import block_reduce

# from sensor import *
# from parameter import *


# class Env():
#     def __init__(self, map_index, num_agents=1, plot=False, test=False):
#         self.num_agents = num_agents
#         self.agent_positions = [None] * num_agents  # Initialize list for agent positions

#         # Other initializations
#         self.test = test
#         if self.test:
#             self.map_dir = f'DungeonMaps/test'
#         else:
#             self.map_dir = f'DungeonMaps/train'
#         self.map_list = os.listdir(self.map_dir)
#         self.map_list.sort(reverse=True)
#         self.map_index = map_index % np.size(self.map_list)
#         self.ground_truth, self.agent_positions = self.import_ground_truth(self.map_dir + '/' + self.map_list[self.map_index])
#         self.ground_truth_size = np.shape(self.ground_truth)
#         self.robot_belief = np.ones(self.ground_truth_size) * 127
        
#         self.finish_percent = 0.985
#         self.resolution = 4
#         self.sensor_range = 80
#         self.old_robot_belief = copy.deepcopy(self.robot_belief)

#         self.plot = plot
#         self.frame_files = []
#         if self.plot:
#             # initialize the route
#             self.xPoints = [self.agent_positions[0][0]]  # Assuming at least one agent
#             self.yPoints = [self.agent_positions[0][1]]

#         self.travel_dist = 0
#         self.explored_rate = 0
#         self.route_node = [self.agent_positions[0]]
#         self.frontiers = None
#         self.downsampled_belief = None

#     def import_ground_truth(self, map_index):
#         # Same as before, but ensure it initializes self.agent_positions properly
#         ground_truth = (io.imread(map_index, 1) * 255).astype(int)
#         robot_locations = np.nonzero(ground_truth == 208)
#         self.agent_positions = [np.array([robot_locations[1][idx], robot_locations[0][idx]]) for idx in range(self.num_agents)]
#         ground_truth = (ground_truth > 150)
#         ground_truth = ground_truth * 254 + 1
#         return ground_truth, self.agent_positions


#     def begin(self):
#         for pos in self.agent_positions:
#             self.robot_belief = self.update_robot_belief(pos, self.sensor_range, self.robot_belief, self.ground_truth)
#         self.downsampled_belief = block_reduce(self.robot_belief.copy(), block_size=(self.resolution, self.resolution), func=np.min)

#         self.frontiers = self.find_frontier()
        
#         self.old_robot_belief = copy.deepcopy(self.robot_belief)

#     def step(self, next_node_coords):
#         distances = [np.linalg.norm(next_node_coords[i] - self.agent_positions[i]) for i in range(self.num_agents)]
#         self.travel_dist += sum(distances)
#         self.agent_positions = next_node_coords
#         self.route_nodes.append(self.agent_positions)
        
#         for i in range(self.num_agents):
#             self.robot_belief = self.update_robot_belief(self.agent_positions[i], self.sensor_range, self.robot_belief, self.ground_truth)
#         self.downsampled_belief = block_reduce(self.robot_belief.copy(), block_size=(self.resolution, self.resolution), func=np.min)

#         frontiers = self.find_frontier()
#         self.explored_rate = self.evaluate_exploration_rate()

#         if self.plot:
#             self.xPoints = [pos[0] for pos in self.agent_positions]
#             self.yPoints = [pos[1] for pos in self.agent_positions]

#         self.frontiers = frontiers
#         done = self.check_done()

#         return done

#     # def import_ground_truth(self, map_index):
#     #     # occupied 1, free 255, unexplored 127

#     #     ground_truth = (io.imread(map_index, 1) * 255).astype(int)
#     #     robot_locations = np.nonzero(ground_truth == 208)
#     #     if self.num_agents == 1:
#     #         robot_positions = [np.array([robot_locations[1][127], robot_locations[0][127]])]
#     #     else:
#     #         robot_positions = [np.array([robot_locations[1][idx], robot_locations[0][idx]]) for idx in range(self.num_agents)]
#     #     ground_truth = (ground_truth > 150)
#     #     ground_truth = ground_truth * 254 + 1
#     #     return ground_truth, robot_positions

#     def free_cells(self):
#         index = np.where(self.ground_truth == 255)
#         free = np.asarray([index[1], index[0]]).T
#         return free

#     def update_robot_belief(self, robot_position, sensor_range, robot_belief, ground_truth):
#         robot_belief = sensor_work(robot_position, sensor_range, robot_belief, ground_truth)
#         return robot_belief

#     def check_done(self):
#         done = False
#         if np.sum(self.ground_truth == 255) - np.sum(self.robot_belief == 255) <= 250:
#             done = True
#         return done

#     def evaluate_exploration_rate(self):
#         rate = np.sum(self.robot_belief == 255) / np.sum(self.ground_truth == 255)
#         return rate

#     def calculate_new_free_area(self):
#         old_free_area = self.old_robot_belief == 255
#         current_free_area = self.robot_belief == 255

#         new_free_area = (current_free_area.astype(np.int) - old_free_area.astype(np.int)) * 255

#         return new_free_area, np.sum(old_free_area)

#     def calculate_utility_along_path(self, path, nodes_list):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1, 2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1] * 1j)

#         return unique_frontiers.shape[0]

#     def calculate_node_gain_over_path(self, node_index, path, nodes_list):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1, 2)
#         pre_unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1] * 1j)
#         observable_frontiers += nodes_list[node_index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1, 2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1] * 1j)

#         return unique_frontiers.shape[0] - pre_unique_frontiers.shape[0]

#     def calculate_dist_path(self, path, node_list):
#         dist = 0
#         start = path[0]
#         end = path[-1]
#         for index in path:
#             if index == end:
#                 break
#             dist += np.linalg.norm(node_list[start].coords - node_list[index].coords)
#             start = index
#         return dist

#     def find_frontier(self):
#         y_len = self.downsampled_belief.shape[0]
#         x_len = self.downsampled_belief.shape[1]
#         mapping = self.downsampled_belief.copy()
#         belief = self.downsampled_belief.copy()
#         # 0-1 unknown area map
#         mapping = (mapping == 127) * 1
#         mapping = np.lib.pad(mapping, ((1, 1), (1, 1)), 'constant', constant_values=0)
#         fro_map = mapping[2:][:, 1:x_len + 1] + mapping[:y_len][:, 1:x_len + 1] + mapping[1:y_len + 1][:, 2:] + \
#                   mapping[1:y_len + 1][:, :x_len] + mapping[:y_len][:, 2:] + mapping[2:][:, :x_len] + mapping[2:][:,
#                                                                                                       2:] + \
#                   mapping[:y_len][:, :x_len]
#         ind_free = np.where(belief.ravel(order='F') == 255)[0]
#         ind_fron_1 = np.where(1 < fro_map.ravel(order='F'))[0]
#         ind_fron_2 = np.where(fro_map.ravel(order='F') < 8)[0]
#         ind_fron = np.intersect1d(ind_fron_1, ind_fron_2)
#         ind_to = np.intersect1d(ind_free, ind_fron)

#         map_x = x_len
#         map_y = y_len
#         x = np.linspace(0, map_x - 1, map_x)
#         y = np.linspace(0, map_y - 1, map_y)
#         t1, t2 = np.meshgrid(x, y)
#         points = np.vstack([t1.T.ravel(), t2.T.ravel()]).T

#         f = points[ind_to]
#         f = f.astype(int)

#         f = f * self.resolution

#         return f

#     def plot_env(self, n, path, step, planned_route=None):
#         plt.switch_backend('agg')
#         plt.cla()
#         plt.imshow(self.robot_belief, cmap='gray')
#         plt.axis((0, self.ground_truth_size[1], self.ground_truth_size[0], 0))
#         if planned_route:
#             for p in planned_route:
#                 planned_x, planned_y = [], []
#                 for coords in p:
#                     planned_x.append(coords[0])
#                     planned_y.append(coords[1])
#                 plt.plot(planned_x, planned_y, c='r', linewidth=2, zorder=2)
#         plt.plot(self.xPoints, self.yPoints, 'b', linewidth=2)
#         plt.plot(self.agent_positions[0][0], self.agent_positions[0][1], 'mo', markersize=8)
#         plt.plot(self.xPoints[0], self.yPoints[0], 'co', markersize=8)
#         plt.scatter(self.frontiers[:, 0], self.frontiers[:, 1], c='r', s=2, zorder=3)
#         plt.suptitle('Explored ratio: {:.4g}  Travel distance: {:.4g}'.format(self.explored_rate, self.travel_dist))
#         plt.tight_layout()
#         plt.savefig('{}/{}_{}_samples.png'.format(path, n, step), dpi=150)
#         frame = '{}/{}_{}_samples.png'.format(path, n, step)
#         self.frame_files.append(frame)




#best work multi
# from scipy import spatial
# from skimage import io
# import numpy as np
# import time
# import sys
# from scipy import ndimage
# import matplotlib.pyplot as plt
# import os
# import copy
# from skimage.measure import block_reduce

# from sensor import *
# from parameter import *

# class Env():
#     def __init__(self, map_index, plot=False, test=False, num_agents=2):
        
#         self.test = test
#         if self.test:
#             self.map_dir = f'DungeonMaps/easy'
#         else:
#             self.map_dir = f'DungeonMaps/train'
#         self.map_list = os.listdir(self.map_dir)
#         self.map_list.sort(reverse=True)
#         self.map_index = map_index % np.size(self.map_list)
#         self.ground_truth, self.start_position = self.import_ground_truth(self.map_dir + '/' + self.map_list[self.map_index])
#         self.ground_truth_size = np.shape(self.ground_truth) # (480, 640)
        
#         self.num_agents = num_agents
#         self.robot_positions = self.generate_start_positions(num_agents)
#         self.robot_beliefs = [np.ones(self.ground_truth_size) * 127 for _ in range(num_agents)]  # unexplored 127
        
#         self.finish_percent = 0.985
#         self.resolution = 4
#         self.sensor_range = 80
#         self.old_robot_beliefs = [copy.deepcopy(belief) for belief in self.robot_beliefs]

#         self.plot = plot
#         self.frame_files = []
#         if self.plot:
#             # initialize the routes
#             self.xPoints = [[pos[0]] for pos in self.robot_positions]
#             self.yPoints = [[pos[1]] for pos in self.robot_positions]

#         self.travel_dists = [0 for _ in range(num_agents)]
#         self.explored_rate = 0
#         self.route_nodes = [[pos.copy()] for pos in self.robot_positions]
#         self.frontiers = None
#         self.downsampled_belief = None

#     def generate_start_positions(self, num_agents):
#         positions = [self.start_position.copy()]
#         for _ in range(1, num_agents):
#             new_pos = self.find_new_start_position(positions)
#             positions.append(new_pos)
#         return positions

#     def find_new_start_position(self, existing_positions, max_attempts=100):
#         free_cells = self.free_cells()
#         for _ in range(max_attempts):
#             candidate = free_cells[np.random.randint(len(free_cells))]
#             if all(np.linalg.norm(candidate - pos) > 20 for pos in existing_positions):
#                 return candidate
        
#         # If we couldn't find a position after max_attempts, just return a random free cell
#         return free_cells[np.random.randint(len(free_cells))]

#     def begin(self):
#         for i in range(self.num_agents):
#             self.robot_beliefs[i] = self.update_robot_belief(self.robot_positions[i], self.sensor_range, self.robot_beliefs[i], self.ground_truth)
        
#         combined_belief = np.min(self.robot_beliefs, axis=0)
#         self.downsampled_belief = block_reduce(combined_belief.copy(), block_size=(self.resolution, self.resolution), func=np.min)

#         self.frontiers = self.find_frontier()
        
#         self.old_robot_beliefs = [copy.deepcopy(belief) for belief in self.robot_beliefs]

#     def step(self, next_node_coords):
#         for i in range(self.num_agents):
#             dist = np.linalg.norm(next_node_coords[i] - self.robot_positions[i])
#             self.travel_dists[i] += dist
#             self.robot_positions[i] = next_node_coords[i]
#             self.route_nodes[i].append(self.robot_positions[i])
#             self.robot_beliefs[i] = self.update_robot_belief(self.robot_positions[i], self.sensor_range, self.robot_beliefs[i], self.ground_truth)

#         combined_belief = np.min(self.robot_beliefs, axis=0)
#         self.downsampled_belief = block_reduce(combined_belief.copy(), block_size=(self.resolution, self.resolution), func=np.min)

#         frontiers = self.find_frontier()
#         self.explored_rate = self.evaluate_exploration_rate()

#         if self.plot:
#             for i in range(self.num_agents):
#                 self.xPoints[i].append(self.robot_positions[i][0])
#                 self.yPoints[i].append(self.robot_positions[i][1])

#         self.frontiers = frontiers

#         done = self.check_done()

#         return done

#     def import_ground_truth(self, map_index):
#         # occupied 1, free 255, unexplored 127
#         ground_truth = (io.imread(map_index, 1) * 255).astype(int)
#         robot_location = np.nonzero(ground_truth == 208)
#         robot_location = np.array([np.array(robot_location)[1, 127], np.array(robot_location)[0, 127]])
#         ground_truth = (ground_truth > 150)
#         ground_truth = ground_truth * 254 + 1
#         return ground_truth, robot_location

#     def free_cells(self):
#         index = np.where(self.ground_truth == 255)
#         free = np.asarray([index[1], index[0]]).T
#         return free

#     def update_robot_belief(self, robot_position, sensor_range, robot_belief, ground_truth):
#         robot_belief = sensor_work(robot_position, sensor_range, robot_belief, ground_truth)
#         return robot_belief

#     def check_done(self):
#         done = False
#         if np.sum(self.ground_truth == 255) - np.sum(np.min(self.robot_beliefs, axis=0) == 255) <= 250:
#             done = True
#         return done

#     def evaluate_exploration_rate(self):
#         combined_belief = np.min(self.robot_beliefs, axis=0)
#         rate = np.sum(combined_belief == 255) / np.sum(self.ground_truth == 255)
#         return rate

#     def calculate_new_free_area(self):
#         old_free_area = np.min(self.old_robot_beliefs, axis=0) == 255
#         current_free_area = np.min(self.robot_beliefs, axis=0) == 255

#         new_free_area = (current_free_area.astype(np.int) - old_free_area.astype(np.int)) * 255

#         return new_free_area, np.sum(old_free_area)

#     def calculate_utility_along_path(self, path, nodes_list):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

#         return unique_frontiers.shape[0]

#     def calculate_node_gain_over_path(self, node_index, path, nodes_list):
#         observable_frontiers = []
#         for index in path:
#             observable_frontiers += nodes_list[index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         pre_unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)
#         observable_frontiers += nodes_list[node_index].observable_frontiers
#         np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
#         unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

#         return unique_frontiers.shape[0] - pre_unique_frontiers.shape[0]

#     def calculate_dist_path(self, path, node_list):
#         dist = 0
#         start = path[0]
#         end = path[-1]
#         for index in path:
#             if index == end:
#                 break
#             dist += np.linalg.norm(node_list[start].coords - node_list[index].coords)
#             start = index
#         return dist

#     # def find_frontier(self):
#     #     y_len = self.downsampled_belief.shape[0]
#     #     x_len = self.downsampled_belief.shape[1]
#     #     mapping = self.downsampled_belief.copy()
#     #     belief = self.downsampled_belief.copy()
#     #     # 0-1 unknown area map
#     #     mapping = (mapping == 127) * 1
#     #     mapping = np.lib.pad(mapping, ((1, 1), (1, 1)), 'constant', constant_values=0)
#     #     fro_map = mapping[2:][:, 1:x_len + 1] + mapping[:y_len][:, 1:x_len + 1] + mapping[1:y_len + 1][:, 2:] + \
#     #               mapping[1:y_len + 1][:, :x_len] + mapping[:y_len][:, 2:] + mapping[2:][:, :x_len] + mapping[2:][:,
#     #                                                                                                   2:] + \
#     #               mapping[:y_len][:, :x_len]
#     #     ind_free = np.where(belief.ravel(order='F') == 255)[0]
#     #     ind_fron_1 = np.where(1 < fro_map.ravel(order='F'))[0]
#     #     ind_fron_2 = np.where(fro_map.ravel(order='F') < 8)[0]
#     #     ind_fron = np.intersect1d(ind_fron_1, ind_fron_2)
#     #     ind_to = np.intersect1d(ind_free, ind_fron)

#     #     map_x = x_len
#     #     map_y = y_len
#     #     x = np.linspace(0, map_x - 1, map_x)
#     #     y = np.linspace(0, map_y - 1, map_y)
#     #     t1, t2 = np.meshgrid(x, y)
#     #     points = np.vstack([t1.T.ravel(), t2.T.ravel()]).T

#     #     f = points[ind_to]
#     #     f = f.astype(int)

#     #     f = f * self.resolution

#     #     return f


#     def find_frontier(self):
#         y_len = self.downsampled_belief.shape[0]
#         x_len = self.downsampled_belief.shape[1]
#         mapping = self.downsampled_belief.copy()
#         belief = self.downsampled_belief.copy()
#         # 0-1 unknown area map
#         mapping = (mapping == 127) * 1
#         mapping = np.lib.pad(mapping, ((1, 1), (1, 1)), 'constant', constant_values=0)
#         fro_map = mapping[2:][:, 1:x_len + 1] + mapping[:y_len][:, 1:x_len + 1] + mapping[1:y_len + 1][:, 2:] + \
#                   mapping[1:y_len + 1][:, :x_len] + mapping[:y_len][:, 2:] + mapping[2:][:, :x_len] + mapping[2:][:,
#                                                                                                       2:] + \
#                   mapping[:y_len][:, :x_len]
#         ind_free = np.where(belief.ravel(order='F') == 255)[0]
#         ind_fron_1 = np.where(1 < fro_map.ravel(order='F'))[0]
#         ind_fron_2 = np.where(fro_map.ravel(order='F') < 8)[0]
#         ind_fron = np.intersect1d(ind_fron_1, ind_fron_2)
#         ind_to = np.intersect1d(ind_free, ind_fron)

#         map_x = x_len
#         map_y = y_len
#         x = np.linspace(0, map_x - 1, map_x)
#         y = np.linspace(0, map_y - 1, map_y)
#         t1, t2 = np.meshgrid(x, y)
#         points = np.vstack([t1.T.ravel(), t2.T.ravel()]).T

#         f = points[ind_to]
#         f = f.astype(int)

#         f = f * self.resolution

#         return f
    
#     # def reset(self):
#     #  self.ground_truth, self.start_position = self.import_ground_truth(self.map_dir + '/' + self.map_list[self.map_index])
#     #  self.robot_positions = self.generate_start_positions(self.num_agents)
#     #  self.robot_beliefs = [np.ones(self.ground_truth_size) * 127 for _ in range(self.num_agents)]
#     #  self.old_robot_beliefs = [copy.deepcopy(belief) for belief in self.robot_beliefs]
#     #  self.travel_dists = [0 for _ in range(self.num_agents)]
#     #  self.explored_rate = 0
#     #  self.route_nodes = [[pos.copy()] for pos in self.robot_positions]
#     #  self.frontiers = None
#     #  self.downsampled_belief = None
    
#     #  if self.plot:
#     #     self.xPoints = [[pos[0]] for pos in self.robot_positions]
#     #     self.yPoints = [[pos[1]] for pos in self.robot_positions]
#     #     self.frame_files = []

#     #  self.begin()

#     def plot_env(self, n, path, step, planned_routes=None):
#         plt.switch_backend('agg')
#         plt.cla()
#         combined_belief = np.min(self.robot_beliefs, axis=0)
#         plt.imshow(combined_belief, cmap='gray')
#         plt.axis((0, self.ground_truth_size[1], self.ground_truth_size[0], 0))
#         if planned_routes:
#             for agent_id, agent_routes in enumerate(planned_routes):
#                 for p in agent_routes:
#                     planned_x, planned_y = [], []
#                     for coords in p:
#                         planned_x.append(coords[0])
#                         planned_y.append(coords[1])
#                     plt.plot(planned_x, planned_y, c=['r', 'g', 'b', 'y'][agent_id % 4], linewidth=2, zorder=2)
#         for i in range(self.num_agents):
#             plt.plot(self.xPoints[i], self.yPoints[i], ['b', 'g', 'r', 'y'][i % 4], linewidth=2)
#             plt.plot(self.robot_positions[i][0], self.robot_positions[i][1], 'mo', markersize=8)
#         for i, start_pos in enumerate(self.robot_positions):
#             plt.plot(start_pos[0], start_pos[1], 'co', markersize=8)
#         plt.scatter(self.frontiers[:, 0], self.frontiers[:, 1], c='r', s=2, zorder=3)
#         plt.suptitle('Explored ratio: {:.4g}  Travel distances: {}'.format(self.explored_rate, [round(d, 2) for d in self.travel_dists]))
#         plt.tight_layout()
#         plt.savefig('{}/{}_{}_samples.png'.format(path, n, step, dpi=150))
#         frame = '{}/{}_{}_samples.png'.format(path, n, step)
#         self.frame_files.append(frame)



#     # def plot_env(self, n, path, step, planned_route=None):
#     #     plt.switch_backend('agg')
#     #     # plt.ion()
#     #     plt.cla()
#     #     plt.imshow(self.robot_belief, cmap='gray')
#     #     plt.axis((0, self.ground_truth_size[1], self.ground_truth_size[0], 0))
#     #     if planned_route:
#     #         for p in planned_route:
#     #             planned_x, planned_y = [], []
#     #             for coords in p:
#     #                 planned_x.append(coords[0])
#     #                 planned_y.append(coords[1])
#     #             plt.plot(planned_x, planned_y, c='r', linewidth=2, zorder=2)
#     #     plt.plot(self.xPoints, self.yPoints, 'b', linewidth=2)
#     #     plt.plot(self.robot_position[0], self.robot_position[1], 'mo', markersize=8)
#     #     plt.plot(self.xPoints[0], self.yPoints[0], 'co', markersize=8)
#     #     plt.scatter(self.frontiers[:, 0], self.frontiers[:, 1], c='r', s=2, zorder=3)
#     #     # plt.pause(0.1)
#     #     plt.suptitle('Explored ratio: {:.4g}  Travel distance: {:.4g}'.format(self.explored_rate, self.travel_dist))
#     #     plt.tight_layout()
#     #     plt.savefig('{}/{}_{}_samples.png'.format(path, n, step, dpi=150))
#     #     # plt.show()
#     #     frame = '{}/{}_{}_samples.png'.format(path, n, step)
#     #     self.frame_files.append(frame)




from scipy import spatial
from skimage import io, measure
import numpy as np
import time
import sys
from scipy import ndimage
import matplotlib.pyplot as plt
import os
import copy
from skimage.measure import block_reduce
from shapely.geometry import Polygon, Point

from sensor import sensor_work
from parameter import *

class Env():
    def __init__(self, map_index, plot=False, test=False):
        self.test = test
        if self.test:
            self.map_dir = f'DungeonMaps/test'
        else:
            self.map_dir = f'DungeonMaps/train'
        self.map_list = os.listdir(self.map_dir)
        self.map_list.sort(reverse=True)
        self.map_index = map_index % np.size(self.map_list)
        self.ground_truth, self.robot_position = self.import_ground_truth(self.map_dir + '/' + self.map_list[self.map_index])
        self.ground_truth_size = np.shape(self.ground_truth) # (480, 640)
        self.robot_belief = np.ones(self.ground_truth_size) * 127 # unexplored 127
        
        self.finish_percent = 0.985
        self.resolution = 4
        self.sensor_range = 80
        self.old_robot_belief = copy.deepcopy(self.robot_belief)

        self.plot = plot
        self.frame_files = []
        if self.plot:
            # initialize the route
            self.xPoints = [self.robot_position[0]]
            self.yPoints = [self.robot_position[1]]

        self.travel_dist = 0
        self.explored_rate = 0
        self.route_node = [self.robot_position]
        self.frontiers = None
        self.downsampled_belief = None

    def begin(self):
        self.robot_belief = self.update_robot_belief(self.robot_position, self.sensor_range, self.robot_belief, self.ground_truth)
        self.downsampled_belief = block_reduce(self.robot_belief.copy(), block_size=(self.resolution, self.resolution), func=np.min)

        self.frontiers = self.find_frontier()
        
        self.old_robot_belief = copy.deepcopy(self.robot_belief)

    def step(self, next_node_coords):
        dist = np.linalg.norm(next_node_coords - self.robot_position)

        self.travel_dist += dist
        self.robot_position = next_node_coords
        self.route_node.append(self.robot_position)
        self.robot_belief = self.update_robot_belief(self.robot_position, self.sensor_range, self.robot_belief,
                                                     self.ground_truth)
        self.downsampled_belief = block_reduce(self.robot_belief.copy(), block_size=(self.resolution, self.resolution),
                                               func=np.min)

        frontiers = self.find_frontier()
        self.explored_rate = self.evaluate_exploration_rate()

        if self.plot:
            self.xPoints.append(self.robot_position[0])
            self.yPoints.append(self.robot_position[1])

        self.frontiers = frontiers

        done = self.check_done()

        return done

    def import_ground_truth(self, map_index):
        # occupied 1, free 255, unexplored 127

        ground_truth = (io.imread(map_index, 1) * 255).astype(int)
        robot_location = np.nonzero(ground_truth == 208)
        robot_location = np.array([np.array(robot_location)[1, 127], np.array(robot_location)[0, 127]])
        ground_truth = (ground_truth > 150)
        ground_truth = ground_truth * 254 + 1
        return ground_truth, robot_location

    def free_cells(self):
        index = np.where(self.ground_truth == 255)
        free = np.asarray([index[1], index[0]]).T
        return free

    def update_robot_belief(self, robot_position, sensor_range, robot_belief, ground_truth):
        robot_belief = sensor_work(robot_position, sensor_range, robot_belief, ground_truth)
        return robot_belief

    def check_done(self):
        done = False
        if np.sum(self.ground_truth == 255) - np.sum(self.robot_belief == 255) <= 250:
            done = True
        return done

    def evaluate_exploration_rate(self):
        rate = np.sum(self.robot_belief == 255) / np.sum(self.ground_truth == 255)
        return rate

    def calculate_new_free_area(self):
        old_free_area = self.old_robot_belief == 255
        current_free_area = self.robot_belief == 255

        new_free_area = (current_free_area.astype(np.int) - old_free_area.astype(np.int)) * 255

        return new_free_area, np.sum(old_free_area)

    def calculate_utility_along_path(self, path, nodes_list):
        observable_frontiers = []
        for index in path:
            observable_frontiers += nodes_list[index].observable_frontiers
        np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
        unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

        return unique_frontiers.shape[0]

    def calculate_node_gain_over_path(self, node_index, path, nodes_list):
        observable_frontiers = []
        for index in path:
            observable_frontiers += nodes_list[index].observable_frontiers
        np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
        pre_unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)
        observable_frontiers += nodes_list[node_index].observable_frontiers
        np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
        unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

        return unique_frontiers.shape[0] - pre_unique_frontiers.shape[0]

    def calculate_dist_path(self, path, node_list):
        dist = 0
        start = path[0]
        end = path[-1]
        for index in path:
            if index == end:
                break
            dist += np.linalg.norm(node_list[start].jpc - node_list[index].jpc)
            start = index
        return dist

    def find_frontier(self):
        y_len = self.downsampled_belief.shape[0]
        x_len = self.downsampled_belief.shape[1]
        mapping = self.downsampled_belief.copy()
        belief = self.downsampled_belief.copy()
        # 0-1 unknown area map
        mapping = (mapping == 127) * 1
        mapping = np.lib.pad(mapping, ((1, 1), (1, 1)), 'constant', constant_values=0)
        fro_map = mapping[2:][:, 1:x_len + 1] + mapping[:y_len][:, 1:x_len + 1] + mapping[1:y_len + 1][:, 2:] + \
                  mapping[1:y_len + 1][:, :x_len] + mapping[:y_len][:, 2:] + mapping[2:][:, :x_len] + mapping[2:][:,
                                                                                                      2:] + \
                  mapping[:y_len][:, :x_len]
        ind_free = np.where(belief.ravel(order='F') == 255)[0]
        ind_fron_1 = np.where(1 < fro_map.ravel(order='F'))[0]
        ind_fron_2 = np.where(fro_map.ravel(order='F') < 8)[0]
        ind_fron = np.intersect1d(ind_fron_1, ind_fron_2)
        ind_to = np.intersect1d(ind_free, ind_fron)

        map_x = x_len
        map_y = y_len
        x = np.linspace(0, map_x - 1, map_x)
        y = np.linspace(0, map_y - 1, map_y)
        t1, t2 = np.meshgrid(x, y)
        points = np.vstack([t1.T.ravel(), t2.T.ravel()]).T

        f = points[ind_to]
        f = f.astype(int)

        f = f * self.resolution

        return f

    def plot_env(self, n, path, step, planned_route=None):
     plt.switch_backend('agg')
     plt.cla()
     plt.imshow(self.robot_belief, cmap='gray')
     plt.axis((0, self.ground_truth_size[1], self.ground_truth_size[0], 0))
    
     if planned_route:
        # Ensure planned_route is a list of routes, even if it's just a single route
        if isinstance(planned_route[0], (list, np.ndarray)):
            planned_route = [planned_route]  # Wrap in a list if it's a single route
        
        # Plot each planned route
        for route in planned_route:
            planned_x, planned_y = [], []
            for coords in route:
                if isinstance(coords, (list, tuple, np.ndarray)):  # Ensure coords are iterable
                    planned_x.append(coords[0])
                    planned_y.append(coords[1])
            plt.plot(planned_x, planned_y, c='r', linewidth=2, zorder=2)
    
     plt.plot(self.xPoints, self.yPoints, 'b', linewidth=2)  # Plot path history
     plt.plot(self.robot_position[0], self.robot_position[1], 'mo', markersize=8)  # Plot robot position
     plt.plot(self.xPoints[0], self.yPoints[0], 'co', markersize=8)  # Plot start point
     plt.scatter(self.frontiers[:, 0], self.frontiers[:, 1], c='r', s=2, zorder=3)  # Plot frontiers
    
     plt.suptitle(f'Explored ratio: {self.explored_rate:.4g}  Travel distance: {self.travel_dist:.4g}')
     plt.tight_layout()
    
    # Save the figure
     plt.savefig(f'{path}/{n}_{step}_samples.png', dpi=150)
    
    # Add the saved frame to the list of frame files
     frame = f'{path}/{n}_{step}_samples.png'
     self.frame_files.append(frame)


    def compute_visibility(self, position):
        visible_area = sensor_work(position, self.sensor_range, np.ones_like(self.ground_truth) * 127, self.ground_truth)
        return visible_area == 255

    def get_polygon_from_binary_mask(self, binary_mask):
        contours = measure.find_contours(binary_mask, 0.5)
        polygons = []
        for contour in contours:
            if len(contour) > 2:  # Ensure we have at least 3 points to form a polygon
                polygon = Polygon(contour)
                if polygon.is_valid and polygon.area > 0:
                    polygons.append(polygon)
        return polygons