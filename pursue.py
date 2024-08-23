# import numpy as np
# import random
# from shapely.geometry import LineString, Point
# from NBVP_env import Env
# from parameter import *
# gifs_path = f'results/SPGEG'

# class Shadow:
#     def __init__(self, polygon, is_contaminated=False):
#         self.polygon = polygon
#         self.is_contaminated = is_contaminated

#     def intersects(self, other):
#         return self.polygon.intersects(other.polygon)

#     def union(self, other):
#         return Shadow(self.polygon.union(other.polygon), 
#                       self.is_contaminated or other.is_contaminated)

# class ShadowLabel:
#     def __init__(self, shadows):
#         self.shadows = shadows

#     def update(self, new_shadows):
#         updated_shadows = []
#         for new_shadow in new_shadows:
#             intersecting = [s for s in self.shadows if s.intersects(new_shadow)]
#             if intersecting:
#                 new_shadow.is_contaminated = any(s.is_contaminated for s in intersecting)
#             updated_shadows.append(new_shadow)
#         self.shadows = updated_shadows

# def compute_shadows(jpc, env):
#     visible_area = env.compute_visibility(jpc)
#     non_visible_area = ~visible_area
#     shadow_polygons = env.get_polygon_from_binary_mask(non_visible_area)
#     return [Shadow(polygon) for polygon in shadow_polygons]

# def compute_new_label(source_jpc, target_jpc, source_labels, env):
#     num_samples = 10
#     jpc_samples = [source_jpc + (target_jpc - source_jpc) * i / num_samples for i in range(num_samples + 1)]
    
#     current_label = source_labels[0] if source_labels else ShadowLabel([])
    
#     for jpc in jpc_samples:
#         new_shadows = compute_shadows(jpc, env)
#         current_label.update(new_shadows)
    
#     return current_label

# class Vertex:
#     def __init__(self, id, parent_id, jpc, env):
#         self.id = id
#         self.parent_id = parent_id
#         self.jpc = jpc
#         self.env = env
#         self.reachable_labels = []
#         self.observable_frontiers = []
#         self.gain = 0
#         self.branch_index = []
#         self.branch_coords = []
#         self.initialize_observable_frontiers()

#     def initialize_observable_frontiers(self):
#         dist_list = np.linalg.norm(self.env.frontiers - self.jpc, axis=-1)
#         frontiers_in_range = self.env.frontiers[dist_list < self.env.sensor_range - 10]
#         for point in frontiers_in_range:
#             collision = check_collision(self.jpc, point, self.env.robot_belief, self.reachable_labels)
#             if not collision:
#                 self.observable_frontiers.append(point)

#     def add_reachable_label(self, label):
#         if not any(self.dominates(existing, label) for existing in self.reachable_labels):
#             self.reachable_labels = [existing for existing in self.reachable_labels 
#                                      if not self.dominates(label, existing)]
#             self.reachable_labels.append(label)

#     @staticmethod
#     def dominates(label1, label2):
#         return sum(s.is_contaminated for s in label1.shadows) <= sum(s.is_contaminated for s in label2.shadows)

# class SGPEGTree:
#     def __init__(self, current_coords, env):
#         self.initial = 0
#         self.vertices = {}
#         self.vertices_indices = []
#         self.env = env

#         vertex = Vertex(self.initial, -1, current_coords, self.env)
#         self.add_vertex(vertex)

#     def add_vertex(self, vertex):
#         self.vertices[vertex.id] = vertex
#         self.vertices_indices.append(vertex.id)
#         vertex.branch_index, vertex.branch_coords = self.extract_branch(vertex)
#         visible = self.env.calculate_utility_along_path(vertex.branch_index, self.vertices)
#         dist = self.env.calculate_dist_path(vertex.branch_index, self.vertices)
#         vertex.gain = visible * np.exp(-10 * dist / 640)

#     def add_edge(self, source_vertex, target_vertex):
#         new_label = compute_new_label(source_vertex.jpc, target_vertex.jpc, source_vertex.reachable_labels, self.env)
#         target_vertex.add_reachable_label(new_label)

#     def extract_branch(self, vertex):
#         branch_index = [vertex.id]
#         while vertex.parent_id != -1:
#             vertex = self.vertices[vertex.parent_id]
#             branch_index.append(vertex.id)
#             if vertex.parent_id == -1:
#                 break
#         branch_coords = [self.vertices[index].jpc for index in branch_index]
#         return branch_index, branch_coords

# def check_collision(start, end, robot_belief, shadow_labels):
#     collision = False
#     line = LineString([start, end])

#     # Check collision with robot belief
#     sortx = np.sort([int(start[0]), int(end[0])])
#     sorty = np.sort([int(start[1]), int(end[1])])
    
#     # Ensure indices are within bounds
#     sortx = np.clip(sortx, 0, robot_belief.shape[1] - 1)
#     sorty = np.clip(sorty, 0, robot_belief.shape[0] - 1)
    
#     robot_belief_section = robot_belief[sorty[0]:sorty[1] + 1, sortx[0]:sortx[1] + 1]
#     occupied_area_index = np.where(robot_belief_section == 1)
#     occupied_area_coords = np.asarray([occupied_area_index[1] + sortx[0], occupied_area_index[0] + sorty[0]]).T
    
#     for coords in occupied_area_coords:
#         obstacle = Point(coords).buffer(5)
#         if line.intersects(obstacle):
#             collision = True
#             break

#     # Check collision with shadows
#     if not collision:
#         for label in shadow_labels:
#             for shadow in label.shadows:
#                 if line.intersects(shadow.polygon):
#                     collision = True
#                     break
#             if collision:
#                 break

#     return collision

# class NBVPWorkerSGPEG:
#     def __init__(self, metaAgentID, global_step, save_image=False):
#         self.metaAgentID = metaAgentID
#         self.global_step = global_step
#         self.save_image = save_image
#         self.step_length = 30
#         self.pre_best_path = []
#         self.planned_paths = []
#         self.env = Env(map_index=self.global_step, plot=save_image, test=True)

#     def find_next_best_viewpoints(self):
#         max_iter_steps = 100
#         tree = SGPEGTree(self.env.robot_position, self.env)
#         self.planned_paths = []
#         if self.pre_best_path:
#             self.initialize_tree_from_path(tree, self.pre_best_path)
#         g_best = 0
#         best_route = []
#         free_area = self.env.free_cells()
#         frontiers = self.env.frontiers
#         indices_to_sample = list(range(free_area.shape[0]))
#         frontiers_to_sample = list(range(len(frontiers)))
        
#         for i in range(max_iter_steps):
#             if np.random.random() > 0.2:
#                 sample_index = random.choice(indices_to_sample)
#                 sample_coords = free_area[sample_index]
#             else:
#                 sample_index = random.choice(frontiers_to_sample)
#                 sample_coords = frontiers[sample_index]
            
#             nearest_vertex = self.find_nearest_vertex(tree, sample_coords)
#             new_vertex_coords = self.steer(nearest_vertex.jpc, sample_coords, self.step_length)
            
#             if not check_collision(nearest_vertex.jpc, new_vertex_coords, self.env.robot_belief, nearest_vertex.reachable_labels):
#                 new_vertex = Vertex(len(tree.vertices), nearest_vertex.id, new_vertex_coords, self.env)
#                 tree.add_vertex(new_vertex)
#                 tree.add_edge(nearest_vertex, new_vertex)
                
#                 route = new_vertex.branch_coords[::-1]
#                 self.planned_paths.append(route)
                
#                 if new_vertex.gain > g_best:
#                     g_best = new_vertex.gain
#                     best_route = route
        
#         self.pre_best_path = best_route
#         return best_route[1], self.planned_paths

#     def find_nearest_vertex(self, tree, coords):
#         distances = [np.linalg.norm(v.jpc - coords) for v in tree.vertices.values()]
#         nearest_index = np.argmin(distances)
#         return tree.vertices[nearest_index]

#     def steer(self, start, goal, step_length):
#         direction = goal - start
#         distance = np.linalg.norm(direction)
#         if distance > step_length:
#             return start + direction / distance * step_length
#         return goal

#     def initialize_tree_from_path(self, tree, path):
#         for i, coords in enumerate(path[1:], start=1):
#             vertex = Vertex(i, i-1, coords, self.env)
#             tree.add_vertex(vertex)
#             tree.add_edge(tree.vertices[i-1], vertex)

#     def run_episode(self, currEpisode):
#         perf_metrics = dict()
#         done = False
#         self.env.begin()
#         i = 0
#         while not done:
#             i += 1
#             next_node_coords, planned_route = self.find_next_best_viewpoints()
#             done = self.env.step(next_node_coords)
            
#             if self.save_image:
#                 self.env.plot_env(self.global_step, gifs_path, i, planned_route)
            
#             if done:
#                 perf_metrics['travel_dist'] = self.env.travel_dist
#                 perf_metrics['explored_rate'] = self.env.explored_rate
#                 perf_metrics['success_rate'] = True
#                 perf_metrics['relax_success_rate'] = True if self.env.explored_rate > self.env.finish_percent else False
#                 break

#         return perf_metrics

# if __name__ == "__main__":
#     total_episode = 40
#     total_dist = 0
#     SAVE_GIFS=True
#     for i in range(total_episode):
#         worker = NBVPWorkerSGPEG(metaAgentID=0, global_step=i, save_image=SAVE_GIFS)
#         performance = worker.run_episode(i)
#         total_dist += performance["travel_dist"]
#         mean_dist = total_dist / (i + 1)
#         print(f"Episode {i+1}, Mean Distance: {mean_dist}")



import numpy as np
import random
from shapely.geometry import LineString, Point
from NBVP_env import Env
from parameter import *
gifs_path = f'results/SPGEG'

class Shadow:
    def __init__(self, polygon, is_contaminated=False):
        self.polygon = polygon
        self.is_contaminated = is_contaminated

    def intersects(self, other):
        return self.polygon.intersects(other.polygon)

    def union(self, other):
        return Shadow(self.polygon.union(other.polygon), 
                      self.is_contaminated or other.is_contaminated)

class ShadowLabel:
    def __init__(self, shadows):
        self.shadows = shadows

    def update(self, new_shadows):
        updated_shadows = []
        for new_shadow in new_shadows:
            intersecting = [s for s in self.shadows if s.intersects(new_shadow)]
            if intersecting:
                new_shadow.is_contaminated = any(s.is_contaminated for s in intersecting)
            updated_shadows.append(new_shadow)
        self.shadows = updated_shadows

def compute_shadows(jpc, env):
    print(f"Computing shadows for jpc: {jpc}")
    visible_area = env.compute_visibility(jpc)
    non_visible_area = ~visible_area
    shadow_polygons = env.get_polygon_from_binary_mask(non_visible_area)
    return [Shadow(polygon) for polygon in shadow_polygons]

def compute_new_label(source_jpc, target_jpc, source_labels, env):
    print(f"Computing new label from source_jpc: {source_jpc} to target_jpc: {target_jpc}")
    num_samples = 10
    jpc_samples = [source_jpc + (target_jpc - source_jpc) * i / num_samples for i in range(num_samples + 1)]
    
    current_label = source_labels[0] if source_labels else ShadowLabel([])
    
    for jpc in jpc_samples:
        new_shadows = compute_shadows(jpc, env)
        current_label.update(new_shadows)
    
    return current_label

class Vertex:
    def __init__(self, id, parent_id, jpc, env):
        print(f"Creating vertex with id: {id}, parent_id: {parent_id}, jpc: {jpc}")
        self.id = id
        self.parent_id = parent_id
        self.jpc = jpc
        self.env = env
        self.reachable_labels = []
        self.observable_frontiers = []
        self.gain = 0
        self.branch_index = []
        self.branch_coords = []
        self.initialize_observable_frontiers()

    def initialize_observable_frontiers(self):
        print(f"Initializing observable frontiers for vertex {self.id}")
        dist_list = np.linalg.norm(self.env.frontiers - self.jpc, axis=-1)
        frontiers_in_range = self.env.frontiers[dist_list < self.env.sensor_range - 10]
        for point in frontiers_in_range:
            collision = check_collision(self.jpc, point, self.env.robot_belief, self.reachable_labels)
            if not collision:
                self.observable_frontiers.append(point)

    def add_reachable_label(self, label):
        print(f"Adding reachable label to vertex {self.id}")
        if not any(self.dominates(existing, label) for existing in self.reachable_labels):
            self.reachable_labels = [existing for existing in self.reachable_labels 
                                     if not self.dominates(label, existing)]
            self.reachable_labels.append(label)

    @staticmethod
    def dominates(label1, label2):
        return sum(s.is_contaminated for s in label1.shadows) <= sum(s.is_contaminated for s in label2.shadows)

class SGPEGTree:
    def __init__(self, current_coords, env, save_image=False, gifs_path=None, global_step=0):
        self.initial = 0
        self.vertices = {}
        self.vertices_indices = []
        self.env = env
        self.save_image = save_image
        self.gifs_path = gifs_path
        self.global_step = global_step

        vertex = Vertex(self.initial, -1, current_coords, self.env)
        self.add_vertex(vertex)

    def add_vertex(self, vertex):
        self.vertices[vertex.id] = vertex
        self.vertices_indices.append(vertex.id)
        vertex.branch_index, vertex.branch_coords = self.extract_branch(vertex)
        visible = self.env.calculate_utility_along_path(vertex.branch_index, self.vertices)
        dist = self.env.calculate_dist_path(vertex.branch_index, self.vertices)
        vertex.gain = visible * np.exp(-10 * dist / 640)

        # Save an image when a new vertex is added
        if self.save_image:
            step_number = len(self.vertices_indices)
            print(f"Saving image for vertex {vertex.id} at step {step_number}")
            self.env.plot_env(self.global_step, self.gifs_path, step_number, vertex.branch_coords)


    def add_edge(self, source_vertex, target_vertex):
        print(f"Adding edge between vertex {source_vertex.id} and vertex {target_vertex.id}")
        new_label = compute_new_label(source_vertex.jpc, target_vertex.jpc, source_vertex.reachable_labels, self.env)
        target_vertex.add_reachable_label(new_label)

    def extract_branch(self, vertex):
     branch_index = [vertex.id]
     branch_coords = [vertex.jpc]  # Initialize with the current vertex's jpc

     while vertex.parent_id != -1:
        vertex = self.vertices[vertex.parent_id]
        branch_index.append(vertex.id)
        branch_coords.append(vertex.jpc)  # Append each parent vertex's jpc
    
     return branch_index, branch_coords


def check_collision(start, end, robot_belief, shadow_labels):
    print(f"Checking collision from {start} to {end}")
    collision = False
    line = LineString([start, end])

    # Check collision with robot belief
    sortx = np.sort([int(start[0]), int(end[0])])
    sorty = np.sort([int(start[1]), int(end[1])])
    
    # Ensure indices are within bounds
    sortx = np.clip(sortx, 0, robot_belief.shape[1] - 1)
    sorty = np.clip(sorty, 0, robot_belief.shape[0] - 1)
    
    robot_belief_section = robot_belief[sorty[0]:sorty[1] + 1, sortx[0]:sortx[1] + 1]
    occupied_area_index = np.where(robot_belief_section == 1)
    occupied_area_coords = np.asarray([occupied_area_index[1] + sortx[0], occupied_area_index[0] + sorty[0]]).T
    
    for coords in occupied_area_coords:
        obstacle = Point(coords).buffer(5)
        if line.intersects(obstacle):
            collision = True
            break

    # Check collision with shadows
    if not collision:
        for label in shadow_labels:
            for shadow in label.shadows:
                if line.intersects(shadow.polygon):
                    collision = True
                    break
            if collision:
                break

    return collision

class NBVPWorkerSGPEG:
    def __init__(self, metaAgentID, global_step, save_image=False):
        self.metaAgentID = metaAgentID
        self.global_step = global_step
        self.save_image = save_image
        self.step_length = 30
        self.pre_best_path = []
        self.planned_paths = []
        self.env = Env(map_index=self.global_step, plot=save_image, test=True)

    def find_next_best_viewpoints(self):
        max_iter_steps = 100
        tree = SGPEGTree(self.env.robot_position, self.env, save_image=self.save_image, gifs_path=gifs_path, global_step=self.global_step)
        self.planned_paths = []
        if self.pre_best_path:
            self.initialize_tree_from_path(tree, self.pre_best_path)
        g_best = 0
        best_route = []
        free_area = self.env.free_cells()
        frontiers = self.env.frontiers
        indices_to_sample = list(range(free_area.shape[0]))
        frontiers_to_sample = list(range(len(frontiers)))

        for i in range(max_iter_steps):
            if np.random.random() > 0.2:
                sample_index = random.choice(indices_to_sample)
                sample_coords = free_area[sample_index]
            else:
                sample_index = random.choice(frontiers_to_sample)
                sample_coords = frontiers[sample_index]

            nearest_vertex = self.find_nearest_vertex(tree, sample_coords)
            new_vertex_coords = self.steer(nearest_vertex.jpc, sample_coords, self.step_length)

            if not check_collision(nearest_vertex.jpc, new_vertex_coords, self.env.robot_belief, nearest_vertex.reachable_labels):
                new_vertex = Vertex(len(tree.vertices), nearest_vertex.id, new_vertex_coords, self.env)
                tree.add_vertex(new_vertex)
                tree.add_edge(nearest_vertex, new_vertex)

                route = new_vertex.branch_coords[::-1]
                self.planned_paths.append(route)

                if new_vertex.gain > g_best:
                    g_best = new_vertex.gain
                    best_route = route

        self.pre_best_path = best_route
        return best_route[1], self.planned_paths


    def find_nearest_vertex(self, tree, coords):
        print(f"Finding nearest vertex to coords: {coords}")
        distances = [np.linalg.norm(v.jpc - coords) for v in tree.vertices.values()]
        nearest_index = np.argmin(distances)
        return tree.vertices[nearest_index]

    def steer(self, start, goal, step_length):
        print(f"Steering from {start} to {goal} with step length {step_length}")
        direction = goal - start
        distance = np.linalg.norm(direction)
        if distance > step_length:
            return start + direction / distance * step_length
        return goal

    def initialize_tree_from_path(self, tree, path):
        print(f"Initializing tree from path: {path}")
        for i, coords in enumerate(path[1:], start=1):
            vertex = Vertex(i, i-1, coords, self.env)
            tree.add_vertex(vertex)
            tree.add_edge(tree.vertices[i-1], vertex)

    def run_episode(self, currEpisode):
        print(f"Running episode {currEpisode}")
        perf_metrics = dict()
        done = False
        self.env.begin()
        i = 0
        while not done:
            i += 1
            next_node_coords, planned_route = self.find_next_best_viewpoints()
            done = self.env.step(next_node_coords)
            
            if self.save_image:
                self.env.plot_env(self.global_step, gifs_path, i, planned_route)
            
            if done:
                perf_metrics['travel_dist'] = self.env.travel_dist
                perf_metrics['explored_rate'] = self.env.explored_rate
                perf_metrics['success_rate'] = True
                perf_metrics['relax_success_rate'] = True if self.env.explored_rate > self.env.finish_percent else False
                break

        return perf_metrics

if __name__ == "__main__":
    total_episode = 40
    total_dist = 0
    SAVE_GIFS=True
    for i in range(total_episode):
        print(f"Starting episode {i+1}")
        worker = NBVPWorkerSGPEG(metaAgentID=0, global_step=i, save_image=SAVE_GIFS)
        performance = worker.run_episode(i)
        total_dist += performance["travel_dist"]
        mean_dist = total_dist / (i + 1)
        print(f"Episode {i+1}, Mean Distance: {mean_dist}")
