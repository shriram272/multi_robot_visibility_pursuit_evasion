# from shapely.geometry import Polygon, LineString, Point, MultiPolygon
# from shapely.ops import unary_union
# import matplotlib.pyplot as plt
# from matplotlib.patches import Polygon as pltPolygon
# import random
# import math
# from queue import PriorityQueue

# # Define the Shadow class
# class Shadow:
#     def __init__(self, polygon):
#         self.polygon = polygon
    
#     def intersects(self, other):
#         return self.polygon.intersects(other.polygon)

# # Calculate connected components of shadow regions
# def connected_components(shadow_region):
#     multi_polygon = unary_union(shadow_region)
    
#     if isinstance(multi_polygon, Polygon):
#         return [Shadow(multi_polygon)]
#     elif isinstance(multi_polygon, MultiPolygon):
#         return [Shadow(polygon) for polygon in multi_polygon.geoms]
#     else:
#         return []

# # Define the Environment class
# class Environment:
#     def __init__(self, boundary, obstacles):
#         self.boundary = Polygon(boundary)
#         self.obstacles = [Polygon(obstacle) for obstacle in obstacles]

#     def visibility_condition(self, p1, p2):
#         return all(self.in_visibility(p1[i], p2[i]) for i in range(len(p1)))
    
#     def in_visibility(self, q1, q2):
#         line_of_sight = LineString([q1, q2])
#         return not any(line_of_sight.crosses(obstacle) for obstacle in self.obstacles)
    
#     def visibility_region(self, pursuer, radius=2):
#         point = Point(pursuer)
#         visibility_polygon = point.buffer(radius).intersection(self.boundary)
        
#         for obstacle in self.obstacles:
#             visibility_polygon = visibility_polygon.difference(obstacle)
        
#         return visibility_polygon

#     def full_region(self):
#         return [self.boundary] + self.obstacles

# # Discretize the path between two points for each pursuer
# def discretize(p, p_prime, step_size=0.3):
#     max_dists = [max(math.dist(p[i], p_prime[i]) for i in range(len(p))) for j in range(len(p))]
#     max_dist = max(max_dists)  # Get the maximum distance among all dimensions
    
#     if max_dist == 0:  # If the maximum distance is zero, handle this case
#         return [p]  # Return the single point if start and end points are the same
    
#     num_steps = int(max_dist / step_size)
#     if num_steps == 0:
#         num_steps = 1  # Ensure at least one step if the distance is very small compared to step size
    
#     discretized = []
#     for t in range(num_steps + 1):
#         segment = []
#         for i in range(len(p)):
#             point = [p[i][j] + (p_prime[i][j] - p[i][j]) * t / num_steps for j in range(len(p[i]))]
#             segment.append(tuple(point))
#         discretized.append(segment)
    
#     return discretized

# # Calculate the shadow region for a given configuration
# def shadow_region(p, environment):
#     visible_region = unary_union([environment.visibility_region(pursuer) for pursuer in p])
#     full_region = unary_union(environment.full_region())
#     shadow_region = full_region.difference(visible_region)
    
#     if shadow_region.is_empty:
#         return []
#     if isinstance(shadow_region, Polygon):
#         shadow_region = [shadow_region]
#     return connected_components(shadow_region)

# # Define the Vertex class
# class Vertex:
#     def __init__(self, jpc):
#         self.jpc = jpc  # Joint pursuer configuration
#         self.reachable_labels = set()
#         self.neighbors = []
#         self.parent = None  # Reference to parent for solution extraction

#     def __repr__(self):
#         return f"Vertex(jpc={self.jpc})"

# # Define the SGPEG class with A* and adaptive sampling
# class SGPEG:
#     def __init__(self, environment, starting_configuration):
#         self.vertices = []
#         self.environment = environment
#         self.initial_configuration = starting_configuration
#         initial_shadow_regions = shadow_region(starting_configuration, environment)
#         self.root = self.add_vertex(starting_configuration, [1] * len(initial_shadow_regions))  # All shadows contaminated

#     def heuristic(self, configuration1, configuration2):
#         # Simple Euclidean distance heuristic for A*
#         return sum(math.dist(c1, c2) for c1, c2 in zip(configuration1, configuration2))

#     def a_star_search(self, start, goal):
#         open_set = PriorityQueue()
#         open_set.put((0, start))
#         came_from = {start: None}
#         g_score = {start: 0}
#         f_score = {start: self.heuristic(start, goal)}

#         while not open_set.empty():
#             _, current = open_set.get()

#             if current == goal:
#                 return self.reconstruct_path(came_from, current)
            
#             for neighbor in self.get_neighbors(current):
#                 tentative_g_score = g_score[current] + self.distance(current, neighbor)
#                 if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
#                     came_from[neighbor] = current
#                     g_score[neighbor] = tentative_g_score
#                     f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
#                     open_set.put((f_score[neighbor], neighbor))
        
#         return []  # No path found

#     def get_neighbors(self, configuration):
#         # Implement neighbor retrieval based on your graph structure
#         return [self.add_sample(self.sample_around_configuration(config)) for config in configuration]

#     def distance(self, config1, config2):
#         # Implement the distance function for your configuration space
#         return sum(math.dist(c1, c2) for c1, c2 in zip(config1, config2))

#     def reconstruct_path(self, came_from, current):
#         path = [current]
#         while came_from[current] is not None:
#             current = came_from[current]
#             path.append(current)
#         path.reverse()
#         return path

#     def adaptive_sampling(self, configuration, num_samples=20):
#         samples = []
#         for _ in range(num_samples):
#             if random.random() < 0.5:
#                 samples.append(self.sample_around_configuration(configuration))
#             else:
#                 samples.append(self.sample_with_high_density(configuration))
#         return samples

#     def sample_around_configuration(self, configuration):
#         # Sample points around the given configuration
#         return [(c[0] + random.uniform(-1, 1), c[1] + random.uniform(-1, 1)) for c in configuration]

#     def sample_with_high_density(self, configuration):
#         # Sample more densely in complex regions
#         return [(c[0] + random.uniform(-0.1, 0.1), c[1] + random.uniform(-0.1, 0.1)) for c in configuration]

#     def add_sample(self, configuration):
#         if isinstance(configuration[0], float):
#             configuration = [configuration]
        
#         new_vertex = self.add_vertex(configuration, [])
#         for vertex in self.vertices:
#             if vertex != new_vertex:
#                 print(f"Checking edge validity between {vertex.jpc} and {new_vertex.jpc}")
#                 if self.environment.visibility_condition(vertex.jpc, new_vertex.jpc):
#                     self.add_edge(vertex, new_vertex)

#     def add_vertex(self, configuration, initial_label):
#         new_vertex = Vertex(configuration)
#         new_vertex.reachable_labels.add(tuple(initial_label))
#         self.vertices.append(new_vertex)
#         print(f"Added vertex: {new_vertex} with label: {initial_label}")
#         return new_vertex
    
#     def add_edge(self, source, target):
#         source.neighbors.append(target)
#         target.neighbors.append(source)
#         target.parent = source
#         print(f"Added edge between {source} and {target}")
#         for label in source.reachable_labels:
#             updated_label = self.compute_label(source.jpc, list(label), target.jpc)
#             self.add_reachable(target, updated_label)
    
#     def add_reachable(self, v, l):
#         if any(self.dominates(existing_label, l) for existing_label in v.reachable_labels):
#             return
        
#         v.reachable_labels.add(tuple(l))
#         v.reachable_labels = {lbl for lbl in v.reachable_labels if not self.dominates(l, lbl)}
        
#         if all_clear(l):
#             print(f"Solution found at vertex {v} with label: {l}")
#             return
        
#         for neighbor in v.neighbors:
#             new_label = self.compute_label(v.jpc, list(l), neighbor.jpc)
#             self.add_reachable(neighbor, new_label)
    
#     def compute_label(self, p, l, p_prime):
#         label = l
#         discretized_points = discretize(p, p_prime)

#         for i in range(len(discretized_points) - 1):
#             pi = discretized_points[i]
#             pi_next = discretized_points[i + 1]
#             old_shadows = shadow_region(pi, self.environment)
#             new_shadows = shadow_region(pi_next, self.environment)

#             if len(label) < len(old_shadows):
#                 label.extend([0] * (len(old_shadows) - len(label)))

#             new_label = [0] * len(new_shadows)

#             for idx_new, s_prime in enumerate(new_shadows):
#                 for idx_old, s in enumerate(old_shadows):
#                     if label[idx_old] == 1 and s_prime.intersects(s):
#                         new_label[idx_new] = 1

#             if len(new_shadows) > len(old_shadows):
#                 for idx_new in range(len(old_shadows), len(new_shadows)):
#                     new_label[idx_new] = 0

#             label = new_label

#         return label

#     def dominates(self, l1, l2):
#         return all(l1[i] >= l2[i] for i in range(len(l1)))

# # Check if all shadows are clear
# def all_clear(label):
#     return all(l == 0 for l in label)

# # Extract the solution path from the SG-PEG for each pursuer
# def extract_solution(graph):
#     paths = {i: [] for i in range(len(graph.root.jpc))}
#     for vertex in graph.vertices:
#         if any(all_clear(label) for label in vertex.reachable_labels):
#             current = vertex
#             while current is not None:
#                 for i in range(len(current.jpc)):
#                     paths[i].append(current.jpc[i])
#                 current = current.parent
#             break
#     for path in paths.values():
#         path.reverse()
#     return paths

# # Define the Sampler class with adaptive and importance sampling
# class Sampler:
#     def __init__(self, environment):
#         self.environment = environment
    
#     def get_sample(self):
#         return self.importance_sampling() if random.random() < 0.5 else self.adaptive_sampling()
    
#     def importance_sampling(self):
#         # Focus on areas around obstacles
#         obstacles = [obstacle.buffer(1.5).envelope for obstacle in self.environment.obstacles]
#         sample_points = []
#         for _ in range(5):
#             while True:
#                 x = random.uniform(0, 10)
#                 y = random.uniform(0, 10)
#                 point = Point(x, y)
#                 if any(point.within(obs) for obs in obstacles):
#                     sample_points.append((x, y))
#                     break
#         return sample_points

#     def adaptive_sampling(self):
#         # Sample with higher density in complex regions
#         sample_points = []
#         num_samples = 10
#         for _ in range(num_samples):
#             x = random.uniform(0, 10)
#             y = random.uniform(0, 10)
#             sample_points.append((x, y))
#         return sample_points

# # Visualize the environment and the solution path
# def visualize_environment(environment, solution_paths, starting_configuration):
#     fig, ax = plt.subplots()

#     # Plot boundary
#     boundary_patch = pltPolygon(environment.boundary.exterior.coords, fill=True, color='lightgrey')
#     ax.add_patch(boundary_patch)
    
#     # Plot obstacles
#     for obstacle in environment.obstacles:
#         obstacle_patch = pltPolygon(obstacle.exterior.coords, fill=True, color='black')
#         ax.add_patch(obstacle_patch)
    
#     # Plot starting positions of pursuers
#     for idx, start in enumerate(starting_configuration):
#         x, y = start
#         ax.scatter(x, y, color=['red', 'green'][idx], label=f'Pursuer {idx + 1} Start', s=100, edgecolor='black')

#     # Plot solution path
#     if solution_paths:
#         for i in range(len(starting_configuration)):
#             x_coords, y_coords = zip(*solution_paths[i])
#             ax.plot(x_coords, y_coords, color=['red', 'green'][i], marker='o', linestyle='-', label=f'Pursuer {i + 1} Path')

#     ax.set_xlim(0, 10)
#     ax.set_ylim(0, 10)
#     ax.set_aspect('equal')
#     plt.xlabel("X")
#     plt.ylabel("Y")
#     plt.title("Environment and Solution Path")
#     plt.legend()
#     plt.show()

# # Define the environment
# boundary = [(0, 0), (10, 0), (10, 10), (0, 10)]
# obstacles = [
#     [(3, 5), (4, 5), (4, 6), (3, 6)],
#     [(5, 8), (6, 8), (6, 9), (5, 9)],
#     [(8, 4), (9, 4), (9, 5), (8, 5)],
#     [(1, 3), (2, 3), (2, 4), (1, 4)],
#     [(7, 7), (8, 7), (8, 8), (7, 8)],
#     [(10, 1), (11, 1), (11, 2), (10, 2)],
#     [(2, 6), (3, 6), (3, 7), (2, 7)],
# ]

# environment = Environment(boundary, obstacles)

# # Define the starting configuration of pursuers
# starting_configuration = [(2.5, 3), (9, 9)]

# # Create the SGPEG instance and sample configurations
# graph = SGPEG(environment, starting_configuration)
# sampler = Sampler(environment)

# # Sample a number of points
# num_samples = 139
# for _ in range(num_samples):
#     sample = sampler.get_sample()
#     graph.add_sample(sample)

# # Extract the solution path
# solution_paths = extract_solution(graph)

# # Visualize the environment and the solution path
# visualize_environment(environment, solution_paths, starting_configuration)



from shapely.geometry import Polygon, LineString, Point, MultiPolygon
from shapely.ops import unary_union
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as pltPolygon
import random
import math

# Define the Shadow class
class Shadow:
    def __init__(self, polygon):
        self.polygon = polygon
    
    def intersects(self, other):
        return self.polygon.intersects(other.polygon)

# Calculate connected components of shadow regions
def connected_components(shadow_region):
    multi_polygon = unary_union(shadow_region)
    
    if isinstance(multi_polygon, Polygon):
        return [Shadow(multi_polygon)]
    elif isinstance(multi_polygon, MultiPolygon):
        return [Shadow(polygon) for polygon in multi_polygon.geoms]
    else:
        return []

# Define the Environment class
class Environment:
    def __init__(self, boundary, obstacles):
        self.boundary = Polygon(boundary)
        self.obstacles = [Polygon(obstacle) for obstacle in obstacles]

    def visibility_condition(self, p1, p2):
        # Ensure both p1 and p2 are properly formatted and of the same length
        if len(p1) != len(p2):
            raise ValueError("Configurations p1 and p2 must have the same length")
        
        return all(self.in_visibility(p1[i], p2[i]) for i in range(len(p1)))
    
    def in_visibility(self, q1, q2):
        line_of_sight = LineString([q1, q2])
        return not any(line_of_sight.crosses(obstacle) for obstacle in self.obstacles)
    
    def visibility_region(self, pursuer, radius=2):
        point = Point(pursuer)
        visibility_polygon = point.buffer(radius).intersection(self.boundary)
        
        for obstacle in self.obstacles:
            visibility_polygon = visibility_polygon.difference(obstacle)
        
        return visibility_polygon

    def full_region(self):
        return [self.boundary] + self.obstacles

# Discretize the path between two points for each pursuer
def discretize(p, p_prime, step_size=0.3):
    max_dists = [max(math.dist(p[i], p_prime[i]) for i in range(len(p))) for j in range(len(p))]
    max_dist = max(max_dists)  # Get the maximum distance among all dimensions
    
    if max_dist == 0:  # If the maximum distance is zero, handle this case
        return [p]  # Return the single point if start and end points are the same
    
    num_steps = int(max_dist / step_size)
    if num_steps == 0:
        num_steps = 1  # Ensure at least one step if the distance is very small compared to step size
    
    discretized = []
    for t in range(num_steps + 1):
        segment = []
        for i in range(len(p)):
            point = [p[i][j] + (p_prime[i][j] - p[i][j]) * t / num_steps for j in range(len(p[i]))]
            segment.append(tuple(point))
        discretized.append(segment)
    
    return discretized

# Calculate the shadow region for a given configuration
def shadow_region(p, environment):
    visible_region = unary_union([environment.visibility_region(pursuer) for pursuer in p])
    full_region = unary_union(environment.full_region())
    shadow_region = full_region.difference(visible_region)
    
    if shadow_region.is_empty:
        return []
    if isinstance(shadow_region, Polygon):
        shadow_region = [shadow_region]
    return connected_components(shadow_region)

# Define the Vertex class
class Vertex:
    def __init__(self, jpc):
        self.jpc = jpc  # Joint pursuer configuration
        self.reachable_labels = set()
        self.neighbors = []
        self.parent = None  # Reference to parent for solution extraction

    def __repr__(self):
        return f"Vertex(jpc={self.jpc})"

# Define the SGPEG class
class SGPEG:
    def __init__(self, environment, starting_configuration):
        self.vertices = []
        self.environment = environment
        initial_shadow_regions = shadow_region(starting_configuration, environment)
        self.root = self.add_vertex(starting_configuration, [1] * len(initial_shadow_regions))  # All shadows contaminated
    
    def add_vertex(self, configuration, initial_label):
        new_vertex = Vertex(configuration)
        new_vertex.reachable_labels.add(tuple(initial_label))
        self.vertices.append(new_vertex)
        print(f"Added vertex: {new_vertex} with label: {initial_label}")
        return new_vertex
    
    def add_edge(self, source, target):
        source.neighbors.append(target)
        target.neighbors.append(source)
        target.parent = source
        print(f"Added edge between {source} and {target}")
        for label in source.reachable_labels:
            updated_label = self.compute_label(source.jpc, list(label), target.jpc)
            self.add_reachable(target, updated_label)
    
    def add_sample(self, configuration):
        if isinstance(configuration[0], float):
            configuration = [configuration]
        
        new_vertex = self.add_vertex(configuration, [])
        for vertex in self.vertices:
            if vertex != new_vertex:
                print(f"Checking edge validity between {vertex.jpc} and {new_vertex.jpc}")
                if self.environment.visibility_condition(vertex.jpc, new_vertex.jpc):
                    self.add_edge(vertex, new_vertex)

    def add_reachable(self, v, l):
        if any(self.dominates(existing_label, l) for existing_label in v.reachable_labels):
            return
        
        v.reachable_labels.add(tuple(l))
        v.reachable_labels = {lbl for lbl in v.reachable_labels if not self.dominates(l, lbl)}
        
        if all_clear(l):
            print(f"Solution found at vertex {v} with label: {l}")
            return
        
        for neighbor in v.neighbors:
            new_label = self.compute_label(v.jpc, list(l), neighbor.jpc)
            self.add_reachable(neighbor, new_label)
    
    def compute_label(self, p, l, p_prime):
        label = l
        discretized_points = discretize(p, p_prime)

        for i in range(len(discretized_points) - 1):
            pi = discretized_points[i]
            pi_next = discretized_points[i + 1]
            old_shadows = shadow_region(pi, self.environment)
            new_shadows = shadow_region(pi_next, self.environment)

            if len(label) < len(old_shadows):
                label.extend([0] * (len(old_shadows) - len(label)))

            new_label = [0] * len(new_shadows)

            for idx_new, s_prime in enumerate(new_shadows):
                for idx_old, s in enumerate(old_shadows):
                    if label[idx_old] == 1 and s_prime.intersects(s):
                        new_label[idx_new] = 1

            if len(new_shadows) > len(old_shadows):
                for idx_new in range(len(old_shadows), len(new_shadows)):
                    new_label[idx_new] = 0

            label = new_label

        return label

    def dominates(self, l1, l2):
        return all(l1[i] >= l2[i] for i in range(len(l1)))

# Check if all shadows are clear
def all_clear(label):
    return all(l == 0 for l in label)

# Extract the solution path from the SG-PEG for each pursuer
def extract_solution(graph):
    paths = {i: [] for i in range(len(graph.root.jpc))}
    for vertex in graph.vertices:
        if any(all_clear(label) for label in vertex.reachable_labels):
            current = vertex
            while current is not None:
                for i in range(len(current.jpc)):
                    paths[i].append(current.jpc[i])
                current = current.parent
            break
    for path in paths.values():
        path.reverse()
    return paths

# Define the Sampler class with adaptive and importance sampling
class Sampler:
    def __init__(self, environment):
        self.environment = environment
    
    def get_sample(self):
        return self.importance_sampling() if random.random() < 0.5 else self.adaptive_sampling()
    
    def importance_sampling(self):
        # Focus on areas around obstacles
        obstacles = [obstacle.buffer(1.5).envelope for obstacle in self.environment.obstacles]
        sample_points = []
        for _ in range(5):
            while True:
                x = random.uniform(0, 10)
                y = random.uniform(0, 10)
                point = Point(x, y)
                if any(point.within(obs) for obs in obstacles):
                    sample_points.append((x, y))
                    break
        return sample_points

    def adaptive_sampling(self):
        # Sample with higher density in complex regions
        sample_points = []
        num_samples = 10
        for _ in range(num_samples):
            x = random.uniform(0, 10)
            y = random.uniform(0, 10)
            sample_points.append((x, y))
        return sample_points

# Visualize the environment and the solution path
def visualize_environment(environment, solution_paths, starting_configuration):
    fig, ax = plt.subplots()

    # Plot boundary
    boundary_patch = pltPolygon(environment.boundary.exterior.coords, fill=True, color='lightgrey')
    ax.add_patch(boundary_patch)
    
    # Plot obstacles
    for obstacle in environment.obstacles:
        obstacle_patch = pltPolygon(obstacle.exterior.coords, fill=True, color='black')
        ax.add_patch(obstacle_patch)
    
    # Plot starting positions of pursuers
    for idx, start in enumerate(starting_configuration):
        x, y = start
        ax.scatter(x, y, color=['red', 'green'][idx], label=f'Pursuer {idx + 1} Start', s=100, edgecolor='black')

    # Plot solution path
    if solution_paths:
        # Plot the paths
        for i in range(len(starting_configuration)):
            x_coords, y_coords = zip(*solution_paths[i])
            ax.plot(x_coords, y_coords, color=['red', 'green'][i], marker='o', linestyle='-', label=f'Pursuer {i + 1} Path')

    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_aspect('equal')
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Environment and Solution Path")
    plt.legend()
    plt.show()

# Define the environment
boundary = [(0, 0), (10, 0), (10, 10), (0, 10)]
obstacles = [
    [(3, 5), (4, 5), (4, 6), (3, 6)],
    [(5, 8), (6, 8), (6, 9), (5, 9)],
    [(8, 4), (9, 4), (9, 5), (8, 5)],
    [(1, 3), (2, 3), (2, 4), (1, 4)],
    [(7, 7), (8, 7), (8, 8), (7, 8)],
    [(10, 1), (11, 1), (11, 2), (10, 2)],
    [(2, 6), (3, 6), (3, 7), (2, 7)],
]

environment = Environment(boundary, obstacles)

# Define the starting configuration of pursuers
starting_configuration = [(2.5, 3), (9, 9)]

# Create the SGPEG instance and sample configurations
graph = SGPEG(environment, starting_configuration)
sampler = Sampler(environment)

# Sample a number of points
num_samples = 139
for _ in range(num_samples):
    sample = sampler.get_sample()
    graph.add_sample(sample)

# Extract the solution path
solution_paths = extract_solution(graph)

# Visualize the environment and the solution path
visualize_environment(environment, solution_paths, starting_configuration)
