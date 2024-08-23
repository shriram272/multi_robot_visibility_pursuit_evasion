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
def discretize(p, p_prime, step_size=0.1):
    max_dists = [max(math.dist(p[i], p_prime[i]) for i in range(len(p))) for j in range(len(p))]
    num_steps = int(max(max_dists) / step_size)
    
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

# Define the Sampler class
class Sampler:
    def __init__(self, environment):
        self.environment = environment
    
    def get_sample(self):
        while True:
            sample = [(random.uniform(0, 10), random.uniform(0, 10)) for _ in range(2)]
            if all(self.environment.in_visibility(sample[i], sample[j]) for i in range(len(sample)) for j in range(i + 1, len(sample))):
                return sample

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
        # Connect starting positions to the first segment of the path
        for i in range(len(starting_configuration)):
            x_coords = [starting_configuration[i][0], solution_paths[i][0][0]]
            y_coords = [starting_configuration[i][1], solution_paths[i][0][1]]
            ax.plot(x_coords, y_coords, color='blue', marker='o')

        # Plot the rest of the path
        for key, path in solution_paths.items():
            x_coords, y_coords = zip(*path)
            ax.plot(x_coords, y_coords, color='blue', marker='o')
    
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
obstacles = [    [(3, 3), (4, 3), (4, 4), (3, 4)],
    [(7, 7), (8, 7), (8, 8), (7, 8)]]
environment = Environment(boundary, obstacles)

# Define the starting configuration of pursuers
starting_configuration = [(1, 1), (9, 9)]

# Create the SGPEG instance and sample configurations
graph = SGPEG(environment, starting_configuration)
sampler = Sampler(environment)

# Sample a number of points
num_samples = 10
for _ in range(num_samples):
    sample = sampler.get_sample()
    graph.add_sample(sample)

# Extract the solution path
solution_paths = extract_solution(graph)

# Visualize the environment and the solution path
visualize_environment(environment, solution_paths, starting_configuration)
