This project was made for comparing baselines for a research paper contribution related to pursuit evasion during internship with NUS Singapore and submitted to ICRA 2025.
The projects are developed with referwnces from following papers - 

1.A. Bircher, M. Kamel, K. Alexis, H. Oleynikova and R. Siegwart, "Receding Horizon "Next-Best-View" Planner for 3D Exploration," 2016 IEEE International Conference on Robotics and Automation (ICRA)
2.Trevor Olsen, Anne M. Tumlin, Nicholas M. Stiffler and Jason M. O’Kane "A Visibility Roadmap Sampling Approach for a Multi-Robot Visibility-Based Pursuit-Evasion Problem "

The main goal of the code is to help a robot explore an environment efficiently by finding the best viewpoints that maximize the explored area while avoiding collisions with obstacles and considering shadowed regions.'

1. Shadow and ShadowLabel Classes

    Shadow: Represents a polygonal area that is either visible (not shadowed) or contaminated (shadowed). It includes methods to check for intersections with other shadows and to merge two shadows into a single one.
    ShadowLabel: Manages a collection of Shadow objects. It updates the shadowed regions based on new observations, ensuring that contaminated regions are appropriately tracked.

2. Shadow Computation Functions

    compute_shadows(jpc, env): Given a Joint Point Cloud (JPC), this function calculates the areas in the environment that are shadowed (not visible) from that point. It returns a list of Shadow objects representing these areas.
    compute_new_label(source_jpc, target_jpc, source_labels, env): This function calculates the new shadow label when moving from one JPC to another. It does so by sampling points between the source and target JPCs and updating the shadow labels at each step.

3. Vertex Class

    Vertex: Represents a point in the exploration tree. Each vertex has a unique ID, a parent vertex, its location (JPC), and various attributes like observable frontiers, reachable labels (shadow labels), and a gain value, which quantifies its utility.
    initialize_observable_frontiers(): Identifies the frontiers (unexplored areas) that are within sensor range and not blocked by obstacles or shadows.
    add_reachable_label(label): Adds a new shadow label to the vertex's reachable labels if it provides new information.

4. SGPEGTree Class

    SGPEGTree: This class manages the exploration tree, which consists of vertices connected by edges. Each vertex in the tree represents a potential viewpoint in the environment.
    add_vertex(vertex): Adds a new vertex to the tree and updates its gain value based on the utility of its path and the distance from the root.
    add_edge(source_vertex, target_vertex): Adds an edge between two vertices, updating the shadow labels for the target vertex.
    extract_branch(vertex): Extracts the path (branch) from the root to a given vertex.

5. Collision Checking

    check_collision(start, end, robot_belief, shadow_labels): Checks whether the path between two points (start and end) collides with any obstacles in the robot's belief map or intersects with any shadowed areas.

6. NBVPWorkerSGPEG Class

    NBVPWorkerSGPEG: This is the main worker class responsible for running the exploration algorithm. It initializes the environment, manages the exploration process, and determines the next best viewpoints for the robot to move to.
    find_next_best_viewpoints(): This function is the core of the exploration process. It iteratively builds the exploration tree by sampling points in the environment, checking for collisions, and adding new vertices and edges to the tree. The function returns the best route found during the exploration.
    find_nearest_vertex(tree, coords): Finds the vertex in the tree that is closest to a given set of coordinates.
    steer(start, goal, step_length): Determines a new point in the direction of the goal but constrained by a maximum step length.
    initialize_tree_from_path(tree, path): Initializes the exploration tree from a previously determined best path.
    run_episode(currEpisode): Runs a single episode of exploration, where the robot repeatedly finds the next best viewpoints until it completes the exploration task or reaches a stopping condition. It also records performance metrics and optionally saves images of the exploration process.

7. Main Execution

    The main script runs multiple exploration episodes, initializing the worker class for each episode and collecting performance metrics. It prints the mean travel distance across episodes.

Key Concepts:

    Exploration Tree: The exploration tree is a structure that represents potential paths the robot can take. Each vertex in the tree is a viewpoint, and edges represent possible paths between viewpoints.
    Shadow Management: The concept of shadows is integrated into the exploration process to handle areas that are not directly visible to the robot. This ensures that the robot doesn't just avoid obstacles but also accounts for areas that may become visible later.
    Collision Checking: Collision checking is crucial for safe exploration, ensuring that the robot doesn't plan paths that intersect with obstacles or shadowed regions.




RESULTS- 
Detailed iterations and snaps of vertex and graph expansion is present in SPGEG folder
These are additional results which show the exploration and the pursuers and evader movement.
![Screenshot from 2024-08-11 00-39-19](https://github.com/user-attachments/assets/c8c35eab-cbdc-44c7-a3dd-d81480638e9d)

![Screenshot from 2024-08-09 20-01-26](https://github.com/user-attachments/assets/d47adbce-20da-434d-9490-b93b52828058)

![3_explored_rate_1_length_759 8](https://github.com/user-attachments/assets/2350cc27-1ef1-4620-981d-01ce2401e01b)

