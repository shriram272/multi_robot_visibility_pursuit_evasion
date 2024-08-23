import numpy as np
import copy

# def collision_check(x0, y0, x1, y1, ground_truth, robot_belief):
#     x0 = x0.round()
#     y0 = y0.round()
#     x1 = x1.round()
#     y1 = y1.round()
#     dx, dy = abs(x1 - x0), abs(y1 - y0)
#     x, y = x0, y0
#     error = dx - dy
#     x_inc = 1 if x1 > x0 else -1
#     y_inc = 1 if y1 > y0 else -1
#     dx *= 2
#     dy *= 2

#     collision_flag = 0
#     max_collision = 10

#     while 0 <= x < ground_truth.shape[1] and 0 <= y < ground_truth.shape[0]:
#         k = ground_truth.item(y, x)
#         if k == 1 and collision_flag < max_collision:
#             collision_flag += 1
#             if collision_flag >= max_collision:
#                 break

#         if k !=1 and collision_flag > 0:
#             break

#         if x == x1 and y == y1:
#             break

#         robot_belief.itemset((y, x), k)

#         if error > 0:
#             x += x_inc
#             error -= dy
#         else:
#             y += y_inc
#             error += dx

#     return robot_belief


# def sensor_work(robot_position, sensor_range, robot_belief, ground_truth):
#     sensor_angle_inc = 0.5 / 180 * np.pi
#     sensor_angle = 0
#     x0 = robot_position[0]
#     y0 = robot_position[1]
#     while sensor_angle < 2 * np.pi:
#         x1 = x0 + np.cos(sensor_angle) * sensor_range
#         y1 = y0 + np.sin(sensor_angle) * sensor_range
#         robot_belief = collision_check(x0, y0, x1, y1, ground_truth, robot_belief)
#         sensor_angle += sensor_angle_inc
#     return robot_belief

import numpy as np

def collision_check(x0, y0, x1, y1, ground_truth, robot_belief):
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x = int(x0)
    y = int(y0)
    n = 1 + dx + dy
    x_inc = 1 if x1 > x0 else -1
    y_inc = 1 if y1 > y0 else -1
    error = dx - dy
    dx *= 2
    dy *= 2

    for i in range(n):
        # Ensure x and y are within bounds
        x = np.clip(x, 0, ground_truth.shape[1] - 1)
        y = np.clip(y, 0, ground_truth.shape[0] - 1)
        
        k = ground_truth[y, x]
        robot_belief[y, x] = k

        if k == 1:
            return robot_belief

        if error > 0:
            x += x_inc
            error -= dy
        else:
            y += y_inc
            error += dx

    return robot_belief

def sensor_work(robot_position, sensor_range, robot_belief, ground_truth):
    x0, y0 = robot_position.astype(int)
    x_max, y_max = ground_truth.shape[1], ground_truth.shape[0]

    for i in range(max(0, x0 - sensor_range), min(x_max, x0 + sensor_range + 1)):
        for j in range(max(0, y0 - sensor_range), min(y_max, y0 + sensor_range + 1)):
            if (i - x0)**2 + (j - y0)**2 <= sensor_range**2:
                robot_belief = collision_check(x0, y0, i, j, ground_truth, robot_belief)

    return robot_belief


def unexplored_area_check(x0, y0, x1, y1, current_belief):
    x0 = x0.round()
    y0 = y0.round()
    x1 = x1.round()
    y1 = y1.round()
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    x, y = x0, y0
    error = dx - dy
    x_inc = 1 if x1 > x0 else -1
    y_inc = 1 if y1 > y0 else -1
    dx *= 2
    dy *= 2

    while 0 <= x < current_belief.shape[1] and 0 <= y < current_belief.shape[0]:
        k = current_belief.item(y, x)
        if x == x1 and y == y1:
            break

        if k == 1:
            break

        if k == 127:
            current_belief.itemset((y, x), 0)
            break

        if error > 0:
            x += x_inc
            error -= dy
        else:
            y += y_inc
            error += dx

    return current_belief


def calculate_utility(waypoint_position, sensor_range, robot_belief):
    sensor_angle_inc = 5 / 180 * np.pi
    sensor_angle = 0
    x0 = waypoint_position[0]
    y0 = waypoint_position[1]
    current_belief = copy.deepcopy(robot_belief)
    while sensor_angle < 2 * np.pi:
        x1 = x0 + np.cos(sensor_angle) * sensor_range
        y1 = y0 + np.sin(sensor_angle) * sensor_range
        current_belief = unexplored_area_check(x0, y0, x1, y1, current_belief)
        sensor_angle += sensor_angle_inc
    utility = np.sum(robot_belief == 127) - np.sum(current_belief == 127)
    return utility



# import numpy as np
# import copy

# def collision_check(x0, y0, x1, y1, ground_truth, robot_belief):
#     x0 = x0.round()
#     y0 = y0.round()
#     x1 = x1.round()
#     y1 = y1.round()
#     dx, dy = abs(x1 - x0), abs(y1 - y0)
#     x, y = x0, y0
#     error = dx - dy
#     x_inc = 1 if x1 > x0 else -1
#     y_inc = 1 if y1 > y0 else -1
#     dx *= 2
#     dy *= 2

#     collision_flag = 0
#     max_collision = 10

#     while 0 <= x < ground_truth.shape[1] and 0 <= y < ground_truth.shape[0]:
#         k = ground_truth.item(y, x)
#         if k == 1 and collision_flag < max_collision:
#             collision_flag += 1
#             if collision_flag >= max_collision:
#                 break

#         if k != 1 and collision_flag > 0:
#             break

#         if x == x1 and y == y1:
#             break

#         robot_belief.itemset((y, x), k)

#         if error > 0:
#             x += x_inc
#             error -= dy
#         else:
#             y += y_inc
#             error += dx

#     return robot_belief


# def sensor_work(robot_position, sensor_range, robot_belief, ground_truth, alpha_h, sensor_max_dist):
#     # Constrain the sensor angle to the horizontal FoV (alpha_h)
#     sensor_angle_inc = 0.5 / 180 * np.pi  # Increment in radians
#     sensor_angle_start = -alpha_h / 2
#     sensor_angle_end = alpha_h / 2
#     x0 = robot_position[0]
#     y0 = robot_position[1]

#     sensor_angle = sensor_angle_start
#     while sensor_angle <= sensor_angle_end:
#         x1 = x0 + np.cos(sensor_angle) * sensor_max_dist
#         y1 = y0 + np.sin(sensor_angle) * sensor_max_dist
#         robot_belief = collision_check(x0, y0, x1, y1, ground_truth, robot_belief)
#         sensor_angle += sensor_angle_inc

#     return robot_belief


# def unexplored_area_check(x0, y0, x1, y1, current_belief):
#     x0 = x0.round()
#     y0 = y0.round()
#     x1 = x1.round()
#     y1 = y1.round()
#     dx, dy = abs(x1 - x0), abs(y1 - y0)
#     x, y = x0, y0
#     error = dx - dy
#     x_inc = 1 if x1 > x0 else -1
#     y_inc = 1 if y1 > y0 else -1
#     dx *= 2
#     dy *= 2

#     while 0 <= x < current_belief.shape[1] and 0 <= y < current_belief.shape[0]:
#         k = current_belief.item(y, x)
#         if x == x1 and y == y1:
#             break

#         if k == 1:
#             break

#         if k == 127:
#             current_belief.itemset((y, x), 0)
#             break

#         if error > 0:
#             x += x_inc
#             error -= dy
#         else:
#             y += y_inc
#             error += dx

#     return current_belief


# def calculate_utility(waypoint_position, sensor_range, robot_belief, alpha_h):
#     # Constrain the sensor angle to the horizontal FoV (alpha_h)
#     sensor_angle_inc = 5 / 180 * np.pi
#     sensor_angle_start = -alpha_h / 2
#     sensor_angle_end = alpha_h / 2
#     x0 = waypoint_position[0]
#     y0 = waypoint_position[1]
#     current_belief = copy.deepcopy(robot_belief)

#     sensor_angle = sensor_angle_start
#     while sensor_angle <= sensor_angle_end:
#         x1 = x0 + np.cos(sensor_angle) * sensor_range
#         y1 = y0 + np.sin(sensor_angle) * sensor_range
#         current_belief = unexplored_area_check(x0, y0, x1, y1, current_belief)
#         sensor_angle += sensor_angle_inc

#     utility = np.sum(robot_belief == 127) - np.sum(current_belief == 127)
#     return utility




# import numpy as np
# import copy

# def collision_check(x0, y0, x1, y1, ground_truth, robot_belief, robot_position, horizontal_fov_angle):
#     def is_in_horizontal_fov(point, robot_position, horizontal_fov_angle):
#         direction_vector = point - robot_position
#         angle = np.arctan2(direction_vector[1], direction_vector[0])  # Angle in radians

#         # Assuming robot's forward direction is along the positive x-axis
#         robot_direction_angle = 0  # Adjust if robot's default direction is different

#         relative_angle = np.abs(angle - robot_direction_angle)

#         # Normalize angle to the range [-π, π]
#         relative_angle = (relative_angle + np.pi) % (2 * np.pi) - np.pi

#         # Check if the relative angle is within the horizontal FoV
#         horizontal_fov_rad = np.deg2rad(horizontal_fov_angle / 2.0)
#         return np.abs(relative_angle) <= horizontal_fov_rad

#     x0 = x0.round()
#     y0 = y0.round()
#     x1 = x1.round()
#     y1 = y1.round()
#     dx, dy = abs(x1 - x0), abs(y1 - y0)
#     x, y = x0, y0
#     error = dx - dy
#     x_inc = 1 if x1 > x0 else -1
#     y_inc = 1 if y1 > y0 else -1
#     dx *= 2
#     dy *= 2

#     collision_flag = 0
#     max_collision = 10

#     while 0 <= x < ground_truth.shape[1] and 0 <= y < ground_truth.shape[0]:
#         if not is_in_horizontal_fov(np.array([x, y]), robot_position, horizontal_fov_angle):
#             break

#         k = ground_truth.item(y, x)
#         if k == 1 and collision_flag < max_collision:
#             collision_flag += 1
#             if collision_flag >= max_collision:
#                 break

#         if k != 1 and collision_flag > 0:
#             break

#         if x == x1 and y == y1:
#             break

#         robot_belief.itemset((y, x), k)

#         if error > 0:
#             x += x_inc
#             error -= dy
#         else:
#             y += y_inc
#             error += dx

#     return robot_belief


# def sensor_work(robot_position, sensor_range, robot_belief, ground_truth, horizontal_fov_angle):
#     sensor_angle_inc = 0.5 / 180 * np.pi
#     sensor_angle = 0
#     x0 = robot_position[0]
#     y0 = robot_position[1]
#     while sensor_angle < 2 * np.pi:
#         x1 = x0 + np.cos(sensor_angle) * sensor_range
#         y1 = y0 + np.sin(sensor_angle) * sensor_range
#         robot_belief = collision_check(x0, y0, x1, y1, ground_truth, robot_belief, robot_position, horizontal_fov_angle)
#         sensor_angle += sensor_angle_inc
#     return robot_belief


# def unexplored_area_check(x0, y0, x1, y1, current_belief, robot_position, horizontal_fov_angle):
#     def is_in_horizontal_fov(point, robot_position, horizontal_fov_angle):
#         direction_vector = point - robot_position
#         angle = np.arctan2(direction_vector[1], direction_vector[0])  # Angle in radians

#         # Assuming robot's forward direction is along the positive x-axis
#         robot_direction_angle = 0  # Adjust if robot's default direction is different

#         relative_angle = np.abs(angle - robot_direction_angle)

#         # Normalize angle to the range [-π, π]
#         relative_angle = (relative_angle + np.pi) % (2 * np.pi) - np.pi

#         # Check if the relative angle is within the horizontal FoV
#         horizontal_fov_rad = np.deg2rad(horizontal_fov_angle / 2.0)
#         return np.abs(relative_angle) <= horizontal_fov_rad

#     x0 = x0.round()
#     y0 = y0.round()
#     x1 = x1.round()
#     y1 = y1.round()
#     dx, dy = abs(x1 - x0), abs(y1 - y0)
#     x, y = x0, y0
#     error = dx - dy
#     x_inc = 1 if x1 > x0 else -1
#     y_inc = 1 if y1 > y0 else -1
#     dx *= 2
#     dy *= 2

#     while 0 <= x < current_belief.shape[1] and 0 <= y < current_belief.shape[0]:
#         if not is_in_horizontal_fov(np.array([x, y]), robot_position, horizontal_fov_angle):
#             break

#         k = current_belief.item(y, x)
#         if x == x1 and y == y1:
#             break

#         if k == 1:
#             break

#         if k == 127:
#             current_belief.itemset((y, x), 0)
#             break

#         if error > 0:
#             x += x_inc
#             error -= dy
#         else:
#             y += y_inc
#             error += dx

#     return current_belief


# def calculate_utility(waypoint_position, sensor_range, robot_belief, horizontal_fov_angle):
#     sensor_angle_inc = 5 / 180 * np.pi
#     sensor_angle = 0
#     x0 = waypoint_position[0]
#     y0 = waypoint_position[1]
#     current_belief = copy.deepcopy(robot_belief)
#     while sensor_angle < 2 * np.pi:
#         x1 = x0 + np.cos(sensor_angle) * sensor_range
#         y1 = y0 + np.sin(sensor_angle) * sensor_range
#         current_belief = unexplored_area_check(x0, y0, x1, y1, current_belief, waypoint_position, horizontal_fov_angle)
#         sensor_angle += sensor_angle_inc
#     utility = np.sum(robot_belief == 127) - np.sum(current_belief == 127)
#     return utility
