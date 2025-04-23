import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline
import math
import time

# 1. Create Waypoints for 3 straight rows
def generate_rows(num_rows=10, row_length=10, spacing=1):
    rows = []
    for i in range(num_rows):
        x = [i * spacing] * 5
        y = np.linspace(0, row_length, 5)
        rows.append(list(zip(x, y)))
    return rows

# 2. Interpolate Waypoints
def interpolate_row(row_points, resolution=0.2):
    x = [p[0] for p in row_points]
    y = [p[1] for p in row_points]
    t = np.linspace(0, 1, len(x))
    cs_x = CubicSpline(t, x)
    cs_y = CubicSpline(t, y)
    t_new = np.linspace(0, 1, int(1 / resolution * len(x)))
    return list(zip(cs_x(t_new), cs_y(t_new)))

# 3. Heading and Navigation Helpers
def distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

def calculate_heading(p1, p2):
    return math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0])) % 360

def heading_difference(current, target):
    diff = (target - current + 540) % 360 - 180
    return diff

def get_nav_command(current_pos, heading, path, idx, lookahead=5, threshold=0.5):
    if idx >= len(path):
        return "End of row", idx
    target_idx = min(idx + lookahead, len(path) - 1)
    target_point = path[target_idx]
    dist = distance(current_pos, target_point)
    if dist < threshold:
        return get_nav_command(current_pos, heading, path, idx + 1, lookahead, threshold)
    desired_heading = calculate_heading(current_pos, target_point)
    diff = heading_difference(heading, desired_heading)
    if abs(diff) < 5:
        return "Drive straight", idx
    elif diff < 0:
        return "Align right", idx
    else:
        return "Align left", idx

# 4. Simulate Movement
def simulate_navigation():
    rows = generate_rows()
    paths = [interpolate_row(r) for r in rows]
    all_paths = []
    for i, row in enumerate(paths):
        all_paths.extend(row if i % 2 == 0 else row[::-1])  # zig-zag pattern

    rover_pos = list(all_paths[0])
    heading = 90  # facing up initially
    idx = 0

    plt.ion()
    fig, ax = plt.subplots()

    path_x, path_y = zip(*all_paths)
    ax.plot(path_x, path_y, 'k--', label="Path")

    while idx < len(all_paths):
        ax.clear()
        ax.plot(path_x, path_y, 'k--', alpha=0.5)
        ax.plot(rover_pos[0], rover_pos[1], 'ro', label="Rover")
        ax.quiver(rover_pos[0], rover_pos[1],
                  math.cos(math.radians(heading)),
                  math.sin(math.radians(heading)),
                  scale=10, color='blue', label="Heading")
        ax.set_xlim(-2, 15)
        ax.set_ylim(-1, 12)
        ax.set_aspect('equal')
        ax.legend()
        plt.draw()
        plt.pause(0.1)

        command, idx = get_nav_command(rover_pos, heading, all_paths, idx)
        print(f"[{idx}] Command: {command}")

        if command == "Drive straight":
            # simulate rover forward move
            dx = 0.2 * math.cos(math.radians(heading))
            dy = 0.2 * math.sin(math.radians(heading))
            rover_pos[0] += dx
            rover_pos[1] += dy
        elif command == "Align left":
            heading += 3
        elif command == "Align right":
            heading -= 3
        elif command.startswith("End of row"):
            print("Reached end of path.")
            break

simulate_navigation()
