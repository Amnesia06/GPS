import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math

# Read waypoints CSV
def read_waypoints(filename):
    df = pd.read_csv(filename)
    df.columns = df.columns.str.strip()  # Strip any extra whitespace
    return df

# Distance function
def distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

# Heading calculation
def calculate_heading(p1, p2):
    return math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0])) % 360

# Heading difference
def heading_difference(current, target):
    diff = (target - current + 540) % 360 - 180
    return diff

# Get navigation command (drive or align)
def get_nav_command(current_pos, heading, path_xy, idx, lookahead=5, threshold=0.5):
    if idx >= len(path_xy):
        return "End of path", idx
    target_idx = min(idx + lookahead, len(path_xy) - 1)
    target_point = path_xy[target_idx]
    dist = distance(current_pos, target_point)
    if dist < threshold:
        return get_nav_command(current_pos, heading, path_xy, idx + 1, lookahead, threshold)
    desired_heading = calculate_heading(current_pos, target_point)
    diff = heading_difference(heading, desired_heading)
    if abs(diff) < 5:
        return "Drive straight", idx
    elif diff < 0:
        return "Align right", idx
    else:
        return "Align left", idx

# Simulate Rover Navigation
def simulate_navigation(filename):
    waypoints = read_waypoints(filename)
    path_xy = list(zip(waypoints['x'], waypoints['y']))
    
    rover_pos = list(path_xy[0])
    heading = 90  # Initial heading (facing up)
    idx = 0
    
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 6))
    # Make room on the right for the legend
    fig.subplots_adjust(right=0.75)

    path_x, path_y = zip(*path_xy)
    ax.plot(path_x, path_y, 'k--', alpha=0.5, label="Planned Path")

    while idx < len(path_xy):
        ax.clear()
        ax.plot(path_x, path_y, 'k--', alpha=0.5, label="Planned Path")
        ax.plot(rover_pos[0], rover_pos[1], 'ro', label="Rover")
        ax.quiver(
            rover_pos[0], rover_pos[1],
            math.cos(math.radians(heading)),
            math.sin(math.radians(heading)),
            scale=10, color='blue', label="Heading"
        )
        
        ax.set_xlim(min(path_x) - 1, max(path_x) + 1)
        ax.set_ylim(min(path_y) - 1, max(path_y) + 1)
        ax.set_aspect('equal')
        # Place legend outside to the right
        ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1.0))
        
        plt.draw()
        plt.pause(0.1)
        
        command, idx = get_nav_command(rover_pos, heading, path_xy, idx)
        print(f"[{idx}] Command: {command}")

        if command == "Drive straight":
            dx = 0.2 * math.cos(math.radians(heading))
            dy = 0.2 * math.sin(math.radians(heading))
            rover_pos[0] += dx
            rover_pos[1] += dy
        elif command == "Align left":
            heading = (heading + 3) % 360
        elif command == "Align right":
            heading = (heading - 3) % 360
        elif command == "End of path":
            print("Reached the end of the path.")
            break

    plt.ioff()
    plt.show()

if __name__ == "__main__":
    simulate_navigation(r"F:\GPS\task_2_waypoints\waypoints.csv")
