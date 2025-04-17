import numpy as np
import matplotlib.pyplot as plt
from math import atan2, degrees, radians, sin, cos, sqrt
import time

class Rover:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y
        self.heading = 0  # radians (0 = East)
        self.history = [(x, y)]
        
    def move_forward(self, distance):
        target_dist = self.distance_to(*waypoint)
        if target_dist <= 0.001:
            return 0
        
        actual_move = min(distance, target_dist)
        self.x += actual_move * cos(self.heading)
        self.y += actual_move * sin(self.heading)
        self.history.append((self.x, self.y))
        print(f"\nCommand: Move forward {actual_move:.2f}m")
        print(f"   ➤ Rover Position: ({self.x:.3f}, {self.y:.3f})")
        print(f"   ➤ Waypoint: ({waypoint[0]:.3f}, {waypoint[1]:.3f})")
        print(f"   ➤ Remaining distance: {self.distance_to(*waypoint):.3f}m")
        return actual_move
        
    def turn(self, angle_degrees):
        self.heading = (self.heading + radians(angle_degrees)) % (2*np.pi)
        direction = "left" if angle_degrees > 0 else "right"
        print(f"\nCommand: Turn {direction} {abs(angle_degrees):.1f}°")
        print(f"   ➤ Rover Heading: {degrees(self.heading):.1f}°")
        print(f"   ➤ Rover Position: ({self.x:.3f}, {self.y:.3f})")
        print(f"   ➤ Remaining distance: {self.distance_to(*waypoint):.3f}m")

    def calculate_heading_to(self, target_x, target_y):
        dx = target_x - self.x
        dy = target_y - self.y
        return atan2(dy, dx)
    
    def distance_to(self, target_x, target_y):
        return sqrt((target_x - self.x)**2 + (target_y - self.y)**2)

# === Setup ===
rover = Rover()
waypoint = (5, 3)
step_size = 0.2
tolerance = 0.001

# === Plot Setup ===
plt.ion()
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_xlim(-1, 6)
ax.set_ylim(-1, 6)
ax.grid(True)
ax.set_title('Rover Navigation - Smooth Turn Then Drive')
ax.scatter(0, 0, c='green', label='Start')
ax.scatter(*waypoint, c='red', label='Waypoint')
path_line, = ax.plot([], [], 'b-', alpha=0.5, label='Path')
heading_arrow = ax.arrow(0, 0, 0.1, 0, head_width=0.2, fc='k')
ax.legend()

# === Initial Movement for Heading Determination ===
initial_move = 1.0
while initial_move > 0:
    move_dist = min(0.1, initial_move)
    rover.move_forward(move_dist)
    initial_move -= move_dist

    path_line.set_data(*zip(*rover.history))
    heading_arrow.remove()
    heading_arrow = ax.arrow(
        rover.x, rover.y,
        0.3 * cos(rover.heading),
        0.3 * sin(rover.heading),
        head_width=0.2, fc='k'
    )
    fig.canvas.flush_events()
    time.sleep(0.05)

print(f"\nInitial heading: {degrees(rover.heading):.1f}°")

# === Smooth One-Time Turn to Face Waypoint ===
target_angle = rover.calculate_heading_to(*waypoint)
turn_angle = degrees((target_angle - rover.heading + np.pi) % (2 * np.pi) - np.pi)

if abs(turn_angle) > 1:
    num_turn_steps = int(abs(turn_angle) / 2) or 1
    for _ in range(num_turn_steps):
        small_turn = turn_angle / num_turn_steps
        rover.turn(small_turn)

        heading_arrow.remove()
        heading_arrow = ax.arrow(
            rover.x, rover.y,
            0.3 * cos(rover.heading),
            0.3 * sin(rover.heading),
            head_width=0.2, fc='k'
        )
        fig.canvas.flush_events()
        time.sleep(0.05)

# === Move Straight Toward Waypoint ===
while rover.distance_to(*waypoint) > tolerance:
    move_dist = min(step_size, rover.distance_to(*waypoint))
    rover.move_forward(move_dist)

    path_line.set_data(*zip(*rover.history))
    heading_arrow.remove()
    arrow_length = min(0.3, rover.distance_to(*waypoint) * 0.7)
    heading_arrow = ax.arrow(
        rover.x, rover.y,
        arrow_length * cos(rover.heading),
        arrow_length * sin(rover.heading),
        head_width=0.2, fc='k'
    )
    fig.canvas.flush_events()
    time.sleep(0.05)

# === Final Touch ===
heading_arrow.remove()
ax.scatter(waypoint[0], waypoint[1], c='black', marker='>', s=200, label='Final Heading')
ax.legend()
plt.ioff()
print("\n✅ Waypoint reached with perfect precision!")
plt.show()
