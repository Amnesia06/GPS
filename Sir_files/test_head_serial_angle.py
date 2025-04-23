import numpy as np
import matplotlib.pyplot as plt
from math import atan2, degrees, radians, sin, cos, sqrt

class Rover:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y
        self.heading = 0  # Initial heading (radians, 0 = East)
        self.history = [(x, y)]  # Track path
        
    def move_forward(self, distance):
        """Move forward and record position."""
        self.x += distance * cos(self.heading)
        self.y += distance * sin(self.heading)
        self.history.append((self.x, self.y))
        
    def turn(self, angle_degrees):
        """Turn left (positive) or right (negative)."""
        self.heading += radians(angle_degrees)
        
    def calculate_heading_to(self, target_x, target_y):
        """Returns angle (degrees) to face target."""
        dx = target_x - self.x
        dy = target_y - self.y
        return degrees(atan2(dy, dx))
    
    def distance_to(self, target_x, target_y):
        """Returns distance to target."""
        return sqrt((target_x - self.x)**2 + (target_y - self.y)**2)

# Initialize rover and waypoint
rover = Rover(x=0, y=0)
waypoint = (3, 4)  # Target (adjust as needed)

# --- STEP 1: Move 1m to reveal heading ---
rover.move_forward(1)
initial_heading = degrees(rover.heading)
print(f"Rover's initial heading: {initial_heading:.2f}°")

# --- STEP 2: Interactive navigation ---
plt.ion()  # Enable interactive mode
fig, ax = plt.subplots(figsize=(8, 6))

def update_plot():
    """Update the live plot."""
    ax.clear()
    ax.plot(*zip(*rover.history), 'b-', label='Path')
    ax.scatter(0, 0, c='green', label='Start')
    ax.scatter(*waypoint, c='red', label='Waypoint')
    ax.arrow(rover.x, rover.y, 0.5*cos(rover.heading), 0.5*sin(rover.heading), 
             head_width=0.2, fc='black')
    ax.set_xlim(-1, 5)
    ax.set_ylim(-1, 5)
    ax.grid()
    ax.legend()
    ax.set_title("Live Rover Navigation")
    fig.canvas.flush_events()

update_plot()

# Command loop
while (rover.distance_to(*waypoint) > 0.1):  # Stop when close
    angle = rover.calculate_heading_to(*waypoint)
    turn_angle = (angle - degrees(rover.heading) + 180) % 360 - 180  # Normalize to [-180, 180]
    distance = rover.distance_to(*waypoint)
    
    print(f"\nCurrent position: ({rover.x:.2f}, {rover.y:.2f})")
    print(f"Waypoint angle: {angle:.2f}° | Turn needed: {turn_angle:.2f}° | Distance: {distance:.2f}m")
    
    command = input("Command (turn_angle distance, e.g., '30 1.5' or 'q' to quit): ")
    if command == 'q':
        break
    
    try:
        turn, dist = map(float, command.split())
        rover.turn(turn)
        rover.move_forward(dist)
        update_plot()
    except:
        print("Invalid command. Use 'angle distance' (e.g., '45 2')")

plt.ioff()
plt.show()