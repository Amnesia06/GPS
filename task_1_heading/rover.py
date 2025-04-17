import math
import time
import matplotlib.pyplot as plt
import numpy as np

class Rover:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.history = [(self.x, self.y)]
        self.command_count = 0
        self.waypoint = None

    def set_position(self, x, y):
        self.command_count += 1
        self.x, self.y = x, y
        self.history.append((self.x, self.y))
        print(f"Command #{self.command_count}: SET_POSITION")
        print(f"➡️ Rover position: ({self.x:.3f}, {self.y:.3f})")
        if self.waypoint:
            self.report_status()

    def move_forward(self, distance):
        self.command_count += 1
        rad = math.radians(self.heading)
        old_x, old_y = self.x, self.y
        self.x += distance * math.cos(rad)
        self.y += distance * math.sin(rad)
        self.history.append((self.x, self.y))
        print(f"Command #{self.command_count}: MOVE_FORWARD {distance:.2f}")
        print(f"➡️ Rover moved from ({old_x:.3f}, {old_y:.3f}) to ({self.x:.3f}, {self.y:.3f})")
        if self.waypoint:
            self.report_status()

    def rotate_by(self, angle_deg):
        self.command_count += 1
        old_heading = self.heading
        self.heading = (self.heading + angle_deg) % 360
        print(f"Command #{self.command_count}: ROTATE_BY {angle_deg:.1f}°")
        print(f"🔄 Rover rotated from {old_heading:.1f}° to {self.heading:.1f}°")
        if self.waypoint:
            self.report_status()

    def calculate_heading_to(self, tx, ty):
        dx, dy = tx - self.x, ty - self.y
        return math.degrees(math.atan2(dy, dx)) % 360

    def distance_to(self, tx, ty):
        return math.hypot(tx - self.x, ty - self.y)

    def set_waypoint(self, x, y):
        self.waypoint = (x, y)

    def report_status(self):
        if not self.waypoint:
            print("\n--- STATUS REPORT ---")
            print(f"📍 Position: ({self.x:.3f}, {self.y:.3f})")
            print(f"🧭 Heading: {self.heading:.1f}°")
            print("🎯 No waypoint set")
            print("---------------------\n")
            return None, None
        dist = self.distance_to(*self.waypoint)
        desired = self.calculate_heading_to(*self.waypoint)
        diff = (desired - self.heading + 180) % 360 - 180
        print("\n--- STATUS REPORT ---")
        print(f"📍 Position: ({self.x:.3f}, {self.y:.3f})")
        print(f"🧭 Heading: {self.heading:.1f}°")
        print(f"🎯 Distance to waypoint: {dist:.3f}")
        print(f"🔄 Angle adj needed: {diff:.1f}°")
        print("---------------------\n")
        return dist, diff

def visualize_turn(rover, target_heading, ax, fig):
    curr = rover.heading
    diff = (target_heading - curr + 180) % 360 - 180
    if abs(diff) < 5:
        return
    steps = max(5, min(36, abs(int(diff / 10))))
    step_ang = diff / steps

    rad_t = math.radians(target_heading)
    targ = ax.arrow(rover.x, rover.y,
                    STEP * math.cos(rad_t),
                    STEP * math.sin(rad_t),
                    head_width=0.1, fc='red', ec='red', alpha=0.7)
    note = ax.annotate(f"Turning {abs(diff):.1f}°",
                       xy=(rover.x, rover.y),
                       xytext=(rover.x+0.5, rover.y+0.5),
                       arrowprops=dict(facecolor='black', shrink=0.05),
                       fontsize=9)

    arrow = None
    for i in range(steps+1):
        ang = (curr + i*step_ang) % 360
        if arrow:
            arrow.remove()
        rad = math.radians(ang)
        arrow = ax.arrow(rover.x, rover.y,
                         STEP * math.cos(rad),
                         STEP * math.sin(rad),
                         head_width=0.1, fc='blue', ec='blue')
        fig.canvas.draw()
        plt.pause(0.05)

    if arrow: arrow.remove()
    targ.remove()
    note.remove()
    rover.heading = target_heading

# --- MAIN ---
STEP = 0.2
TOLERANCE = 0.3
rover = Rover()

# 1) User inputs
print("🔧 Enter start coords:")
x1 = float(input(" x1: "))
y1 = float(input(" y1: "))
print("🔧 Enter 2nd coords (to set heading):")
x2 = float(input(" x2: "))
y2 = float(input(" y2: "))
print("🎯 Enter waypoint coords:")
wx = float(input(" wx: "))
wy = float(input(" wy: "))
rover.set_waypoint(wx, wy)

# 2) Plot setup
plt.ion()
fig, ax = plt.subplots(figsize=(8,6))
mx, Mx = min(x1,x2,wx)-1, max(x1,x2,wx)+1
my, My = min(y1,y2,wy)-1, max(y1,y2,wy)+1
ax.set_xlim(mx, Mx); ax.set_ylim(my, My); ax.grid(True)
ax.scatter(x1,y1,c='green',s=80,label='Start')
ax.scatter(wx,wy,c='red',s=120,marker='*',label='Waypoint')
path_line, = ax.plot([],[],'b-',alpha=0.5,label='Path')
ax.legend(loc='upper left'); fig.canvas.draw(); plt.pause(0.1)

# 3) Initial move & heading
rover.set_position(x1,y1)
h0 = rover.calculate_heading_to(x2,y2)
rover.heading = h0
print(f"🧭 Initial heading: {rover.heading:.1f}°")
dist = rover.distance_to(x2,y2)
while dist > STEP:
    rover.move_forward(min(STEP,dist))
    path_line.set_data(*zip(*rover.history))
    fig.canvas.draw(); plt.pause(0.05)
    dist = rover.distance_to(x2,y2)
print(f"🧭 Initial move done. Heading: {rover.heading:.1f}°")

# 4) Mark turn start
turn_pt = (rover.x, rover.y)
ax.scatter(turn_pt[0], turn_pt[1],
           c='orange',marker='D',s=100,label='Turn Start')
ax.legend(loc='upper left'); fig.canvas.draw(); plt.pause(0.1)

# 5) Align if >90°
dist, diff = rover.report_status()
if abs(diff) > 90:
    print("⚠️ >90° off — aligning first...")
    tgt = rover.calculate_heading_to(wx,wy)
    visualize_turn(rover,tgt,ax,fig)
    old = rover.heading; rover.heading = tgt; rover.command_count+=1
    print(f"Cmd#{rover.command_count}: ROTATE_BY {diff:.1f}° → {rover.heading:.1f}°")
else:
    print("✅ Within 90° — no preliminary alignment.")

# 6) Drive to waypoint
print("🚗 Driving to waypoint...\n")
dist = rover.distance_to(wx,wy)
arrow = None
while dist > TOLERANCE:
    tgt = rover.calculate_heading_to(wx,wy)
    df = (tgt - rover.heading + 180) % 360 - 180
    if abs(df) > 5:
        visualize_turn(rover,tgt,ax,fig)
        old=rover.heading; rover.heading=tgt; rover.command_count+=1
        print(f"Cmd#{rover.command_count}: ROTATE_BY {df:.1f}° → {rover.heading:.1f}°")
        if arrow: arrow.remove()
    rover.move_forward(min(STEP, dist))
    dist = rover.distance_to(wx,wy)
    path_line.set_data(*zip(*rover.history))
    if arrow: arrow.remove()
    rad = math.radians(rover.heading)
    arrow = ax.arrow(rover.x,rover.y,
                     STEP*math.cos(rad),
                     STEP*math.sin(rad),
                     head_width=0.1,fc='blue',ec='blue')
    fig.canvas.draw(); plt.pause(0.05)

print(f"\n✅ Reached ({rover.x:.3f},{rover.y:.3f}) in {rover.command_count} commands.")
plt.ioff(); plt.show()
