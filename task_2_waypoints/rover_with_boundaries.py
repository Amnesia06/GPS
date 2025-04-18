import math
import time
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
import matplotlib.transforms as transforms

# --- Constants ---
STEP = 0.5  # Increased for faster navigation
TOLERANCE = 0.5  # Increased tolerance for reaching targets
MAX_ATTEMPTS = 200  # Increased maximum attempts
DEBUG = False
ANIMATION_SPEED = 0.05  # Rotation animation speed (lower is faster)


def get_float(prompt):
    """Get a float value from user with error handling"""
    while True:
        try:
            value = float(input(prompt))
            return value
        except ValueError:
            print("⚠️ Please enter a valid number.")


class Rover:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.history = []
        self.command_count = 0
        self.waypoint = None
        self.geofence = None
        self.inside_fence = False
        self.entry_point = None
        self.blocked_directions = set()
        self.stuck_count = 0  # Counter for detecting when rover is stuck
        self.position_history = []  # To detect cycles and stuck conditions
        self.rover_patch = None  # Visual representation of the rover
        self.fence_locked = False  # Track if rover is locked within the fence

    def set_position(self, x, y, heading=None, force=False, add_to_history=True):
        """Set rover position with optional heading"""
        if self.geofence and not force:
            # 1) Check if we're at (or within tolerance of) the entry point
            if self.entry_point and self.is_entry_point(x, y):
                # Entering through entry point: allow and lock inside
                self.inside_fence = True
                self.fence_locked = True
                print(f"🔓 Entering farm at entry point — rover locked inside.")
            else:
                # 2) Perform standard polygon check
                in_fence = self.is_point_in_polygon(x, y, self.geofence)
                # 3) Prevent exit once locked inside
                if self.fence_locked and self.inside_fence and not in_fence:
                    print(f"🔒 Movement blocked: Rover is locked inside the farm")
                    return False
                # 4) Block any crossing unless it is the first entry
                if (in_fence and not self.inside_fence) or (not in_fence and self.inside_fence):
                    print(f"⚠️ Movement blocked: would cross fence boundary")
                    return False
        # Apply new position/heading
        self.x = x
        self.y = y
        if heading is not None:
            self.heading = heading % 360
        if add_to_history:
            self.history.append((x, y))
        return True

    def set_waypoint(self, x, y):
        """Set target waypoint"""
        self.waypoint = (x, y)

    def set_geofence(self, vertices, entry_point):
        """Set geofence polygon and entry point"""
        self.geofence = vertices
        self.entry_point = entry_point
        self.inside_fence = self.is_point_in_polygon(self.x, self.y, vertices)

    def move_forward(self, distance, ax=None, fig=None, rover_patch=None):
        """Move rover forward in current heading direction"""
        if distance <= 0:
            print("⚠️ Invalid distance value <= 0")
            return False
        rad = math.radians(self.heading)
        target_x = self.x + distance * math.cos(rad)
        target_y = self.y + distance * math.sin(rad)
        # Smooth intermediate animation
        if ax and fig and rover_patch and distance > STEP:
            steps = min(int(distance / (STEP/2)), 5)
            if steps > 1:
                step_x = (target_x - self.x) / steps
                step_y = (target_y - self.y) / steps
                for _ in range(steps-1):
                    nx, ny = self.x + step_x, self.y + step_y
                    if not self.set_position(nx, ny, add_to_history=False):
                        return False
                    update_rover_visualization(self, ax, fig, rover_patch)
                    plt.pause(ANIMATION_SPEED/2)
        success = self.set_position(target_x, target_y)
        if success:
            self.command_count += 1
            print(f"Cmd #{self.command_count}: MOVE_FWD {distance:.2f}m → ({self.x:.3f}, {self.y:.3f})")
            if self.waypoint:
                print(f"   📏 Distance to waypoint: {self.distance_to(*self.waypoint):.2f}m")
            if self.entry_point and not self.inside_fence:
                print(f"   📏 Distance to entry point: {self.distance_to(*self.entry_point):.2f}m")
            return True
        else:
            rounded = int(self.heading/10)*10
            self.blocked_directions.add(rounded)
            print(f"⚠️ Movement in direction {rounded}° blocked")
            return False

    def calculate_heading_to(self, tx, ty):
        dx, dy = tx - self.x, ty - self.y
        if abs(dx)<1e-6 and abs(dy)<1e-6:
            return self.heading
        ang = math.degrees(math.atan2(dy, dx))
        return ang if ang>=0 else ang+360

    def distance_to(self, tx, ty):
        return math.hypot(tx-self.x, ty-self.y)

    def is_point_in_polygon(self, x, y, verts):
        n = len(verts)
        inside = False
        p1x, p1y = verts[0]
        for i in range(1, n+1):
            p2x, p2y = verts[i%n]
            if y > min(p1y,p2y) and y <= max(p1y,p2y) and x <= max(p1x,p2x):
                if p1y!=p2y:
                    xin = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                if p1x==p2x or x<=xin:
                    inside = not inside
            p1x,p1y = p2x,p2y
        return inside

    def is_entry_point(self, x, y, tolerance=TOLERANCE):
        if self.entry_point:
            return self.distance_to(*self.entry_point) <= tolerance
        return False

    def detect_and_resolve_stuck(self):
        self.position_history.append((self.x,self.y))
        if len(self.position_history)>10:
            self.position_history.pop(0)
        if len(self.position_history)>=5:
            maxd = max(math.hypot(self.x-px,self.y-py) for px,py in self.position_history)
            if maxd < TOLERANCE/2:
                self.stuck_count += 1
                print(f"⚠️ Possible stuck condition detected ({self.stuck_count}/3)")
                if self.stuck_count>=3:
                    print("🔄 Taking recovery action - making a significant turn")
                    self.blocked_directions.clear()
                    self.stuck_count=0
                    return (self.heading+120)%360
            else:
                self.stuck_count=0
        return None

# Visualization helpers (unchanged)
def safe_remove(element):
    if element:
        try:
            element.remove()
            return True
        except:
            if DEBUG: print(f"Warning: failed to remove {element}")
    return False

def create_rover_patch():
    verts = np.array([[0.7,0],[-0.3,0.4],[-0.3,-0.4]])
    return Polygon(verts, closed=True, fc='blue', ec='black')

def update_rover_visualization(rover, ax, fig, rover_patch=None):
    if rover_patch is None:
        rover_patch = create_rover_patch()
        ax.add_patch(rover_patch)
    tr = transforms.Affine2D().rotate_deg(rover.heading).translate(rover.x,rover.y)
    rover_patch.set_transform(tr+ax.transData)
    if hasattr(ax,'path_line') and len(rover.history)>1:
        ax.path_line.set_data(*zip(*rover.history))
    fig.canvas.draw_idle(); plt.pause(0.01)
    return rover_patch

def visualize_turn(rover, new_heading, ax, fig, rover_patch=None):
    current = rover.heading
    diff = (new_heading-current+180)%360 - 180
    if abs(diff)<5:
        rover.heading = new_heading
        return update_rover_visualization(rover, ax, fig, rover_patch)
    rover.command_count += 1
    print(f"Cmd #{rover.command_count}: ROTATE_TO {new_heading:.1f}° ({diff:.1f}° turn)")
    if rover.waypoint:
        print(f"   📏 Distance to waypoint: {rover.distance_to(*rover.waypoint):.2f}m")
    steps = max(5, min(int(abs(diff)/5), 36))
    step_ang = diff/steps
    try:
        for i in range(1, steps+1):
            rover.heading = (current + step_ang*i)%360
            rover_patch = update_rover_visualization(rover, ax, fig, rover_patch)
            plt.pause(ANIMATION_SPEED)
    except Exception as e:
        if DEBUG: print(f"Turn viz error: {e}")
        rover.heading = new_heading
    return update_rover_visualization(rover, ax, fig, rover_patch)

# Path planning (unchanged)
def find_best_path_angle(rover, tx, ty, blocked_angles=None):
    direct = rover.calculate_heading_to(tx,ty)
    if not blocked_angles or int(direct/10)*10 not in blocked_angles:
        return direct
    for off in range(10,360,10):
        for sign in (1,-1):
            ta = (direct+sign*off)%360
            if int(ta/10)*10 not in blocked_angles:
                return ta
    import random; return random.randint(0,359)


def navigate_to_point(rover, tx, ty, ax, fig, rover_patch=None, step_size=STEP, tolerance=TOLERANCE):
    print(f"\n🚗 Navigating to point ({tx:.3f}, {ty:.3f})...\n")
    dist = rover.distance_to(tx,ty)
    attempts=0; last_dist=float('inf'); alt=False; blocked=0
    while dist>tolerance and attempts<MAX_ATTEMPTS:
        attempts+=1
        rec = rover.detect_and_resolve_stuck()
        if rec is not None:
            rover_patch = visualize_turn(rover, rec, ax, fig, rover_patch)
            alt=True; continue
        if attempts%5==0:
            if dist>last_dist*0.95 and not alt:
                print("⚠️ Limited progress detected, trying alternative approach...")
                rover.blocked_directions.clear()
                angle=(rover.heading+90+attempts%90)%360
                rover_patch=visualize_turn(rover,angle,ax,fig,rover_patch)
                step_size=min(step_size*2,dist/2); alt=True
            else:
                alt=False; step_size=min(STEP,dist/2)
            last_dist=dist
        if blocked>2:
            tgt=find_best_path_angle(rover,tx,ty,rover.blocked_directions)
            blocked=0
        else:
            tgt=rover.calculate_heading_to(tx,ty)
        diff=(tgt-rover.heading+180)%360-180
        if abs(diff)>5:
            rover_patch=visualize_turn(rover,tgt,ax,fig,rover_patch)
        step=min(step_size,dist)
        ok=rover.move_forward(step,ax,fig,rover_patch)
        rover_patch=update_rover_visualization(rover,ax,fig,rover_patch)
        dist=rover.distance_to(tx,ty)
        if not ok:
            blocked+=1
            if blocked>=2:
                ch=45+blocked*15
                ch=min(ch,180)
                rover_patch=visualize_turn(rover,(rover.heading+ch)%360,ax,fig,rover_patch)
        else:
            blocked=0
    if dist<=tolerance:
        print(f"✅ Reached target point ({rover.x:.3f}, {rover.y:.3f})")
        return True, rover_patch
    print("🔄 Making final approach attempt with larger step size...")
    direct = rover.calculate_heading_to(tx,ty)
    rover_patch=visualize_turn(rover,direct,ax,fig,rover_patch)
    rover.move_forward(dist*0.9,ax,fig,rover_patch)
    fd=rover.distance_to(tx,ty)
    if fd<=tolerance*1.5:
        print(f"✅ Reached target point on final attempt ({rover.x:.3f}, {rover.y:.3f})")
        return True, rover_patch
    print(f"⚠️ Could not reach target point. Current position: ({rover.x:.3f}, {rover.y:.3f})")
    print(f"   Distance to target: {fd:.3f}")
    return False, rover_patch

# --- MAIN ---
def main():
    plt.rcParams['figure.max_open_warning'] = 50
    rover = Rover()
    print("🔧 Enter farm rectangle coordinates:")
    min_x = get_float(" Min X: ")
    max_x = get_float(" Max X: ")
    min_y = get_float(" Min Y: ")
    max_y = get_float(" Max Y: ")
    verts = [(min_x,min_y),(max_x,min_y),(max_x,max_y),(min_x,max_y)]
    entry = verts[0]
    print(f"🚪 Entry point set to bottom-left corner: ({entry[0]:.2f}, {entry[1]:.2f})")
    rover.set_geofence(verts,entry)
    print("🔧 Enter starting position:")
    while True:
        x1=get_float(" x1: "); y1=get_float(" y1: ")
        if rover.is_point_in_polygon(x1,y1,verts):
            print("⚠️ Starting point must be outside the farm. Please enter new coordinates.")
        else: break
    print(f"✅ Valid starting position: ({x1:.2f}, {y1:.2f})")
    print("🔧 Enter waypoint coords:")
    while True:
        wx=get_float(" wx: "); wy=get_float(" wy: ")
        if not rover.is_point_in_polygon(wx,wy,verts):
            print("⚠️ Waypoint must be inside the farm. Please enter new coordinates.")
        else: break
    print(f"✅ Valid waypoint: ({wx:.2f}, {wy:.2f})")
    rover.set_waypoint(wx,wy)
    try:
        plt.ion(); fig, ax = plt.subplots(figsize=(10,8))
        xs=[v[0] for v in verts]+[x1,wx,entry[0]]; ys=[v[1] for v in verts]+[y1,wy,entry[1]]
        mxx, Mxx = min(xs)-2, max(xs)+2; myy, Myy = min(ys)-2, max(ys)+2
        ax.set_xlim(mxx,Mxx); ax.set_ylim(myy,Myy); ax.grid(True)
        ax.set_title("Rover Farm Navigation Simulation")
        fence = Polygon(np.array(verts),closed=True,facecolor='lightgreen',edgecolor='darkgreen',alpha=0.3)
        ax.add_patch(fence)
        ax.scatter(entry[0],entry[1],c='purple',s=100,marker='o',label='Farm Entry')
        ax.scatter(x1,y1,c='green',s=80,label='Start (Outside)')
        ax.scatter(wx,wy,c='red',s=120,marker='*',label='Waypoint')
        pl,=ax.plot([],[], 'b-',alpha=0.5,label='Path'); ax.path_line=pl; ax.legend(loc='upper left')
        fig.canvas.draw_idle(); plt.pause(0.5)
    except Exception as e:
        print(f"Error in plot setup: {e}")
        fig, ax = plt.subplots(figsize=(8,6)); pl,=ax.plot([]); ax.path_line=pl; ax.set_title("Rover Navigation (Limited View)")
    rover.set_position(x1,y1,force=True,add_to_history=False)
    rover.history.append((rover.x,rover.y))
    rover_patch = update_rover_visualization(rover,ax,fig)
    print("\n🚜 Moving rover from outside farm to entry point...\n")
    print(f"📏 Initial distance to entry point: {rover.distance_to(*entry):.2f}m")
    for attempt in range(1,4):
        print(f"\n🔄 Entry point navigation attempt {attempt}/3...")
        reached, rover_patch = navigate_to_point(rover, entry[0],entry[1],ax,fig,rover_patch, step_size=STEP*attempt, tolerance=TOLERANCE)
        if reached:
            break
        if attempt<3:
            rover.blocked_directions.clear(); print("🔄 Retrying entry point navigation with new parameters...")
    if reached:
        try:
            ax.scatter(rover.x,rover.y,c='cyan',s=80,marker='^',label='Entry Reached'); ax.legend(loc='upper left'); fig.canvas.draw_idle(); plt.pause(1)
        except: pass
        rover.blocked_directions.clear(); rover.position_history.clear()
        print("\n🚜 Moving rover to waypoint inside farm...\n")
        print(f"📏 Initial distance to waypoint: {rover.distance_to(*rover.waypoint):.2f}m")
        for attempt in range(1,4):
            print(f"\n🔄 Waypoint navigation attempt {attempt}/3...")
            ok, rover_patch = navigate_to_point(rover, wx,wy,ax,fig,rover_patch, step_size=STEP*attempt, tolerance=TOLERANCE)
            if ok: break
            if attempt<3:
                rover.blocked_directions.clear(); print("🔄 Retrying waypoint navigation with new parameters...")
        if ok:
            try:
                ax.scatter(rover.x,rover.y,c='magenta',s=100,marker='o',label='Final Position'); ax.legend(loc='upper left'); fig.canvas.draw_idle(); plt.pause(0.5)
            except: pass
            print(f"\n✅ Mission complete! Reached waypoint in {rover.command_count} commands.")
        else:
            print("\n⚠️ Could not reach waypoint after multiple attempts.")
            print("   Consider adjusting sim parameters or waypoint location.")
    else:
        print("\n⚠️ Could not reach farm entry point after multiple attempts.")
        print("   Consider adjusting sim parameters or entry point location.")
    try:
        plt.ioff(); plt.title("Rover Farm Navigation Simulation"); plt.show(block=True)
    except Exception as e:
        print(f"Error in final plot display: {e}")
        print("Simulation completed without final visualization.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n🛑 Simulation terminated by user.")
    except Exception as e:
        print(f"\n❌ Simulation error: {e}")
        if DEBUG:
            import traceback; traceback.print_exc()
