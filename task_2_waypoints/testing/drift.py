import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline
import math
import random

# ——— Configuration —————————————————————————————————————————
TURN_RATE_PER_CM = 20            # degrees per cm of turn capability
DISTANCE_PER_STEP = 0.2          # cm traveled per simulation step
LOOKAHEAD_STEPS = 5              # baseline lookahead for drift recovery

# ——— Helpers ———————————————————————————————————————————————

def generate_farm_rows(num_rows=2, row_length=20, points_per_row=20, row_spacing=2):
    ys = np.linspace(0, row_length, points_per_row)
    return [[(i * row_spacing, y) for y in ys] for i in range(num_rows)]


def interpolate_row(pts, res=0.2):
    x, y = zip(*pts)
    t = np.linspace(0, 1, len(x))
    csx, csy = CubicSpline(t, x), CubicSpline(t, y)
    t2 = np.linspace(0, 1, int(1/res * len(x)))
    return list(zip(csx(t2), csy(t2)))


def dist(a, b): return math.hypot(b[0] - a[0], b[1] - a[1])
def head(a, b): return math.degrees(math.atan2(b[1]-a[1], b[0]-a[0])) % 360
def diff_h(c, t): return (t - c + 540) % 360 - 180

# ——— Drawing ————————————————————————————————————
def draw(ax, paths, pts, pos, hdg, title, xlim=(-2, 6), ylim=(-2, 22)):
    ax.clear()
    for p in paths:
        xs, ys = zip(*p)
        ax.plot(xs, ys, 'k--', lw=2, alpha=0.3)
    if len(pts) > 1:
        xs, ys = zip(*pts)
        ax.plot(xs, ys, 'b-', lw=3, alpha=0.7)
        ax.scatter(xs, ys, c='blue', s=60, alpha=0.6)
    ax.plot(pos[0], pos[1], 'ro', markersize=12)
    ax.quiver(pos[0], pos[1], math.cos(math.radians(hdg)), math.sin(math.radians(hdg)),
              scale=5, width=0.005)
    ax.set_title(title, fontsize=14)
    ax.set_aspect('equal')
    ax.set_autoscale_on(False)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.grid(True)
    plt.draw(); plt.pause(0.05)

# ——— Traverse one row with drift & recovery ——————————————————————
def traverse_row(ax, paths, row_idx, pos, hdg, start_idx=0):
    path = paths[row_idx]
    idx = start_idx
    trigger = start_idx + (len(path) - start_idx) // 3
    lookahead = LOOKAHEAD_STEPS
    end_idx = min(trigger + lookahead, len(path) - 1)

    # unit tangent vector
    dx, dy = path[end_idx][0] - path[trigger][0], path[end_idx][1] - path[trigger][1]
    mag = math.hypot(dx, dy)
    ux, uy = (dx/mag, dy/mag) if mag != 0 else (1.0, 0.0)

    # drift at ±45° relative to path heading
    angle = 45 if random.choice([True, False]) else -45
    rad = math.radians(angle)
    c, s = math.cos(rad), math.sin(rad)
    rx = ux * c + uy * s
    ry = -ux * s + uy * c

    # determine recovery index based on drift length and turn rate
    turn_dist = abs(angle) / TURN_RATE_PER_CM             # cm needed to turn back
    extra_skip = int(turn_dist / DISTANCE_PER_STEP)       # steps traveled during turn
    recovery_idx = min(trigger + lookahead + extra_skip, len(path) - 1)
    recovery_target = path[recovery_idx]

    drift_len, steps = 1.0, 20
    drift_delta = (rx * drift_len/steps, ry * drift_len/steps)

    drifted = False
    drifting = False
    recovering = False
    step = 0

    while idx < len(path):
        # trigger drift
        if idx == trigger and not drifted:
            drifting = True
            drifted = True
            step = 0

        # DRIFT PHASE
        if drifting:
            pos[0] += drift_delta[0]
            pos[1] += drift_delta[1]
            step += 1
            draw(ax, paths, [pos, recovery_target], pos, hdg, f"Row{row_idx+1} Drifting")
            if step >= steps:
                drifting = False
                recovering = True
            continue

        # RECOVERY PHASE
        if recovering:
            d = dist(pos, recovery_target)
            desired = head(pos, recovery_target)
            dh = diff_h(hdg, desired)
            turn_step = TURN_RATE_PER_CM * DISTANCE_PER_STEP
            if abs(dh) > 5:
                hdg += turn_step if dh > 0 else -turn_step
            else:
                pos[0] += DISTANCE_PER_STEP * math.cos(math.radians(hdg))
                pos[1] += DISTANCE_PER_STEP * math.sin(math.radians(hdg))
            draw(ax, paths, [pos, recovery_target], pos, hdg, f"Row{row_idx+1} Recovering")
            if d < 0.5:
                pos[:] = recovery_target
                idx = recovery_idx + 1
                recovering = False
            continue

        # NORMAL TRAVERSE
        target = path[idx]
        d = dist(pos, target)
        desired = head(pos, target)
        dh = diff_h(hdg, desired)
        turn_step = TURN_RATE_PER_CM * DISTANCE_PER_STEP
        if abs(dh) > 5:
            hdg += turn_step if dh > 0 else -turn_step
        else:
            pos[0] += DISTANCE_PER_STEP * math.cos(math.radians(hdg))
            pos[1] += DISTANCE_PER_STEP * math.sin(math.radians(hdg))
        draw(ax, paths, [target], pos, hdg, f"Row{row_idx+1} Traversing")
        if d < 0.5:
            idx += 1

    # finish row
    pos[:] = list(path[-1])
    draw(ax, paths, path, pos, hdg, f"Row{row_idx+1} Completed")
    plt.pause(1)
    return pos, hdg, 0

# ——— Transition & traverse —————————————————————————————
def transition_and_traverse(ax, paths, current_pos, hdg, next_row_idx):
    path = paths[next_row_idx]
    d0 = dist(current_pos, path[0])
    dn = dist(current_pos, path[-1])
    if dn < d0:
        paths[next_row_idx] = list(reversed(path))
        path = paths[next_row_idx]
    target = path[0]

    while dist(current_pos, target) > 0.5:
        desired = head(current_pos, target)
        dh = diff_h(hdg, desired)
        turn_step = TURN_RATE_PER_CM * DISTANCE_PER_STEP
        if abs(dh) > 5:
            hdg += turn_step if dh > 0 else -turn_step
        else:
            current_pos[0] += DISTANCE_PER_STEP * math.cos(math.radians(hdg))
            current_pos[1] += DISTANCE_PER_STEP * math.sin(math.radians(hdg))
        draw(ax, paths, [current_pos, target], current_pos, hdg,
             f"Transition to Row{next_row_idx+1}")

    return traverse_row(ax, paths, next_row_idx, current_pos, hdg, start_idx=0)

# ——— Main Simulation —————————————————————————————
def simulate_full():
    rows = generate_farm_rows(num_rows=2, row_length=20, points_per_row=30, row_spacing=3)
    paths = [interpolate_row(r) for r in rows]
    fig, ax = plt.subplots(figsize=(12, 9))
    plt.ion()

    pos = list(paths[0][0])
    hdg = 90

    pos, hdg, _ = traverse_row(ax, paths, 0, pos, hdg)
    for next_idx in range(1, len(paths)):
        pos, hdg, _ = transition_and_traverse(ax, paths, pos, hdg, next_idx)

    plt.ioff()
    plt.show()

if __name__ == "__main__":
    simulate_full()
