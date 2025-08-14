# car_path_follow_anim.py
import os, sys, csv, argparse, math
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import Polygon
from matplotlib.transforms import Affine2D

# ---------- helpers ----------
def load_nodes(path):
    nodes = {}
    with open(path, newline='') as f:
        r = csv.reader(f)
        for row in r:
            if not row or row[0].strip().startswith('#'): 
                continue
            try:
                nid = int(row[0].strip())
                x = float(row[1].strip()); y = float(row[2].strip())
                nodes[nid] = (x, y)
            except Exception:
                continue
    return nodes

def load_path_ids(path):
    with open(path, newline='') as f:
        line = f.readline()
        if not line:
            return []
        return [int(tok.strip()) for tok in line.split(',') if tok.strip().lstrip('-').isdigit()]

def load_obstacles(path):
    obs = []
    with open(path, newline='') as f:
        r = csv.reader(f)
        for row in r:
            if not row or row[0].strip().startswith('#'):
                continue
            try:
                obs.append((float(row[0]), float(row[1]), float(row[2])))
            except Exception:
                continue
    return obs

def resample_polyline(xy, ds=0.0025):
    if len(xy) < 2:
        return xy, np.array([0.0])
    d = np.diff(xy, axis=0)
    seg = np.hypot(d[:,0], d[:,1])
    s = np.r_[0.0, np.cumsum(seg)]
    L = s[-1]
    if L <= 1e-12:
        return xy, s
    s_new = np.arange(0.0, L, ds)
    x = np.interp(s_new, s, xy[:,0])
    y = np.interp(s_new, s, xy[:,1])
    return np.c_[x, y], s_new

def compute_heading(xy):
    if len(xy) < 2:
        return np.zeros((len(xy),))
    g = np.gradient(xy, axis=0)
    return np.unwrap(np.arctan2(g[:,1], g[:,0]))

def build_car_polygon(length=0.035, width=0.02):
    nose = np.array([+length/2, 0.0])
    rear_left  = np.array([-length/2, +width/2])
    rear_right = np.array([-length/2, -width/2])
    return np.vstack([nose, rear_left, rear_right])

# ---------- main ----------
def main():
    here = Path(__file__).resolve().parent
    csv_dir = here / "CSV"

    p = argparse.ArgumentParser()
    # Default to CSV/… relative to this script
    p.add_argument("--nodes", default=str(csv_dir / "nodes.csv"))
    p.add_argument("--path",  default=str(csv_dir / "path.csv"))
    p.add_argument("--obstacles", default=str(csv_dir / "obstacles.csv"))
    p.add_argument("--output", default="car_on_my_path")
    p.add_argument("--interval-ms", type=int, default=35)
    p.add_argument("--size", type=float, default=6.0)
    p.add_argument("--speed", type=float, default=0.6, help="constant speed (world units/s)")
    p.add_argument("--ds", type=float, default=0.0025, help="resampling spacing along path")
    args = p.parse_args()

    # Load data
    nodes = load_nodes(args.nodes)
    path_ids = load_path_ids(args.path)
    obstacles = load_obstacles(args.obstacles)

    # Build polyline from IDs
    poly = [nodes[i] for i in path_ids if i in nodes]
    if len(poly) < 2:
        raise SystemExit("Path has fewer than 2 valid nodes or IDs missing in nodes.csv.")
    poly = np.array(poly, dtype=float)

    # Resample + headings
    poly_u, s_u = resample_polyline(poly, ds=args.ds)
    heading = compute_heading(poly_u)

    # Time & kinematics (constant speed MVP)
    dif = np.diff(poly_u, axis=0)
    total_len = float(np.sum(np.hypot(dif[:,0], dif[:,1])))
    dt = args.interval_ms / 1000.0
    total_time = max(1.0, total_len / max(args.speed, 1e-6))
    T = np.arange(0.0, total_time, dt)
    v = np.full_like(T, args.speed)
    s = np.cumsum(v) * dt
    s = np.clip(s, 0.0, total_len)

    # Map s(t) to positions/heading
    seg = np.hypot(dif[:,0], dif[:,1])
    cs = np.cumsum(np.r_[0.0, seg])
    X = np.interp(s, cs, poly_u[:,0])
    Y = np.interp(s, cs, poly_u[:,1])
    H = np.interp(s, cs, heading)

    # Figure
    fig, ax = plt.subplots(figsize=(args.size, args.size))
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(-0.6, 0.6); ax.set_ylim(-0.6, 0.6)
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_title("Car Following Planned Path")

    # Boundary
    bx = [-0.55, 0.55, 0.55, -0.55, -0.55]
    by = [ 0.55, 0.55, -0.55, -0.55,  0.55]
    ax.plot(bx, by, 'k-', linewidth=2)

    # Obstacles
    for (ox, oy, d) in obstacles:
        circ = plt.Circle((ox, oy), d/2, color=(0, 0.4, 0.8), alpha=0.25, ec=None)
        ax.add_patch(circ)

    # Path
    ax.plot(poly[:,0], poly[:,1], color='red', lw=2.5, alpha=0.9, zorder=2)

    # Car
    car_shape = build_car_polygon(length=0.035, width=0.02)
    car_patch = Polygon(car_shape, closed=True, fc=(0.2,0.2,0.2), ec='black', lw=1.0, zorder=3)
    ax.add_patch(car_patch)
    hud = ax.text(0.02, 0.96, "", transform=ax.transAxes, ha='left', va='top', fontsize=9)

    def init():
        car_patch.set_xy(car_shape)
        hud.set_text("")
        return (car_patch, hud)

    def animate(i):
        x, y, ang = X[i], Y[i], H[i]
        trans = Affine2D().rotate(ang).translate(x, y) + ax.transData
        car_patch.set_transform(trans)
        hud.set_text(f"t = {T[i]:.2f} s   v = {v[i]:.2f} (units/s)")
        return (car_patch, hud)

    ani = animation.FuncAnimation(fig, animate, init_func=init,
                                  frames=len(T), interval=args.interval_ms, blit=True)

    out_base = os.path.splitext(args.output)[0]
    ok = False
    try:
        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=int(1000/args.interval_ms), metadata=dict(artist='CarPathFollow'), bitrate=2000)
        mp4 = out_base + ".mp4"
        ani.save(mp4, writer=writer)
        print("Saved:", mp4)
        ok = True
    except Exception as e:
        print("MP4 save failed:", e)

    if not ok:
        try:
            gif = out_base + ".gif"
            ani.save(gif, writer="pillow", fps=int(1000/args.interval_ms))
            print("Saved:", gif)
            ok = True
        except Exception as e:
            print("GIF save failed:", e)

    if not ok:
        print("No output saved. Please install ffmpeg or pillow.")

if __name__ == "__main__":
    main()
