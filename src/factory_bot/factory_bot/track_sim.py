"""Kinematic test bench for the controllers (like Lab 4's teaching.navigation 'follow'): a
simulated robot drives a planned path with NO Gazebo, so controllers can be compared quickly
under identical conditions. Plain Python + NumPy, no ROS.

The robot model: unicycle (x += v cos(yaw) dt, y += v sin(yaw) dt, yaw += w dt) with the
same speed and acceleration limits as our diff_drive_controller, and optional pose noise
(the controller sees a slightly wrong pose, like ICP's ~2-5 cm).
Metrics (what the coursework asks for: path validity, goal error, what the robot did):
  reached, time_s, goal_error_m, cross_track mean/max (m), min_clearance_m, jitter
"""
import math

import numpy as np

V_MAX, W_MAX = 0.5, 1.8           # same limits as config/fleet_controllers.yaml
A_MAX, ALPHA_MAX = 1.0, 3.0       # m/s^2, rad/s^2


def simulate(follower, path_xy, start_pose, dt=0.05, max_time=300.0, pose_noise=0.0, seed=0):
    """Drive the path. Returns dict with 'trajectory' (N x 3 array of x, y, yaw), 'commands'
    (N x 2 of v, w), 'reached' and 'time_s'."""
    rng = np.random.default_rng(seed)
    follower.set_path(path_xy)
    x, y, yaw = start_pose
    v = w = 0.0
    traj, cmds = [(x, y, yaw)], []
    for k in range(int(max_time / dt)):
        seen = (x + rng.normal(0, pose_noise), y + rng.normal(0, pose_noise),
                yaw + rng.normal(0, pose_noise)) if pose_noise else (x, y, yaw)
        v_cmd, w_cmd, done = follower.step(seen, dt)
        if done:
            return {"trajectory": np.array(traj), "commands": np.array(cmds), "reached": True,
                    "time_s": k * dt}
        # the wheels can't change speed instantly: limit acceleration, then speed
        v += max(-A_MAX * dt, min(A_MAX * dt, v_cmd - v))
        w += max(-ALPHA_MAX * dt, min(ALPHA_MAX * dt, w_cmd - w))
        v, w = max(-V_MAX, min(V_MAX, v)), max(-W_MAX, min(W_MAX, w))
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw = math.atan2(math.sin(yaw + w * dt), math.cos(yaw + w * dt))
        traj.append((x, y, yaw))
        cmds.append((v, w))
    return {"trajectory": np.array(traj), "commands": np.array(cmds), "reached": False, "time_s": max_time}


def cross_track(path_xy, traj):
    """Distance from every robot position to the nearest point ON the path (segments, not
    just vertices: measuring to vertices alone adds up to half a cell of fake error)."""
    a = np.asarray(path_xy[:-1], float)
    b = np.asarray(path_xy[1:], float)
    ab = b - a
    len2 = np.maximum((ab ** 2).sum(axis=1), 1e-12)
    out = np.empty(len(traj))
    for i, p in enumerate(traj[:, :2]):
        t = np.clip(((p - a) * ab).sum(axis=1) / len2, 0.0, 1.0)
        out[i] = np.min(np.linalg.norm(a + t[:, None] * ab - p, axis=1))
    return out


def min_clearance(grid, traj, robot_radius=0.234):
    """Smallest gap between the robot's body (a circle) and any wall cell along the trajectory.
    Negative = the body overlapped a wall."""
    rows, cols = np.nonzero(grid.occupied)
    walls = np.column_stack([grid.origin[0] + (cols + 0.5) * grid.res, grid.origin[1] + (rows + 0.5) * grid.res])
    best = math.inf
    for p in traj[::2, :2]:                                   # every 2nd pose is plenty
        d = np.linalg.norm(np.maximum(np.abs(walls - p) - grid.res / 2, 0), axis=1)   # to each square's edge
        best = min(best, float(d.min()) - robot_radius)
    return best


def jitter(commands, dt=0.05):
    """How jerky the steering was: spread of the change in w between cycles (rad/s per cycle)."""
    if len(commands) < 3:
        return 0.0
    return float(np.std(np.diff(commands[:, 1])))


def evaluate(grid, follower, path_xy, start_pose, **kw):
    """simulate() + every metric, as one flat dict for the CSV."""
    r = simulate(follower, path_xy, start_pose, **kw)
    ct = cross_track(path_xy, r["trajectory"])
    end = r["trajectory"][-1]
    return {"reached": r["reached"], "time_s": round(r["time_s"], 2),
            "goal_error_m": round(math.dist(end[:2], path_xy[-1]), 3),
            "cross_track_mean_m": round(float(ct.mean()), 3), "cross_track_max_m": round(float(ct.max()), 3),
            "min_clearance_m": round(min_clearance(grid, r["trajectory"]), 3),
            "jitter": round(jitter(r["commands"]), 4), "_trajectory": r["trajectory"]}
