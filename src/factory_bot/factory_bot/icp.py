"""Scan-to-map ICP localization (Lab 4's ICP, used to keep the robot located on the saved map).
Plain Python + NumPy, no ROS, so it can be unit-tested.

Idea: wheel odometry gives a GUESS of where the robot is. We place the lidar points on the
map at that guess. If the guess is slightly wrong, the points land slightly off the walls.
ICP repeatedly (1) pairs every scan point with its nearest wall point, (2) finds the rotation
+ translation that best moves the points onto their partners, and (3) applies it, until the
points sit on the walls. The final pose is the corrected robot pose.

Like the course says: ICP needs a good initial guess and overlap, and can lock onto a wrong
alignment in repetitive geometry. Odometry gives the guess; checks below reject bad results.
best_rigid_transform() follows icp() in the course's teaching/navigation.py (Lab 4).
"""
import math

import numpy as np


def best_rigid_transform(a, b):
    """Rotation R (2x2) and translation t that best move points a onto points b
    (least squares, the SVD method used in Lab 4). Both are (N, 2) arrays, row i paired."""
    ac, bc = a.mean(axis=0), b.mean(axis=0)
    u, _, vt = np.linalg.svd((a - ac).T @ (b - bc))
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:          # a mirror image is not a real robot motion
        vt[-1] *= -1
        rotation = vt.T @ u.T
    return rotation, bc - rotation @ ac


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max, laser_x=0.10, step=2):
    """Lidar ranges -> (N, 2) points in the ROBOT frame (x forward, y left).
    Drops broken readings (inf, NaN, too close, at max range). Uses every `step`-th beam."""
    r = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r > range_min) & (r < range_max * 0.99)
    ok[np.arange(len(r)) % step != 0] = False
    return np.column_stack([laser_x + r[ok] * np.cos(angles[ok]), r[ok] * np.sin(angles[ok])])


def transform(points, pose):
    """Robot-frame points -> map-frame points for robot pose (x, y, yaw)."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return points @ np.array([[c, s], [-s, c]]) + np.array([x, y])


class WallLookup:
    """For every cell of the map, remembers the NEAREST wall point (within max_distance).
    Built once at start; afterwards 'which wall point is closest to here?' is one array
    lookup instead of comparing against thousands of wall points."""

    def __init__(self, occupied, resolution, origin, max_distance=0.5):
        self.res = resolution
        self.origin = (float(origin[0]), float(origin[1]))
        self.h, self.w = occupied.shape
        rows, cols = np.nonzero(occupied)                       # row 0 = bottom of the map
        self.walls = np.column_stack([self.origin[0] + (cols + 0.5) * resolution,
                                      self.origin[1] + (rows + 0.5) * resolution])
        if len(self.walls) < 3:
            raise ValueError("map has fewer than 3 wall cells")
        # Label every wall cell with its number, then spread the labels outwards ring by ring
        # (nearest offsets first), so every free cell learns its nearest wall cell.
        wall_id = np.full((self.h, self.w), -1, dtype=np.int32)
        wall_id[rows, cols] = np.arange(len(rows))
        nearest = np.full((self.h, self.w), -1, dtype=np.int32)
        reach = int(math.ceil(max_distance / resolution))
        offsets = sorted(((dr, dc) for dr in range(-reach, reach + 1) for dc in range(-reach, reach + 1)
                          if dr * dr + dc * dc <= reach * reach), key=lambda o: o[0] ** 2 + o[1] ** 2)
        for dr, dc in offsets:
            # cells (r, c) whose neighbour (r + dr, c + dc) is a wall get that wall, if still unset
            src = wall_id[max(dr, 0):self.h + min(dr, 0), max(dc, 0):self.w + min(dc, 0)]
            dst = nearest[max(-dr, 0):self.h + min(-dr, 0), max(-dc, 0):self.w + min(-dc, 0)]
            take = (dst < 0) & (src >= 0)
            dst[take] = src[take]
        self.nearest = nearest.ravel()

    def lookup(self, points):
        """For map-frame points: (nearest wall point for each, mask of points that have one)."""
        cols = np.floor((points[:, 0] - self.origin[0]) / self.res).astype(int)
        rows = np.floor((points[:, 1] - self.origin[1]) / self.res).astype(int)
        inside = (cols >= 0) & (cols < self.w) & (rows >= 0) & (rows < self.h)
        idx = np.full(len(points), -1)
        idx[inside] = self.nearest[rows[inside] * self.w + cols[inside]]
        found = idx >= 0
        partners = np.zeros_like(points)
        partners[found] = self.walls[idx[found]]
        return partners, found


def icp_localize(scan_points, guess, walls, iterations=20, max_distance=0.3):
    """Correct a pose guess so the scan lands on the map's walls.

    scan_points : (N, 2) lidar points in the robot frame (from scan_to_points)
    guess       : (x, y, yaw) starting guess in the map frame (from odometry)
    walls       : WallLookup of the map
    Returns (pose, info). info: rms (m), matched (fraction of points with a wall partner),
    iterations used. The caller decides whether to trust it."""
    x, y, yaw = guess
    rms, matched, n_used = float("inf"), 0.0, 0
    for n_used in range(1, iterations + 1):
        moved = transform(scan_points, (x, y, yaw))
        partners, found = walls.lookup(moved)
        dist = np.linalg.norm(moved - partners, axis=1)
        keep = found & (dist < max_distance)                   # far points = not a wall (forklift...)
        matched = keep.mean() if len(keep) else 0.0
        if keep.sum() < 10:
            return (x, y, yaw), {"rms": float("inf"), "matched": float(matched), "iterations": n_used}
        rot, t = best_rigid_transform(moved[keep], partners[keep])
        # apply the small correction to the pose: new = correction o old
        x, y = rot @ np.array([x, y]) + t
        yaw = math.atan2(math.sin(yaw + math.atan2(rot[1, 0], rot[0, 0])),
                         math.cos(yaw + math.atan2(rot[1, 0], rot[0, 0])))
        rms = float(np.sqrt(np.mean(dist[keep] ** 2)))
        if np.linalg.norm(t) < 1e-4 and abs(math.atan2(rot[1, 0], rot[0, 0])) < 1e-4:
            break                                               # stopped moving: converged
    moved = transform(scan_points, (x, y, yaw))                # final quality check
    partners, found = walls.lookup(moved)
    dist = np.linalg.norm(moved - partners, axis=1)
    keep = found & (dist < max_distance)
    if keep.any():
        rms = float(np.sqrt(np.mean(dist[keep] ** 2)))
    return (float(x), float(y), float(yaw)), {"rms": rms, "matched": float(keep.mean()), "iterations": n_used}


def relocalize(scan_points, guess, walls, xy_range=3.0, xy_step=0.25, yaw_range=1.0, yaw_step=0.1, keep=10):
    """'I am lost' recovery (e.g. the robot was pushed): search many poses around `guess`
    (+-xy_range metres, +-yaw_range radians) for the one where the scan best lands on the walls.
      1. quick score for every candidate: fraction of scan points within 0.15 m of a wall
      2. full ICP from the `keep` best candidates; return the best result
    About 0.5-1 s, so it is only used after normal ICP has failed for a while."""
    offsets = np.arange(-xy_range, xy_range + 1e-9, xy_step)
    candidates = []
    for dyaw in np.arange(-yaw_range, yaw_range + 1e-9, yaw_step):
        yaw = guess[2] + dyaw
        turned = transform(scan_points, (0.0, 0.0, yaw))          # rotate once per heading ...
        for dx in offsets:
            for dy in offsets:
                x, y = guess[0] + dx, guess[1] + dy
                moved = turned + np.array([x, y])                 # ... then just shift
                partners, found = walls.lookup(moved)
                near = found & (np.linalg.norm(moved - partners, axis=1) < 0.15)
                candidates.append((near.mean(), (x, y, yaw)))
    candidates.sort(key=lambda c: -c[0])
    best_pose, best_info = guess, {"rms": float("inf"), "matched": 0.0, "iterations": 0}
    for _, start in candidates[:keep]:
        pose, info = icp_localize(scan_points, start, walls)
        if (info["matched"], -info["rms"]) > (best_info["matched"], -best_info["rms"]):
            best_pose, best_info = pose, info
    return best_pose, best_info
