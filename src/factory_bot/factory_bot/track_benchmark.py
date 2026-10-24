"""track_benchmark: PID vs Pure Pursuit on every station-to-station route of the factory
(A* routes, kinematic robot, same start, same noise), plus one picture for the report.
  ros2 run factory_bot track_benchmark
  ros2 run factory_bot track_benchmark --noise 0.05 --example TOOL_CRIB ASSEMBLY_A
Outputs in ~/factory_ws/results/: tracking_benchmark.csv, tracking_example.png
"""
import argparse
import csv
import itertools
import os
import statistics

import numpy as np
import yaml

from factory_bot.controllers import make_follower
from factory_bot.planners import path_to_world, plan
from factory_bot.planning_grid import PlanningGrid
from factory_bot.show_grid import grid_image, share_dir
from factory_bot.track_sim import evaluate

CONTROLLERS = ("pid", "pure_pursuit")
COLOURS = {"pid": (220, 40, 40), "pure_pursuit": (20, 160, 60)}


def run_all(grid, stations, settings, noise):
    rows, examples = [], {}
    for a, b in itertools.combinations(stations, 2):
        allow = "QC_LAB" in (a, b)
        route = plan(grid, stations[a][:2], stations[b][:2], "astar", allow_restricted=allow)
        if not route.success:
            print(f"  {a} -> {b}: no route ({route.reason})")
            continue
        path = path_to_world(grid, route.path)
        start = (path[0][0], path[0][1], stations[a][2])          # face the station's direction
        for name in CONTROLLERS:
            m = evaluate(grid, make_follower(name, settings), path, start, pose_noise=noise, seed=1)
            examples[(a, b, name)] = (path, m.pop("_trajectory"))
            rows.append({"from": a, "to": b, "controller": name, "path_m": round(route.length_m, 2), **m})
    return rows, examples


def summary(rows):
    print(f"\n{'controller':13s} {'reached':>8s} {'time s':>7s} {'goal err m':>11s} {'track mean':>11s} "
          f"{'track max':>10s} {'min clear m':>12s} {'jitter':>7s}")
    for name in CONTROLLERS:
        mine = [r for r in rows if r["controller"] == name]
        mean = lambda k: statistics.mean(r[k] for r in mine)     # noqa: E731
        print(f"{name:13s} {sum(r['reached'] for r in mine):3d}/{len(mine):<4d} {mean('time_s'):7.1f} "
              f"{mean('goal_error_m'):11.3f} {mean('cross_track_mean_m'):11.3f} "
              f"{max(r['cross_track_max_m'] for r in mine):10.3f} {min(r['min_clearance_m'] for r in mine):12.3f} "
              f"{mean('jitter'):7.4f}")
    print("(averages over all routes; 'track max' = worst over all routes, 'min clear' = closest any route came)")


def draw_example(grid, examples, a, b, out):
    """Left: the whole route. Right: a 2.5 m close-up where the two controllers differ most."""
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    import pygame
    pygame.init()
    font = pygame.font.SysFont(None, 22)
    size = 520                                                # each panel is size x size pixels
    img = grid_image(grid)
    path = examples[(a, b, CONTROLLERS[0])][0]
    trajs = {n: examples[(a, b, n)][1] for n in CONTROLLERS}

    def panel(x0, y0, span):
        """Draw the square of the map with lower-left corner (x0, y0) and side `span` metres."""
        c0, r0 = int((x0 - grid.origin[0]) / grid.res), int((y0 - grid.origin[1]) / grid.res)
        n = int(span / grid.res)
        crop = np.full((n, n, 3), 150, dtype=np.uint8)        # grey outside the map
        rs, cs = slice(max(r0, 0), min(r0 + n, grid.height)), slice(max(c0, 0), min(c0 + n, grid.width))
        crop[rs.start - r0:rs.stop - r0, cs.start - c0:cs.stop - c0] = img[rs, cs]
        surf = pygame.transform.scale(pygame.surfarray.make_surface(crop[::-1].transpose(1, 0, 2)), (size, size))
        k = size / span
        to_px = lambda x, y: ((x - x0) * k, size - (y - y0) * k)          # noqa: E731
        pygame.draw.lines(surf, (40, 70, 230), False, [to_px(*p) for p in path], 2)
        for name, traj in trajs.items():
            pygame.draw.lines(surf, COLOURS[name], False, [to_px(x, y) for x, y, _ in traj], 2)
        return surf

    whole = max(grid.width, grid.height) * grid.res
    pid, pp = trajs["pid"], trajs["pure_pursuit"]
    m = min(len(pid), len(pp))
    worst = int(np.argmax(np.linalg.norm(pid[:m, :2] - pp[:m, :2], axis=1)))  # where they differ most
    cx, cy = pid[worst, :2]
    sheet = pygame.Surface((2 * size + 10, size + 40))
    sheet.fill((255, 255, 255))
    sheet.blit(panel(grid.origin[0], grid.origin[1], whole), (0, 0))
    sheet.blit(panel(cx - 1.25, cy - 1.25, 2.5), (size + 10, 0))
    sheet.blit(font.render(f"{a} -> {b}: blue = A* path, red = PID, green = Pure Pursuit"
                           f"   (right: 2.5 m close-up)", True, (0, 0, 0)), (6, size + 12))
    pygame.image.save(sheet, out)


def main():
    share = share_dir()
    ap = argparse.ArgumentParser(description="Compare PID and Pure Pursuit path tracking")
    ap.add_argument("--map", default=os.path.join(share, "maps", "factory_map.yaml"))
    ap.add_argument("--noise", type=float, default=0.02, help="pose noise in m (ICP is ~0.02-0.05)")
    ap.add_argument("--example", nargs=2, default=["STORES", "ASSEMBLY_B"], metavar=("FROM", "TO"))
    ap.add_argument("--out", default=os.path.expanduser("~/factory_ws/results"))
    a = ap.parse_args()

    planning = os.path.join(share, "config", "planning.yaml")
    grid = PlanningGrid.from_files(a.map, planning, os.path.join(share, "config", "zones.yaml"))
    with open(planning) as f:
        settings = (yaml.safe_load(f) or {}).get("controller", {})
    with open(os.path.join(share, "config", "stations.yaml")) as f:
        stations = {n: (s["x"], s["y"], s["yaw"]) for n, s in yaml.safe_load(f)["stations"].items()}
    print(f"tracking {len(stations) * (len(stations) - 1) // 2} A* routes with PID and Pure Pursuit "
          f"(pose noise {a.noise} m) ...")

    rows, examples = run_all(grid, stations, settings, a.noise)
    os.makedirs(a.out, exist_ok=True)
    csv_path = os.path.join(a.out, "tracking_benchmark.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary(rows)
    png = os.path.join(a.out, "tracking_example.png")
    draw_example(grid, examples, *a.example, png)
    print(f"\nCSV:     {csv_path}\npicture: {png}")


if __name__ == "__main__":
    main()
