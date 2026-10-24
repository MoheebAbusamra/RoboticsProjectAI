"""plan_benchmark: runs BFS, Dijkstra and A* between EVERY pair of stations on the factory
map, checks every route, and saves the numbers and one picture for the report.
  ros2 run factory_bot plan_benchmark
  ros2 run factory_bot plan_benchmark --example STORES ASSEMBLY_B --repeat 5
Outputs in ~/factory_ws/results/: planner_benchmark.csv, planner_example.png
AI-assisted (Claude): cite in report."""
import argparse
import csv
import itertools
import os
import statistics

import numpy as np
import yaml

from factory_bot.planners import path_is_valid, plan
from factory_bot.planning_grid import PlanningGrid
from factory_bot.show_grid import grid_image, share_dir

ALGORITHMS = ("bfs", "dijkstra", "astar")


def run_all(grid, stations, repeat):
    """Every unordered station pair x every algorithm. Time = best of `repeat` runs."""
    rows = []
    for a, b in itertools.combinations(stations, 2):
        allow = "QC_LAB" in (a, b)                 # only jobs to/from the QC lab may enter it
        for alg in ALGORITHMS:
            runs = [plan(grid, stations[a], stations[b], alg, allow_restricted=allow) for _ in range(repeat)]
            r = runs[0]
            rows.append({"from": a, "to": b, "algorithm": alg, "success": r.success,
                         "valid": path_is_valid(grid, r.path, allow), "length_m": round(r.length_m, 3),
                         "cost": round(r.cost, 2), "expanded": r.expanded,
                         "time_ms": round(min(x.time_ms for x in runs), 2)})
    return rows


def summary(rows):
    print(f"\n{'algorithm':10s} {'routes':>7s} {'valid':>6s} {'length m':>9s} {'cost':>8s} "
          f"{'expanded':>9s} {'time ms':>8s}   (averages over all station pairs)")
    for alg in ALGORITHMS:
        mine = [r for r in rows if r["algorithm"] == alg and r["success"]]
        n_all = sum(r["algorithm"] == alg for r in rows)
        mean = lambda k: statistics.mean(r[k] for r in mine)     # noqa: E731
        print(f"{alg:10s} {len(mine):3d}/{n_all:<3d} {sum(r['valid'] for r in mine):6d} {mean('length_m'):9.2f} "
              f"{mean('cost'):8.1f} {mean('expanded'):9.0f} {mean('time_ms'):8.1f}")


def draw_example(grid, stations, a, b, path_out):
    """Three panels side by side: the cells each algorithm expanded (tinted) and its route."""
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    import pygame
    pygame.init()
    font = pygame.font.SysFont(None, 22)
    scale = 400 / max(grid.width, grid.height)               # each panel about 400 px wide
    pw, ph = int(grid.width * scale), int(grid.height * scale)
    sheet = pygame.Surface((3 * pw + 20, ph + 95))
    sheet.fill((255, 255, 255))
    allow = "QC_LAB" in (a, b)
    for i, alg in enumerate(ALGORITHMS):
        r = plan(grid, stations[a], stations[b], alg, allow_restricted=allow)
        img = grid_image(grid).astype(float)
        for row, col in r.closed:                  # expanded cells: blend towards green
            img[row, col] = 0.55 * img[row, col] + 0.45 * np.array([60, 200, 90])
        img = img.astype(np.uint8)[::-1]           # flip: screen row 0 is the TOP
        surf = pygame.transform.scale(pygame.surfarray.make_surface(img.transpose(1, 0, 2)), (pw, ph))
        x0 = i * (pw + 10)
        sheet.blit(surf, (x0, 0))
        pts = [(x0 + (c + 0.5) * scale, (grid.height - row - 0.5) * scale) for row, c in r.path]
        if len(pts) > 1:
            pygame.draw.lines(sheet, (30, 60, 220), False, pts, 3)
        lines = ([alg.upper(), f"{r.length_m:.2f} m, cost {r.cost:.0f}",
                  f"{r.expanded} cells expanded, {r.time_ms:.0f} ms"] if r.success else [alg.upper(), r.reason])
        for k, text in enumerate(lines):
            sheet.blit(font.render(text, True, (0, 0, 0)), (x0 + 4, ph + 6 + 20 * k))
    sheet.blit(font.render(f"{a} -> {b}   (green = cells expanded, blue = route)", True, (60, 60, 60)),
               (4, ph + 72))
    pygame.image.save(sheet, path_out)


def main():
    share = share_dir()
    ap = argparse.ArgumentParser(description="Compare BFS, Dijkstra and A* on the factory map")
    ap.add_argument("--map", default=os.path.join(share, "maps", "factory_map.yaml"))
    ap.add_argument("--example", nargs=2, default=["STORES", "ASSEMBLY_B"], metavar=("FROM", "TO"))
    ap.add_argument("--repeat", type=int, default=3, help="runs per route; the fastest time is kept")
    ap.add_argument("--out", default=os.path.expanduser("~/factory_ws/results"))
    a = ap.parse_args()

    grid = PlanningGrid.from_files(a.map, os.path.join(share, "config", "planning.yaml"),
                                   os.path.join(share, "config", "zones.yaml"))
    with open(os.path.join(share, "config", "stations.yaml")) as f:
        stations = {n: (s["x"], s["y"]) for n, s in yaml.safe_load(f)["stations"].items()}
    print(f"{len(stations)} stations -> {len(stations) * (len(stations) - 1) // 2} routes x 3 algorithms ...")

    rows = run_all(grid, stations, a.repeat)
    os.makedirs(a.out, exist_ok=True)
    csv_path = os.path.join(a.out, "planner_benchmark.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary(rows)
    failed = [r for r in rows if not r["success"] or not r["valid"]]
    print(f"\n{'all routes found and valid' if not failed else f'{len(failed)} PROBLEM ROUTES - see the CSV'}")
    png = os.path.join(a.out, "planner_example.png")
    draw_example(grid, stations, *a.example, png)
    print(f"CSV:     {csv_path}\npicture: {png}")


if __name__ == "__main__":
    main()
