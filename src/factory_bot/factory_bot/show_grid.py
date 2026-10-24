"""show_grid: draws the planning grid in a Pygame window, checks every station, and saves a
picture for the report (~/factory_ws/results/planning_grid.png). No ROS nodes involved.
  ros2 run factory_bot show_grid
  ros2 run factory_bot show_grid --no-window          # only print the checks and save the picture
"""
import argparse
import os

import numpy as np
import yaml

from factory_bot.planning_grid import PlanningGrid

# colour of each kind of cell (later entries are painted on top of earlier ones)
COLOURS = [("free floor", (255, 255, 255)), ("near wall (cost 2)", (205, 225, 245)),
           ("slow zone (cost 2)", (250, 200, 140)), ("QC lab (authorised only)", (250, 235, 120)),
           ("inflation (robot can't fit)", (245, 170, 170)), ("forbidden zone", (220, 40, 40)),
           ("unknown", (150, 150, 150)), ("wall", (0, 0, 0))]


def share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("factory_bot")
    except Exception:                                      # running from the source folder
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def grid_image(g):
    """(height, width, 3) colour picture of the grid, row 0 = bottom."""
    layers = [np.ones_like(g.occupied), g.near_wall, g.slow, g.restricted, g.inflated,
              g.forbidden, g.unknown & ~g.occupied, g.occupied]
    img = np.zeros((g.height, g.width, 3), dtype=np.uint8)
    for mask, (_, colour) in zip(layers, COLOURS):
        img[mask] = colour
    return img


def main():
    share = share_dir()
    ap = argparse.ArgumentParser(description="Show and check the planning grid")
    ap.add_argument("--map", default=os.path.join(share, "maps", "factory_map.yaml"))
    ap.add_argument("--save", default=os.path.expanduser("~/factory_ws/results/planning_grid.png"))
    ap.add_argument("--no-window", action="store_true")
    a = ap.parse_args()

    g = PlanningGrid.from_files(a.map, os.path.join(share, "config", "planning.yaml"),
                                os.path.join(share, "config", "zones.yaml"))
    with open(os.path.join(share, "config", "stations.yaml")) as f:
        stations = yaml.safe_load(f)["stations"]

    passable = ~np.isinf(g.cost)
    print(f"grid {g.width} x {g.height} cells of {g.res:.2f} m | passable {100 * passable.mean():.1f} % "
          f"| walls {g.occupied.sum()} | inflation {g.inflated.sum()} | unknown {g.unknown.sum()} cells")
    all_ok = True
    for name, s in stations.items():
        row, col = g.world_to_cell(s["x"], s["y"])
        restricted = g.in_bounds(row, col) and bool(g.restricted[row, col])
        ok = g.passable(row, col, allow_restricted=True)
        all_ok &= ok
        note = " (inside the QC lab: authorised jobs only)" if restricted else ""
        print(f"  station {name:11s} at ({s['x']:5.2f}, {s['y']:5.2f}) -> cell ({row:3d}, {col:3d}) "
              f"{'OK' if ok else 'BLOCKED - move it'}{note}")
    print("all stations reachable" if all_ok else "SOME STATIONS ARE BLOCKED - edit config/stations.yaml")

    import pygame                                          # imported here so --help works without it
    if a.no_window:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    pygame.init()
    scale = max(1, 700 // max(g.width, g.height))
    img = grid_image(g)[::-1]                              # flip: screen row 0 is the TOP
    surface = pygame.transform.scale(pygame.surfarray.make_surface(img.transpose(1, 0, 2)),
                                     (g.width * scale, g.height * scale))
    legend_h = 22 * ((len(COLOURS) + 1) // 2) + 10
    screen = pygame.Surface((g.width * scale, g.height * scale + legend_h))
    screen.fill((255, 255, 255))
    screen.blit(surface, (0, 0))
    font = pygame.font.SysFont(None, 18)
    for name, s in stations.items():
        row, col = g.world_to_cell(s["x"], s["y"])
        px, py = int((col + 0.5) * scale), int((g.height - row - 0.5) * scale)
        pygame.draw.circle(screen, (40, 90, 230), (px, py), 6)
        text = font.render(name, True, (20, 40, 140))
        tx = px + 8 if px + 8 + text.get_width() < g.width * scale else px - 8 - text.get_width()
        screen.blit(text, (tx, py - 6))
    for i, (label, colour) in enumerate(COLOURS):
        x, y = 10 + (i % 2) * (g.width * scale // 2), g.height * scale + 8 + (i // 2) * 22
        pygame.draw.rect(screen, colour, (x, y, 16, 16))
        pygame.draw.rect(screen, (0, 0, 0), (x, y, 16, 16), 1)
        screen.blit(font.render(label, True, (0, 0, 0)), (x + 22, y + 2))
    os.makedirs(os.path.dirname(a.save), exist_ok=True)
    pygame.image.save(screen, a.save)
    print(f"picture saved: {a.save}")
    if a.no_window:
        return
    window = pygame.display.set_mode(screen.get_size())
    pygame.display.set_caption("Planning grid (close the window to quit)")
    window.blit(screen, (0, 0))
    pygame.display.flip()
    while not any(e.type == pygame.QUIT for e in pygame.event.get()):
        pygame.time.wait(100)
    pygame.quit()


if __name__ == "__main__":
    main()
