"""BFS, Dijkstra and A* on the planning grid (Lab 4). Plain Python, no ROS.

All three keep a FRONTIER (cells found but not yet expanded). They differ only in which
frontier cell they expand next (the table in the Lab 4 manual):
  BFS       the earliest found          -> fewest MOVES; ignores cell costs (baseline)
  Dijkstra  the smallest cost so far g  -> cheapest route
  A*        the smallest g + h          -> cheapest route, usually far fewer expansions
            (h = octile distance to the goal: never more than the real remaining cost,
             because every cell costs at least 1, so A* stays optimal)
Moves: 8 neighbours (diagonals cost sqrt 2). A diagonal is allowed only if both cells it
squeezes between are passable, so a path never cuts a wall corner.
Step cost = step length (1 or sqrt 2) x cost of the cell entered (planning_grid.py).
AI-assisted (Claude): cite in report. Compare with your own Lab 4 planner_skeleton.py."""
import heapq
import math
import time
from collections import deque
from dataclasses import dataclass, field

SQRT2 = math.sqrt(2.0)
MOVES = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
         (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]


@dataclass
class PlanResult:
    algorithm: str
    success: bool
    path: list = field(default_factory=list)       # [(row, col), ...] start .. goal
    cost: float = math.inf                         # sum of step costs (cells)
    length_m: float = 0.0                          # geometric length in metres
    expanded: int = 0                              # cells taken out of the frontier
    time_ms: float = 0.0
    reason: str = ""
    closed: set = field(default_factory=set, repr=False)   # expanded cells (for pictures)


def octile(a, b):
    """Shortest 8-connected distance between two cells if nothing were in the way."""
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dr + dc) + (SQRT2 - 2.0) * min(dr, dc)


def neighbours(cell, ok, h, w):
    """Passable neighbours of a cell with the length of the step. ok[r][c] = passable."""
    r, c = cell
    for dr, dc, step in MOVES:
        nr, nc = r + dr, c + dc
        if not (0 <= nr < h and 0 <= nc < w) or not ok[nr][nc]:
            continue
        if dr and dc and not (ok[r + dr][c] and ok[r][c + dc]):   # don't cut corners
            continue
        yield (nr, nc), step


def rebuild(parent, goal):
    """Follow parents back from the goal to the start."""
    path = [goal]
    while parent[path[-1]] is not None:
        path.append(parent[path[-1]])
    return path[::-1]


def bfs(ok, cost, start, goal):
    """Breadth-first: expand cells in the order they were found (a FIFO queue)."""
    h, w = len(ok), len(ok[0])
    parent = {start: None}
    queue, closed = deque([start]), set()
    while queue:
        cell = queue.popleft()
        closed.add(cell)
        if cell == goal:
            return rebuild(parent, goal), closed
        for nb, _ in neighbours(cell, ok, h, w):
            if nb not in parent:                   # first time seen = fewest moves to it
                parent[nb] = cell
                queue.append(nb)
    return None, closed


def best_first(ok, cost, start, goal, use_heuristic):
    """Dijkstra (use_heuristic=False) or A* (True): expand the cell with the smallest
    priority from a heap. priority = g (cost so far) [+ h (estimate to goal) for A*]."""
    h, w = len(ok), len(ok[0])
    est = (lambda cell: octile(cell, goal)) if use_heuristic else (lambda cell: 0.0)
    g = {start: 0.0}
    parent = {start: None}
    heap = [(est(start), 0, start)]               # (priority, tie-break counter, cell)
    counter, closed = 0, set()
    while heap:
        _, _, cell = heapq.heappop(heap)
        if cell in closed:                         # an older, worse copy of this cell
            continue
        closed.add(cell)
        if cell == goal:                           # stop only when the GOAL is expanded
            return rebuild(parent, goal), closed
        for nb, step in neighbours(cell, ok, h, w):
            if nb in closed:
                continue
            new_g = g[cell] + step * cost[nb[0]][nb[1]]
            if new_g < g.get(nb, math.inf):        # found a cheaper way to nb
                g[nb] = new_g
                parent[nb] = cell
                counter += 1
                heapq.heappush(heap, (new_g + est(nb), counter, nb))
    return None, closed


def path_cost(path, cost):
    total = 0.0
    for a, b in zip(path, path[1:]):
        step = SQRT2 if a[0] != b[0] and a[1] != b[1] else 1.0
        total += step * cost[b[0]][b[1]]
    return total


def path_is_valid(grid, path, allow_restricted=False):
    """Every cell passable and every step to an 8-neighbour (no jumps)."""
    if not path:
        return False
    if not all(grid.passable(r, c, allow_restricted) for r, c in path):
        return False
    return all(max(abs(a[0] - b[0]), abs(a[1] - b[1])) == 1 for a, b in zip(path, path[1:]))


def plan(grid, start_xy, goal_xy, algorithm="astar", allow_restricted=False, snap_cells=10):
    """Plan from a start to a goal given in METRES (map frame) on a PlanningGrid.
    A start or goal inside the inflation band is moved to the nearest passable cell
    (up to snap_cells away); further than that, the request fails with a reason."""
    result = PlanResult(algorithm, False)
    ok = (~(grid.cost == math.inf) & (allow_restricted | ~grid.restricted)).tolist()   # plain lists:
    cost = grid.cost.tolist()                                                           # faster to index
    t0 = time.perf_counter()                       # time the search itself, not the set-up
    cells = []
    for name, xy in (("start", start_xy), ("goal", goal_xy)):
        cell = grid.world_to_cell(*xy)
        if not grid.passable(*cell, allow_restricted):
            cell = grid.nearest_passable(*cell, max_cells=snap_cells, allow_restricted=allow_restricted)
            if cell is None:
                result.reason = f"{name} {xy} is not in free space (wall, zone or unknown)"
                return result
        cells.append(cell)
    start, goal = cells
    if algorithm == "bfs":
        path, closed = bfs(ok, cost, start, goal)
    elif algorithm in ("dijkstra", "astar"):
        path, closed = best_first(ok, cost, start, goal, use_heuristic=(algorithm == "astar"))
    else:
        raise ValueError(f"unknown algorithm {algorithm!r}: use bfs, dijkstra or astar")
    result.time_ms = 1000 * (time.perf_counter() - t0)
    result.expanded, result.closed = len(closed), closed
    if path is None:
        result.reason = "no route: the goal cannot be reached from the start"
        return result
    result.success, result.path = True, path
    result.cost = path_cost(path, cost)
    result.length_m = grid.res * sum(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(path, path[1:]))
    return result


def path_to_world(grid, path):
    """Cells -> list of (x, y) metres (cell centres), for the controller."""
    return [grid.cell_to_world(r, c) for r, c in path]
