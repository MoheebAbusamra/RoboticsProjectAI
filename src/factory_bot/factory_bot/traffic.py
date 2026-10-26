"""Traffic rules for the robots: who goes first, who waits, who makes room. Plain Python, no ROS.

Every robot gets a PRIORITY score (higher = goes first; equal -> robot1 before robot2):
  +100  almost at its goal (less than 1 m of route left): let it finish and get out of the way
  +30 / +20 / +10   its job is emergency / urgent / routine     (no job, e.g. going home: +0)
  +5    it is carrying cargo (on its way to the drop-off)
Because the order is strict (no ties), two robots can never both wait for each other.

The rules, checked in this order every update:
  1. MAKE ROOM is sticky: a robot that stepped aside stays there until the robot it made room for has passed.
  2. MAKE ROOM:  a robot standing still (waiting, or parked with nothing to do) is in a moving robot's way:
                   the mover has been BLOCKED by it for blocked_s, or
                   the mover's route has run straight through it for blocked_s (no way round), or
                   the mover has got no closer to its goal for stuck_s while it is close by
                 -> the standing robot drives to the nearest free spot clear of the mover's route and waits
                 there. (This is what breaks deadlocks: in a narrow aisle "wait" alone would leave both
                 robots stuck nose to nose for ever.)
  3. STATION BUSY: a robot's destination is taken by a robot standing there:
                   loading / unloading -> wait outside until it leaves;  just parked -> it makes room.
  4. GIVE WAY:   the next lookahead_m of two moving robots' routes come closer than clearance_m
                 -> the lower-priority robot stops and waits (until the routes are clear for release_s).
                 A robot driving off to make room counts as highest priority while it does so.
                 Not when they drive the same way one behind the other: the rear one just slows down
                 (the navigator's lidar slow-down / stop does that).
AI-assisted (Claude): cite in report."""
import math
import heapq
from dataclasses import dataclass, field

import numpy as np

from factory_bot.planning_grid import inflate
from factory_bot.planners import neighbours
from factory_bot.backup_recovery import peer_blocks_reverse

JOB_POINTS = {"emergency": 30, "urgent": 20, "routine": 10}
MOVING = ("DRIVING", "BLOCKED", "ALIGNING", "RECOVERING")
BUSY_HERE = ("LOADING", "UNLOADING")                  # fleet states that mean "can't leave right now"


@dataclass
class RobotView:
    """What the traffic rules need to know about one robot."""
    name: str
    pose: tuple = None              # (x, y, yaw) on the map, None = unknown
    active: bool = False            # following a route (navigator DRIVING / BLOCKED / ALIGNING)
    route: list = field(default_factory=list)   # what is LEFT of its route, [(x, y), ...]
    fleet_state: str = "IDLE"       # IDLE, TO_PICKUP, LOADING, TO_DROPOFF, UNLOADING, RETURNING
    job_priority: str = None        # emergency / urgent / routine / None
    blocked_s: float = 0.0          # how long the navigator has been BLOCKED (stopped by an obstacle)
    recovery: dict = None
    fresh: bool = True
    stopped: bool = False
    traffic_hold: str = None

    @property
    def xy(self):
        return (self.pose[0], self.pose[1])

    @property
    def goal(self):
        return self.route[-1] if self.route else None


# ---------------------------------------------------------------- geometry helpers
def remaining_route(path, pose):
    """The part of the route still in front of the robot: from the route point nearest to it on."""
    if not path or pose is None:
        return []
    pts = np.asarray(path, dtype=float)
    i = int(np.argmin(np.hypot(pts[:, 0] - pose[0], pts[:, 1] - pose[1])))
    return [tuple(p) for p in path[i:]]


def ahead(route, xy, metres):
    """The robot's position plus the next `metres` of its route."""
    out, travelled, last = [tuple(xy)], 0.0, tuple(xy)
    for p in route:
        travelled += math.dist(last, p)
        if travelled > metres:
            break
        out.append(tuple(p))
        last = tuple(p)
    return out


def route_length(route):
    return sum(math.dist(a, b) for a, b in zip(route, route[1:]))


def closest(a, b):
    """Smallest distance between any point of list a and any point of list b (math.inf if one is empty)."""
    if not a or not b:
        return math.inf
    pa, pb = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    return float(np.min(np.hypot(pa[:, None, 0] - pb[None, :, 0], pa[:, None, 1] - pb[None, :, 1])))


def following(a, b, max_angle_deg=45.0):
    """True if a and b drive roughly the same way, one behind the other (no head-on, no crossing)."""
    if abs(math.remainder(a.pose[2] - b.pose[2], 2 * math.pi)) > math.radians(max_angle_deg):
        return False
    dx, dy = b.pose[0] - a.pose[0], b.pose[1] - a.pose[1]
    along = dx * math.cos(a.pose[2]) + dy * math.sin(a.pose[2])           # how far b is in front of a
    across = -dx * math.sin(a.pose[2]) + dy * math.cos(a.pose[2])         # how far to the side
    return abs(along) > abs(across)                                       # one is really behind the other


# ---------------------------------------------------------------- priority
def priority(view, near_goal_m=1.0):
    """Higher = goes first (see the module docstring)."""
    score = JOB_POINTS.get(view.job_priority, 0) if view.fleet_state != "RETURNING" else 0
    if view.fleet_state == "TO_DROPOFF":
        score += 5
    if view.active and view.route and route_length(view.route) < near_goal_m:
        score += 100
    return score


def right_of_way_order(views):
    """Robot names, the one that goes first first. Ties -> by name (robot1 before robot2)."""
    return sorted(views, key=lambda n: (-priority(views[n]), n))


# ---------------------------------------------------------------- making room
def find_aside_spot(grid, start_xy, other_route, other_xy, clearance=0.9, body=0.6, max_m=6.0,
                    allow_restricted=False):
    """Nearest place (by driving distance) the robot at start_xy can move to so that it is at least
    `clearance` from every point of the other robot's route. The search floods outwards over free cells
    but never through the other robot itself (within `body` of other_xy). -> (x, y) or None."""
    keep_off = np.zeros((grid.height, grid.width), dtype=bool)
    for x, y in list(other_route) + [tuple(other_xy)]:
        r, c = grid.world_to_cell(x, y)
        if grid.in_bounds(r, c):
            keep_off[r, c] = True
    keep_off = inflate(keep_off, clearance / grid.res + 1.0)      # +1 cell: cells are 5 cm squares
    other_body = np.zeros_like(keep_off)
    r, c = grid.world_to_cell(*other_xy)
    if grid.in_bounds(r, c):
        other_body[r, c] = True
    other_body = inflate(other_body, body / grid.res)

    start = grid.world_to_cell(*start_xy)
    if not grid.passable(*start, allow_restricted):
        start = grid.nearest_passable(*start, allow_restricted=allow_restricted)
        if start is None:
            return None
    # Use the SAME diagonal/corner rules as A*. A candidate found by an
    # unrestricted 8-neighbour flood could otherwise be unreachable by the
    # navigator. Distances include sqrt(2) for a diagonal, in metres.
    ok = (np.isfinite(grid.cost) & (allow_restricted | ~grid.restricted) & ~other_body).tolist()
    ok[start[0]][start[1]] = True  # let a robot leave its current cell
    distance, queue = {start: 0.0}, [(0.0, start)]
    while queue:
        travelled, cell = heapq.heappop(queue)
        if travelled > distance[cell] + 1e-9:
            continue
        r, c = cell
        if not keep_off[r, c]:
            return grid.cell_to_world(r, c)
        for n, step in neighbours(cell, ok, grid.height, grid.width):
            new_distance = travelled + step * grid.res
            if new_distance <= max_m + 1e-9 and new_distance < distance.get(n, math.inf):
                distance[n] = new_distance
                heapq.heappush(queue, (new_distance, n))
    return None


# ---------------------------------------------------------------- the rules
class TrafficRules:
    def __init__(self, grid, lookahead_m=2.5, clearance_m=0.9, release_s=1.0, blocked_s=2.0,
                 aside_timeout_s=60.0, station_m=0.7, aside_clearance_m=0.9, through_m=0.5, stuck_s=8.0):
        self.grid = grid
        self.lookahead_m, self.clearance_m, self.release_s = lookahead_m, clearance_m, release_s
        self.blocked_s, self.aside_timeout_s, self.station_m = blocked_s, aside_timeout_s, station_m
        self.aside_clearance_m, self.through_m = aside_clearance_m, through_m
        self.stuck_s = stuck_s
        self.in_way_since = {}        # (mover, stander) -> time the mover's route first ran through the stander
        self.progress = {}            # robot -> (goal, nearest it has been to it, when)
        self.cmd = {}                 # robot -> {"cmd": go/wait/aside, "because", "rule", "spot", "since"}
        self.clear_since = {}         # robot -> time its wait condition first looked clear
        self.backup_owner = None      # (robot, unique request id); never preempt a backing robot

    def _backup_commands(self, views, now):
        requests={n:v for n,v in views.items() if v.recovery and
                  v.recovery.get('phase') in ('waiting','backing')}
        if self.backup_owner:
            name,identifier=self.backup_owner;current=views.get(name)
            # A stale owner must explicitly return before another robot gets a lease.
            if current is not None and current.fresh and (name not in requests or
                    current.recovery.get('id')!=identifier):self.backup_owner=None
        if not self.backup_owner and not requests:return None
        if not self.backup_owner:
            backing={n:v for n,v in requests.items() if v.recovery.get('phase')=='backing'}
            candidates=backing or requests
            name=right_of_way_order(candidates)[0]
            self.backup_owner=(name,candidates[name].recovery.get('id'))
        name,identifier=self.backup_owner;owner=views.get(name)
        token=f'{name}:{identifier}';new={}
        data=owner.recovery if owner and owner.recovery else {}
        path=data.get('path') or []
        valid=(owner is not None and owner.pose is not None and owner.fresh and
               data.get('id')==identifier and name in requests and len(path)>=2 and
               all(len(p)==2 and all(math.isfinite(x) for x in p) for p in path))
        remaining=math.dist(owner.xy,path[-1]) if valid else math.inf
        valid=valid and remaining<=.55
        granted=valid and all(v.fresh and v.pose is not None for v in views.values())
        if data.get('phase')!='backing':granted=granted and bool(owner and owner.stopped)
        for peer,v in views.items():
            if peer==name:continue
            conflict=(not valid or v.pose is None or peer in requests or
                closest(path,ahead(v.route,v.xy,self.lookahead_m))<self.clearance_m)
            if conflict:
                c=self._make(peer,'wait',name,'backup recovery',now);c['backup_hold']=token
                new[peer]=c
                granted=granted and v.fresh and v.stopped and v.traffic_hold==token
            else:new[peer]=self._make(peer,'go',None,None,now)
            if valid and v.pose is not None and peer_blocks_reverse(owner.pose,remaining,v.xy):
                granted=False
        command=self._make(name,'go' if granted else 'wait',None if granted else name,
                           'backup recovery' if granted else 'backup awaiting clearance',now)
        command.update(backup_granted=bool(granted),backup_id=identifier)
        new[name]=command
        return new

    def _make(self, robot, cmd, because, rule, now, spot=None):
        old = self.cmd.get(robot, {})
        same = old.get("cmd") == cmd and old.get("because") == because
        return {"cmd": cmd, "because": because, "rule": rule, "spot": old.get("spot") if same and spot is None
                else spot, "since": old["since"] if same else now}

    def _held(self, name):
        return self.cmd.get(name, {}).get("cmd", "go") != "go"

    def _stationary(self, v, new):
        return (not v.active or v.blocked_s >= self.blocked_s
                or new.get(v.name, self.cmd.get(v.name, {})).get("cmd", "go") != "go")

    @staticmethod
    def _can_move(v):
        """False while loading/unloading - or just arrived for it (navigator done, fleet not updated yet)."""
        if v.fleet_state in BUSY_HERE:
            return False
        return v.active or v.fleet_state not in ("TO_PICKUP", "TO_DROPOFF")

    def _passed(self, mover, waiter):
        """Has `mover` gone by the robot that made room for it?"""
        if not mover.active:
            return True
        return closest(mover.route, [waiter.xy]) > self.aside_clearance_m + 0.3 and \
            math.dist(mover.xy, waiter.xy) > 1.0

    def _aside(self, new, mover, stander, rule, now):
        """Send `stander` somewhere clear of mover's route. False if there is no such place."""
        r, c = self.grid.world_to_cell(*stander.xy)
        allow = self.grid.in_bounds(r, c) and bool(self.grid.restricted[r, c])
        spot = find_aside_spot(self.grid, stander.xy, mover.route, mover.xy, self.aside_clearance_m,
                               allow_restricted=allow)
        if spot is None:
            return False
        new[stander.name] = self._make(stander.name, "aside", mover.name, rule, now, spot=spot)
        return True

    def _stuck(self, v, now):
        """True if robot v has been driving but got no closer than 0.3 m nearer its goal for stuck_s."""
        # Status and Path arrive on different topics. An active status can
        # arrive first, or the manager can join after a goal was published.
        if not v.active or v.goal is None or self._held(v.name):
            self.progress.pop(v.name, None)
            return False
        d = math.dist(v.xy, v.goal)
        goal, best, since = self.progress.get(v.name, (None, math.inf, now))
        if goal is None or math.dist(goal, v.goal) > 0.5 or d < best - 0.3:      # new goal, or progress
            self.progress[v.name] = (v.goal, d, now)
            return False
        return now - since >= self.stuck_s

    def update(self, views, now):
        """views: {name: RobotView}. -> {name: command dict} for every robot."""
        recovering=self._backup_commands(views,now)
        if recovering is not None:
            self.cmd=recovering
            return recovering
        ready = {n: v for n, v in views.items() if v.pose is not None}
        new = {}
        stuck = {n: self._stuck(v, now) for n, v in ready.items()}

        # 1. robots that stepped aside stay there until the robot they made room for has passed
        for n, c in self.cmd.items():
            if c["cmd"] == "aside" and n in ready:
                mover = ready.get(c["because"])
                if mover is not None and not self._passed(mover, ready[n]) and \
                        now - c["since"] < self.aside_timeout_s:
                    new[n] = c

        # 2. a robot standing still in a moving robot's way for a while -> the standing one makes room
        for x in ready.values():
            if not x.active or self._held(x.name) or x.name in new:
                continue
            for y in ready.values():
                if y.name == x.name or y.name in new or not self._stationary(y, new) or not self._can_move(y):
                    self.in_way_since.pop((x.name, y.name), None)
                    continue
                blocked = (x.blocked_s >= self.blocked_s
                           and closest(ahead(x.route, x.xy, 1.5), [y.xy]) < self.clearance_m)
                through = closest(ahead(x.route, x.xy, self.lookahead_m + 0.5), [y.xy]) < self.through_m
                if through:
                    self.in_way_since.setdefault((x.name, y.name), now)
                else:
                    self.in_way_since.pop((x.name, y.name), None)
                near = closest(ahead(x.route, x.xy, self.lookahead_m), [y.xy]) < self.clearance_m
                if blocked or (through and now - self.in_way_since[(x.name, y.name)] >= self.blocked_s) or \
                        (stuck[x.name] and near):
                    self._aside(new, x, y, "in the way", now)

        # 3. station busy: the destination is taken by a robot standing on it
        for b in ready.values():
            if b.name in new or not b.active or b.goal is None:
                continue
            for a in ready.values():
                if a.name == b.name or a.name in new or not self._stationary(a, new):
                    continue
                if math.dist(a.xy, b.goal) < self.station_m and math.dist(a.xy, b.xy) < 2.0:
                    if self._can_move(a):                          # just parked there: it moves
                        self._aside(new, b, a, "parked on the destination", now)
                    else:                                          # loading / unloading: wait outside
                        new[b.name] = self._make(b.name, "wait", a.name, "station busy", now)
                    break

        # A close rear robot waits for the leader regardless of job priority.
        # Following does not justify driving into a leader that is physically held.
        names = list(ready)
        for i, an in enumerate(names):
            a = ready[an]
            for bn in names[i+1:]:
                b = ready[bn]
                if not (a.active and b.active and following(a, b)):
                    continue
                if math.dist(a.xy, b.xy) >= self.clearance_m:
                    continue
                along = ((b.pose[0]-a.pose[0])*math.cos(a.pose[2])
                         + (b.pose[1]-a.pose[1])*math.sin(a.pose[2]))
                leader, rear = (b, a) if along > 0 else (a, b)
                if rear.name not in new:
                    new[rear.name] = self._make(rear.name, "wait", leader.name, "following distance", now)

        # 4. give way: routes of two moving robots about to cross -> lower priority waits
        #    (a robot that is on its way to make room goes first: don't drive into it while it does)
        making_room = [n for n in right_of_way_order(ready) if new.get(n, {}).get("cmd") == "aside"]
        order = making_room + [n for n in right_of_way_order(ready) if n not in making_room]
        for i, bn in enumerate(order):
            b = ready[bn]
            if bn in new or not b.active:
                continue
            for an in order[:i]:
                a = ready[an]
                if not a.active or (following(a, b) and a.blocked_s < self.blocked_s
                                    and b.blocked_s < self.blocked_s):
                    continue
                was_waiting = self.cmd.get(bn, {}).get("cmd") == "wait" and self.cmd[bn]["because"] == an
                limit = self.clearance_m * (1.25 if was_waiting else 1.0)      # hysteresis: no flicker
                gap = closest(ahead(a.route, a.xy, self.lookahead_m), ahead(b.route, b.xy, self.lookahead_m))
                if gap < limit:
                    new[bn] = self._make(bn, "wait", an, "give way", now)
                    break

        # 5. a waiting robot only goes once its way has been clear for release_s
        for n in ready:
            old = self.cmd.get(n, {})
            if n in new or old.get("cmd") != "wait":
                self.clear_since.pop(n, None)
                continue
            first = self.clear_since.setdefault(n, now)
            if now - first < self.release_s:
                new[n] = old
            else:
                self.clear_since.pop(n, None)

        for n in views:
            new.setdefault(n, self._make(n, "go", None, None, now))
        self.cmd = new
        return new
