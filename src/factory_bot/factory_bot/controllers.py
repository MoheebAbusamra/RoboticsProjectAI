"""Path-following controllers (Lab 4): turn a planned path into speed commands (v, w).
Plain Python, no ROS. Both controllers have the same interface:

    ctrl.set_path([(x, y), ...])                 # metres, map frame (from planners.path_to_world)
    v, w, done = ctrl.step((x, y, yaw), dt)      # call every control cycle with the robot's pose

PID           steer so the heading error to a "carrot" point ahead on the path goes to zero
              w = kp*e + ki*sum(e*dt) + kd*(de/dt)
Pure Pursuit  drive the circular arc that passes through a point L metres ahead on the path
              curvature = 2*y_L / L^2  (y_L = how far left of the robot that point is),  w = v*curvature
Shared rules (both): the target point only moves FORWARD along the path (never jumps back to
an earlier part that passes nearby); if the target is far behind, turn on the spot first;
slow down near the goal; never exceed the speed limits.
Follows starters/algorithms/control.py in the course repo. """
import math


def wrap(a):
    """Angle to (-pi, pi]: 179 deg to -179 deg is a 2 deg turn, not 358."""
    return math.atan2(math.sin(a), math.cos(a))


def to_robot_frame(pose, point):
    """A map point seen from the robot: x = ahead, y = to the left."""
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return c * dx + s * dy, -s * dx + c * dy


class PathFollower:
    """What both controllers share: the path, progress along it, goal checks, limits."""

    def __init__(self, lookahead=0.40, speed=0.25, max_w=1.5, goal_tolerance=0.10,
                 turn_in_place=math.radians(60), slow_radius=0.5):
        self.lookahead = lookahead            # m: how far ahead the target point is
        self.speed = speed                    # m/s: cruising speed
        self.max_w = max_w                    # rad/s: turning limit
        self.goal_tolerance = goal_tolerance  # m: "arrived"
        self.turn_in_place = turn_in_place    # rad: bigger heading error -> stop and turn
        self.slow_radius = slow_radius        # m: start slowing down this close to the goal
        self.path, self.index = [], 0

    def set_path(self, points):
        self.path, self.index = [tuple(p) for p in points], 0
        self.reset()

    def reset(self):
        """Clear memory between paths (PID uses it)."""

    def target(self, pose):
        """First path point at least `lookahead` ahead, searching forward from the last one."""
        i = self.index
        while i < len(self.path) - 1 and math.dist(self.path[i], pose[:2]) < self.lookahead:
            i += 1
        self.index = i
        return self.path[i]

    def step(self, pose, dt, speed_limit=None):
        """One control cycle -> (v, w, done)."""
        if not self.path:
            return 0.0, 0.0, True
        to_goal = math.dist(self.path[-1], pose[:2])
        if to_goal <= self.goal_tolerance:
            return 0.0, 0.0, True
        tx, ty = to_robot_frame(pose, self.target(pose))
        heading_error = math.atan2(ty, tx)
        if abs(heading_error) > self.turn_in_place:           # facing the wrong way: turn first
            return 0.0, math.copysign(0.6 * self.max_w, heading_error), False
        v_max = self.speed if speed_limit is None else min(self.speed, speed_limit)
        v_max *= min(1.0, max(0.25, to_goal / self.slow_radius))   # gentle stop at the goal
        v, w = self.command(tx, ty, heading_error, v_max, dt)
        return v, max(-self.max_w, min(self.max_w, w)), False

    def command(self, tx, ty, heading_error, v_max, dt):
        raise NotImplementedError


class PIDFollower(PathFollower):
    """PID on the heading error to the target point; drive slower while badly aligned."""

    def __init__(self, kp=2.0, ki=0.0, kd=0.10, integral_limit=0.5, **kw):
        super().__init__(**kw)
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral_limit = integral_limit  # anti-windup: the sum can never grow past this
        self.reset()

    def reset(self):
        self.integral, self.previous = 0.0, None

    def command(self, tx, ty, error, v_max, dt):
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral + error * dt))
        derivative = 0.0 if self.previous is None else wrap(error - self.previous) / dt
        self.previous = error
        w = self.kp * error + self.ki * self.integral + self.kd * derivative
        v = v_max * max(0.0, math.cos(error))                 # aligned -> full speed, sideways -> stop
        return v, w


class PurePursuitFollower(PathFollower):
    """Pure Pursuit with simple regulation: slower on tight arcs."""

    def __init__(self, min_speed=0.06, regulate=True, regulation_gain=0.5, **kw):
        super().__init__(**kw)
        self.min_speed, self.regulate = min_speed, regulate
        self.regulation_gain = regulation_gain    # bigger = slows down more on tight arcs

    def command(self, tx, ty, error, v_max, dt):
        curvature = 2.0 * ty / max(tx * tx + ty * ty, 1e-9)
        v = v_max
        if self.regulate:                                     # tight arc (small radius) -> slow down
            v = max(self.min_speed, v_max / (1.0 + self.regulation_gain * abs(curvature)))
        return v, v * curvature


def make_follower(name, settings=None):
    """'pid' or 'pure_pursuit' with settings from config/planning.yaml (controller: section)."""
    settings = dict(settings or {})
    common = {k: settings.pop(k) for k in list(settings)
              if k in ("lookahead", "speed", "max_w", "goal_tolerance", "slow_radius")}
    if name == "pid":
        return PIDFollower(**common, **settings.get("pid", {}))
    if name == "pure_pursuit":
        return PurePursuitFollower(**common, **settings.get("pure_pursuit", {}))
    raise ValueError(f"unknown controller {name!r}: use pid or pure_pursuit")
