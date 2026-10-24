"""Delivery jobs, the waiting queue, and the choice of which robot does which job.
Plain Python, no ROS, so it can be unit-tested.

Job:    pick something up at one station and drop it at another, with a priority.
Queue:  waiting jobs, most important first: emergency > urgent > routine; equal priority ->
        the one that has waited longest first.
Assign: give waiting jobs to idle robots. cost(robot, job) = how far that robot must drive to
        reach the job's pickup (an A* route length, math.inf if it can't get there).
  greedy   take the most important job, give it to the cheapest idle robot, repeat.
  optimal  take as many of the most important jobs as there are idle robots, then try EVERY way
           of pairing them with those robots and keep the pairing with the smallest total cost.
           (With 2 robots that is at most 2 pairings - cheap. Greedy can be short-sighted:
           it may give robot A the first job even though A is the only one close to the second.)
"""
import itertools
import math
from dataclasses import dataclass, field

PRIORITY_RANK = {"emergency": 0, "urgent": 1, "routine": 2}


@dataclass
class Job:
    id: str
    pickup: str
    dropoff: str
    priority: str = "routine"
    created: float = 0.0
    state: str = "WAITING"           # WAITING -> ASSIGNED -> PICKED -> DELIVERED   (or FAILED)
    robot: str = ""
    assigned: float = None
    picked: float = None
    delivered: float = None
    attempts: int = 0
    interrupted: int = 0             # times a robot dropped this job for an emergency
    distance_m: float = 0.0
    log: list = field(default_factory=list)

    def sort_key(self):
        return (PRIORITY_RANK.get(self.priority, 9), self.created)


def waiting_in_order(jobs):
    """Waiting jobs, most important first, then oldest first."""
    return sorted((j for j in jobs if j.state == "WAITING"), key=Job.sort_key)


def assign(idle_robots, jobs, cost, mode="greedy"):
    """-> list of (robot, job) pairs to start now. cost(robot, job) -> metres or math.inf."""
    queue = waiting_in_order(jobs)
    robots = list(idle_robots)
    if not queue or not robots:
        return []
    if mode == "greedy":
        pairs = []
        for job in queue:
            if not robots:
                break
            best = min(robots, key=lambda r: cost(r, job))
            if math.isinf(cost(best, job)):
                continue                                   # nobody can reach it right now
            pairs.append((best, job))
            robots.remove(best)
        return pairs
    if mode == "optimal":
        chosen = queue[:len(robots)]                       # priority decides WHICH jobs go now
        best_pairs, best_total = [], math.inf
        for order in itertools.permutations(robots, len(chosen)):
            pairs = [(r, j) for r, j in zip(order, chosen) if not math.isinf(cost(r, j))]
            total = sum(cost(r, j) for r, j in pairs) - 1000.0 * len(pairs)   # more jobs started wins
            if total < best_total:
                best_pairs, best_total = pairs, total
        return best_pairs
    raise ValueError(f"unknown allocation mode {mode!r}: use greedy or optimal")


def pick_robot_to_interrupt(busy, cost, carrying, carry_penalty=5.0):
    """No robot is free but an emergency is waiting: which busy robot should drop what it is doing?
    busy      robots that may be interrupted (not already on an emergency)
    cost      cost(robot) -> metres to the emergency pickup (math.inf = cannot get there)
    carrying  carrying(robot) -> True if it already has its cargo on board (it will have to finish
              that delivery afterwards, so it is slightly less attractive: + carry_penalty metres)
    -> the robot to interrupt, or None."""
    scored = [(cost(r) + (carry_penalty if carrying(r) else 0.0), r) for r in busy]
    scored = [(s, r) for s, r in scored if not math.isinf(s)]
    return min(scored)[1] if scored else None
