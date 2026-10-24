"""Unit tests for the job queue and robot allocation (scheduler.py). """
import math

from factory_bot.scheduler import Job, assign, waiting_in_order

# simple 1-D world for the tests: robots and stations are just numbers on a line (metres)
WHERE = {"A": 0.0, "B": 8.0, "C": 12.0, "D": 20.0}


def cost_from(robot_positions):
    return lambda robot, job: abs(robot_positions[robot] - WHERE[job.pickup])


def test_queue_order_is_priority_then_age():
    jobs = [Job("r1", "A", "B", "routine", created=1.0), Job("u1", "A", "B", "urgent", created=5.0),
            Job("r0", "A", "B", "routine", created=0.0), Job("e1", "A", "B", "emergency", created=9.0),
            Job("done", "A", "B", "urgent", created=0.0, state="DELIVERED")]
    assert [j.id for j in waiting_in_order(jobs)] == ["e1", "u1", "r0", "r1"]


def test_greedy_gives_the_job_to_the_nearest_idle_robot():
    jobs = [Job("j1", "C", "A")]
    pairs = assign(["robot1", "robot2"], jobs, cost_from({"robot1": 0.0, "robot2": 11.0}), "greedy")
    assert [(r, j.id) for r, j in pairs] == [("robot2", "j1")]


def test_optimal_beats_greedy_when_greedy_is_short_sighted():
    # robot1 at 9 m, robot2 at 0 m. Job j1 (pickup B at 8 m) is nearest to robot1, so greedy gives
    # it to robot1 - but then robot2 must drive 20 m to job j2 (pickup D at 20 m).
    positions = {"robot1": 9.0, "robot2": 0.0}
    jobs = [Job("j1", "B", "A", created=0.0), Job("j2", "D", "A", created=1.0)]
    cost = cost_from(positions)
    greedy = assign(["robot1", "robot2"], jobs, cost, "greedy")
    optimal = assign(["robot1", "robot2"], jobs, cost, "optimal")
    total = lambda pairs: sum(cost(r, j) for r, j in pairs)   # noqa: E731
    assert total(greedy) == 1 + 20       # robot1 -> j1 (1 m), robot2 -> j2 (20 m)
    assert total(optimal) == 8 + 11      # robot2 -> j1 (8 m), robot1 -> j2 (11 m): 2 m less driving
    assert {(r, j.id) for r, j in optimal} == {("robot2", "j1"), ("robot1", "j2")}


def test_priority_decides_which_jobs_start_when_robots_are_scarce():
    jobs = [Job("routine_near", "A", "B", "routine", created=0.0),
            Job("urgent_far", "D", "A", "urgent", created=5.0)]
    for mode in ("greedy", "optimal"):
        pairs = assign(["robot1"], jobs, cost_from({"robot1": 0.0}), mode)
        assert [j.id for _, j in pairs] == ["urgent_far"]


def test_unreachable_jobs_are_left_waiting():
    jobs = [Job("j1", "A", "B")]
    for mode in ("greedy", "optimal"):
        assert assign(["robot1"], jobs, lambda r, j: math.inf, mode) == []


def test_nothing_to_do():
    assert assign([], [Job("j1", "A", "B")], cost_from({}), "greedy") == []
    assert assign(["robot1"], [], cost_from({"robot1": 0.0}), "optimal") == []


def test_emergency_interrupts_the_nearest_busy_robot():
    from factory_bot.scheduler import pick_robot_to_interrupt
    dist = {"robot1": 12.0, "robot2": 4.0}
    assert pick_robot_to_interrupt(["robot1", "robot2"], dist.get, lambda r: False) == "robot2"


def test_a_robot_already_carrying_cargo_is_less_attractive():
    from factory_bot.scheduler import pick_robot_to_interrupt
    dist = {"robot1": 6.0, "robot2": 4.0}
    carrying = {"robot1": False, "robot2": True}.get           # robot2: 4 + 5 = 9 > 6
    assert pick_robot_to_interrupt(["robot1", "robot2"], dist.get, carrying) == "robot1"


def test_no_robot_can_be_interrupted():
    from factory_bot.scheduler import pick_robot_to_interrupt
    assert pick_robot_to_interrupt([], {}.get, lambda r: False) is None
    assert pick_robot_to_interrupt(["robot1"], lambda r: math.inf, lambda r: False) is None
