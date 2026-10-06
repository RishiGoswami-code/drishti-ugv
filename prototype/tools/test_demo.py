#!/usr/bin/env python3
"""Regression tests for the prototype.

A demo that quietly stops demonstrating what it claims is worse than no demo,
because it will be believed. Each test below pins one of the claims the demo is
used to make.

    python tools/test_demo.py
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROTO = os.path.dirname(HERE)
sys.path.insert(0, PROTO)

from drishti_proto.planner import astar                    # noqa: E402
from drishti_proto.sim import Fault, Simulation            # noqa: E402
from drishti_proto.supervisor import Action, Reason        # noqa: E402
from drishti_proto.world import WORLDS                     # noqa: E402

_checks = 0
_failures = 0
_case = ""


def case(name):
    global _case
    _case = name


def CHECK(cond, note=""):
    global _checks, _failures
    import inspect
    _checks += 1
    if not cond:
        _failures += 1
        print("  FAIL  [%s] line %d %s"
              % (_case, inspect.currentframe().f_back.f_lineno, note))


def run(world="easy", faults=None, limit=2000):
    sim = Simulation(WORLDS[world](0), faults=faults)
    trail, reasons = [], []
    while not sim.finished and len(trail) < limit:
        t = sim.step()
        trail.append(sim.pose[:2])
        reasons.append(t.reason)
    return sim, trail, reasons


def test_clean_runs_reach_the_goal():
    case("every world is solvable when nothing is broken")
    for world in sorted(WORLDS):
        sim, _, _ = run(world)
        CHECK(sim.outcome == "success",
              "%s ended as %s" % (world, sim.outcome))


def test_the_ditch_is_gone_around_not_across():
    case("T07: the ditch is avoided, not crossed")
    # The claim made about this scenario. The ditch spans x 9.0-10.5 for y below 9.5,
    # and the detour is north of it.
    sim, trail, _ = run("hard")
    CHECK(sim.outcome == "success", "outcome %s" % sim.outcome)
    inside = [p for p in trail if 9.0 <= p[0] <= 10.5 and p[1] <= 9.5]
    through = [p for p in trail if 9.0 <= p[0] <= 10.5 and p[1] > 9.5]
    CHECK(not inside, "%d points inside the ditch footprint" % len(inside))
    CHECK(through, "never used the northern detour")


def test_no_lethal_terrain_is_ever_traversed():
    case("no run ever stands on lethal terrain")
    for world in sorted(WORLDS):
        sim, trail, _ = run(world)
        w = sim.world
        worst = 0.0
        for x, y in trail:
            cx, cy = w.to_cell(x, y)
            if w.in_bounds(cx, cy):
                worst = max(worst, w.step_at(cx, cy))
        CHECK(worst < 0.25, "%s traversed a %.2f m step" % (world, worst))


def test_a_frozen_camera_stops_the_vehicle():
    case("D19: a frozen camera stops the vehicle")
    # The failure a liveness check cannot see. Frames keep arriving with fresh
    # timestamps; only the content signature catches it.
    sim, _, reasons = run("easy", [Fault(9.0, "camera_freeze")])
    CHECK(Reason.CAMERA_FROZEN in reasons, "never reported CAMERA_FROZEN")
    CHECK(sim.outcome == "safe_abort", "outcome %s" % sim.outcome)
    # And it must NOT be caught as staleness: the age stays small throughout,
    # which is the entire point.
    CHECK(Reason.CAMERA_STALE not in reasons,
          "reported CAMERA_STALE -- then the freeze was never the thing tested")


def test_the_freeze_trips_at_the_configured_threshold():
    case("the freeze trips at t_frame_static, not before")
    sim = Simulation(WORLDS["easy"](0), faults=[Fault(9.0, "camera_freeze")])
    tripped = None
    while not sim.finished and sim.t < 60:
        t = sim.step()
        if t.reason == Reason.CAMERA_FROZEN and tripped is None:
            tripped = t.t
    CHECK(tripped is not None, "never tripped")
    if tripped is not None:
        want = 9.0 + sim.supervisor.params.t_frame_static
        CHECK(abs(tripped - want) < 0.25,
              "tripped at %.2f s, expected about %.2f s" % (tripped, want))


def test_each_fault_reports_its_own_reason():
    case("each fault reports its own reason, not a neighbouring one")
    # Reporting the right action for the wrong reason sends you debugging the
    # wrong sensor.
    expect = {
        "camera_silence": Reason.CAMERA_STALE,
        "depth_silence": Reason.DEPTH_STALE,
        "slam_loss": Reason.LOCALIZATION_LOST,
        "camera_freeze": Reason.CAMERA_FROZEN,
    }
    for kind, reason in sorted(expect.items()):
        sim, _, reasons = run("easy", [Fault(9.0, kind)])
        CHECK(reason in reasons, "%s never reported %s" % (kind, reason.name))
        CHECK(sim.outcome == "safe_abort",
              "%s ended as %s" % (kind, sim.outcome))


def test_every_fault_halts_the_vehicle():
    case("every injected fault ends in a safe halt, never a collision")
    for kind in ("camera_freeze", "camera_silence", "depth_silence", "slam_loss"):
        sim, _, _ = run("easy", [Fault(9.0, kind)])
        CHECK(sim.outcome != "collision", "%s collided" % kind)
        CHECK(sim.telemetry.cmd_out[0] == 0.0,
              "%s left a non-zero command on the wire" % kind)


def test_unobserved_space_starts_expensive():
    case("SPEC 6.2: unobserved space starts expensive, not free")
    sim = Simulation(WORLDS["easy"](0))
    costs = [sim.cost[cy][cx]
             for cy in range(sim.world.height) for cx in range(sim.world.width)]
    CHECK(min(costs) > 0.5, "cheapest unobserved cell is %.2f" % min(costs))
    CHECK(not any(any(row) for row in sim.lethal),
          "unknown must be expensive, never lethal")


def test_observation_reduces_cost_where_the_ground_is_good():
    case("observing good ground makes it cheap")
    sim = Simulation(WORLDS["easy"](0))
    before = sim.cost[sim.world.to_cell(*sim.world.start)[1]][
        sim.world.to_cell(*sim.world.start)[0]]
    for _ in range(30):
        sim.step()
    observed = [sim.cost[cy][cx]
                for cy in range(sim.world.height)
                for cx in range(sim.world.width) if sim.observed[cy][cx]]
    CHECK(observed, "nothing was observed at all")
    CHECK(min(observed) < 0.3,
          "best observed cell is %.2f; flat dirt should be cheap" % min(observed))
    CHECK(before > 0.5, "start cell was not expensive before observation")


def test_the_supervisor_is_the_only_thing_that_moves_the_vehicle():
    case("the supervisor output is what drives, not the planner output")
    sim, _, _ = run("easy", [Fault(5.0, "slam_loss")])
    t = sim.telemetry
    # Nav2 still wants to move; the wheels get nothing.
    CHECK(t.action == Action.STOP)
    CHECK(t.cmd_out == (0.0, 0.0), "wheels got %r" % (t.cmd_out,))


def test_astar_refuses_to_cross_lethal_cells():
    case("A* treats lethal as a constraint, not a high price")
    n = 12
    cost = [[0.0] * n for _ in range(n)]
    lethal = [[False] * n for _ in range(n)]
    for y in range(n):                       # a full wall
        lethal[y][6] = True
    CHECK(astar(cost, lethal, (1, 1), (10, 10), n, n) == [],
          "found a path through a solid wall")

    lethal[0][6] = False                     # one gap
    path = astar(cost, lethal, (1, 1), (10, 10), n, n)
    CHECK(path, "no path through the gap")
    CHECK(all(not lethal[y][x] for x, y in path), "path crosses a lethal cell")


def test_runs_are_deterministic():
    case("the same seed gives the same run")
    a, trail_a, _ = run("hard")
    b, trail_b, _ = run("hard")
    CHECK(a.outcome == b.outcome)
    CHECK(len(trail_a) == len(trail_b))
    CHECK(all(abs(p[0] - q[0]) < 1e-9 and abs(p[1] - q[1]) < 1e-9
              for p, q in zip(trail_a, trail_b)), "trajectories diverged")


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print("\n%d checks, %d failures across %d tests"
          % (_checks, _failures, len(tests)))
    if _failures == 0:
        print("prototype: OK")
    return 1 if _failures else 0


if __name__ == "__main__":
    sys.exit(main())
