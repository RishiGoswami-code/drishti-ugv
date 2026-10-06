# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""A* over the traversability grid, and pure pursuit along the result.

Standing in for Nav2's planner and MPPI controller. Both are simplified, and
deliberately so: the point of the demo is the terrain cost and the safety gate,
and a faithful MPPI would add a great deal of code without changing what you
see.

What is NOT simplified is how cost enters the search. The planner minimises
accumulated traversability cost, not distance -- which is why it routes around
an expensive-but-passable mud patch, and why unobserved space (priced at
unknown_cost by the real cost function) pushes it towards ground it has
actually seen.
"""
from __future__ import annotations

import heapq
import math
from typing import List, Optional, Sequence, Tuple

#: Cost multiplier. A cell at cost 1.0 is this many times more expensive to
#: cross than a free one, so the planner will accept a detour of up to roughly
#: this many cells to avoid one bad cell.
COST_GAIN = 12.0

NEIGHBOURS = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
              (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142))


def astar(cost: Sequence[Sequence[float]],
          lethal: Sequence[Sequence[bool]],
          start: Tuple[int, int], goal: Tuple[int, int],
          width: int, height: int,
          max_expansions: int = 200000) -> List[Tuple[int, int]]:
    """Least-cost path in cells, or [] when there is none.

    Lethal cells are not merely expensive: they are removed from the graph. A
    ditch must not be crossable at any price, however inconvenient the detour,
    which is the difference between a cost function and a constraint.
    """
    if not (0 <= start[0] < width and 0 <= start[1] < height):
        return []
    if not (0 <= goal[0] < width and 0 <= goal[1] < height):
        return []
    if lethal[goal[1]][goal[0]]:
        return []

    def h(cx, cy):
        return math.hypot(cx - goal[0], cy - goal[1])

    open_heap = [(h(*start), 0.0, start, None)]
    came: dict = {}
    best: dict = {start: 0.0}
    expansions = 0

    while open_heap:
        _, g, node, parent = heapq.heappop(open_heap)
        if node in came:
            continue
        came[node] = parent
        if node == goal:
            break
        expansions += 1
        if expansions > max_expansions:
            return []

        cx, cy = node
        for dx, dy, step in NEIGHBOURS:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            if lethal[ny][nx] or (nx, ny) in came:
                continue
            # Distance plus terrain penalty. Both matter: pure distance ignores
            # the terrain, pure cost wanders.
            ng = g + step * (1.0 + COST_GAIN * cost[ny][nx])
            if ng < best.get((nx, ny), float("inf")):
                best[(nx, ny)] = ng
                heapq.heappush(open_heap, (ng + h(nx, ny), ng, (nx, ny), node))

    if goal not in came:
        return []

    path = []
    node: Optional[Tuple[int, int]] = goal
    while node is not None:
        path.append(node)
        node = came[node]
    path.reverse()
    return path


def pure_pursuit(pose: Tuple[float, float, float],
                 path: Sequence[Tuple[float, float]],
                 lookahead: float = 1.2,
                 v_nominal: float = 1.0,
                 max_omega: float = 1.6) -> Tuple[float, float]:
    """Velocity command toward a point `lookahead` metres along the path.

    Returns the command Nav2 would produce. It goes to the supervisor, never
    straight to the vehicle -- the same seam as /cmd_vel_nav in the real stack.
    """
    if len(path) < 2:
        return 0.0, 0.0

    x, y, th = pose
    target = path[-1]
    for px, py in path:
        if math.hypot(px - x, py - y) >= lookahead:
            target = (px, py)
            break

    dx, dy = target[0] - x, target[1] - y
    distance = math.hypot(dx, dy)
    if distance < 1e-6:
        return 0.0, 0.0

    heading = math.atan2(dy, dx)
    error = math.atan2(math.sin(heading - th), math.cos(heading - th))

    omega = max(-max_omega, min(max_omega, 2.0 * error))
    # Slow into turns: a differential-drive vehicle that keeps full speed
    # through a sharp correction overshoots and oscillates.
    speed = v_nominal * max(0.15, math.cos(min(abs(error), math.pi / 2)))
    if abs(error) > 1.2:
        speed = 0.0                     # turn on the spot first
    return speed, omega
