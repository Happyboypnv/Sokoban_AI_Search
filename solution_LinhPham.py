
from sokoban import sokoban_goal_state
from collections import deque
from itertools import count
import heapq
import time

def bfs(state):
    distances = {}
    queue = deque()

    for goal in state.storage:
        distances[goal] = 0
        queue.append(goal)

    directions = ((0, 1), (1, 0), (0, -1), (-1, 0))

    while queue:
        x, y = queue.popleft()

        for dx, dy in directions:
            neighbor = (x + dx, y + dy)
            nx, ny = neighbor

            if ((nx < 0 or nx >= state.width) or (ny < 0 or ny >= state.height)):
                continue
            if (neighbor in state.obstacles or neighbor in distances):
                continue

            distances[neighbor] = distances[(x, y)] + 1
            queue.append(neighbor)

    return distances

def heuristics(state, goal_distances):
    total = 0

    for box in state.boxes:
        if box not in goal_distances:
            return float("inf")

        total += goal_distances[box]

    return total



def solve(initial_state, timebound=120):
    if sokoban_goal_state(initial_state):
        return initial_state

    if timebound <= 0:
        return False

    deadline = time.monotonic() + timebound

    goal_distances = bfs(initial_state)

    if(time.monotonic() >= deadline):
        return False

    initial_h = heuristics(initial_state, goal_distances)

    if initial_h == float("inf"):
        return False

    order = count()
    frontier = []

    heapq.heappush(frontier, (initial_state.gval + initial_h, next(order), initial_state))

    best_g = {initial_state.hashable_state(): initial_state.gval}

    while frontier and time.monotonic() < deadline:
        _, _, current = heapq.heappop(frontier)

        current_key = current.hashable_state()

        if current.gval != best_g[current_key]:
            continue

        if sokoban_goal_state(current):
            return current

        for successor in current.successors():
            successor_key = successor.hashable_state()

            if successor.gval >= best_g.get(successor_key, float("inf")):
                continue

            h_value = heuristics(successor, goal_distances)

            if h_value == float("inf"):
                continue

            best_g[successor_key] = successor.gval

            f_value = successor.gval + h_value

            heapq.heappush(frontier, (f_value, next(order), successor))

    return False
