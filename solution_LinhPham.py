
from sokoban import sokoban_goal_state
from collections import deque
from itertools import count
import heapq
import time

def bfs(state, goal):
    distances = {goal : 0}
    queue = deque([goal])

    directions = ((0, 1), (1, 0), (0, -1), (-1, 0))

    while queue:
        x, y = queue.popleft()

        for dx, dy in directions:
            neighbor = (x + dx, y + dy)
            nx, ny = neighbor

            if (nx < 0 or nx >= state.width) or (ny < 0 or ny >= state.height):
                continue
            if neighbor in state.obstacles or neighbor in distances:
                continue

            distances[neighbor] = distances[(x, y)] + 1
            queue.append(neighbor)

    return distances

def hungarian(costs):
    n = len(costs)

    if n == 0:
        return 0

    m = len(costs[0])

    if n > m:
        return float("inf")

    matching = [0] * (m + 1)
    lx = [0] * (n + 1)
    ly = [0] * (m + 1)
    trace = [0] * (m + 1)

    for i in range(1, n + 1):
        lx[i] = min(costs[i - 1])

    for i in range(1, n + 1):
        matching[0] = i
        j0 = 0
        minv = [float("inf")] * (m + 1)
        visited = [False] * (m + 1)

        while True:
            visited[j0] = True
            i0 = matching[j0]
            delta = float("inf")
            j1 = 0

            for j in range(1, m + 1):
                if visited[j]:
                    continue

                slack = costs[i0 - 1][j - 1] - lx[i0] - ly[j]

                if slack < minv[j]:
                    minv[j] = slack
                    trace[j] = j0

                if minv[j] < delta:
                    delta = minv[j]
                    j1 = j

            for j in range(m + 1):
                if visited[j]:
                    lx[matching[j]] += delta
                    ly[j] -= delta

                else:
                    minv[j] -= delta

            j0 = j1

            if matching[j0] == 0:
                break

        while True:
            previous_j = trace[j0]
            matching[j0] = matching[previous_j]
            j0 = previous_j

            if j0 == 0:
                break

    total_cost = 0

    for j in range(1, m + 1):
        if matching[j] != 0:
            total_cost += costs[matching[j] -1][j - 1]

    return total_cost

def heuristics(state, goal_distances):
    boxes = sorted(state.boxes)
    goals = sorted(state.storage)

    if len(boxes) > len(goals):
        return float("inf")

    if not boxes:
        return 0

    unreachable = (state.width * state.height + 1) * (len(boxes) + 1)

    costs = []

    for box in boxes:
        row = []

        for goal in goals:
            distance = goal_distances[goal].get(box, unreachable)
            row.append(distance)

        costs.append(row)

    total = hungarian(costs)

    if total >= unreachable:
        return float("inf")

    return total
    
def solve(initial_state, timebound=120):
    if sokoban_goal_state(initial_state):
        return initial_state

    if timebound <= 0:
        return False

    deadline = time.monotonic() + timebound

    goal_distances = {goal: bfs(initial_state, goal) for goal in initial_state.storage}

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
