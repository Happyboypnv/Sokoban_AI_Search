import time
from heapq import heappush, heappop
from collections import deque
from sokoban import sokoban_goal_state

INF = 10 ** 6


# 1. PULL-BFS TÍNH KHOẢNG CÁCH ĐẨY THỰC TẾ TỪ ĐÍCH
def precompute_distance(initial_state):
    distances = {}
    width, height = initial_state.width, initial_state.height
    obstacles = initial_state.obstacles

    for storage in initial_state.storage:
        distances[storage] = {storage: 0}
        queue = deque([storage])

        while queue:
            current_pos = queue.popleft()
            dist = distances[storage][current_pos]

            for dx, dy in [(0, -1), (0, 1), (1, 0), (-1, 0)]:
                prev_box_pos = (current_pos[0] + dx, current_pos[1] + dy)
                robot_pos = (current_pos[0] + 2 * dx, current_pos[1] + 2 * dy)

                if not (0 <= prev_box_pos[0] < width and 0 <= prev_box_pos[1] < height):
                    continue
                if not (0 <= robot_pos[0] < width and 0 <= robot_pos[1] < height):
                    continue
                if prev_box_pos in obstacles or robot_pos in obstacles:
                    continue

                if prev_box_pos not in distances[storage]:
                    distances[storage][prev_box_pos] = dist + 1
                    queue.append(prev_box_pos)

    return distances


# 2. XÁC ĐỊNH Ô DEADLOCK TĨNH TỰ ĐỘNG TỪ BẢNG KHOẢNG CÁCH 
def get_dead_squares(state, distances):
    dead_squares = set()
    for x in range(state.width):
        for y in range(state.height):
            pos = (x, y)
            if pos not in state.obstacles:
                if all(pos not in distances[st] for st in state.storage):
                    dead_squares.add(pos)
    return dead_squares


# 3. KIỂM TRA FREEZE DEADLOCK ĐỘNG (ITERATIVE UNFREEZING)
def is_freeze_deadlock(state, dead_squares, pushed_box):
    width, height = state.width, state.height
    obstacles = state.obstacles
    boxes = state.boxes
    storages = state.storage

    def is_wall(x, y):
        return x < 0 or x >= width or y < 0 or y >= height or (x, y) in obstacles

    def is_axis_statically_blocked(x, y, dx, dy):
        pos1, pos2 = (x - dx, y - dy), (x + dx, y + dy)
        if is_wall(pos1[0], pos1[1]) or is_wall(pos2[0], pos2[1]):
            return True
        return pos1 in dead_squares and pos2 in dead_squares

    # Gom cụm các thùng liên thông với thùng vừa đẩy
    connected_boxes = {pushed_box}
    stack = [pushed_box]
    while stack:
        bx, by = stack.pop()
        for dx, dy in [(0, -1), (0, 1), (1, 0), (-1, 0)]:
            nb = (bx + dx, by + dy)
            if nb in boxes and nb not in connected_boxes:
                connected_boxes.add(nb)
                stack.append(nb)

    # Gỡ dần các thùng không bị khóa đồng thời cả 2 trục Ngang & Dọc
    frozen_boxes = set(connected_boxes)
    changed = True
    while changed:
        changed = False
        for (bx, by) in list(frozen_boxes):
            blocked_h = (
                is_axis_statically_blocked(bx, by, 1, 0)
                or (bx - 1, by) in frozen_boxes
                or (bx + 1, by) in frozen_boxes
            )
            blocked_v = (
                is_axis_statically_blocked(bx, by, 0, 1)
                or (bx, by - 1) in frozen_boxes
                or (bx, by + 1) in frozen_boxes
            )
            if not (blocked_h and blocked_v):
                frozen_boxes.remove((bx, by))
                changed = True

    return any(box not in storages for box in frozen_boxes)


# 4. MA TRẬN CHI PHÍ & THUẬT TOÁN HUNGARIAN (N THÙNG x M ĐÍCH)
def build_cost_matrix(state, distances):
    boxes = list(state.boxes)
    storages = list(state.storage)
    matrix = [[INF] * len(storages) for _ in range(len(boxes))]
    for i, box in enumerate(boxes):
        for j, storage in enumerate(storages):
            if box in distances[storage]:
                matrix[i][j] = distances[storage][box]
    return matrix


def hungarian_algorithm(cost_matrix):
    n = len(cost_matrix)
    if n == 0:
        return 0
    m = len(cost_matrix[0])

    u = [0] * (n + 1)
    v = [0] * (m + 1)
    p = [0] * (m + 1)
    way = [0] * (m + 1)

    for i in range(1, n + 1):
        u[i] = min(cost_matrix[i - 1][j - 1] for j in range(1, m + 1))

    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = [float('inf')] * (m + 1)
        used = [False] * (m + 1)
        while p[j0] != 0:
            used[j0] = True
            i0 = p[j0]
            delta = float('inf')
            j1 = 0
            for j in range(1, m + 1):
                if not used[j]:
                    cur = cost_matrix[i0 - 1][j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta = minv[j]
                        j1 = j
            for j in range(m + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
        while j0 != 0:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1

    total_cost = sum(cost_matrix[p[j] - 1][j - 1] for j in range(1, m + 1) if p[j] != 0)
    return min(total_cost, INF)


def heuristic(state, distances, heuristic_cache):
    boxes_key = frozenset(state.boxes)
    if boxes_key in heuristic_cache:
        return heuristic_cache[boxes_key]

    if len(state.boxes) == 1:
        box = next(iter(state.boxes))
        h_val = min((distances[st].get(box, INF) for st in state.storage), default=INF)
    else:
        matrix = build_cost_matrix(state, distances)
        h_val = hungarian_algorithm(matrix)

    heuristic_cache[boxes_key] = h_val
    return h_val


# 5. THUẬT TOÁN TÌM KIẾM ANYTIME WEIGHTED A* (ARA*)
def solve(initial_state, timebound=120):
    if timebound is None or timebound <= 0:
        return False
    if sokoban_goal_state(initial_state):
        return initial_state

    start_time = time.monotonic()
    deadline = start_time + 0.85 * timebound

    distances = precompute_distance(initial_state)
    dead_squares = get_dead_squares(initial_state, distances)
    heuristic_cache = {}

    if len(initial_state.boxes) > len(initial_state.storage):
        return False
    for box in initial_state.boxes:
        if box in dead_squares:
            return False
        if box not in initial_state.storage and is_freeze_deadlock(initial_state, dead_squares, box):
            return False

    h0 = heuristic(initial_state, distances, heuristic_cache)
    if h0 >= INF:
        return False

    best_goal_state = False
    best_cost = INF
    weights = (6, 3, 2, 1.5, 1.2, 1)

    for w in weights:
        pq = []
        heappush(pq, (w * h0, h0, initial_state.index, initial_state))
        best_g = {initial_state.hashable_state(): 0}
        found_better = False
        expansions = 0

        while pq:
            expansions += 1
            if expansions % 128 == 0 and time.monotonic() > deadline:
                return best_goal_state

            f_score, h_val, c_index, current_state = heappop(pq)
            g_val = current_state.gval

            if g_val + h_val >= best_cost:
                continue
            if best_g.get(current_state.hashable_state(), INF) < g_val:
                continue

            for successor in current_state.successors():
                succ_g = successor.gval
                pushed_boxes = successor.boxes - current_state.boxes

                # Chỉ kiểm tra Deadlock và tính lại Heuristic khi có thùng bị đẩy
                if pushed_boxes:
                    pushed_box = next(iter(pushed_boxes))
                    if sokoban_goal_state(successor):
                        if succ_g < best_cost:
                            best_cost = succ_g
                            best_goal_state = successor
                            found_better = True
                            if w > 1:
                                break
                        continue

                    if pushed_box in dead_squares or is_freeze_deadlock(successor, dead_squares, pushed_box):
                        continue

                    succ_h = heuristic(successor, distances, heuristic_cache)
                    if succ_h >= INF:
                        continue
                else:
                    succ_h = h_val

                if succ_g + succ_h >= best_cost:
                    continue

                succ_key = successor.hashable_state()
                if best_g.get(succ_key, INF) <= succ_g:
                    continue
                best_g[succ_key] = succ_g

                heappush(pq, (succ_g + w * succ_h, succ_h, successor.index, successor))

            if found_better and w > 1:
                break

        if not found_better:
            break

    return best_goal_state