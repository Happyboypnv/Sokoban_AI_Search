
from sokoban import sokoban_goal_state

import heapq
import time

DIR_NAMES = ("up", "right", "down", "left")
DELTAS = ((0, -1), (1, 0), (0, 1), (-1, 0))
BIG = 10 ** 6
INF = float('inf') 
MAX_NODES = 1500000
WEIGHTS = (6, 3, 2, 1.5, 1.2, 1)
TIME_FRACTION = 0.8

def in_code(x, y, W):
    return y * W + x

def check_Pos(x, y, W, H):
    return 0 <= x and x < W and 0 <= y and y < H

def hungarian(cost, n, m):
    a = [[0] * (m + 1) for i in range(n + 1)]

    for i in range(n):
        for j in range(m):
            a[i + 1][j + 1] = cost[i][j]

    matching = [0] * (m + 1)
    lx = [0] * (n + 1)
    ly = [0] * (m + 1)
    trace = [0] * (m + 1)

    for i in range(1, n + 1):
        lx[i] = min(a[i][j] for j in range(1, m + 1))

    for i in range(1, n + 1):
        matching[0] = i
        j0 = 0
        visited = [False] * (m + 1)
        minv = [INF] * (m + 1)

        while True:
            visited[j0] = True
            i0 = matching[j0]
            delta = INF
            j1 = 0

            for j in range(1, m + 1):
                if not visited[j]:
                    cur_slack = a[i0][j] - lx[i0] - ly[j]

                    if cur_slack < minv[j]:
                        minv[j] = cur_slack
                        trace[j] = j0

                    if minv[j] < delta:
                        delta = minv[j]
                        j1 = j

            for j in range(0, m + 1):
                if visited[j]:
                    lx[matching[j]] += delta
                    ly[j] -= delta

                else:
                    minv[j] -= delta

            j0 = j1

            if matching[j0] == 0:
                break

        while True:
            pre_j = trace[j0]
            matching[j0] = matching[pre_j]
            j0 = pre_j
            if j0 == 0:
                break

    ans = 0

    for j in range(1, m + 1):
        if matching[j] != 0:
            ans += a[matching[j]][j]

    return ans

class Solver:
    def __init__(self, state):
        W = state.width
        H = state.height
        C = W * H
        self.C = C
        walls = {in_code(x, y, W) for (x, y) in state.obstacles}
        self.nb = [[-1] * C for i in range(4)]

        for y in range(H):
            for x in range(W):
                c = in_code(x, y, W)

                if c in walls:
                    continue

                for d, (dx, dy) in enumerate(DELTAS):
                    newX, newY = x + dx, y + dy

                    if check_Pos(newX, newY, W, H) and in_code(newX, newY, W) not in walls:
                        self.nb[d][c] = in_code(newX, newY, W)

        goals = sorted(in_code(x, y, W) for (x, y) in state.storage)
        self.goals = goals
        self.is_goals = [False] * C
        self.goal_mask = 0

        for g in goals:
            self.is_goals[g] = True
            self.goal_mask |= 1 << g

        pushdist = []

        for g in goals:
            dist = [BIG] * C
            dist[g] = 0
            queue = [g]

            for q in queue:
                for d in range(4):
                    back = (d + 2) % 4
                    p = self.nb[back][q]

                    if p < 0 or dist[p] != BIG:
                        continue

                    r = self.nb[back][p]

                    if r < 0:
                        continue

                    dist[p] = dist[q] + 1
                    queue.append(p)

            pushdist.append(dist)

        self.cost_rows = [[pd[c] for pd in pushdist] for c in range(C)]
        self.mindist = [min(row) if row else BIG for row in self.cost_rows]
        self.dead = [self.mindist[c] >= BIG for c in range(C)]
        self.r0 = tuple(in_code(x, y, W) for (x, y) in state.robots)
        self.m0 = 0

        for (x, y) in state.boxes:
            self.m0 |= 1 << in_code(x, y, W)

        self.heuristicCache = {}
        self.sblk = [[False] * C for i in range(2)]
        self.nbm = [[0] * C for i in range(2)]
        for ax, (da, db) in enumerate(((1, 3), (0, 2))):
            for c in range(C):
                a, b = self.nb[da][c], self.nb[db][c]

                if a < 0 or b < 0 or (self.dead[a] and self.dead[b]):
                    self.sblk[ax][c] = True

                else:
                    self.nbm[ax][c] = (1 << a) | (1 << b)

    def heuristic(self, mask):
        ans = self.heuristicCache.get(mask)

        if ans is not None:
            return ans

        cells = []
        bitmask = mask

        while bitmask:
            low = bitmask & -bitmask
            cells.append(low.bit_length() - 1)
            bitmask ^= low

        if len(cells) == 1:
            ans = self.mindist[cells[0]]

        else:
            ans = hungarian([self.cost_rows[c] for c in cells], len(cells), len(self.goals))

        if ans >= BIG:
            ans = BIG

        self.heuristicCache[mask] = ans
    
        return ans

    def frozen(self, c, mask):
        comp, stack = 1 << c, [c]

        while stack:
            x = stack.pop()

            for d in range(4):
                y = self.nb[d][x]

                if 0 <= y and (mask >> y) & 1 and not (comp >> y) & 1:
                    comp |=1 << y
                    stack.append(y)

        F, changed = comp, True

        while changed:
            tmp, changed = F, False

            while tmp:
                low = tmp & -tmp
                pos = low.bit_length() - 1
                tmp ^= low

                if pos < 0:
                    continue

                h = self.sblk[0][pos] or (F & self.nbm[0][pos])
                v = self.sblk[1][pos] or (F & self.nbm[1][pos])

                if not (h and v):
                    F ^= low
                    changed = True

        return bool(F & ~self.goal_mask)

    def check(self):
        m = self.m0

        if bin(m).count("1") > len(self.goals):
            return True

        for c in range(self.C):
            if (m >> c) & 1:
                if self.dead[c]:
                    return True

                if not self.is_goals[c] and self.frozen(c, m):
                    return True

        return self.heuristic(m) >= BIG

    def search(self, dl):
        nb = self.nb
        dead = self.dead
        hfun = self.heuristic
        goal_mask = self.goal_mask

        r0, m0 = self.r0, self.m0
        start_key = (tuple(sorted(r0)), m0)

        gl = [0]         
        par = [-1]   
        act = [-1]   
        rob = [r0]  
        msk = [m0] 
        keys = [start_key]

        best_g = {start_key: 0}
        best_node = {start_key: 0}

        open_set = {start_key}
        closed = set()
        incons = set()

        serial = 0
        h0 = hfun(m0)
        heap = [(WEIGHTS[0] * h0, h0, serial, 0)]

        best_cost = INF
        best_actions = None
        expansions = 0

        def make_actions(node_id):
            actions = []

            while par[node_id] >= 0:
                actions.append(act[node_id])
                node_id = par[node_id]
            
            actions.reverse()

            return actions

        for weight_index, weight in enumerate(WEIGHTS):
            while open_set and time.monotonic() < dl:
                while heap:
                    _, _, _, node_id = heap[0]
                    node_key = keys[node_id]

                    if node_key in open_set and best_node.get(node_key) == node_id:
                        break

                    heapq.heappop(heap)

                if not heap:
                    break

                if best_actions is not None and heap[0][0] >= best_cost:
                    break

                _, h_value, _, node_id = heapq.heappop(heap)
                node_key = keys[node_id]

                if node_key not in open_set:
                    continue

                if best_node.get(node_key) != node_id:
                    continue

                open_set.remove(node_key)
                closed.add(node_key)

                g_value = gl[node_id]

                if g_value != best_g[node_key]:
                    continue

                expansions += 1

                if expansions % 256 == 0:
                    if time.monotonic() >= dl or len(gl) >= MAX_NODES:
                        return best_actions

                robot_positions = rob[node_id]
                box_mask = msk[node_id]

                if (box_mask | goal_mask) == goal_mask:
                    if g_value < best_cost:
                        best_cost = g_value
                        best_actions = make_actions(node_id)
                        
                    continue

                next_g = g_value + 1

                for robot_index, position in enumerate(robot_positions):
                    for direction in range(4):
                        next_cell = nb[direction][position]

                        if next_cell < 0 or next_cell in robot_positions:
                            continue

                        next_mask = box_mask
                        pushed = False

                        if (box_mask >> next_cell) & 1:
                            box_destination = nb[direction][next_cell]

                            if (
                                box_destination < 0
                                or dead[box_destination]
                                or (box_mask >> box_destination) & 1
                                or box_destination in robot_positions
                            ):
                                continue

                            next_mask = (box_mask ^ (1 << next_cell)) | (1 << box_destination)
                            pushed = True

                        is_goal = (next_mask | goal_mask) == goal_mask

                        if pushed and not is_goal:
                            if self.frozen(box_destination, next_mask):
                                continue

                        if is_goal:
                            next_h = 0

                        elif pushed:
                            next_h = hfun(next_mask)

                            if next_h >= BIG:
                                continue

                        else:
                            next_h = h_value

                        if next_g + next_h >= best_cost:
                            continue

                        next_robots = robot_positions[:robot_index] + (next_cell,) + robot_positions[robot_index + 1: ]
                        next_key = (tuple(sorted(next_robots)), next_mask)

                        if next_g >= best_g.get(next_key, INF):
                            continue

                        best_g[next_key] = next_g

                        new_node = len(gl)
                        best_node[next_key] = new_node

                        gl.append(next_g)
                        par.append(node_id)
                        act.append(in_code(direction, robot_index, 4))
                        rob.append(next_robots)
                        msk.append(next_mask)
                        keys.append(next_key)

                        if next_key in closed:
                            incons.add(next_key)

                        else:
                            open_set.add(next_key)
                            serial += 1
                            priority = in_code(next_g, next_h, weight)

                            heapq.heappush(heap, (priority, next_h, serial, new_node))

            if weight_index == len(WEIGHTS) - 1:
                break

            if time.monotonic() >= dl or len(gl) >= MAX_NODES:
                break

            open_set.update(incons)
            incons.clear()
            closed.clear()

            next_weight = WEIGHTS[weight_index + 1]
            heap = []

            for key in open_set:
                node_id = best_node[key]
                node_h = hfun(msk[node_id])
                serial += 1
                priority = gl[node_id] + next_weight * node_h

                heapq.heappush(heap, (priority, node_h, serial, node_id))

        return best_actions

def solve(initial_state, timebound = 120):
    start = time.monotonic()

    if timebound is None or timebound <= 0:
        return False

    if sokoban_goal_state(initial_state):
        return initial_state

    dl = start + TIME_FRACTION * timebound

    solver = Solver(initial_state)

    if not solver.r0 or solver.check():
        return False

    best_actions = solver.search(dl)

    if best_actions is None:
        return False

    state = initial_state
    for code in best_actions:
        name = "%d %s" % (code // 4, DIR_NAMES[code % 4])

        for succ in state.successors():
            if succ.action == name:
                state = succ
                break

        else:
            return False

    return state if sokoban_goal_state(state) else False