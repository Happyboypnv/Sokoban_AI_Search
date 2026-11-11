"""Sokoban solver based on an ARA* search with a goal-distance heuristic.

The implementation is optimized for the Sokoban problem used in this course: a
board may contain multiple robots, boxes, storage cells, and obstacles. Each
move is one robot step or one push, and the solver tries to find a valid path to
 a goal state where every box is on a storage tile.

Main ideas:
- Precompute cell indexes and neighbors for each board cell.
- Model each box placement as a bitmask; the search state is a pair of robot
  positions and the box configuration.
- Use a lower-bound heuristic based on assignment cost between boxes and goals.
- Run an anytime ARA* search with decreasing weights to find a near-optimal
  solution under a time budget.
"""

import heapq
import time
from collections import deque

from sokoban import sokoban_goal_state

_DIR_NAMES = ("up", "right", "down", "left")
_DELTAS = ((0, -1), (1, 0), (0, 1), (-1, 0))
_BIG = 10 ** 6
_MAX_NODES = 800_000
_WEIGHTS = (6, 3, 2, 1.5, 1.2, 1)
_LEGACY_WEIGHTS = (100, 20, 6, 3, 2, 1.5, 1.2, 1)
_TIME_FRACTION = 0.95
_LEGACY_TIME_FRACTION = 0.35


def in_code(x, y, W, H):
    if not (0 <= x < W and 0 <= y < H):
        raise ValueError(f"Out of bounds: {(x, y)} for board {W}x{H}")
    return y * W + x


def check_Pos(x, y, W, H):
    """Return True when a coordinate lies within the board boundaries."""
    return x >= 0 and x < W and y >= 0 and y < H


def _hungarian(arr, n, m):
    """Solve the assignment problem using the Hungarian algorithm.

    The input is a matrix of size n x m, where each row represents a box and each
    column represents a storage goal. The algorithm minimizes the total cost of
    assigning boxes to reachable goals, which is used as a heuristic estimate of
    how far the current box arrangement is from the goal.
    """
    a = [[0] * (m + 1) for _ in range(n + 1)]
    for i in range(n):
        for j in range(m):
            a[i + 1][j + 1] = arr[i][j]
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
        minv = [float('inf')] * (m + 1)
        while True:
            visited[j0] = True
            i0 = matching[j0]
            delta = float('inf')
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
    """Build a search model for one Sokoban instance.

    The solver precomputes:
    - neighboring cells for each direction,
    - distances from every goal tile to every board cell (push distances),
    - a deadlock mask for cells that are effectively unreachable or blocked,
    - a bitmask representation of the current box configuration.
    """

    def __init__(self, state):
        W = state.width
        H = state.height
        self.board_width = W
        C = W * H
        self.C = C
        walls = {in_code(x, y, W, H) for (x, y) in state.obstacles}
        self.nb = [[-1] * C for _ in range(4)]
        for y in range(H):
            for x in range(W):
                c = in_code(x, y, W, H)
                if c in walls:
                    continue
                for d, (dx, dy) in enumerate(_DELTAS):
                    newX, newY = x + dx, y + dy
                    if check_Pos(newX, newY, W, H) and in_code(newX, newY, W, H) not in walls:
                        self.nb[d][c] = in_code(newX, newY, W, H)

        goals = sorted(in_code(x, y, W, H) for (x, y) in state.storage)
        self.goals = goals
        self.is_goals = [False] * C
        self.goal_mask = 0
        for g in goals:
            self.is_goals[g] = True
            self.goal_mask |= 1 << g

        pushdist = []
        for g in goals:
            dist = [_BIG] * C
            dist[g] = 0
            queue = [g]
            for q in queue:
                for d in range(4):
                    back = (d + 2) % 4
                    p = self.nb[back][q]
                    if p < 0 or dist[p] != _BIG:
                        continue
                    r = self.nb[back][p]
                    if r < 0:
                        continue
                    dist[p] = dist[q] + 1
                    queue.append(p)
            pushdist.append(dist)

        self.cost_rows = [[pd[c] for pd in pushdist] for c in range(C)]
        self.mindist = [min(row) if row else _BIG for row in self.cost_rows]
        self.dead = [self.mindist[c] >= _BIG for c in range(C)]
        self.r0 = tuple(in_code(x, y, W, H) for (x, y) in state.robots)
        self.m0 = 0
        for (x, y) in state.boxes:
            self.m0 |= 1 << in_code(x, y, W, H)
        self.heuristicCache = {}
        self.reachabilityCache = {}
        self.robotHeuristicCache = {}
        self.sblk = [[False] * C for _ in range(2)]
        self.nbm = [[0] * C for _ in range(2)]
        for ax, (da, db) in enumerate(((1, 3), (0, 2))):
            for c in range(C):
                a, b = self.nb[da][c], self.nb[db][c]
                if a < 0 or b < 0 or (self.dead[a] and self.dead[b]):
                    self.sblk[ax][c] = True
                else:
                    self.nbm[ax][c] = (1 << a) | (1 << b)

    def HeuristicFunction(self, mask):
        """Estimate the minimum cost from the current box configuration to goals.

        The heuristic converts the box bitmask into a list of occupied cells and
        then solves a minimum-cost assignment between boxes and goals. If no valid
        assignment exists or a box is in a dead cell, the heuristic becomes
        infinite.
        """
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
            ans = _hungarian([self.cost_rows[c] for c in cells], len(cells), len(self.goals))
        if ans >= _BIG:
            ans = _BIG
        self.heuristicCache[mask] = ans
        return ans

    def _frozen(self, c, mask):
        """Check whether a box is trapped in a reachable but unsolvable region.

        The method grows a connected component from the given cell and removes any
        cell whose two orthogonal directions are not both valid. If the connected
        region contains any cell outside the goal set, the box is considered frozen.
        """
        comp, stack = 1 << c, [c]
        while stack:
            x = stack.pop()
            for d in range(4):
                y = self.nb[d][x]
                if y >= 0 and (mask >> y) & 1 and not (comp >> y) & 1:
                    comp |= 1 << y
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

    def inititally_hopeless(self):
        """Fast rejection test for impossible initial layouts.

        This catches obvious dead situations such as too many boxes for goals,
        boxes sitting in dead cells, or frozen configurations that cannot ever be
        resolved.
        """
        m = self.m0
        if bin(m).count("1") > len(self.goals):
            return True
        for c in range(self.C):
            if (m >> c) & 1:
                if self.dead[c]:
                    return True
                if not self.is_goals[c] and self._frozen(c, m):
                    return True
        return self.HeuristicFunction(m) >= _BIG

    def _walk_reachability(self, robot, robots, mask):
        """Return and cache shortest walks for one robot with fixed boxes."""
        other_robots = tuple(
            sorted(other for other in robots if other != robot)
        )
        key = (robot, other_robots, mask)
        cached = self.reachabilityCache.get(key)
        if cached is not None:
            return cached

        dist = [_BIG] * self.C
        parent = [-1] * self.C
        move = [-1] * self.C
        blocked = set(other_robots)
        dist[robot] = 0
        queue = deque([robot])
        while queue:
            current = queue.popleft()
            for direction in range(4):
                nxt = self.nb[direction][current]
                if nxt < 0 or dist[nxt] != _BIG:
                    continue
                if (mask >> nxt) & 1 or nxt in blocked:
                    continue
                dist[nxt] = dist[current] + 1
                parent[nxt] = current
                move[nxt] = direction
                queue.append(nxt)

        result = (tuple(dist), tuple(parent), tuple(move))
        self.reachabilityCache[key] = result
        return result

    @staticmethod
    def _walk_path(target, parent, move):
        path = []
        while parent[target] >= 0:
            path.append(move[target])
            target = parent[target]
        path.reverse()
        return path

    def _robot_walk_lower_bound(self, robots, mask):
        """Return a cheap lower bound for walking to the next legal push."""
        key = (robots, mask)
        cached = self.robotHeuristicCache.get(key)
        if cached is not None:
            return cached

        occupied = set(robots)
        boxes = []
        remaining = mask
        while remaining:
            low = remaining & -remaining
            boxes.append(low.bit_length() - 1)
            remaining ^= low

        best_walk = _BIG
        for robot in robots:
            robot_x = robot % self.board_width
            robot_y = robot // self.board_width
            for box in boxes:
                for direction in range(4):
                    front = self.nb[(direction + 2) % 4][box]
                    destination = self.nb[direction][box]
                    if front < 0 or destination < 0:
                        continue
                    if (mask >> destination) & 1:
                        continue
                    if destination in occupied or self.dead[destination]:
                        continue
                    front_x = front % self.board_width
                    front_y = front // self.board_width
                    distance = abs(robot_x - front_x) + abs(robot_y - front_y)
                    if distance < best_walk:
                        best_walk = distance

        self.robotHeuristicCache[key] = best_walk
        return best_walk

    def _macro_pushes(self, robots, mask):
        """Generate successors containing a walk followed by one push."""
        occupied = set(robots)
        boxes = []
        remaining = mask
        while remaining:
            low = remaining & -remaining
            boxes.append(low.bit_length() - 1)
            remaining ^= low

        for robot_id, robot in enumerate(robots):
            dist, parent, move = self._walk_reachability(robot, robots, mask)
            for box in boxes:
                for direction in range(4):
                    front = self.nb[(direction + 2) % 4][box]
                    destination = self.nb[direction][box]
                    if front < 0 or destination < 0:
                        continue
                    if dist[front] >= _BIG:
                        continue
                    if (mask >> destination) & 1:
                        continue
                    if destination in occupied or self.dead[destination]:
                        continue

                    newmask = (mask ^ (1 << box)) | (1 << destination)
                    if (newmask | self.goal_mask) != self.goal_mask:
                        if self._frozen(destination, newmask):
                            continue
                        if self.HeuristicFunction(newmask) >= _BIG:
                            continue

                    walk = self._walk_path(front, parent, move)
                    actions = [robot_id * 4 + d for d in walk]
                    actions.append(robot_id * 4 + direction)
                    newrobots = list(robots)
                    newrobots[robot_id] = box
                    yield tuple(newrobots), newmask, len(actions), actions

    @staticmethod
    def _reconstruct_actions(node, parents, segments):
        result = []
        chain = []
        while node >= 0:
            if segments[node]:
                chain.append(segments[node])
            node = parents[node]
        for segment in reversed(chain):
            result.extend(segment)
        return result

    def search_legacy(self, w, best, deadline):
        """Run the original step-by-step weighted A* search.

        This fallback is useful on small or open boards where walking steps are
        cheap to search directly. It is intentionally bounded so the macro-push
        search still receives most of the caller's time budget.
        """
        nb, dead = self.nb, self.dead
        hfun, goal_mask = self.HeuristicFunction, self.goal_mask
        r0, m0 = self.r0, self.m0
        gl, par, act = [0], [-1], [-1]
        rob, msk = [r0], [m0]
        keys = [(tuple(sorted(r0)), m0)]
        bestg = {keys[0]: 0}
        h0 = hfun(m0)
        heap = [(w * h0, h0, 0)]
        best_actions = None
        expansions = 0
        push, pop = heapq.heappush, heapq.heappop

        while heap:
            _, hv, i = pop(heap)
            g = gl[i]
            if g + hv >= best:
                continue
            if bestg[keys[i]] < g:
                continue
            expansions += 1
            if expansions % 256 == 0:
                if time.monotonic() > deadline or len(gl) > _MAX_NODES:
                    return "limit", best, best_actions

            r, m = rob[i], msk[i]
            newg = g + 1
            for ri in range(len(r)):
                pos = r[ri]
                for direction in range(4):
                    nxt = nb[direction][pos]
                    if nxt < 0 or nxt in r:
                        continue
                    newmask = m
                    pushed = False
                    if (m >> nxt) & 1:
                        box_destination = nb[direction][nxt]
                        if (
                            box_destination < 0
                            or dead[box_destination]
                            or (m >> box_destination) & 1
                            or box_destination in r
                        ):
                            continue
                        newmask = (m ^ (1 << nxt)) | (1 << box_destination)
                        pushed = True

                    if pushed:
                        if (newmask | goal_mask) == goal_mask:
                            if newg < best:
                                best = newg
                                actions = [ri * 4 + direction]
                                node = i
                                while par[node] >= 0:
                                    actions.append(act[node])
                                    node = par[node]
                                actions.reverse()
                                best_actions = actions
                                if w > 1:
                                    return "found", best, best_actions
                            continue
                        if self._frozen(box_destination, newmask):
                            continue
                        newh = hfun(newmask)
                        if newh >= _BIG:
                            continue
                    else:
                        newh = hv

                    if newg + newh >= best:
                        continue
                    newr = r[:ri] + (nxt,) + r[ri + 1:]
                    key = (tuple(sorted(newr)), newmask)
                    if bestg.get(key, _BIG * 10) <= newg:
                        continue
                    bestg[key] = newg
                    gl.append(newg)
                    par.append(i)
                    act.append(ri * 4 + direction)
                    rob.append(newr)
                    msk.append(newmask)
                    keys.append(key)
                    push(heap, (newg + w * newh, newh, len(gl) - 1))

        return "exhausted", best, best_actions

    def search(self, w, best, deadline):
        """Run a weighted A* search for a better solution under the time limit.

        Parameters:
            w: heuristic weight, used by ARA*.
            best: current best known cost bound.
            deadline: monotonic timestamp after which the search must stop.

        Returns a tuple (status, bound, actions):
            - status in {"found", "exhausted", "limit"}
            - bound is the best cost found so far
            - actions is the best action sequence, if found
        """
        r0, m0 = self.r0, self.m0
        gl, par = [0], [-1]
        segments = [[]]
        rob, msk = [r0], [m0]
        keys = [(r0, m0)]
        bestg = {keys[0]: 0}
        hfun = self.HeuristicFunction
        h0 = hfun(m0)
        robot_h0 = self._robot_walk_lower_bound(r0, m0)
        heap = [(w * h0, robot_h0, 0)]
        best_actions = None
        expansions = 0
        push, pop = heapq.heappush, heapq.heappop

        while heap:
            f, robot_hv, i = pop(heap)
            g = gl[i]
            hv = hfun(msk[i])
            if g + hv >= best:
                continue
            if bestg[keys[i]] < g:
                continue
            expansions += 1
            if expansions % 256 == 0:
                if time.monotonic() > deadline or len(gl) > _MAX_NODES:
                    return "limit", best, best_actions
            r, m = rob[i], msk[i]
            for newr, newmask, step_cost, macro_actions in self._macro_pushes(r, m):
                newg = g + step_cost
                if (newmask | self.goal_mask) == self.goal_mask:
                    if newg < best:
                        best = newg
                        best_actions = (
                            self._reconstruct_actions(i, par, segments)
                            + macro_actions
                        )
                        if w > 1:
                            return "found", best, best_actions
                    continue
                newh = hfun(newmask)
                if newg + newh >= best:
                    continue
                robot_h = self._robot_walk_lower_bound(newr, newmask)
                k = (newr, newmask)
                if bestg.get(k, _BIG * 10) <= newg:
                    continue
                bestg[k] = newg
                gl.append(newg)
                par.append(i)
                segments.append(macro_actions)
                rob.append(newr)
                msk.append(newmask)
                keys.append(k)
                push(heap, (newg + w * newh, robot_h, len(gl) - 1))

        return "exhausted", best, best_actions


def solve(initial_state, timebound=120):
    """Return a valid goal state found by the Sokoban solver.

    The function runs an anytime search loop with different weights and returns the
    first valid goal path found before the time budget expires. If no solution can
    be discovered within the allowed time, it returns False.
    """
    start = time.monotonic()
    if timebound is None or timebound <= 0:
        return False
    if sokoban_goal_state(initial_state):
        return initial_state
    deadline = start + _TIME_FRACTION * timebound
    solver = Solver(initial_state)
    if not solver.r0 or solver.inititally_hopeless():
        return False

    # Try the original step-level search first. A short private deadline keeps
    # the macro search available for boards where walking states explode.
    legacy_deadline = min(
        deadline,
        start + _LEGACY_TIME_FRACTION * timebound,
    )
    legacy_actions = None
    legacy_best = _BIG * 10
    for w in _LEGACY_WEIGHTS:
        status, legacy_best, acts = solver.search_legacy(
            w, legacy_best, legacy_deadline
        )
        if acts is not None:
            legacy_actions = acts
        if time.monotonic() >= legacy_deadline:
            break

    if legacy_actions is not None:
        best_actions = legacy_actions
    else:
        best, best_actions = _BIG * 10, None
        for w in _WEIGHTS:
            status, best, acts = solver.search(w, best, deadline)
            if acts is not None:
                best_actions = acts
            if status != "found":
                break

    if best_actions is None:
        return False

    state = initial_state
    for code in best_actions:
        name = "%d %s" % (code // 4, _DIR_NAMES[code % 4])
        for succ in state.successors():
            if succ.action == name:
                state = succ
                break
        else:
            return False
    return state if sokoban_goal_state(state) else False
