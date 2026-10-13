import heapq
import time

from sokoban import sokoban_goal_state

_DIR_NAMES = ("up", "right", "down", "left")
_DELTAS = ((0, -1), (1, 0), (0, 1), (-1, 0))
_BIG = 10 ** 6                 # chi phi "khong toi duoc"
_MAX_NODES = 1_500_000         # tran bo nho cho moi luot tim kiem
_WEIGHTS = (6, 3, 2, 1.5, 1.2, 1)
_TIME_FRACTION = 0.8           # dung toi da 80% timebound


def in_code(x, y, W, H):
    return y*W + x
def check_Pos(x, y, W, H):
    return x >= 0 and x < W and y >= 0 and y < H
def _hungarian(arr, n, m):
    a = [[0]*(m + 1) for _ in range(n + 1)]
    for i in range(n):
        for j in range(m):
            a[i + 1][j + 1] = arr[i][j]
    matching = [0]*(m + 1)
    lx = [0]*(n + 1)
    ly = [0]*(m + 1)
    trace = [0]*(m + 1)
    for i in range(1, n + 1):
        lx[i] = min(a[i][j] for j in range(1, m + 1))
    for i in range(1, n + 1):
        matching[0] = i
        j0 = 0

        visited = [False]*(m + 1)
        minv = [float('inf')]*(m + 1)
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
            if j0 == 0: break
    ans = 0
    for j in range(1, m + 1):
        if matching[j] != 0:
            ans += a[matching[j]][j]
    return ans
class Solver:
    def __init__(self, state):
        W = state.width
        H = state.height
        C = W*H
        self.C = C
        walls = {in_code(x, y, W, H) for (x, y) in state.obstacles}
        #build nb
        #nb[d][c] = o ke c theo huong d
        #nb[d][c] = -1 neu nam ngoai bang hoac la tuong
        self.nb = [[-1]*C for _ in range(4)]
        for y in range(H):
            for x in range(W):
                c = in_code(x, y, W, H)
                if c in walls: continue
                for d, (dx, dy) in enumerate(_DELTAS):
                    newX, newY = x + dx, y + dy
                    if check_Pos(newX, newY, W, H) and in_code(newX, newY, W, H) not in walls:
                        self.nb[d][c] = in_code(newX, newY, W, H)
        # build goals_mask : day bit 0100100.. voi 1 la o chua dich
        goals = sorted(in_code(x, y, W, H) for (x, y) in state.storage)
        self.goals = goals
        self.is_goals = [False]*C
        self.goal_mask = 0
        for g in goals:
            self.is_goals[g] = True
            self.goal_mask |= 1 << g
        #pushdist : BFS nguoc (keo thung) tu tung dich
        nb = self.nb
        pushdist = []
        for g in goals:
            dist = [_BIG]*C
            dist[g] = 0
            queue = [g]
            for q in queue:
                for d in range(4):
                    back = (d + 2)%4
                    p = nb[back][q] # o truoc do
                    if p < 0 or dist[p] != _BIG: continue
                    r = nb[back][p] # pos cua robot de day
                    if r < 0: continue
                    dist[p] = dist[q] + 1
                    queue.append(p)
            pushdist.append(dist)
        self.cost_rows = [[pd[c] for pd in pushdist] for c in range(C)] # cost tu 1 o den goals
        self.mindist = [min(row) if row else _BIG for row in self.cost_rows] # min cost tu 1 o den goal[]
        self.dead = [self.mindist[c] >= _BIG for c in range(C)]  # 1 o ma co min dist la _BIG thi la o dead khong di vao
        self.r0 = tuple(in_code(x, y, W, H) for (x, y) in state.robots)
        self.m0 = 0
        for (x, y) in state.boxes:
            self.m0 |= 1 << (in_code(x, y, W, H))
        self.heuristicCache = {}
        self.sblk = [[False]*C for _ in range(2)]
        self.nbm = [[0]*C for _ in range(2)]
        for ax, (da, db) in enumerate(((1, 3), (0, 2))):
            for c in range(C):
                a, b = self.nb[da][c], self.nb[db][c]
                if a < 0 or b < 0 or (self.dead[a] and self.dead[b]):
                    self.sblk[ax][c] = True
                else:
                    self.nbm[ax][c] = (1 << a)|(1 << b)
    # build heuristic function voi state la bitmask cua cac o thung
    def HeuristicFunction(self, mask):
        ans = self.heuristicCache.get(mask)
        if ans is not None: return ans
        # convert to cells
        cells = []
        bitmask = mask
        while bitmask:
            low = bitmask&-bitmask
            #         x = 12 -> 0 0 0 0 1 1 0 0
            #        ~x      -> 1 1 1 1 0 0 1 1
            #    ~x + 1 = -12-> 1 1 1 1 0 1 0 0
            #    -> x&-x     -> 0 0 0 0 0 1 0 0
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
    # build deadlock
    def _frozen(self, c, mask):
        comp, stack = 1 << c, [c]
        while stack:
            x = stack.pop()
            for d in range(0, 4):
                y = self.nb[d][x]
                if y >= 0 and (mask >> y)&1 and not (comp >> y) & 1:
                    comp |= (1 << y)
                    stack.append(y)
        F, changed = comp, True
        while changed:
            tmp, changed = F, False
            while tmp:
                low = tmp&-tmp
                pos = low.bit_length() - 1
                tmp ^= low
                if pos < 0: continue
                h = self.sblk[0][pos] or (F & self.nbm[0][pos])
                v = self.sblk[1][pos] or (F & self.nbm[1][pos])
                if not (h and v):
                    F ^= low
                    changed = True
        return bool(F & ~self.goal_mask)



    def inititally_hopeless(self):
        m = self.m0
        if bin(m).count("1") > len(self.goals): return True
        for c in range(self.C):
            if(m >> c)&1:
                if self.dead[c]:
                    return True
                if not self.is_goals[c] and self._frozen(c, m):
                    return True
        return self.HeuristicFunction(m) >= _BIG
    def search(self, w, best, deadline):
        nb, dead, hfun, goal_mask = self.nb, self.dead, self.HeuristicFunction, self.goal_mask
        r0, m0 = self.r0, self.m0
        gl, par, act, rob, msk, keys = [0], [-1], [-1], [r0], [m0], [(tuple(sorted(r0)), m0)]
        bestg = {keys[0]: 0}
        h0 = hfun(m0)
        # (f, h, node_index)
        heap = [(w*h0, h0, 0)]
        best_actions = None
        expansions = 0
        push, pop = heapq.heappush, heapq.heappop

        while heap:
            f, hv, i = pop(heap)
            g = gl[i]
            # branch and bound
            if g + hv >= best:
                continue
            if bestg[keys[i]] < g:
                continue
            expansions += 1
            if expansions % 256 == 0:
                if time.monotonic() > deadline or len(gl) > _MAX_NODES:
                    return "limit", best, best_actions
            r , m= rob[i], msk[i]
            newg = g + 1
            for ri in range(len(r)):
                pos = r[ri]
                for d in range(4):
                    n = nb[d][pos]
                    if n < 0 or n in r:
                        continue
                    newmask = m
                    pushed = False
                    if (m >> n)&1:
                        b2 = nb[d][n]
                        if b2 < 0 or dead[b2] or (m >> b2)&1 or b2 in r:
                            continue
                        newmask = (m^(1 << n)) | (1 << b2)
                        pushed = True
                    if pushed:
                        if (newmask | goal_mask) == goal_mask:
                            if newg < best:
                                best = newg
                                acts = [ri*4 + d]
                                j = i
                                while par[j] >= 0:
                                    acts.append(act[j])
                                    j = par[j]
                                acts.reverse()
                                best_actions = acts
                                if w > 1:
                                    return "found", best, best_actions
                            continue
                        if self._frozen(b2, newmask):
                            continue
                        newh = hfun(newmask)
                        if newh >= _BIG:
                            continue
                    else:
                        newh = hv
                    if newg + newh >= best: continue
                    newr = r[:ri] + (n, ) + r[ri + 1:]
                    k = (tuple(sorted(newr)), newmask)
                    if bestg.get(k, _BIG*10) <= newg:
                        continue
                    bestg[k] = newg
                    gl.append(newg)
                    par.append(i)
                    act.append(ri*4 + d)
                    rob.append(newr)
                    msk.append(newmask)
                    keys.append(k)
                    push(heap, (newg + w*newh, newh, len(gl) - 1))

        return "exhausted", best, best_actions

def solve(initial_state, timebound=120):
    """Return the best goal found by ARA*, or False if no goal is found."""
    start = time.monotonic()
    if timebound is None or timebound <= 0:
        return False
    if sokoban_goal_state(initial_state):
        return initial_state
    deadline = start + _TIME_FRACTION*timebound
    solver = Solver(initial_state)
    if not solver.r0 or solver.inititally_hopeless():
        return False

    best, best_actions = _BIG*10, None
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
        name = "%d %s" %(code//4, _DIR_NAMES[code%4])
        for succ in state.successors():
            if succ.action == name:
                state = succ
                break
        else: return False
    return state if sokoban_goal_state(state) else False
