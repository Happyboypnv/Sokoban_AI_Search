import heapq
import time
from collections import deque
from sokoban import sokoban_goal_state

INF = 10**6
DL_CHECK = True
WEIGHTS = (6, 3, 2, 1.5, 1.2, 1) 
TIME_FRACTION = 0.8 
MAX_NODES = 1_000_000

# BFS => tính khoảng cách từ mỗi đích đến mọi vtri
def bfs_cost(state):
    storages = list(state.storage)

    def safe(c):
        return 0 <= c[0] < state.width and 0 <= c[1] <= state.height and c not in state.obstacles

    distances = []
    for s in storages:
        dist = {s: 0}
        q = deque([s])
        while q:
            x, y = q.popleft()
            for dx, dy in ((0, -1), (1, 0), (0, 1), (-1, 0)):
                b = (x + dx, y + dy)
                r = (x + 2*dx, y + 2*dy)     # vị trí robot đứng để đẩy thùng
                if safe(b) and safe(r) and b not in dist:
                    dist[b] = dist[(x, y)] + 1
                    q.append(b)
        distances.append(dist)

    # cost_table[ô] = [khoảng cách tới từng đích]; ô không có trong bảng = ô deadlock
    cost_table = {}
    for dist in distances:
        for cell in dist:
            if cell not in cost_table:
                cost_table[cell] = [d.get(cell, INF) for d in distances]

    # min_cost[ô] = đích gần nhất -> heuristic của 1 thùng: tra bảng O(1)
    min_cost = {c: min(row) for c, row in cost_table.items()}
    return cost_table, min_cost


""" 
Mỗi ô (0 phải deadlock), mỗi trục (0: ngang, 1: dọc), lưu:
   neigh[ax][c]      = (hxóm 1, hxóm 2) theo trục đó  
   static_blk[ax][c] = True nếu bị chặn: tường/ 2 bên là ô deadlock 
"""
def build_static(state, cost_table):
    width, height, walls = state.width, state.height, state.obstacles

    def is_wall(c):
        return c[0] < 0 or c[0] >= width or c[1] < 0 or c[1] >= height or c in walls

    neigh = ({}, {})
    static_blk = ({}, {})
    for c in cost_table:
        for ax, (dx, dy) in enumerate(((1, 0), (0, 1))):
            n1 = (c[0] - dx, c[1] - dy)
            n2 = (c[0] + dx, c[1] + dy)
            neigh[ax][c] = (n1, n2)
            static_blk[ax][c] = (is_wall(n1) or is_wall(n2) or (n1 not in cost_table and n2 not in cost_table))
    return static_blk, neigh


# DEADLOCK
"""
Coi mọi thùng bị freeze, lặp: loại dần khỏi tập nếu 0 bị chặn trên cả 2 trục
Còn thùng mà 0 trên đích -> deadlock
"""
def is_deadlock(boxes, storages, static_blk, neigh):
    frozen = set(boxes)
    changed = True
    while changed:
        changed = False
        for b in list(frozen):
            for ax in (0, 1):
                n1, n2 = neigh[ax][b]
                if not (static_blk[ax][b] or n1 in frozen or n2 in frozen):
                    frozen.discard(b)
                    changed = True
                    break
    return any(b not in storages for b in frozen)


# HUNGARIAN => slack = A[i][j] - u[i] - v[j]
def hungarian(a):
    n, m = len(a), len(a[0])
    u, v = [0]*(n+1), [0]*(m+1)
    matching, trace = [0]*(m+1), [0]*(m+1)    
    
    for i in range(1, n + 1):
        matching[0] = i
        j0 = 0
        minv = [float('inf')]*(m+1)          
        visited = [False]*(m+1)
        
        while True:                          
            visited[j0] = True
            i0 = matching[j0]
            delta, j1 = float('inf'), 0      
            
            for j in range(1, m + 1):
                if not visited[j]:
                    cur = a[i0-1][j-1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        trace[j] = j0        
                    if minv[j] < delta:     
                        delta = minv[j]
                        j1 = j
            
            for j in range(m+1):
                if visited[j]:
                    u[matching[j]] += delta     
                    v[j] -= delta
                else:
                    minv[j] -= delta            
                    
            j0 = j1
            if matching[j0] == 0: break         
            
        while j0:
            pre = trace[j0]
            matching[j0] = matching[pre]
            j0 = pre
            
    return -v[0]     
                          

# Bảng tính trước Heuristic
def build_heuristic(state, cost_table, min_cost, static_blk, neigh):
    storages = state.storage
    n_storage = len(storages)
    cache = {}

    def heuristic(s):
        boxes = s.boxes
        h = cache.get(boxes)
        if h is not None: return h

        if not boxes: h = 0
        elif len(boxes) > n_storage: h = INF                    # thungf > dichs
        elif any(b not in min_cost for b in boxes): h = INF     # thungf tren o dl
        elif DL_CHECK and is_deadlock(boxes, storages, static_blk, neigh): h = INF
        elif len(boxes) == 1:                                   # 1 thungf -> return min
            h = min_cost[next(iter(boxes))]
        else:
            total = hungarian([cost_table[b] for b in boxes])
            h = INF if total >= INF else total

        cache[boxes] = h
        return h
    
    return heuristic


# INITIAL HOPELESS
def initially_hopeless(state, min_cost, static_blk, neigh, heuristic):
    boxes = state.boxes
    if len(state.robots) == 0: return True
    if len(boxes) > len(state.storage): return True
    if any(b not in min_cost for b in boxes): return True
    if is_deadlock(boxes, state.storage, static_blk, neigh): return True
    return heuristic(state) >= INF


# ARA*
def ara(initial_state, w, best, deadline, heuristic):
    #return (status, best, goal_state). status: found / exhausted / limit
    best_goal = None
    counter = 0
    h0 = heuristic(initial_state)
    heap = [(initial_state.gval + w * h0, h0, counter, initial_state)]
    best_g = {initial_state.hashable_state(): initial_state.gval}
    expansions = 0

    while heap:
        f, h, _, cur = heapq.heappop(heap)
        g = cur.gval
        if g + h >= best: continue
        if g > best_g.get(cur.hashable_state(), float("inf")): continue

        
        if time.time() > deadline or len(best_g) > MAX_NODES:
            return "limit", best, best_goal

        for nxt in cur.successors():
            ng = nxt.gval
            nkey = nxt.hashable_state()
            if ng >= best_g.get(nkey, float("inf")):
                continue
            nh = heuristic(nxt)
            if nh >= INF or ng + nh >= best:
                continue
            best_g[nkey] = ng
 
            if sokoban_goal_state(nxt):
                best, best_goal = ng, nxt
                if w > 1:
                    return "found", best, best_goal
                continue
 
            counter += 1
            heapq.heappush(heap, (ng + w * nh, nh, counter, nxt))
 
    return "exhausted", best, best_goal


def solve(initial_state, timebound=120):
    start = time.time()

    if timebound is None or timebound <= 0: return False
    if sokoban_goal_state(initial_state): return initial_state
        
    cost_table = bfs_cost(initial_state)
    heuristic_cache = {}

    deadline = start + TIME_FRACTION * timebound
 
    # tính trước toàn bộ bảng phụ thuộc map
    cost_table, min_cost = bfs_cost(initial_state)
    static_blk, neigh = build_static(initial_state, cost_table)
    heuristic = build_heuristic(initial_state, cost_table, min_cost, static_blk, neigh)
 
    if initially_hopeless(initial_state, min_cost, static_blk, neigh, heuristic):
        return False
 
    best_cost, best_goal = float("inf"), None
    for w in WEIGHTS:
        status, best_cost, goal = ara(initial_state, w, best_cost, deadline, heuristic)
        if goal is not None:
            best_goal = goal
        if status != "found":
            break
 
    return best_goal if best_goal is not None else False

    