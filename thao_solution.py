import heapq
import time
from collections import deque
from sokoban import sokoban_goal_state

INF = 10**6  # xly cac o deadlock

# TIEN XU LY: bfs => tinh khoang cach ngan nhat tu moi vtri -> dich
def bfs_cost(state):
    storages = list(state.storage)
    distances = []
    for s in storages: 
        dist = {s: 0}        #bfs tu dich s {vtri: so buoc}
        q = deque([s])
        while q:
            x, y = q.popleft()
            for dx, dy in ((0,-1),(1,0),(0,1),(-1,0)):
                nx, ny = x+dx, y+dy
                if 0<=nx<state.width and 0<=ny<state.height and (nx,ny) not in state.obstacles and (nx,ny) not in dist:
                    dist[(nx,ny)] = dist[(x,y)] + 1
                    q.append((nx,ny))
        distances.append(dist)
    
    cost_table = {}
    for dist in distances:
        for cell in dist:
            if cell not in cost_table:
                cost_table[cell] = [d.get(cell, INF) for d in distances]
    return cost_table

# HUNGARIAN => tinh slack = A[i][j] - u[i] - v[j], tim delta min trong minv
def hungarian(a):
    n, m = len(a), len(a[0])
    u, v = [0]*(n+1), [0]*(m+1)
    matching, trace = [0]*(m+1), [0]*(m+1)    #match[j] -> dich j gan voi thung nao, trace[j] -> luu vet
    
    for i in range(1, n+1):
        matching[0] = i
        j0 = 0
        minv = [float('inf')]*(m+1)          #gtri slack min cua dich j
        visited = [False]*(m+1)
        
        while True:                          #tim augmenting path
            visited[j0] = True
            i0 = matching[j0]
            delta, j1 = float('inf'), 0      #delta: khoang cach min de tao new rigid edge
            
            # update slack + tim delta
            for j in range(1, m+1):
                if not visited[j]:
                    cur = a[i0-1][j-1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        trace[j] = j0        # luu trace tu j0 -> j
                    if minv[j] < delta:      #tim delta min
                        delta = minv[j]
                        j1 = j
            
            # update u, v
            for j in range(m+1):
                if visited[j]:
                    u[matching[j]] += delta     #thung +delta, dich -delta
                    v[j] -= delta
                else:
                    minv[j] -= delta            #giam slack cua dich chua tham
                    
            j0 = j1
            if matching[j0] == 0: break         #dich trong -> tim dc augmenting path
            
        # Truy vet augmeting path
        while j0:
            pre = trace[j0]
            matching[j0] = matching[pre]
            j0 = pre
            
    return -v[0]                               #tong cost = -v[0]

# A* SEARCH + HEURISTIC
def solve(initial_state, timebound=120):
    start = time.time()

    if timebound <= 0: return False
    if sokoban_goal_state(initial_state): return initial_state
        
    cost_table = bfs_cost(initial_state)
    heuristic_cache = {}

    # Heuristic
    def heuristic(state):
        boxes = state.boxes
        if boxes in heuristic_cache:
            return heuristic_cache[boxes]
        matrix = [cost_table.get(b) for b in boxes]
        if any(r is None for r in matrix): h = INF   #deadlock
        else:
            total = min(matrix[0]) if len(matrix) == 1 else hungarian(matrix)
            h = INF if total >= INF else total

        heuristic_cache[boxes] = h
        return h

    open_heap = []
    counter = 0

    h_start = heuristic(initial_state)
    if h_start >= INF: return False

    f_start = initial_state.gval + h_start     
    heapq.heappush(open_heap, (f_start, counter, initial_state))

    best_g = {}
    best_g[initial_state.hashable_state()] = initial_state.gval

    # A* loop
    while open_heap:
        if time.time() - start >= timebound:
            return False

        f_val, _, current_state = heapq.heappop(open_heap)

        if sokoban_goal_state(current_state):
            return current_state

        current_key = current_state.hashable_state()
        if current_state.gval > best_g.get(current_key, float("inf")):
            continue

        for next_state in current_state.successors():
            next_key = next_state.hashable_state()
            next_g = next_state.gval

            if next_g < best_g.get(next_key, float("inf")):
                h_val = heuristic(next_state)
                if h_val >= INF:  continue
                best_g[next_key] = next_g
                f_next = next_g + h_val
                counter += 1
                heapq.heappush(open_heap, (f_next, counter, next_state))
                
    return False