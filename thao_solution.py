import heapq
import time
from collections import deque
from sokoban import sokoban_goal_state

INF = 10**6  
DL_CHECK = True 

# BFS => tính khoảng cách từ mỗi đích đến mọi vtri
def bfs_cost(state):
    storages = list(state.storage)

    def safe(c): 
        return 0 <= c[0] < state.width and 0 <= c[1] < state.height and c not in state.obstacles
    
    distances = []
    for s in storages: 
        dist = {s: 0}        
        q = deque([s])
        while q:
            x, y = q.popleft()
            for dx, dy in ((0,-1), (1,0), (0,1), (-1,0)):
                e = (x + dx, y + dy)            
                f = (x + 2*dx, y + 2*dy)         #vtri robot đứng để đẩy thùng
                if safe(e) and safe(f) and e not in dist:
                    dist[e] = dist[(x, y)] + 1
                    q.append(e)
        distances.append(dist)

    #cost_table[ô] = [khoảng cách tới từng đích]; ô k có trong bảng = ô deadlock
    cost_table = {}
    for dist in distances:
        for cell in dist:
            if cell not in cost_table:
                cost_table[cell] = [d.get(cell, INF) for d in distances]
    return cost_table


# DEADLOCK
"""
Gồm: khối 2x2, các thùng kẹt dọc tường
1 thùng bị chặn theo chiều ngang(tương tự dọc):
- bên trái/phải là tường
- 2 bên là ô deadlock
- bên trái/phải là thùng bị chặn theo trục dọc
Thùng bị freeze không phải đích -> bỏ """

def is_deadlock(boxes, state, cost_table):
    walls, storages = state.obstacles, state.storage
    width, height = state.width, state.height

    def is_wall(c):
        return c[0] < 0 or c[0] >= width or c[1] < 0 or c[1] >= height or c in walls

    def blocked(b, horizontal, s):
        # s: các thùng đg xét ở tầng trên
        dx, dy = (1, 0) if horizontal else (0, 1)
        n1 = (b[0] - dx, b[1] - dy)
        n2 = (b[0] + dx, b[1] + dy)
        if is_wall(n1) or is_wall(n2): return True
        if n1 not in cost_table and n2 not in cost_table: return True
        for n in (n1, n2):
            if n in s: return True
            if n in boxes and blocked(n, not horizontal, s | {b}):
                return True
        return False

    empty = frozenset()
    for b in boxes:
        if b not in storages and blocked(b, True, empty) and blocked(b, False, empty):
            return True
    return False


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
            
            # update slack + tìm delta
            for j in range(1, m+1):
                if not visited[j]:
                    cur = a[i0-1][j-1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        trace[j] = j0        
                    if minv[j] < delta:      #tìm delta min
                        delta = minv[j]
                        j1 = j
            
            # update u, v
            for j in range(m+1):
                if visited[j]:
                    u[matching[j]] += delta     #thùng +delta, đihcs -delta
                    v[j] -= delta
                else:
                    minv[j] -= delta            
                    
            j0 = j1
            if matching[j0] == 0: break         #đích trống -> tìm dc augmenting path
            
        # Truy vet augmeting path
        while j0:
            pre = trace[j0]
            matching[j0] = matching[pre]
            j0 = pre
            
    return -v[0]     
                          

# A* + HEURISTIC
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
        if any(r is None for r in matrix):                          #thùng nằm trên ô deadlock
            h = INF   
        elif DL_CHECK and is_deadlock(boxes, state, cost_table):    #thùng freeze
            h = INF
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

    best_g = {initial_state.hashable_state() : initial_state.gval}

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
