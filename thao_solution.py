import heapq
import time
from collections import deque
from sokoban import sokoban_goal_state

INF = 10**6  

#bfs tu dich -> vtri i
def bfs(state, storage): 
    
    distance = {storage: 0}
    queue = deque([storage])
    while queue:
        x, y = queue.popleft()
        for dx, dy in ((0, -1), (1, 0), (0, 1), (-1, 0)):
            nxt = (x + dx, y + dy)
            if not (0 <= nxt[0] < state.width and 0 <= nxt[1] < state.height):
                continue                      # ngoai map
            if nxt in state.obstacles or nxt in distance:
                continue                      # gap tuong / da tham

            distance[nxt] = distance[(x, y)] + 1
            queue.append(nxt)

    return distance

def build_cost(state):
    storages = list(state.storage)
    # khoag cach tu moi o -> dich
    distances = [bfs(state, s) for s in storages]
    
    cost_table = {}
    for dis in distances:
        for k in dis:
            if k not in cost_table:
                # moi o luu cost den dich
                cost_table[k] = [d.get(k, INF) for d in distances]
    return cost_table

# hungarian
def hungarian(a):
   
    n = len(a)
    m = len(a[0])
    inf = float("inf")

    u = [0] * (n + 1)                
    v = [0] * (m + 1)                
    matching = [0] * (m + 1)         
    trace = [0] * (m + 1)            

    for i in range(1, n + 1):
        matching[0] = i               
        j0 = 0
        minv = [inf] * (m + 1)        
        visited = [False] * (m + 1)   

        while True:
            visited[j0] = True
            i0 = matching[j0]         # thung dag gan voi dich vua tham
            row = a[i0 - 1]
            u_i0 = u[i0]
            delta = inf              
            j1 = 0

            for j in range(1, m + 1):
                if not visited[j]:
                    cur_slack = row[j - 1] - u_i0 - v[j]
                    if cur_slack < minv[j]:
                        minv[j] = cur_slack
                        trace[j] = j0     
                    if minv[j] < delta:
                        delta = minv[j]
                        j1 = j

            for j in range(m + 1):
                if visited[j]:
                    u[matching[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta

            j0 = j1
            if matching[j0] == 0:     
                break

        # dao nguoc cap ghep
        while j0:
            pre_j = trace[j0]
            matching[j0] = matching[pre_j]
            j0 = pre_j

    # v[0] gom tong cac delta => -v[0] = cost min
    return -v[0]


# heuristic
def heuristic(boxes, cost_table):
    cost_matrix = []
    for box in boxes:
        row = cost_table.get(box)
        if row is None:                   # state chet
            return INF
        cost_matrix.append(row)

    if len(cost_matrix) == 1:             # 1 thung, k can hungarian
        total = min(cost_matrix[0])
    else:
        total = hungarian(cost_matrix)
        
    return INF if total >= INF else total


# A* search
def solve(initial_state, timebound=120):
    start_time = time.time()

    if timebound <= 0: return False

    if sokoban_goal_state(initial_state):
        return initial_state

    cost_table = build_cost(initial_state)   
    heuristic_cache = {}                          

    def get_h(state):
        h = heuristic_cache.get(state.boxes)
        if h is None:
            h = heuristic(state.boxes, cost_table)
            heuristic_cache[state.boxes] = h
        return h

    h_start = get_h(initial_state)
    if h_start >= INF:
        return False

    open_heap = []
    counter = 0                                    
    heapq.heappush(open_heap, (initial_state.gval + h_start, counter, initial_state))
    best_g = {initial_state.hashable_state(): initial_state.gval}

    while open_heap:
        if time.time() - start_time >= timebound:
            return False

        _, _, current_state = heapq.heappop(open_heap)

        if sokoban_goal_state(current_state):
            return current_state

        if current_state.gval > best_g.get(current_state.hashable_state(), float("inf")):
            continue                               

        for next_state in current_state.successors():
            next_key = next_state.hashable_state()
            next_g = next_state.gval
            if next_g < best_g.get(next_key, float("inf")):
                h = get_h(next_state)
                if h >= INF: continue                # State chet 
                best_g[next_key] = next_g
                counter += 1
                heapq.heappush(open_heap, (next_g + h, counter, next_state))
    return False