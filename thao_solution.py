import heapq
import time
from sokoban import sokoban_goal_state

def heuristic(state):
    total = 0
    boxes = state.boxes
    storages = state.storage

    for bi, bj in boxes:
        if (bi, bj) in storages: #neu thung da nam tren o dich, chi phi = 0
            continue

        #tim khoang cach min tu thung -> o dich
        min_dis = float("inf")
        for si, sj in storages:
            dis = abs(bi - si) + abs(bj - sj)
            if dis < min_dis:
                min_dis = dis
        total += min_dis
    return total


# A* algorithm
def solve(initial_state, timebound=120):
    start_time = time.time()

    if timebound <= 0: return False

    if sokoban_goal_state(initial_state): #neu tat ca thung o dung dich
        return initial_state

    ## Open set
    open_heap = []
    counter = 0

    h_start = heuristic(initial_state)
    f_start = initial_state.gval + h_start  #ban dau gval = 0

    heapq.heappush(open_heap, (f_start, counter, initial_state))

    ## Closed set
    best_g = {}
    best_g[initial_state.hashable_state()] = initial_state.gval

    # A* loop
    while open_heap:
        if time.time() - start_time >= timebound:
            return False

        f_val, _, current_state = heapq.heappop(open_heap)

        if sokoban_goal_state(current_state): #neu den dich -> tra ve current_state de trace nguoc parent
            return current_state

        current_key = current_state.hashable_state()
        if current_state.gval > best_g.get(current_key, float("inf")):
            continue

        for next_state in current_state.successors():
            next_key = next_state.hashable_state()
            next_g = next_state.gval

            if next_g < best_g.get(next_key, float("inf")):
                best_g[next_key] = next_g
                h_val = heuristic(next_state)
                f_next = next_g + h_val
                counter += 1
                heapq.heappush(open_heap, (f_next, counter, next_state))
    return False

     

