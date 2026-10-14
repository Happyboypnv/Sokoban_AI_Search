import time
from heapq import heappush, heappop
from sokoban import sokoban_goal_state
from collections import deque

# def distance(p1,p2):
#     return abs(p1[0]-p2[0])+abs(p1[1]-p2[1])

def precompute_distance(initial_state):
    distances = {}
    # duyet bfs nguoc tu o dich 
    for storage in initial_state.storage:
        distances[storage] = {}
        queue = deque([(storage, 0)]) # toa do va khoang cach 
        visited = set([storage])

        while queue:
            current_pos, dist = queue.popleft() 
            distances[storage][current_pos] = dist 
            # loang ra 4 huong 
            for dx, dy in [(0, -1), (0, 1), (1, 0), (-1, 0)]:
                next_pos = (current_pos[0] + dx, current_pos[1] + dy) 
                if next_pos[0] < 0 or next_pos[0] >= initial_state.width: 
                    continue 
                if next_pos[1] < 0 or next_pos[1] >= initial_state.height:
                    continue 
                if next_pos in initial_state.obstacles:
                    continue 

                if next_pos not in visited:
                    visited.add(next_pos)
                    queue.append((next_pos, dist + 1))
    return distances


# tao ma tran chi phi 
def build_cost_matrix(state, distances):
    boxes = list(state.boxes)
    storages = list(state.storage)
    n = len(boxes) 
    INF = 999999
    matrix = [[INF] * n for _ in range(n)]
    for i, box in enumerate(boxes):
        for j, storage in enumerate(storages) : 
            if box in distances[storage]:
                matrix[i][j] = distances[storage][box] 

    return matrix 

def hungarian_algorithm(cost_matrix):
    n=len(cost_matrix)
    if n == 0 :
        return 0 
    u = [0] * (n+1) 
    v = [0] * (n+1) 
    p = [0] * (n+1) # luu index cua hang ghep cot j  
    way = [0] * (n+1) 
    for i in range(1, n+1):
        p[0] = i 
        j0 = 0 
        minv = [float('inf')] * (n+1) 
        used = [False] * (n+1) 
        while p[j0] != 0 :
            used[j0] = True 
            i0 = p[j0]
            delta = float('inf') 
            j1 = 0 
            # cap nhat mang minv 
            for j in range(1, n+1):
                if not used[j]:
                    cur = cost_matrix[i0 - 1][j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0 
                    if minv[j] < delta:
                        delta = minv[j]
                        j1 = j
            # cap nhat cach nhan 
            for j in range(n+1):
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
    # total ghep cao 
    total_cost = 0 
    for j in range(1, n+1):
        if p[j] != 0:
            total_cost += cost_matrix[p[j]-1][j-1] 

    return total_cost


def heuristic(state, distances):

    matrix = build_cost_matrix(state, distances)
    
    return hungarian_algorithm(matrix)


def solve(initial_state, timebound=120):
    # A* serach algorithm
    start_time = time.time() 
    distances = precompute_distance(initial_state)
    pq = []
    # push trang thai dau tien vao
    heappush(pq, (heuristic(initial_state, distances), initial_state.index, initial_state))
    visited = set()
    while pq: 
        if time.time()-start_time>timebound:
            return False
        f_score, c_index, current_state = heappop(pq) 

        if sokoban_goal_state(current_state):
            return current_state 
        # xu ly de tranh trung lap
        state_key = current_state.hashable_state() 
        if state_key in visited:
            continue 
        visited.add(state_key)

        # mo rong cac trang thai con 
        for successor in current_state.successors() :
            succ_key = successor.hashable_state() 
            if succ_key not in visited: 
                # f(n) = g(n) + h(n)
                g_val = successor.gval
                h_val = heuristic(successor, distances)
                f_val = g_val + h_val
                heappush(pq, (f_val, successor.index, successor))

    return False



# def heuristic(state):
#     total_cost = 0
#     # tạo một bản sao danh sách đích 
#     available_storage = list(state.storage)
#     for box in state.boxes:
#         # box goal 
#         if box in available_storage:
#             available_storage.remove(box) 
#             continue 
#         # tim distance min box
#         min_dist = float('inf')
#         best_storage = None
#         for st in available_storage:
#             d = distance(box, st) 
#             if (d<min_dist) :
#                 min_dist = d 
#                 best_storage = st
#         # total cost if true 
#         if best_storage: 
#             total_cost += min_dist
#             available_storage.remove(best_storage)

#     return total_cost

""" def solve(initial_state, timebound=120):
    # Return the best goal found, or False if no goal is found.
    start_time = time.time()
    
    queue = deque([initial_state])
    
    # Tập hợp visited lưu lại các cấu hình bản đồ đã đi qua để tránh lặp chu trình
    visited = set()
    visited.add(initial_state.hashable_state())
    
    while queue:
        # Kiểm tra điều kiện ngắt thời gian để không bị quá hạn
        if time.time() - start_time > timebound:
            return False
            
        # Lấy trạng thái ở đầu hàng đợi ra (First-In, First-Out)
        current_state = queue.popleft()
        
        # Nếu trạng thái hiện tại là đích, framework sẽ tự dò ngược chuỗi parent để lấy đường đi
        if sokoban_goal_state(current_state):
            return current_state
            
        # Sinh các trạng thái kế tiếp hợp lệ
        for successor in current_state.successors():
            succ_key = successor.hashable_state()
            
            # Nếu trạng thái này chưa từng được duyệt, thêm vào hàng đợi
            if succ_key not in visited:
                visited.add(succ_key)
                queue.append(successor)
                
    return False
    """

# hàm heuristic sử dụng bfs duyệt ngược từ đích về các thùng 
# def heuristic(state, distances):
   
#     matrix = build_cost_matrix(state, distances)
    
#     total_cost = 0
#     used_cols = set()
    
#     for i in range(len(matrix)):
#         min_val = float('inf')
#         best_j = -1
#         for j in range(len(matrix[i])):
#             if j not in used_cols and matrix[i][j] < min_val:
#                 min_val = matrix[i][j]
#                 best_j = j
                
#         if best_j != -1:
#             total_cost += min_val
#             used_cols.add(best_j)
            
#     return total_cost