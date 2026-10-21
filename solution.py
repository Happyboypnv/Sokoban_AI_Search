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
# Quét tìm ô deadlock 
def get_dead_squares(state):
    def is_wall(x, y):
        # True nếu ô là tường hoặc nắm ngoài bản đồ 
        return (x, y) in state.obstacles or x < 0 or x >= state.width or y < 0 or y >= state.height
    
    dead_squares = set() 
    corners = set()
    # Tìm Corner Deadlock 
    for x in range(state.width):
        for y in range(state.height):
            pos = (x, y) 
            if pos in state.obstacles or pos in state.storage:
                continue
            up = (x, y-1) in state.obstacles or (y-1) < 0
            down = (x, y+1) in state.obstacles or (y+1) >= state.height 
            left = (x-1, y) in state.obstacles or (x-1) < 0 
            right = (x+1, y) in state.obstacles or (x+1) >= state.width
            if (up or down) and (left or right):
                dead_squares.add(pos) 
                corners.add(pos)

    # Tìm Edge Deadlock nối giữa 2 Corner Deadlock
    corner_list = list(corners)
    for i in range(len(corner_list)):
        for j in range(i+1, len(corner_list)):
            c1, c2 = corner_list[i], corner_list[j]
            if c1[0] == c2[0]: 
                x = c1[0] 
                y_start, y_end = min(c1[1], c2[1]), max(c1[1], c2[1])
                is_dead_edge = True 
                has_wall_left, has_wall_right = True, True 
                for y in range(y_start, y_end + 1):
                    pos=(x,y)
                    # Giữa cạnh có đích hoặc bị ngắt quãng
                    if pos in state.obstacles or pos in state.storage:
                        is_dead_edge = False 
                        break 
                    if not is_wall(x - 1, y): 
                        has_wall_left = False
                    if not is_wall(x + 1, y): 
                        has_wall_right = False
                if is_dead_edge and (has_wall_left or has_wall_right):
                    for y in range(y_start, y_end + 1):
                        dead_squares.add((x,y))

            elif c1[1] == c2[1]:
                y=c1[1] 
                x_start, x_end = min(c1[0], c2[0]), max(c1[0], c2[0])
                
                is_dead_edge = True
                has_wall_up, has_wall_down = True, True
                
                for x in range(x_start, x_end + 1):
                    pos = (x, y)
                    if pos in state.obstacles or pos in state.storage:
                        is_dead_edge = False
                        break
                    if not is_wall(x, y - 1): has_wall_up = False
                    if not is_wall(x, y + 1): has_wall_down = False
                        
                if is_dead_edge and (has_wall_up or has_wall_down):
                    for x in range(x_start, x_end + 1):
                        dead_squares.add((x, y))

    return dead_squares 

# Deadlokk động 
def is_freeze_deadlock(state):
    def is_blocked(x, y):
        # Ô bị chặn nếu là tường , thùng , biên 
        return (x, y) in state.obstacles or (x, y) in state.boxes or x < 0 or x >= state.width or y < 0 or y >= state.height

    for box in state.boxes:
        if box in state.storage:
            continue 
        x, y = box 
        for dx, dy in [(-1, -1), (1, -1), (-1, 1), (1, 1)]:
            # 3 ô còn lại tạo thành khối 2*2 đều bị chặn
            if is_blocked(x + dx, y) and is_blocked(x, y + dy) and is_blocked(x + dx, y + dy):
                return True  

    return False

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
    dead_squares = get_dead_squares(initial_state)
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
                is_deadlocked = False
                # Deadlock tĩnh
                for box in successor.boxes:
                    if box in dead_squares:
                        is_deadlocked = True 
                        break

                # Deadlock động 
                if not is_deadlocked and is_freeze_deadlock(successor):
                    is_deadlocked = True 

                if is_deadlocked:
                    continue
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