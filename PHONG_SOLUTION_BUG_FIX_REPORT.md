# Báo cáo sửa lỗi và thay đổi thuật toán

> Báo cáo này mô tả phiên bản solver hiện tại trong
> [`solution.py`](./solution.py). Tên file báo cáo được giữ theo bản
> `Phong_solution` ban đầu, nhưng nội dung cần được đọc cùng mã nguồn solver
> hiện tại.

## 1. Phạm vi thay đổi

File được sửa là [`solution.py`](./solution.py). Mục tiêu là cải
thiện khả năng xử lý các bài hard, đặc biệt là bài `open10_6r_8b`, đồng thời
vẫn giữ đúng giao diện của framework Sokoban:

- trả về `SokobanState` hợp lệ hoặc `False`;
- mỗi action có dạng `"robot_id direction"`;
- `gval` tăng đúng một cho mỗi bước nguyên thủy;
- chuỗi `parent` có thể được kiểm tra bởi `validate_solution()`;
- không thay đổi trạng thái ban đầu.

## 2. Các bug và điểm nghẽn trước khi sửa

### 2.1. Mở rộng từng bước đi bộ làm không gian trạng thái quá lớn

Phiên bản cũ mở rộng mỗi trạng thái bằng toàn bộ cặp:

```text
robot × 4 hướng
```

Với 6 robot, một trạng thái có thể sinh tối đa 24 successor. Phần lớn các
successor chỉ thay đổi vị trí robot, trong khi cấu hình thùng không thay đổi.
Những trạng thái đi bộ này làm cây tìm kiếm phình to nhanh.

Hậu quả là bài `open10_6r_8b` thường chạm giới hạn node hoặc hết thời gian
trước khi tìm được lời giải, dù bài có lời giải.

### 2.2. Heuristic không phản ánh đầy đủ chi phí đi bộ

Heuristic Hungarian chỉ ước lượng số lần đẩy cần thiết để đưa thùng tới các ô
đích. Trong khi đó chi phí thật của bài toán là tổng số bước, bao gồm cả bước
robot đi bộ.

Vì vậy nhiều trạng thái có cùng cấu hình thùng nhưng robot ở các vị trí khác
nhau nhận heuristic giống nhau. Weighted A* phải mở rộng nhiều trạng thái
robot không cần thiết.

### 2.3. Các vòng tìm kiếm lặp lại công việc

Mỗi trọng số heuristic gọi lại `search()` với heap và bảng `bestg` mới. Cách
này làm lặp lại một phần công việc giữa các vòng trọng số. Đây là hạn chế hiệu
năng còn lại của thiết kế anytime hiện tại, nhưng không làm sai tính đúng đắn.

### 2.4. Lỗi sinh action không hợp lệ trong bản macro đầu tiên

Trong phiên bản macro-push đầu tiên, solver có thể sinh một lần đẩy đưa thùng
vào ô đang có robot khác. Mô hình macro không kiểm tra điều kiện này trước khi
thêm successor.

Khi phát lại action bằng `state.successors()`, action đó không tồn tại và
`validate_solution()` từ chối đường đi.

Bug được sửa bằng điều kiện:

```python
if destination in occupied_robots:
    continue
```

Điều kiện này nằm trong [`_macro_pushes()`](./solution.py) và đảm bảo
ô đích của thùng không bị robot khác chiếm.

### 2.5. BFS vùng robot bị tính lại nhiều lần

Sau khi chuyển sang macro-push, mỗi node cần tìm vùng đi bộ của từng robot.
Nếu chạy BFS lại cho cùng một cấu hình, thời gian bị lãng phí, đặc biệt trong
các vòng Weighted A* khác nhau.

Đã thêm `reachabilityCache`. Kết quả BFS được cache theo:

```text
(vị trí robot đang xét, vị trí các robot còn lại, box_mask)
```

Cache lưu khoảng cách, parent và hướng di chuyển dưới dạng tuple bất biến.
Khi cùng cấu hình xuất hiện lại, solver dùng lại kết quả BFS và chỉ khôi phục
đường đi cần thiết.

## 3. Thuật toán chính sau khi sửa

Thuật toán chính hiện tại là **Weighted A* trên không gian macro-push**.
Nó vẫn sử dụng heuristic khoảng cách đẩy và Hungarian, nhưng không còn coi
mỗi bước đi bộ là một node tìm kiếm độc lập.

Một node tìm kiếm đại diện cho:

```text
(vị trí các robot, cấu hình các thùng)
```

Mỗi cạnh giữa hai node là:

```text
đi bộ tới vị trí đứng sau một thùng + đẩy thùng một lần
```

Chi phí cạnh là:

```text
chi phí macro = số bước đi bộ + 1 bước đẩy
```

Hàm đánh giá vẫn có dạng:

```text
f(n) = g(n) + w * h(n)
```

Trong đó:

- `g(n)` là tổng số bước nguyên thủy của các macro trước đó;
- `h(n)` là heuristic Hungarian trên cấu hình thùng;
- `w` là trọng số của Weighted A*.

## 4. Chi tiết các thành phần mới

### 4.1. BFS vùng đi bộ

Hàm `_walk_reachability()` thực hiện BFS từ một robot trong khi vị trí các
thùng được giữ cố định.

BFS không đi qua:

- biên bàn;
- vật cản;
- ô có thùng;
- vị trí của robot khác.

Hàm lưu ba loại thông tin:

```text
dist[cell]   : khoảng cách ngắn nhất tới cell
parent[cell] : ô trước đó trên đường BFS
move[cell]   : hướng dùng để tới cell
```

Nhờ đó, solver vừa biết robot có thể tới phía sau thùng hay không, vừa khôi
phục được chính xác chuỗi bước đi bộ.

### 4.2. Khôi phục đường đi bộ

Hàm `_walk_path()` lần ngược mảng `parent` từ ô đứng sau thùng về vị trí robot
ban đầu, sau đó đảo ngược danh sách hướng.

Ví dụ một macro có thể được lưu thành:

```text
["0 right", "0 right", "0 down"]
```

Trong code, action được lưu dưới dạng số nguyên:

```python
robot_id * 4 + direction
```

Cách mã hóa này tương thích với phần dựng lại action hiện có.

### 4.3. Sinh macro-push

Hàm `_macro_pushes()` duyệt:

1. từng robot;
2. từng thùng;
3. từng hướng đẩy.

Với hướng `d`, robot phải đi tới ô phía sau thùng:

```python
front = self.nb[(d + 2) % 4][box]
```

Thùng sẽ được đẩy tới:

```python
destination = self.nb[d][box]
```

Một macro chỉ hợp lệ nếu:

- robot tới được `front`;
- `destination` nằm trong bàn;
- `destination` không có thùng;
- `destination` không phải ô dead;
- `destination` không có robot khác;
- cấu hình mới không bị frozen;
- heuristic mới không phải vô hạn.

Sau khi đẩy, robot đứng tại vị trí cũ của thùng, đúng theo luật Sokoban:

```python
newrobots[robot_id] = box
```

### 4.4. Tìm kiếm trên macro

`search()` hiện chỉ thêm vào heap các trạng thái sau một lần đẩy. Điều này
giảm mạnh số node so với phiên bản tìm kiếm từng bước.

Mỗi node lưu:

- `gl`: chi phí tích lũy;
- `par`: node cha;
- `segments`: chuỗi action nguyên thủy của macro;
- `rob`: vị trí robot;
- `msk`: bitmask thùng;
- `bestg`: chi phí tốt nhất đã biết cho mỗi trạng thái.

### 4.5. Heuristic có xét chi phí tiếp cận của robot

Heuristic Hungarian ban đầu chỉ ước lượng số lần đẩy thùng tới các ô đích.
Đã bổ sung `_robot_walk_lower_bound()` để ước lượng khoảng cách Manhattan nhỏ
nhất từ một robot tới vị trí đứng sau một thùng có thể đẩy.

Giá trị này được cache theo:

```text
(robots, box_mask)
```

Chi phí robot không được cộng trực tiếp vào `g + w*h`, vì thử nghiệm cho thấy
cách cộng này làm thay đổi mạnh thứ tự tìm kiếm và khiến bài có nhiều robot
chậm hơn. Thay vào đó, nó được dùng làm tie-breaker trong heap:

```text
(g + w * h_box, robot_walk_lower_bound, node_id)
```

Như vậy solver vẫn ưu tiên heuristic Hungarian trước, nhưng nếu hai node có
cùng độ ưu tiên chính thì chọn node có robot gần vị trí đẩy hơn. Đây là heuristic
phụ rẻ, bỏ qua tường và các cản trở động, nên chỉ dùng để định thứ tự mở rộng,
không dùng để khẳng định nghiệm tối ưu.

Hàm `_reconstruct_actions()` nối các đoạn macro từ node đích về gốc để tạo ra
toàn bộ danh sách action nguyên thủy.

## 5. Vì sao vẫn giữ bước phát lại action?

A* làm việc trên macro để giảm kích thước không gian tìm kiếm, nhưng framework
vẫn yêu cầu chuỗi trạng thái chi tiết theo từng bước robot. Vì vậy sau khi tìm
được danh sách action, `solve()` vẫn phát lại từng action trên:

```python
state.successors()
```

Mỗi successor được framework tạo ra với:

- `parent` đúng;
- `action` đúng;
- `gval` tăng một;
- cấu hình robot/thùng hợp lệ.

Do đó macro chỉ là cấu trúc nội bộ của solver; kết quả trả về vẫn là đường đi
nguyên thủy đầy đủ và tương thích với validator.

## 6. Kết quả kiểm thử

Sau khi sửa, các bài sau đã được chạy với `timebound=20` và kiểm tra bằng
`validate_solution()`:

| Bài kiểm thử | Kết quả |
|---|---|
| `open10_4r_6b` | PASS |
| `open10_5r_7b` | PASS |
| `open10_6r_8b` | PASS |
| `open12_4r_8b` | PASS |
| `scat12_2r_9b` | PASS |
| `scat12_1r_10b` | PASS |
| `maze10_1r_6b` | PASS |
| `maze14_1r_9b` | PASS |

Bài khó mục tiêu `open10_6r_8b` trước đây không pass, nhưng sau khi chuyển
sang macro-push và bổ sung heuristic/tie-breaker cho robot đã pass trong các
lần chạy kiểm thử.

Các kiểm tra kỹ thuật cũng thành công:

```bash
python3 -m py_compile solution.py
git diff --check
```

## 7. Giới hạn còn lại

Phiên bản mới cải thiện đáng kể việc mở rộng trạng thái, nhưng vẫn còn các
giới hạn:

- mỗi robot vẫn có thể cần BFS ở các cấu hình mới, dù kết quả trùng được cache;
- các vòng trọng số vẫn chạy search độc lập;
- heuristic robot dùng Manhattan nên chưa tính chính xác tường và vật cản;
- deadlock detector chưa bao phủ mọi mẫu deadlock Sokoban;
- với các bài rất lớn, số cấu hình thùng vẫn có thể tăng nhanh.

Nếu cần tối ưu thêm, các hướng tiếp theo là tái sử dụng OPEN giữa các trọng số,
gom trạng thái robot theo vùng reachable, hoặc thiết kế heuristic kết hợp chi
phí tiếp cận vị trí đẩy chính xác hơn.

## 8. Bản đồ đầy đủ các hàm trong solver hiện tại

Phần này bổ sung các hàm nền tảng đã có trong mã nhưng chưa được mô tả đầy đủ
ở bản báo cáo trước.

### 8.1. Các hằng số và hàm tiện ích

Các hằng số chính:

| Thành phần | Vai trò |
|---|---|
| `_DIR_NAMES` | Đổi chỉ số hướng thành `up`, `right`, `down`, `left` |
| `_DELTAS` | Độ dời `(dx, dy)` của bốn hướng |
| `_BIG` | Giá trị đại diện cho vô hạn hoặc không thể đạt |
| `_MAX_NODES` | Giới hạn số node lưu trong một lần search |
| `_WEIGHTS` | Các trọng số Weighted A* dùng trong giai đoạn anytime |
| `_TIME_FRACTION` | Phần ngân sách dành cho tìm kiếm, hiện là `0.85` |

`in_code(x, y, W, H)` chuyển tọa độ hai chiều thành chỉ số một chiều
`y * W + x` và báo lỗi nếu tọa độ ngoài bàn. `check_Pos()` là phiên bản kiểm
tra biên trả về `True/False`, được dùng trong tiền xử lý lân cận.

### 8.2. `_hungarian(arr, n, m)`

Hàm này cài đặt thuật toán Hungarian cho bài toán assignment tối thiểu.
Mỗi hàng là một thùng, mỗi cột là một ô đích và giá trị ô là số lần đẩy thấp
nhất theo bảng khoảng cách đã tiền xử lý.

Các mảng `lx`, `ly`, `matching`, `trace`, `visited` và `minv` lần lượt lưu
nhãn hai phía, matching hiện tại, đường truy vết và slack trong quá trình mở
rộng matching. Kết quả trả về là tổng chi phí assignment nhỏ nhất, dùng làm
`h_box`.

Độ phức tạp xấp xỉ `O(B^3)` với `B` là số thùng/đích. Kết quả cuối cùng
được cache gián tiếp trong `HeuristicFunction()` theo `box_mask`.

### 8.3. `Solver.__init__(state)`

Hàm khởi tạo xây dựng toàn bộ mô hình tĩnh của một map:

1. Lưu kích thước bàn và tạo chỉ số ô.
2. Tạo `nb[direction][cell]`, bảng lân cận không đi qua vật cản.
3. Tạo `goals`, `is_goals` và `goal_mask`.
4. Chạy BFS ngược từ từng goal để tính khoảng cách đẩy.
5. Tạo `cost_rows`, `mindist` và `dead`.
6. Chuyển robot ban đầu thành `r0` và thùng ban đầu thành `m0`.
7. Khởi tạo các cache:
   - `heuristicCache`: cache Hungarian theo `box_mask`;
   - `reachabilityCache`: cache BFS robot;
   - `robotHeuristicCache`: cache khoảng cách robot phụ.
8. Tạo `sblk` và `nbm`, dữ liệu dùng bởi bộ phát hiện nhóm thùng bị khóa.

Khoảng cách đẩy được tính bằng BFS ngược: muốn đẩy thùng từ `p` tới `q`,
robot phải đứng ở ô phía sau `p`. Nếu không tồn tại chuỗi đẩy hình học tới
goal nào, ô đó được đánh dấu `dead`.

### 8.4. `HeuristicFunction(mask)`

Hàm tách các bit đang bật trong `mask` thành danh sách vị trí thùng. Với một
thùng, nó dùng trực tiếp `mindist`. Với nhiều thùng, nó tạo ma trận
`cost_rows` và gọi Hungarian.

Nếu assignment không tồn tại, hoặc chi phí đạt `_BIG`, hàm trả về `_BIG`.
Kết quả được lưu trong `heuristicCache`, vì các node có thể khác vị trí robot
nhưng có cùng cấu hình thùng.

Đây là heuristic chính của search. Nó bỏ qua chi phí đi bộ robot, xung đột
động và tương tác giữa các thùng, nên là một lower bound định hướng chứ không
phải chi phí thực tế đầy đủ.

### 8.5. `_frozen(c, mask)` và `inititally_hopeless()`

`_frozen()` tìm thành phần liên thông của thùng chứa ô `c`. Sau đó nó loại
dần các ô chưa bị khóa theo cả hai trục. Nếu phần còn lại chứa thùng ngoài
goal, nhóm bị xem là frozen và nhánh bị loại.

`inititally_hopeless()` là bộ lọc trước search. Nó loại:

- nhiều thùng hơn số goal;
- thùng nằm trên ô `dead`;
- cấu hình ban đầu frozen;
- assignment ban đầu không tồn tại.

Tên hàm `inititally_hopeless` giữ nguyên theo mã nguồn hiện tại, dù cách viết
chuẩn tiếng Anh thường là `initially_hopeless`.

### 8.6. `_walk_reachability(robot, robots, mask)`

Đây là BFS trên không gian đi bộ của một robot với box cố định. Khóa cache là:

```text
(robot_position, sorted_other_robot_positions, box_mask)
```

BFS không đi vào ô có thùng, robot khác, vật cản hoặc ngoài bàn. Kết quả gồm:

```text
dist[cell], parent[cell], move[cell]
```

Các mảng được đổi sang tuple trước khi lưu cache để không bị sửa ngoài ý muốn.
BFS này vừa kiểm tra một vị trí đẩy có reachable hay không, vừa cung cấp dữ
liệu để dựng chuỗi đi bộ trong macro.

### 8.7. `_walk_path(target, parent, move)`

Hàm lần ngược `parent` từ vị trí đứng sau thùng về vị trí robot ban đầu,
lấy các hướng trong `move`, rồi đảo danh sách. Đây là phần chuyển kết quả BFS
thành action nguyên thủy.

### 8.8. `_robot_walk_lower_bound(robots, mask)`

Hàm duyệt robot, thùng và bốn hướng đẩy để tìm khoảng cách Manhattan nhỏ nhất
từ một robot tới vị trí `front` cần đứng để thực hiện một push hợp lệ.

Nó bỏ qua tường và đường đi bị chặn, nên chỉ phù hợp làm tie-breaker. Kết quả
được cache theo `(robots, box_mask)`. Trong heap, nó đứng sau khóa chính:

```text
(g + w * h_box, robot_walk_lower_bound, node_id)
```

Do không cộng vào `h_box`, hàm này không làm thay đổi điều kiện cắt nhánh
chính của Weighted A*. Đây là lý do phiên bản hiện tại ổn định hơn thử nghiệm
`h_box + h_robot`.

### 8.9. `_macro_pushes(robots, mask)`

Hàm sinh các cạnh macro. Với mỗi robot, thùng và hướng:

1. Xác định `front`, ô robot cần đứng.
2. Xác định `destination`, ô thùng sẽ tới.
3. Dùng BFS cache để kiểm tra `front` reachable.
4. Kiểm tra destination không có thùng, robot khác, dead cell hoặc vật cản.
5. Tạo `newmask` sau push.
6. Áp dụng `_frozen()` và heuristic vô hạn cho cấu hình chưa hoàn thành.
7. Khôi phục walk path.
8. Thêm action push.
9. Đặt robot vào vị trí cũ của thùng.

Mỗi phần tử yield có dạng:

```text
(new_robots, new_mask, macro_cost, primitive_actions)
```

`macro_cost` là số bước đi bộ cộng một bước đẩy. Vì vậy search macro vẫn tối
ưu theo đúng chi phí framework, không chỉ theo số lần đẩy.

### 8.10. `_reconstruct_actions(node, parents, segments)`

Hàm lần theo `parents` từ node đích về gốc, gom từng `segments` và đảo thứ tự.
Kết quả là một danh sách action nguyên thủy liên tục, dù search nội bộ chỉ
lưu các macro.

### 8.11. `search(w, best, deadline)`

Đây là Weighted A* trên macro graph. Heap ưu tiên:

```text
(g + w * h_box, robot_h, node_id)
```

`bestg` loại trạng thái đã biết với chi phí không tốt hơn. Mỗi node mở rộng
chỉ sinh các trạng thái sau một lần push. Search dừng khi:

- heap cạn;
- vượt deadline;
- vượt `_MAX_NODES`;
- tìm được goal tốt hơn `best` trong vòng trọng số hiện tại.

Tên mô tả trong docstring là ARA*, nhưng code hiện tại là anytime Weighted A*
vì mỗi vòng trọng số tạo heap và bảng node mới, chưa tái sử dụng đầy đủ
`OPEN`, `CLOSED`, `INCONS` như ARA* chuẩn.

### 8.12. `search_legacy(w, best, deadline)`

Đây là phiên bản tìm kiếm cũ theo từng bước robot, được giữ lại như một
fallback nhanh cho các map nhỏ hoặc map mở. Nó sử dụng cùng:

- biểu diễn robot và box mask;
- heuristic Hungarian;
- deadlock pruning;
- `bestg` và heap Weighted A*.

Khác biệt là mỗi successor chỉ là một bước robot hoặc một lần đẩy, không gom
thành macro. Vì vậy nó có thể rất nhanh khi lời giải gần, nhưng dễ bùng nổ
trạng thái trên map nhiều robot.

`search_legacy()` được giới hạn bằng `_LEGACY_TIME_FRACTION`, hiện là `0.35`
ngân sách đầu vào. Giới hạn này ngăn search cũ chiếm toàn bộ thời gian khiến
macro-push không còn cơ hội chạy.

### 8.13. `solve(initial_state, timebound)`

`solve()` là entry point của framework:

1. Từ chối ngân sách không dương.
2. Trả ngay input nếu đã là goal.
3. Tạo `Solver` và chạy kiểm tra hopeless ban đầu.
4. Chạy `search_legacy()` trước với các trọng số lớn.
5. Nếu search cũ tìm được nghiệm, dừng và dùng ngay nghiệm đó.
6. Nếu search cũ không tìm được, chạy `search()` macro-push với deadline chung.
7. Phát lại từng action trên `state.successors()`.
8. Trả state cuối nếu `sokoban_goal_state()` xác nhận thành công; nếu không,
   trả `False`.

Việc phát lại action là bước bắt buộc để đảm bảo `parent`, `action` và `gval`
đúng theo framework. Macro không được trả trực tiếp cho caller.

Kiến trúc hai tầng này là một trade-off có chủ ý: search cũ tận dụng các bài
dễ mà không phải trả chi phí BFS macro, còn macro-push là fallback cho bài có
không gian bước đi bộ quá lớn. Nếu search cũ thất bại, phần thời gian đã dùng
không bị lặp lại trong macro vì cả hai cùng dùng một deadline tuyệt đối.

## 9. Trạng thái hiện tại và đánh giá

Phiên bản hiện tại đã sửa phần lỗi đúng đắn quan trọng và cải thiện khả năng
pass các bài nhiều robot, nhưng chưa chứng minh tối ưu toàn cục. Weighted A*
và deadline khiến kết quả phụ thuộc vào trọng số, cache, thứ tự tie-breaker và
thời gian chạy.

Khi báo cáo kết quả, nên ghi riêng:

- số bài pass;
- số bước của từng nghiệm;
- thời gian từng bài;
- bài timeout/fail;
- không gọi nghiệm hiện tại là nghiệm tối ưu nếu chưa chạy xong `w = 1`.

Các hướng cải thiện tiếp theo là giảm số vòng search độc lập, gom robot theo
vùng reachable, cache macro hợp lệ và xây dựng heuristic robot chính xác hơn
nhưng vẫn giữ được chi phí tính toán chấp nhận được.
