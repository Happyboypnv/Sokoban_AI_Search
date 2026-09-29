# Mô tả thiết kế solver Sokoban

Tài liệu này dùng để giải thích bài nộp, không có điểm riêng trong thang chấm.
Không bắt buộc một thuật toán, heuristic hay cấu trúc dữ liệu cụ thể.

## 1. Thiết kế
Mô tả phương pháp giải, cách tổ chức chương trình và lý do lựa chọn.

Nếu dùng ARA* (Anytime Repairing A*), cần nêu rõ:

- Vòng đầu dùng `f = g + epsilon*h` với `epsilon > 1` để lấy incumbent nhanh.
- `OPEN`, `CLOSED`, `INCONS` được tái sử dụng khi giảm dần `epsilon`; đây là
  điểm khác với việc chạy Weighted A* độc lập nhiều lần.
- Heuristic phải là cận dưới nếu muốn diễn giải bound chất lượng. Trong
  Sokoban, matching Manhattan giữa thùng và đích là cận dưới vì bỏ qua tường,
  robot và xung đột giữa các thùng.

## 2. Chất lượng lời giải
Giải thích phương pháp tìm hoặc cải thiện lời giải. Nếu tuyên bố tối ưu,
trình bày cơ sở cho tuyên bố đó; nếu không, nêu rõ giới hạn.

ARA* có thể trả nghiệm đầu tiên trước deadline và tiếp tục cải thiện nghiệm.
Chỉ khi vòng `epsilon = 1` hoàn tất mới nên tuyên bố tối ưu; nếu hết thời gian
trước đó, hãy báo cáo số bước của incumbent và epsilon cuối cùng.

## 3. Thực nghiệm
Ghi các map đã thử, có giải được hay không và số bước của lời giải.
Có thể ghi thêm thời gian để phân tích, dù thời gian không trực tiếp tính điểm.

## 4. Hạn chế và nguồn tham khảo
Nêu phần chưa hoàn thiện, tài liệu, mã nguồn và công cụ AI đã sử dụng.
