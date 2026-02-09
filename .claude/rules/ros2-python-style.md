---
trigger: glob
globs: "**/*.py"
---

# CHUẨN MỰC MÃ HÓA PYTHON CHO ROS 2

## 1. Tuân Thủ PEP 8 & Linter
*   Mã nguồn phải vượt qua kiểm tra của `flake8` với cấu hình được định nghĩa trong `setup.cfg`.
*   Docstrings là bắt buộc cho module, class, và function, tuân thủ chuẩn PEP 257.

## 2. Cấu Trúc Node
*   Mọi Node phải là một lớp kế thừa từ `rclpy.node.Node`.
*   Hàm `__init__` phải gọi `super().__init__('node_name')` ở dòng đầu tiên.
*   Không được thực hiện các tác vụ blocking (chặn) trong callback. Sử dụng `async/await` hoặc đẩy tác vụ nặng sang luồng khác nếu cần.

## 3. Điểm Nhập (Entry Point)
*   Mỗi module phải có khối `if __name__ == '__main__':` gọi đến hàm main().
*   Hàm `main()` phải thực hiện quy trình chuẩn:
    1.  `rclpy.init(args=args)`
    2.  Khởi tạo Node.
    3.  `rclpy.spin(node)`
    4.  Xử lý ngoại lệ `KeyboardInterrupt` để dọn dẹp (`destroy_node`).
    5.  `rclpy.shutdown()`.