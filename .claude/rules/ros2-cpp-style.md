---
trigger: glob
globs: "/*.cpp", "/.hpp", "**/.h"
---

# CHUẨN MỰC MÃ HÓA C++ CHO ROS 2

## 1. Quy Ước Đặt Tên (Naming Conventions)
Khác với chuẩn Google thuần túy, ROS 2 áp dụng các ngoại lệ sau để đồng bộ với hệ sinh thái:

| Thành phần | Quy tắc | Ví dụ |
| :--- | :--- | :--- |
| Tên Gói (Package) | snake_case | `my_robot_driver` |
| Tên Tệp (File) | snake_case | `sensor_handler.cpp` |
| Lớp (Class/Type) | PascalCase | `LaserScanProcessor` |
| Phương Thức (Method) | snake_case | `publish_point_cloud()` |
| Biến (Variable) | snake_case | `current_velocity` |
| Biến Thành Viên | snake_case + _ | `publisher_` |
| Biến Toàn Cục | g_ + snake_case | `g_shutdown_signal` |

## 2. Thiết Kế Hướng Đối Tượng (OOP & Composition)
*   **Component-Based**: Mọi Node C++ phải được thiết kế dưới dạng `Component` (Shared Library) kế thừa từ `rclcpp::Node` hoặc `rclcpp_lifecycle::LifecycleNode`.
*   **Encapsulation**: 
    *   Thành viên dữ liệu (Data Members) phải là `private` và có hậu tố `_` (ví dụ: `timer_`).
    *   Chỉ expose các phương thức cần thiết qua `public`.
*   **Macro Registration**: Phải sử dụng `RCLCPP_COMPONENTS_REGISTER_NODE` để đăng ký Class vào Executor, tránh viết hàm `main()` cứng trong file component.

## 3. Quản Lý Bộ Nhớ & Con Trỏ
*   Tuyệt đối không sử dụng `new` và `delete` thủ công (Raw Pointers) trừ khi tương tác với thư viện C legacy.
*   Sử dụng `std::shared_ptr` và `std::unique_ptr` cho việc quản lý tài nguyên.
*   Khi tạo Node hoặc Message, ưu tiên `rclcpp::make_shared<T>()` để đảm bảo tương thích với bộ cấp phát bộ nhớ của middleware.

## 3. Định Dạng & Cấu Trúc (Formatting)
*   Sử dụng 2 dấu cách (spaces) cho thụt đầu dòng. Không dùng Tab.
*   Độ rộng dòng tối đa: 100 ký tự.
*   Dấu ngoặc nhọn `{}` là bắt buộc cho mọi khối lệnh điều kiện (if, loop), ngay cả khi chỉ có một dòng lệnh.
*   Sử dụng `#pragma once` trong header file thay vì `ifndef/define` guard cũ.