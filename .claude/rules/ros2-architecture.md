---
trigger: always_on
globs: "/package.xml", "/CMakeLists.txt", "**/setup.py"
---

# QUY TẮC KIẾN TRÚC ROS 2 (ANTIGRAVITY STANDARD) 

## 1. Phân Tách Loại Hình Build (Build Type Segregation)
Mỗi gói tin (package) phải tuân thủ nghiêm ngặt một loại hình build duy nhất. Tuyệt đối không trộn lẫn cấu hình C++ và Python trong cùng một gói tin logic để đảm bảo tính minh bạch của ament.

### C++ Packages:
*   Phải khai báo `<build_type>ament_cmake</build_type>` trong `package.xml`.
*   Phải sử dụng `CMakeLists.txt` làm tệp cấu hình chính.
*   Mã nguồn nằm trong thư mục `src/`, header public nằm trong `include/<package_name>/`.

### Python Packages:
*   Phải khai báo `<build_type>ament_python</build_type>` trong `package.xml`.
*   Phải sử dụng `setup.py` và `setup.cfg`.
*   Cấu trúc thư mục phải chứa `resource/<package_name>` để colcon nhận diện.

### Interface Packages (Interfaces):
*   Các định nghĩa Message (.msg), Service (.srv), và Action (.action) phải được tách riêng vào các gói tin có hậu tố `_interfaces` (ví dụ: `my_robot_interfaces`).
*   Không được chứa mã thực thi (executable code) trong các gói interface.

## 2. Tiêu Chuẩn Thành Phần Hóa (Componentization Standard)
Để tối ưu hóa hiệu năng và khả năng tái sử dụng, hệ thống áp dụng kiến trúc Composition:

### Đối với C++:
*   Hạn chế tối đa việc viết hàm `main()` thủ công để khởi tạo Node.
*   Tất cả logic nghiệp vụ phải được gói gọn trong một lớp kế thừa từ `rclcpp::Node`.
*   Sử dụng macro `RCLCPP_COMPONENTS_REGISTER_NODE` để đăng ký component vào hệ thống class loader.
*   Biên dịch dưới dạng thư viện chia sẻ (Shared Library) thay vì tệp thực thi độc lập.

### Đối với Python:
*   Sử dụng cơ chế `entry_points` trong `setup.py` để định nghĩa các script thực thi (`console_scripts`), thay vì gọi trực tiếp tệp `.py`.

## 3. Quản Lý Phụ Thuộc (Dependency Management)
Tệp `package.xml` là nguồn sự thật duy nhất (Single Source of Truth) cho các phụ thuộc:
*   Phải sử dụng `<package format="3">`.
*   Phân loại rõ ràng:
    *   `<buildtool_depend>`: Cho các công cụ build (`ament_cmake`, `ament_cmake_python`).
    *   `<depend>`: Cho các thư viện dùng chung cả lúc build và runtime (`rclcpp`, `std_msgs`).
    *   `<exec_depend>`: Cho các thư viện chỉ dùng lúc chạy (Python modules, Launch files).
    *   `<test_depend>`: **BẮT BUỘC** phải có `ament_lint_auto` và `ament_lint_common`.