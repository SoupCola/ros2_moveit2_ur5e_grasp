# QUY TẮC MÔ PHỎNG ROS 2 VÀ GAZEBO SIM (ANTIGRAVITY STANDARD)

## 1. Ma Trận Tương Thích (Compatibility Matrix)
Việc tuân thủ ma trận tương thích giữa ROS 2 và Gazebo Sim là yêu cầu **BẮT BUỘC** để đảm bảo tính ổn định của hệ thống.

| Bản Phân Phối ROS 2 | Phiên bản Gazebo Sim | Trạng thái | Gói Bridge | Ghi chú |
| :--- | :--- | :--- | :--- | :--- |
| **Humble Hawksbill** | **Fortress** (LTS) | Recommended | `ros-humble-ros-gz` | Ổn định nhất cho Production. Namespace: `ignition`. CLI: `ign`. |
| **Jazzy Jalisco** | **Harmonic** (LTS) | Standard | `ros-jazzy-ros-gz` | Chuẩn mới 2024-2029. Namespace: `gz`. CLI: `gz`. |
| **Rolling Ridley** | **Ionic** | Experimental | `ros-rolling-ros-gz` | Chỉ dùng cho nghiên cứu tính năng mới (Bleeding Edge). |

> [!IMPORTANT]
> - **Dự án Bảo trì/Kế thừa:** Giữ nguyên Humble + Fortress.
> - **Dự án Mới (Greenfield):** Bắt buộc sử dụng Jazzy + Harmonic. Gazebo Classic (Gazebo 11) đã EOL, **KHÔNG** sử dụng cho dự án mới.

## 2. Quy Tắc "Nguồn Chân Lý Duy Nhất" (Single Source of Truth)
*   **URDF (hoặc Xacro) là nguồn duy nhất** mô tả robot cho cả ROS 2 và Gazebo.
*   **KHÔNG** duy trì file SDF riêng biệt thủ công.
*   Sử dụng node `create` của gói `ros_gz_sim` để convert URDF -> SDF tại runtime.
*   File URDF phải chứa thẻ `<gazebo>` với các plugin hệ thống của Gazebo Sim (khác với plugin Gazebo Classic).

## 3. Quy Tắc Quản Lý Tài Nguyên (Resource Path Management)
Gazebo Sim **KHÔNG** tự động nhận diện đường dẫn mesh trong package ROS 2 nếu không cấu hình đúng.
*   **Bắt buộc thiết lập biến môi trường:** `GZ_SIM_RESOURCE_PATH`.
*   Launch file phải trỏ biến này tới thư mục cha của package chứa mesh.
*   Trong `package.xml`, sử dụng thẻ `<export>` để khai báo đường dẫn model (tùy chọn nhưng khuyến nghị).

## 4. Quy Tắc "Cầu Nối Định Danh" (Explicit Bridging)
Gazebo Sim không tự động publish topic sang ROS 2.
*   **Nguyên tắc:** Mọi dữ liệu (Sensor, Cmd_vel, Odom, TF) phải đi qua `ros_gz_bridge`.
*   **Cấu hình:** Sử dụng file **YAML** để định nghĩa bridge thay vì tham số dòng lệnh rời rạc.
*   **Topic Clock:** Bắt buộc bridge topic `/clock` (GZ -> ROS) nếu sử dụng `use_sim_time`.
