---
description: ROS 2 LOCAL CI/CD PIPELINE
---
# ROS 2 LOCAL CI/CD PIPELINE
// turbo

Quy trình kiểm thử và tích hợp liên tục cục bộ. Thực hiện trước mỗi lần commit.

## Bước 1: Giải Quyết Phụ Thuộc Hệ Thống (Rosdep)
Đảm bảo tất cả thư viện hệ thống (như `libboost`, `opencv`, v.v.) đã được cài đặt.
> `rosdep install --from-paths src --ignore-src -r -y`

## Bước 2: Kiểm Tra Tĩnh (Static Analysis / Linting)
Chạy linter cho các gói tin mục tiêu. Không biên dịch nếu bước này thất bại.
> `colcon test --packages-select <target_package> --ctest-args -R "flake8|cpplint|clang_format|pep257"`
> `colcon test-result --verbose`

## Bước 3: Biên Dịch (Build)
Biên dịch mã nguồn.
> `colcon build --packages-select <target_package> --symlink-install`

## Bước 4: Kiểm Thử Đơn Vị (Unit Testing)
Chạy các bài test GTest (C++) hoặc PyTest.
> `colcon test --packages-select <target_package> --ctest-args -E "lint"`
> `colcon test-result --all`

## Bảng Tổng Kết Trạng Thái (Status Interpretation)

| Exit Code | Ý Nghĩa | Hành Động Khuyến Nghị |
| :--- | :--- | :--- |
| 0 | Success | Có thể commit mã. |
| 1 | Build Failed | Kiểm tra `package.xml` và cú pháp mã. |
| 2 | Test Failed | Kiểm tra logic nghiệp vụ hoặc linter style.
