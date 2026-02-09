---
description: CLEAN WORKSPACE INITIALIZATION
---
# CLEAN WORKSPACE INITIALIZATION
// turbo

Quy trình xóa sạch và khởi tạo lại workspace.

## Các Bước Thực Hiện
1.  **Dừng toàn bộ Node:**
    > `pkill -f ros2` hoặc `pkill -f <node_name>`
2.  **Xóa Artifacts:**
    > `rm -rf build/ install/ log/`
3.  **Cập nhật Index:**
    > `ros2 pkg list` (để refresh cache hệ thống nếu cần)
4.  **Rebuild toàn bộ:**
    > `colcon build --symlink-install`
5.  **Source môi trường mới:**
    > `source install/setup.bash` (Lưu ý: Agent cần thực hiện việc này trong phiên shell mới).
