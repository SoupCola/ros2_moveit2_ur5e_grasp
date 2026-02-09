---
description: INSTALL ROS 2 HUMBLE (Ubuntu 22.04)
---
# INSTALL ROS 2 HUMBLE
// turbo

Quy trình chuẩn cài đặt ROS 2 Humble Hawksbill trên Ubuntu 22.04.

## 1. Thiết lập Locale
Đảm bảo locale hỗ trợ UTF-8.
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

## 2. Thêm Repository
Bật Ubuntu Universe và thêm ROS 2 GPG key.
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## 3. Cài đặt ROS 2 Packages
Cài đặt bản Desktop (gồm ROS-Base + Demos + Tutorials).
```bash
sudo apt update
sudo apt install ros-humble-desktop
```

## 4. Cài đặt Development Tools
Cài colcon, rosdep và các công cụ khác.
```bash
sudo apt install ros-dev-tools
```

## 5. Khởi tạo Rosdep
```bash
sudo rosdep init
rosdep update
```

## 6. Thiết lập Môi trường (Environment Setup)
Tự động source ROS 2 mỗi khi mở terminal mới.
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
