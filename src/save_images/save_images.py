#!/usr/bin/env python3
"""
ROS2 图像保存节点
订阅 /color/image_raw 话题并保存图像到指定文件夹
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import argparse


class ImageSaver(Node):
    def __init__(self, save_dir='saved_images', fps=4):
        super().__init__('image_saver')

        # 参数
        self.save_dir = save_dir
        self.fps = fps
        self.save_interval = 1.0 / fps  # 保存间隔（秒）

        # 创建保存目录
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            self.get_logger().info(f'Created directory: {self.save_dir}')

        # CV Bridge
        self.bridge = CvBridge()

        # 计数器
        self.image_count = 0
        self.last_save_time = self.get_clock().now()

        # 订阅图像话题
        self.subscription = self.create_subscription(
            Image,
            '/color/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info(f'Image Saver Node Started')
        self.get_logger().info(f'Saving to: {os.path.abspath(self.save_dir)}')
        self.get_logger().info(f'Save rate: {self.fps} images/second')
        self.get_logger().info(f'Press Ctrl+C to stop')

    def image_callback(self, msg):
        # 检查是否到了保存时间
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_save_time).nanoseconds / 1e9

        if time_diff < self.save_interval:
            return

        try:
            # 转换 ROS Image 到 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 生成文件名（带时间戳）
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
            filename = f'image_{self.image_count:06d}_{timestamp}.jpg'
            filepath = os.path.join(self.save_dir, filename)

            # 保存图像
            cv2.imwrite(filepath, cv_image)

            self.image_count += 1
            self.last_save_time = current_time

            self.get_logger().info(f'Saved: {filename} (Total: {self.image_count})')

        except Exception as e:
            self.get_logger().error(f'Failed to save image: {str(e)}')


def main(args=None):
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='Save ROS2 camera images')
    parser.add_argument('--dir', type=str, default='saved_images',
                        help='Directory to save images (default: saved_images)')
    parser.add_argument('--fps', type=int, default=4,
                        help='Images per second to save (default: 4)')

    # 只解析已知参数，忽略 ROS2 参数
    parsed_args, unknown = parser.parse_known_args()

    rclpy.init(args=args)

    node = ImageSaver(save_dir=parsed_args.dir, fps=parsed_args.fps)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'\nStopping... Total images saved: {node.image_count}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
