#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointStateInitializer(Node):
    def __init__(self):
        super().__init__('joint_state_initializer')
        
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_initial_state)
        self.count = 0
        self.max_count = 100  # Publish for 10 seconds
        
        # Initial joint positions (from initial_positions.yaml)
        self.initial_positions = {
            'shoulder_pan_joint': 0.0,
            'shoulder_lift_joint': -1.57,
            'elbow_joint': 1.57,
            'wrist_1_joint': -1.57,
            'wrist_2_joint': -1.57,
            'wrist_3_joint': 0.0,
            'robotiq_85_left_knuckle_joint': 0.0,
            'robotiq_85_right_knuckle_joint': 0.0,
        }
        
        self.get_logger().info('Joint State Initializer started')

    def publish_initial_state(self):
        if self.count < self.max_count:
            msg = JointState()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self.initial_positions.keys())
            msg.position = list(self.initial_positions.values())
            msg.velocity = [0.0] * len(msg.name)
            msg.effort = [0.0] * len(msg.name)
            
            self.publisher_.publish(msg)
            self.count += 1
        else:
            self.get_logger().info('Initial states published, stopping')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    joint_state_initializer = JointStateInitializer()
    rclpy.spin(joint_state_initializer)
    joint_state_initializer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()