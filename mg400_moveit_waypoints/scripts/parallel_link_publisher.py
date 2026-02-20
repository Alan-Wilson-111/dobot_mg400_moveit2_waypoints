#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ParallelLinkPublisher(Node):
    def __init__(self):
        super().__init__('parallel_link_publisher')
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.publisher = self.create_publisher(
            JointState, '/joint_states', 10)
        self.j2_val = 0.0
        self.j3_val = 0.0
        self.timer = self.create_timer(0.01, self.publish_compensation)

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == 'j2':
                self.j2_val = msg.position[i]
            elif name == 'j3':
                self.j3_val = msg.position[i]

    def publish_compensation(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['j3_to_link4']
        msg.position = [-(self.j2_val + self.j3_val)]
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = ParallelLinkPublisher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()