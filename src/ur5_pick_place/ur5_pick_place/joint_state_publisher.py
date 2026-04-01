#!/usr/bin/env python3
"""
Publie des joint states fixes pour initialiser MoveIt2
dans une position valide (bras plié, sans auto-collision).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FixedJointStatePublisher(Node):
    def __init__(self):
        super().__init__('fixed_joint_state_publisher')

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish)  # 20 Hz

        # Position valide : bras plié vers le bas, sans collision
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
            'robotiq_85_left_knuckle_joint',
            'robotiq_85_right_knuckle_joint',
            'robotiq_85_left_inner_knuckle_joint',
            'robotiq_85_right_inner_knuckle_joint',
            'robotiq_85_left_finger_tip_joint',
            'robotiq_85_right_finger_tip_joint',
        ]

        self.positions = [
            0.0,    # shoulder_pan
            -1.57,  # shoulder_lift
            1.57,   # elbow
            -1.57,  # wrist_1
            -1.57,  # wrist_2
            0.0,    # wrist_3
            0.0,    # gripper joints
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]

        self.get_logger().info('Publishing fixed joint states (valid start position)')

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = self.joint_names
        msg.position     = self.positions
        msg.velocity     = [0.0] * len(self.joint_names)
        msg.effort       = [0.0] * len(self.joint_names)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FixedJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
