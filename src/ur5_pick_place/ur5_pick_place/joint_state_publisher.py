#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FixedJointStatePublisher(Node):
    def __init__(self):
        super().__init__('fixed_joint_state_publisher')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish)

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

        # Position de départ FIXE — jamais modifiée
        # Bras replié, clairement différent de toutes les cibles
        self.positions = [
            0.5,    # shoulder_pan  — décalé de 0.5 rad
            -1.57,  # shoulder_lift
            1.57,   # elbow
            -1.57,  # wrist_1
            -1.57,  # wrist_2
            0.0,    # wrist_3
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # gripper
        ]
        self.get_logger().info('Publishing fixed joint states — start pos: shoulder_pan=0.5')

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = self.joint_names
        msg.position = self.positions
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort   = [0.0] * len(self.joint_names)
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
