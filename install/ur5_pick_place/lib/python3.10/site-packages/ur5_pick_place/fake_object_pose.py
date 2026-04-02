#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class FakeObjectPosePublisher(Node):
    def __init__(self):
        super().__init__('fake_object_pose_publisher')
        self.pub = self.create_publisher(PoseStamped, '/object_pose', 10)
        self.timer = self.create_timer(0.5, self.publish_pose)
        self.object_x = 0.35
        self.object_y = 0.00
        self.object_z = 0.05
        self.get_logger().info("Publishing fake /object_pose in frame 'world'.")

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = float(self.object_x)
        msg.pose.position.y = float(self.object_y)
        msg.pose.position.z = float(self.object_z)
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeObjectPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
