#!/usr/bin/env python3
"""
fake_object_pose.py — Publie une pose fictive d'objet pour les tests.

CORRECTION : position alignée avec scene_config.yaml pick_object (x=0.5, y=0.1, z=0.05).
L'ancienne position (x=0.35, y=0.0, z=0.05) divergeait de la scène MoveIt2 : l'IK
était calculé pour (0.35,0.0) mais l'objet visuel dans RViz était à (0.5,0.1).

Timer réduit de 0.5s → 2.0s (pose statique — pas besoin de publier 2 fois/seconde).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class FakeObjectPosePublisher(Node):
    def __init__(self):
        super().__init__('fake_object_pose_publisher')
        self.pub = self.create_publisher(PoseStamped, '/object_pose', 10)

        # CORRECTION : coordonnées alignées avec scene_config.yaml → pick_object.position
        # Ancienne valeur : x=0.35, y=0.00 → incohérente avec la scène MoveIt2
        self.object_x = 0.50   # identique à scene_config.yaml
        self.object_y = 0.10   # identique à scene_config.yaml
        self.object_z = 0.05   # sommet de l'objet (hauteur 0.10m / 2 + table)

        # Pose statique : inutile de publier à 2 Hz — 0.5 Hz suffit
        self.timer = self.create_timer(2.0, self.publish_pose)
        self.get_logger().info(
            f"Fake /object_pose @ ({self.object_x}, {self.object_y}, {self.object_z}) "
            f"frame='world'"
        )

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = float(self.object_x)
        msg.pose.position.y = float(self.object_y)
        msg.pose.position.z = float(self.object_z)
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeObjectPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
