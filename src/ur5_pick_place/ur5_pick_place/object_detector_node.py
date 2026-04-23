# object_detector_node.py — Remplacement de fake_object_pose.py
#!/usr/bin/env python3
"""
Détection d'objet robuste via caméra RGB-D (Intel RealSense D435 / ZED2).
Pipeline: détection → clustering → estimation de pose → filtre Kalman.
Compatible ROS2 Jazzy / Humble.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, Image
import numpy as np

# Pour RealSense: pip install pyrealsense2
# Pour ZED: SDK ZED + ros2 wrapper

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        # Paramètres configurables
        self.declare_parameter('detection_method', 'pointcloud')  # 'pointcloud' ou 'yolo'
        self.declare_parameter('object_color_hsv_low',  [20, 100, 100])   # orange bas
        self.declare_parameter('object_color_hsv_high', [35, 255, 255])   # orange haut
        self.declare_parameter('kalman_process_noise', 0.01)
        self.declare_parameter('kalman_measurement_noise', 0.1)
        
        self.method = self.get_parameter('detection_method').value
        
        # Filtre de Kalman simplifié (position 3D)
        self.kf_state = None      # [x, y, z]
        self.kf_P = np.eye(3) * 1.0
        Q_val = self.get_parameter('kalman_process_noise').value
        R_val = self.get_parameter('kalman_measurement_noise').value
        self.kf_Q = np.eye(3) * Q_val   # bruit processus
        self.kf_R = np.eye(3) * R_val   # bruit mesure
        
        # Publishers / Subscribers
        self.pose_pub = self.create_publisher(PoseStamped, '/object_pose', 10)
        
        if self.method == 'pointcloud':
            self.create_subscription(
                PointCloud2, '/camera/depth/color/points',
                self._pointcloud_cb, 10)
        else:
            # YOLO sur image couleur
            self.create_subscription(
                Image, '/camera/color/image_raw',
                self._image_cb, 10)
        
        self.get_logger().info(f'ObjectDetector démarré — méthode: {self.method}')

    def _kalman_update(self, measurement: np.ndarray) -> np.ndarray:
        """Mise à jour filtre de Kalman (modèle statique, objet posé)."""
        if self.kf_state is None:
            self.kf_state = measurement.copy()
            return self.kf_state
        
        # Prédiction (objet statique → pas de dynamique)
        x_pred = self.kf_state
        P_pred = self.kf_P + self.kf_Q
        
        # Correction
        K = P_pred @ np.linalg.inv(P_pred + self.kf_R)
        self.kf_state = x_pred + K @ (measurement - x_pred)
        self.kf_P = (np.eye(3) - K) @ P_pred
        
        return self.kf_state

    def _pointcloud_cb(self, msg: PointCloud2):
        """
        Segmentation couleur dans le nuage de points.
        Retourne le centroïde du cluster dominant.
        """
        try:
            from sensor_msgs_py import point_cloud2 as pc2
            import open3d as o3d  # pip install open3d
            
            # Lire les points
            points = np.array(list(pc2.read_points(
                msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True)))
            
            if len(points) == 0:
                return
            
            xyz = points[:, :3]
            
            # Filtrer par distance (workspace UR5: 0.2m à 1.0m)
            dist = np.sqrt(xyz[:, 0]**2 + xyz[:, 1]**2)
            mask = (dist > 0.2) & (dist < 1.0) & (xyz[:, 2] > 0.01) & (xyz[:, 2] < 0.5)
            xyz_filtered = xyz[mask]
            
            if len(xyz_filtered) < 50:
                self.get_logger().warn('Pas assez de points dans le workspace')
                return
            
            # Clustering DBSCAN pour isoler l'objet
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz_filtered)
            labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=30))
            
            if labels.max() < 0:
                return
            
            # Prendre le cluster le plus proche de la base
            best_cluster = -1
            best_dist = float('inf')
            for label in range(labels.max() + 1):
                cluster_pts = xyz_filtered[labels == label]
                centroid = cluster_pts.mean(axis=0)
                d = np.sqrt(centroid[0]**2 + centroid[1]**2)
                if d < best_dist:
                    best_dist = d
                    best_cluster = label
                    best_centroid = centroid
            
            if best_cluster < 0:
                return
            
            # Filtrage Kalman
            smoothed = self._kalman_update(best_centroid)
            
            # Publication
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = 'world'
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose.position.x = float(smoothed[0])
            pose_msg.pose.position.y = float(smoothed[1])
            pose_msg.pose.position.z = float(smoothed[2])
            pose_msg.pose.orientation.w = 1.0
            
            self.pose_pub.publish(pose_msg)
            
        except Exception as e:
            self.get_logger().error(f'Erreur détection: {e}')

    def _image_cb(self, msg: Image):
        """Détection YOLO si disponible (plus robuste que segmentation couleur)."""
        try:
            import cv2
            from cv_bridge import CvBridge
            # from ultralytics import YOLO  # pip install ultralytics
            
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Segmentation par couleur (fallback sans YOLO)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            low = np.array(self.get_parameter('object_color_hsv_low').value)
            high = np.array(self.get_parameter('object_color_hsv_high').value)
            mask = cv2.inRange(hsv, low, high)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                return
            
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] == 0:
                return
            
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            self.get_logger().debug(f'Objet détecté pixel: ({cx}, {cy})')
            # TODO: déprojecter via camera_info pour obtenir (x,y,z) 3D
            
        except Exception as e:
            self.get_logger().error(f'Erreur YOLO/CV: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
