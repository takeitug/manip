import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class ARMarkerTracker(Node):
    def __init__(self):
        super().__init__('ar_marker_tracker')
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)

        self.marker1_pub = self.create_publisher(PoseStamped, '/marker1_pose', 10)
        self.marker2_pub = self.create_publisher(PoseStamped, '/marker2_pose', 10)

        self.latest_depth = None

        self.timer = self.create_timer(0.1, self.check_tf)

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def image_callback(self, msg):
        if self.latest_depth is None:
            return

        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Detect AR markers (Aruco example, replace with AprilTag if needed)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        corners, ids, _ = detector.detectMarkers(gray)

        if ids is None:
            return

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id not in [1, 2]:
                continue

            c = corners[i][0]
            cx = int(np.mean(c[:, 0]))
            cy = int(np.mean(c[:, 1]))
            depth = self.latest_depth[cy, cx] / 1000.0  # mm → meters

            if depth == 0.0:
                continue

            x = (cx - 320) * depth / 615.0  # intrinsics
            y = (cy - 240) * depth / 615.0
            z = depth

            pose_cam = PoseStamped()
            pose_cam.header.frame_id = self.camera_frame
            pose_cam.header.stamp = msg.header.stamp
            pose_cam.pose.position.x = x
            pose_cam.pose.position.y = y
            pose_cam.pose.position.z = z
            pose_cam.pose.orientation.w = 1.0

            # publish
            if marker_id == 1:
                self.marker1_pub.publish(pose_cam)
            elif marker_id == 2:
                self.marker2_pub.publish(pose_cam)

        cv2.imshow("AR Markers", color_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ARMarkerTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
