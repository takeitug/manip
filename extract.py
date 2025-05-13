import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
import tf2_ros
import tf2_kdl
import PyKDL
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler
from tf2_kdl import transform_to_kdl


def do_transform_pose(pose: PoseStamped, transform) -> PoseStamped:
    t = transform_to_kdl(transform)
    p = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ),
        PyKDL.Vector(
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z
        )
    )
    f = t * p
    res = PoseStamped()
    res.header = transform.header
    res.pose.position.x = f.p[0]
    res.pose.position.y = f.p[1]
    res.pose.position.z = f.p[2]
    q = f.M.GetQuaternion()
    res.pose.orientation.x = q[0]
    res.pose.orientation.y = q[1]
    res.pose.orientation.z = q[2]
    res.pose.orientation.w = q[3]
    return res


def do_transform_point(point: Point, tf1, tf2):
    pose_cam = PoseStamped()
    pose_cam.header.frame_id = tf2.child_frame_id
    pose_cam.header.stamp = rclpy.time.Time().to_msg()
    pose_cam.pose.position = point
    pose_cam.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
    pose_mount = do_transform_pose(pose_cam, tf2)
    pose_world = do_transform_pose(pose_mount, tf1)
    return pose_world.pose.position


class MarkerCloudFilter(Node):
    def __init__(self):
        super().__init__('marker_cloud_filter')

        self.marker1_pose_cam = None
        self.marker2_pose_cam = None
        self.radius = 0.2

        self.create_subscription(PointCloud2, '/camera/depth/color/points', self.cloud_callback, 10)
        self.create_subscription(PoseStamped, '/marker1_pose', self.marker1_callback, 10)
        self.create_subscription(PoseStamped, '/marker2_pose', self.marker2_callback, 10)

        self.marker1_cloud_pub = self.create_publisher(PointCloud2, '/marker1_cloud_transformed', 10)
        self.marker2_cloud_pub = self.create_publisher(PointCloud2, '/marker2_cloud_transformed', 10)
        self.marker1_pub = self.create_publisher(PoseStamped, '/marker1_transformed', 10)
        self.marker2_pub = self.create_publisher(PoseStamped, '/marker2_transformed', 10)

        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("camera_mount_frame", "camera_mount")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("camera_to_mount_translation", [0.0, 0.1, 0.2])
        self.declare_parameter("camera_to_mount_rotation_rpy", [0.0, 0.0, 0.0])

        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.camera_mount_frame = self.get_parameter("camera_mount_frame").get_parameter_value().string_value
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.translation = self.get_parameter("camera_to_mount_translation").value
        self.rotation_rpy = self.get_parameter("camera_to_mount_rotation_rpy").value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.broadcast_static_transform()

    def broadcast_static_transform(self):
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = self.camera_mount_frame
        static_tf.child_frame_id = self.camera_frame

        static_tf.transform.translation.x = self.translation[0]
        static_tf.transform.translation.y = self.translation[1]
        static_tf.transform.translation.z = self.translation[2]

        q = quaternion_from_euler(*self.rotation_rpy)
        static_tf.transform.rotation.x = q[0]
        static_tf.transform.rotation.y = q[1]
        static_tf.transform.rotation.z = q[2]
        static_tf.transform.rotation.w = q[3]

        self.static_broadcaster.sendTransform(static_tf)
        self.get_logger().info(f"Static TF published from {self.camera_mount_frame} to {self.camera_frame}")

    def marker1_callback(self, msg):
        self.marker1_pose_cam = msg.pose

    def marker2_callback(self, msg):
        self.marker2_pose_cam = msg.pose

    def cloud_callback(self, msg: PointCloud2):
        if self.marker1_pose_cam is None or self.marker2_pose_cam is None:
            return

        try:
            tf1 = self.tf_buffer.lookup_transform(self.world_frame, self.camera_mount_frame, rclpy.time.Time())
            tf2 = self.tf_buffer.lookup_transform(self.camera_mount_frame, self.camera_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        cloud_points = list(point_cloud2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))

        cloud_world = []
        for pt in cloud_points:
            point = Point(x=pt[0], y=pt[1], z=pt[2])
            try:
                transformed = do_transform_point(point, tf1, tf2)
                cloud_world.append([transformed.x, transformed.y, transformed.z])
            except Exception as e:
                continue

        cloud_np = np.array(cloud_world)

        marker1_world = do_transform_point(self.marker1_pose_cam.position, tf1, tf2)
        marker2_world = do_transform_point(self.marker2_pose_cam.position, tf1, tf2)

        marker1_points = self.extract_near_points(cloud_np, marker1_world)
        marker2_points = self.extract_near_points(cloud_np, marker2_world)

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.world_frame

        self.marker1_cloud_pub.publish(self.create_cloud(header, marker1_points))
        self.marker2_cloud_pub.publish(self.create_cloud(header, marker2_points))

        self.marker1_pub.publish(self.create_pose_stamped(marker1_world, header))
        self.marker2_pub.publish(self.create_pose_stamped(marker2_world, header))

    def extract_near_points(self, cloud_np, center_point, radius=0.2):
        center = np.array([center_point.x, center_point.y, center_point.z])
        distances = np.linalg.norm(cloud_np[:, :3] - center, axis=1)
        return cloud_np[distances < radius]

    def create_cloud(self, header: Header, points):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        return pc2.create_cloud(header, fields, points)

    def create_pose_stamped(self, point: Point, header: Header):
        pose = PoseStamped()
        pose.header = header
        pose.pose.position = point
        pose.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = MarkerCloudFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
