#!/usr/bin/env python3
import rospy
import math

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from rover_msgs.msg import HazardParams   # New msg type from live UI panel


class HazardVisualizer:
    def __init__(self):
        rospy.loginfo("[hazard_visualizer] Starting hazard overlay node")

        self.params = {
            "min_x": 0.3,
            "max_x": 1.0,
            "max_y": 0.5,
            "min_z": -0.2,
            "max_z": 0.5,
        }

        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/camera/depth/points")

        self.overlay_pub = rospy.Publisher("/hazard/overlay_markers",
                                           MarkerArray, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber(self.cloud_topic, PointCloud2,
                         self.cloud_cb, queue_size=1)
        rospy.Subscriber("/hazard/params", HazardParams,
                         self.params_cb, queue_size=10)

        rospy.loginfo("[hazard_visualizer] Ready.")

    # -------------------------
    # Receive updated parameters
    # -------------------------
    def params_cb(self, msg: HazardParams):
        self.params["min_x"] = msg.min_x
        self.params["max_x"] = msg.max_x
        self.params["max_y"] = msg.max_y
        self.params["min_z"] = msg.min_z
        self.params["max_z"] = msg.max_z

    # -------------------------
    # Main cloud callback
    # -------------------------
    def cloud_cb(self, cloud):
        arr = MarkerArray()

        # 1) Hazard zone cube
        arr.markers.append(self.make_zone_marker())

        # 2) Transform points & keep ones inside zone
        points = self.extract_hazard_points(cloud)
        arr.markers.append(self.make_point_marker(points))

        self.overlay_pub.publish(arr)

    # -------------------------
    # Draw hazard zone cuboid
    # -------------------------
    def make_zone_marker(self):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = rospy.Time.now()
        m.ns = "hazard_zone"
        m.id = 1
        m.type = Marker.CUBE
        m.action = Marker.ADD

        m.color = ColorRGBA(1.0, 0.5, 0.0, 0.25)   # orange transparent

        # Center & size
        w = self.params["max_x"] - self.params["min_x"]
        h = 2 * self.params["max_y"]
        d = self.params["max_z"] - self.params["min_z"]

        m.scale.x = w
        m.scale.y = h
        m.scale.z = d

        center_x = (self.params["max_x"] + self.params["min_x"]) / 2.0
        center_y = 0.0
        center_z = (self.params["max_z"] + self.params["min_z"]) / 2.0

        m.pose.position.x = center_x
        m.pose.position.y = center_y
        m.pose.position.z = center_z

        return m

    # -------------------------
    # Draw red hazard points
    # -------------------------
    def make_point_marker(self, pts):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = rospy.Time.now()
        m.ns = "hazard_points"
        m.id = 2
        m.type = Marker.POINTS
        m.action = Marker.ADD

        m.scale.x = 0.03
        m.scale.y = 0.03
        m.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # red

        for x, y, z in pts:
            p = Point()
            p.x, p.y, p.z = x, y, z
            m.points.append(p)

        return m

    # -------------------------
    # Process cloud & extract hazard points
    # -------------------------
    def extract_hazard_points(self, cloud):
        pts = []

        # Lookup TF transform
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, cloud.header.frame_id,
                cloud.header.stamp, rospy.Duration(0.1))
        except Exception:
            return pts

        for p in point_cloud2.read_points(cloud,
                                          field_names=("x", "y", "z"),
                                          skip_nans=True):
            ps = PointStamped()
            ps.header = cloud.header
            ps.point.x, ps.point.y, ps.point.z = p

            try:
                pt = tf2_geometry_msgs.do_transform_point(ps, tf).point
            except Exception:
                continue

            if (self.params["min_x"] <= pt.x <= self.params["max_x"] and
                abs(pt.y) <= self.params["max_y"] and
                self.params["min_z"] <= pt.z <= self.params["max_z"]):
                pts.append((pt.x, pt.y, pt.z))

        return pts


if __name__ == "__main__":
    rospy.init_node("hazard_visualizer")
    HazardVisualizer()
    rospy.spin()
