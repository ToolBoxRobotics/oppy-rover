#!/usr/bin/env python3
import rospy
import math

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Bool, Float32
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped


class DepthHazardMonitor:
    def __init__(self):
        # Parameters
        self.cloud_topic = rospy.get_param("~cloud_topic", "/camera/depth/points")
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        # Hazard zone (in base_link frame)
        self.min_x = rospy.get_param("~min_x", 0.2)   # start 20 cm in front
        self.max_x = rospy.get_param("~max_x", 1.0)   # up to 1 m
        self.max_y = rospy.get_param("~max_y", 0.4)   # ±0.4 m side
        self.min_z = rospy.get_param("~min_z", -0.2)  # ground tolerance
        self.max_z = rospy.get_param("~max_z", 0.5)   # up to 0.5 m high

        self.max_points = rospy.get_param("~max_points", 5000)

        self.hazard_pub = rospy.Publisher("/hazard/obstacle", Bool, queue_size=1)
        self.min_dist_pub = rospy.Publisher("/hazard/min_distance", Float32, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sub = rospy.Subscriber(self.cloud_topic, PointCloud2,
                                    self.cloud_cb, queue_size=1)

        self.last_hazard = False
        rospy.loginfo("[depth_hazard_monitor] Monitoring %s relative to %s",
                      self.cloud_topic, self.base_frame)

    def cloud_cb(self, cloud: PointCloud2):
        # Try to transform points into base_link frame if needed
        count = 0
        hazard = False
        min_dist = float("inf")

        # We assume cloud.header.frame_id is either base_link-like or transformable
        source_frame = cloud.header.frame_id

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                cloud.header.stamp,
                rospy.Duration(0.1)
            )
        except Exception as e:
            rospy.logwarn_throttle(2.0,
                f"[depth_hazard_monitor] TF transform failed: {e}")
            return

        for p in point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
            count += 1
            if count > self.max_points:
                break

            pt = PointStamped()
            pt.header = cloud.header
            pt.point.x, pt.point.y, pt.point.z = p

            try:
                pt_base = tf2_geometry_msgs.do_transform_point(pt, transform)
            except Exception:
                continue

            x = pt_base.point.x
            y = pt_base.point.y
            z = pt_base.point.z

            # Check if point is inside hazard box
            if (self.min_x <= x <= self.max_x and
                abs(y) <= self.max_y and
                self.min_z <= z <= self.max_z):

                hazard = True
                dist = math.sqrt(x*x + y*y + z*z)
                if dist < min_dist:
                    min_dist = dist

        if not hazard:
            min_dist = float("inf")

        self.last_hazard = hazard
        self.hazard_pub.publish(Bool(data=hazard))
        self.min_dist_pub.publish(Float32(data=min_dist if math.isfinite(min_dist) else -1.0))


if __name__ == "__main__":
    rospy.init_node("depth_hazard_monitor")
    DepthHazardMonitor()
    rospy.spin()
