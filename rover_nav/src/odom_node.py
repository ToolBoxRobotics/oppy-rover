#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from rover_msgs.msg import WheelStates
from geometry_msgs.msg import Quaternion
import tf
import math

class OdomNode:
    def __init__(self):
        self.sub = rospy.Subscriber("/rover/wheel_states", WheelStates, self.cb, queue_size=10)
        self.pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.br = tf.TransformBroadcaster()
        self.x = self.y = self.yaw = 0.0
        self.last_t = None
        self.wheel_base = rospy.get_param("~wheel_base", 0.6)

    def cb(self, msg):
        if self.last_t is None:
            self.last_t = msg.header.stamp
            return
        dt = (msg.header.stamp - self.last_t).to_sec()
        self.last_t = msg.header.stamp
        if dt <= 0: return

        # Approximate: use average left vs right velocities
        v_left  = (msg.wheel_velocity[0] + msg.wheel_velocity[2] + msg.wheel_velocity[4]) / 3.0
        v_right = (msg.wheel_velocity[1] + msg.wheel_velocity[3] + msg.wheel_velocity[5]) / 3.0
        v = (v_left + v_right) / 2.0
        wz = (v_right - v_left) / self.wheel_base

        self.yaw += wz * dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = tf.transformations.quaternion_from_euler(0,0,self.yaw)
        odom.pose.pose.orientation = Quaternion(*q)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = wz
        self.pub.publish(odom)

        self.br.sendTransform(
            (self.x, self.y, 0.0),
            q,
            msg.header.stamp,
            "base_link",
            "odom"
        )

if __name__ == "__main__":
    rospy.init_node("odom_node")
    OdomNode()
    rospy.spin()
