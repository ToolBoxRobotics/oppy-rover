#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class SixWheelKinematics:
    def __init__(self):
        rospy.init_node("six_wheel_kinematics")

        # Parameters (tune for your rover geometry)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.055)     # meters
        self.track_width  = rospy.get_param("~track_width", 0.45)       # meters (distance between left/right wheel groups)

        # Publishers for each wheel controller
        self.pub = {
            "lf": rospy.Publisher("/articulated_rover/left_front_wheel_velocity_controller/command",  Float64, queue_size=1),
            "lm": rospy.Publisher("/articulated_rover/left_middle_wheel_velocity_controller/command", Float64, queue_size=1),
            "lr": rospy.Publisher("/articulated_rover/left_rear_wheel_velocity_controller/command",   Float64, queue_size=1),

            "rf": rospy.Publisher("/articulated_rover/right_front_wheel_velocity_controller/command",  Float64, queue_size=1),
            "rm": rospy.Publisher("/articulated_rover/right_middle_wheel_velocity_controller/command", Float64, queue_size=1),
            "rr": rospy.Publisher("/articulated_rover/right_rear_wheel_velocity_controller/command",   Float64, queue_size=1),
        }

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.loginfo("6-wheel kinematics node started.")
        rospy.spin()

    def cmd_vel_callback(self, msg):
        v = msg.linear.x                      # forward velocity (m/s)
        w = msg.angular.z                     # yaw rate (rad/s)

        # Differential drive equations:
        # Left  side wheels: v_l = (v - w*track_width/2) / r
        # Right side wheels: v_r = (v + w*track_width/2) / r

        v_left  = (v - w * self.track_width / 2.0) / self.wheel_radius
        v_right = (v + w * self.track_width / 2.0) / self.wheel_radius

        # Publish to all 6 wheels
        self.pub["lf"].publish(v_left)
        self.pub["lm"].publish(v_left)
        self.pub["lr"].publish(v_left)

        self.pub["rf"].publish(v_right)
        self.pub["rm"].publish(v_right)
        self.pub["rr"].publish(v_right)

        rospy.logdebug(f"cmd_vel → wheel speeds: L={v_left:.2f}, R={v_right:.2f}")

if __name__ == "__main__":
    try:
        SixWheelKinematics()
    except rospy.ROSInterruptException:
        pass
