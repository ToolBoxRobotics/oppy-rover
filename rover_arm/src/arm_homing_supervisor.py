#!/usr/bin/env python3
import rospy
from rover_msgs.msg import ArmJointStates
from sensor_msgs.msg import JointState

class ArmHomingSupervisor:
    def __init__(self):
        self.homed = [False] * 5
        self.homing_done = False

        self.state_sub = rospy.Subscriber(
            "/arm/joint_states", ArmJointStates, self.state_cb, queue_size=10)

        # Raw command topic (from MoveIt or teleop)
        self.cmd_raw_sub = rospy.Subscriber(
            "/arm/joint_cmd_raw", JointState, self.cmd_raw_cb, queue_size=10)

        # Filtered command output (to Arduino via arm_bridge.py)
        self.cmd_pub = rospy.Publisher(
            "/arm/joint_cmd", JointState, queue_size=10)

        rospy.loginfo("[arm_homing_supervisor] Ready and waiting for homing...")

    def state_cb(self, msg: ArmJointStates):
        self.homed = msg.joint_homed
        if all(self.homed) and not self.homing_done:
            self.homing_done = True
            rospy.loginfo("[arm_homing_supervisor] All joints homed. Arm is ready.")

    def cmd_raw_cb(self, msg: JointState):
        if not self.homing_done:
            rospy.logwarn_throttle(5.0,
                "[arm_homing_supervisor] Command dropped — arm still homing.")
            return

        # Pass through once homed
        self.cmd_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("arm_homing_supervisor")
    ArmHomingSupervisor()
    rospy.spin()
