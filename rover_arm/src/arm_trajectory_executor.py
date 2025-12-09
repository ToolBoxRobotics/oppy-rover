#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

class ArmTrajectoryExecutor:
    def __init__(self):
        self.pub = rospy.Publisher("/arm/joint_cmd_raw", JointState, queue_size=10)
        rospy.Subscriber("/arm_controller/command", JointTrajectory,
                         self.cb, queue_size=10)
        rospy.loginfo("[arm_trajectory_executor] Ready.")

    def cb(self, traj: JointTrajectory):
        if len(traj.points) == 0:
            return

        # MoveIt sends full trajectory; we take final point
        final = traj.points[-1]

        js = JointState()
        js.name = traj.joint_names
        js.position = final.positions
        js.velocity = final.velocities
        js.effort = []

        self.pub.publish(js)

if __name__ == "__main__":
    rospy.init_node("arm_trajectory_executor")
    ArmTrajectoryExecutor()
    rospy.spin()
