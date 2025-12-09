#!/usr/bin/env python3
import rospy
import threading

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from visualization_msgs.msg import InteractiveMarkerFeedback
from geometry_msgs.msg import PoseStamped

import moveit_commander


class ArmInteractiveMarker:
    def __init__(self):
        rospy.loginfo("[arm_interactive_marker] Initializing...")

        # Init MoveIt
        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = rospy.get_param("~group", "arm")
        self.arm_group = moveit_commander.MoveGroupCommander(self.group_name)

        # Planning frame and end-effector link
        self.planning_frame = self.arm_group.get_planning_frame()
        self.eef_link = self.arm_group.get_end_effector_link()
        if not self.eef_link:
            self.eef_link = rospy.get_param("~eef_link", "arm_tool_link")

        rospy.loginfo(f"[arm_interactive_marker] Planning frame: {self.planning_frame}")
        rospy.loginfo(f"[arm_interactive_marker] End-effector link: {self.eef_link}")

        # Interactive marker server
        self.server = InteractiveMarkerServer("arm_interactive_marker")

        # Mutex to avoid overlapping motions
        self.lock = threading.Lock()

        # Create marker at current pose
        initial_pose = self.arm_group.get_current_pose(self.eef_link).pose
        self.create_marker(initial_pose)

        rospy.loginfo("[arm_interactive_marker] Ready. Use RViz 'Interactive Markers' display to manipulate.")

    def create_marker(self, pose):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.planning_frame
        int_marker.name = "arm_target"
        int_marker.description = "Arm EE Target"
        int_marker.pose = pose
        int_marker.scale = 0.3  # marker size

        # Visual representation (a small sphere)
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = 0.05
        sphere.scale.y = 0.05
        sphere.scale.z = 0.05
        sphere.color.r = 0.1
        sphere.color.g = 0.7
        sphere.color.b = 0.1
        sphere.color.a = 1.0

        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.markers.append(sphere)
        int_marker.controls.append(visual_control)

        # 6-DOF controls

        # Move X
        ctrl = InteractiveMarkerControl()
        ctrl.name = "move_x"
        ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        ctrl.orientation.w = 1
        ctrl.orientation.x = 1
        ctrl.orientation.y = 0
        ctrl.orientation.z = 0
        int_marker.controls.append(ctrl)

        # Move Y
        ctrl = InteractiveMarkerControl()
        ctrl.name = "move_y"
        ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        ctrl.orientation.w = 1
        ctrl.orientation.x = 0
        ctrl.orientation.y = 1
        ctrl.orientation.z = 0
        int_marker.controls.append(ctrl)

        # Move Z
        ctrl = InteractiveMarkerControl()
        ctrl.name = "move_z"
        ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        ctrl.orientation.w = 1
        ctrl.orientation.x = 0
        ctrl.orientation.y = 0
        ctrl.orientation.z = 1
        int_marker.controls.append(ctrl)

        # Rotate X
        ctrl = InteractiveMarkerControl()
        ctrl.name = "rotate_x"
        ctrl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        ctrl.orientation.w = 1
        ctrl.orientation.x = 1
        ctrl.orientation.y = 0
        ctrl.orientation.z = 0
        int_marker.controls.append(ctrl)

        # Rotate Y
        ctrl = InteractiveMarkerControl()
        ctrl.name = "rotate_y"
        ctrl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        ctrl.orientation.w = 1
        ctrl.orientation.x = 0
        ctrl.orientation.y = 1
        ctrl.orientation.z = 0
        int_marker.controls.append(ctrl)

        # Rotate Z
        ctrl = InteractiveMarkerControl()
        ctrl.name = "rotate_z"
        ctrl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        ctrl.orientation.w = 1
        ctrl.orientation.x = 0
        ctrl.orientation.y = 0
        ctrl.orientation.z = 1
        int_marker.controls.append(ctrl)

        self.server.insert(int_marker, self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback: InteractiveMarkerFeedback):
        # We only react when the mouse is released after moving: less spam
        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return

        rospy.loginfo("[arm_interactive_marker] New pose requested, planning...")
        with self.lock:
            target = PoseStamped()
            target.header.frame_id = feedback.header.frame_id
            target.pose = feedback.pose

            # Set goal
            self.arm_group.set_pose_target(target, end_effector_link=self.eef_link)

            # Plan and execute
            plan = self.arm_group.plan()
            if not plan or not hasattr(plan, "joint_trajectory") or len(plan.joint_trajectory.points) == 0:
                rospy.logwarn("[arm_interactive_marker] Planning failed.")
                return

            rospy.loginfo("[arm_interactive_marker] Executing plan...")
            self.arm_group.execute(plan, wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            rospy.loginfo("[arm_interactive_marker] Motion done.")

def main():
    rospy.init_node("arm_interactive_marker")
    node = ArmInteractiveMarker()
    rospy.spin()
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
