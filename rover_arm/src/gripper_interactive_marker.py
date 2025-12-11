#!/usr/bin/env python3
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler

from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from visualization_msgs.msg import InteractiveMarkerFeedback
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


class GripperInteractiveMarker:
    def __init__(self):
        rospy.loginfo("[gripper_interactive_marker] Starting...")

        # Publisher to your homing-supervisor gatekeeper
        self.pub = rospy.Publisher("/arm/joint_cmd_raw", JointState, queue_size=10)

        # Marker server
        self.server = InteractiveMarkerServer("gripper_interactive_marker")

        # Create menu
        self.menu = MenuHandler()
        self.h_open = self.menu.insert("Open Gripper", callback=self.open_cb)
        self.h_close = self.menu.insert("Close Gripper", callback=self.close_cb)

        # Submenu: Set Position
        h_set = self.menu.insert("Set Position")
        for pct in [0, 25, 50, 75, 100]:
            self.menu.insert(f"{pct}%",
                             parent=h_set,
                             callback=lambda fb, p=pct: self.set_pct(p))

        # Create interactive marker at arm_tool_link
        self.create_marker()

        self.menu.apply(self.server, "gripper_control")
        self.server.applyChanges()

        rospy.loginfo("[gripper_interactive_marker] Ready.")

    # -------------------------------------------
    # CREATE MARKER (slider on arm_tool_link)
    # -------------------------------------------
    def create_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "arm_tool_link"
        int_marker.name = "gripper_control"
        int_marker.description = "Gripper Control"
        int_marker.scale = 0.25

        # Visual representation: a cube
        cube = Marker()
        cube.type = Marker.CUBE
        cube.scale.x = 0.04
        cube.scale.y = 0.04
        cube.scale.z = 0.04
        cube.color.r = 0.2
        cube.color.g = 0.8
        cube.color.b = 0.2
        cube.color.a = 1.0

        control_visual = InteractiveMarkerControl()
        control_visual.always_visible = True
        control_visual.markers.append(cube)
        int_marker.controls.append(control_visual)

        # Slider control: Move along Z axis (tool frame)
        ctrl = InteractiveMarkerControl()
        ctrl.name = "gripper_slider"
        ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        ctrl.orientation.w = 1
        ctrl.orientation.x = 0
        ctrl.orientation.y = 0
        ctrl.orientation.z = 1
        int_marker.controls.append(ctrl)

        self.server.insert(int_marker, self.process_feedback)

    # -------------------------------------------
    # PROCESS SLIDER MOVEMENT / MENU COMMANDS
    # -------------------------------------------
    def process_feedback(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type in [
            InteractiveMarkerFeedback.MOUSE_UP,
            InteractiveMarkerFeedback.POSE_UPDATE,
        ]:
            # Convert marker Z movement to gripper command
            z = feedback.pose.position.z

            # Clamp slider between -0.05 .. +0.05
            z = max(-0.05, min(0.05, z))

            # Convert to 0..1 gripper command
            # z = -0.05 → 1.0 (closed)
            # z = +0.05 → 0.0 (open)
            pct = (0.05 - z) / 0.10
            pct = max(0.0, min(1.0, pct))

            self.send_gripper_cmd(pct)

    # -------------------------------------------
    # MENU CALLBACKS
    # -------------------------------------------
    def open_cb(self, feedback):
        self.send_gripper_cmd(0.0)

    def close_cb(self, feedback):
        self.send_gripper_cmd(1.0)

    def set_pct(self, pct):
        self.send_gripper_cmd(pct / 100.0)

    # -------------------------------------------
    # SEND TO ARM JOINT STATE COMMANDS
    # -------------------------------------------
    def send_gripper_cmd(self, cmd_01):
        js = JointState()
        js.name = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
        js.position = [float('nan')] * 5       # NaN = no change
        js.effort = [cmd_01]                  # our gripper command channel
        self.pub.publish(js)
        rospy.loginfo_throttle(1.0,
            f"[gripper_interactive_marker] Gripper cmd = {cmd_01:.2f}")


if __name__ == "__main__":
    rospy.init_node("gripper_interactive_marker")
    node = GripperInteractiveMarker()
    rospy.spin()
