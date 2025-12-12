#!/usr/bin/env python3
import sys
import math
import threading
import time
import subprocess
import datetime

import rospy
from python_qt_binding.QtWidgets import (
    QApplication, QWidget, QTabWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QSlider, QDoubleSpinBox, QGroupBox,
    QFormLayout, QCheckBox, QTextEdit
)
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QImage, QPixmap
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Image
from rover_msgs.msg import PowerStatus, ArmJointStates, HazardParams
import tf.transformations as tft
from cv_bridge import CvBridge
import cv2


class MissionControl(QWidget):
    def __init__(self):
        super(MissionControl, self).__init__()

        rospy.init_node("mission_control", anonymous=True, disable_signals=True)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.arm_cmd_pub = rospy.Publisher("/arm/joint_cmd_raw", JointState, queue_size=10)
        self.nav_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

        self.hazard_active = False
        self.hazard_min_distance = -1.0
        self.hazard_params_pub = rospy.Publisher("/hazard/params", HazardParams, queue_size=10)

        # Subscribers
        self.power_sub = rospy.Subscriber("/rover/power", PowerStatus, self.power_cb, queue_size=10)
        self.arm_state_sub = rospy.Subscriber("/arm/joint_states", ArmJointStates, self.arm_state_cb, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=10)

        self.hazard_sub = rospy.Subscriber("/hazard/obstacle", Bool, self.hazard_cb, queue_size=10)
        self.hazard_dist_sub = rospy.Subscriber("/hazard/min_distance", Float32, self.hazard_dist_cb, queue_size=10)

        # Camera topics (params allow override)
        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/rgb/image_color")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")

        self.rgb_sub = rospy.Subscriber(self.rgb_topic, Image, self.rgb_cb, queue_size=1)
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_cb, queue_size=1)

        self.bridge = CvBridge()
        self.rgb_qimage = None
        self.depth_qimage = None
        self.camera_lock = threading.Lock()

        # State
        self.last_voltage = 0.0
        self.last_current = 0.0
        self.last_power = 0.0
        self.last_batt_pct = 0.0
        self.homing_done = False
        self.last_odom = None

        self.drive_enabled = False

        # Mission script state
        self.mission_active = False
        self.mission_index = 0
        self.mission_step_start_time = None
        self.mission_script = self.default_mission_script()

        # Rosbag recorder
        self.rosbag_process = None
        self.rosbag_auto = True          # Auto log missions
        self.rosbag_status = "idle"       # "idle" / "recording"
        self.rosbag_topics = [
            "/odom",
            "/cmd_vel",
            "/arm/joint_states",
            "/arm/joint_cmd",
            "/camera/rgb/image_color",
            "/camera/depth/image_raw",
            "/tf",
            "/tf_static",
            "/rover/power"
        ]

        # Build UI
        self.init_ui()

        # Timers
        self.drive_timer = QTimer(self)
        self.drive_timer.timeout.connect(self.publish_drive_cmd)
        self.drive_timer.start(100)  # 10 Hz

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.update_status_labels)
        self.status_timer.start(200)  # 5 Hz

        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self.update_camera_views)
        self.camera_timer.start(100)  # 10 Hz

        self.mission_timer = QTimer(self)
        self.mission_timer.timeout.connect(self.update_mission)
        self.mission_timer.start(200)  # 5 Hz

        # ROS spin in background
        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()


    # ============================================================
    # ROSBAG CONTROL
    # ============================================================

    def start_rosbag(self):
        """Start rosbag recording."""
        if self.rosbag_process is not None:
            rospy.logwarn("Rosbag already running.")
            return

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        outname = f"/home/ubuntu/rosbags/mission_{timestamp}.bag"

        cmd = ["rosbag", "record", "-O", outname] + self.rosbag_topics
        rospy.loginfo(f"Starting rosbag: {cmd}")

        try:
            self.rosbag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.rosbag_status = "recording"
        except Exception as e:
            rospy.logerr(f"Cannot start rosbag: {e}")
            self.rosbag_process = None
            self.rosbag_status = "error"

    def stop_rosbag(self):
        """Stop rosbag recording."""
        if self.rosbag_process is None:
            return

        rospy.loginfo("Stopping rosbag...")
        try:
            self.rosbag_process.terminate()
            self.rosbag_process.wait(timeout=5)
        except Exception:
            self.rosbag_process.kill()

        self.rosbag_process = None
        self.rosbag_status = "idle"

    def rosbag_status_text(self):
        return f"Rosbag: {self.rosbag_status}"




    # --------------------
    # ROS callbacks
    # --------------------
    def power_cb(self, msg: PowerStatus):
        self.last_voltage = msg.bus_voltage
        self.last_current = msg.bus_current
        self.last_power = msg.bus_power
        self.last_batt_pct = msg.battery_percent

    def arm_state_cb(self, msg: ArmJointStates):
        self.homing_done = all(msg.joint_homed)

    def odom_cb(self, msg: Odometry):
        self.last_odom = msg

    def rgb_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            h, w, ch = cv_img.shape
            bytes_per_line = ch * w
            qimg = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
            with self.camera_lock:
                self.rgb_qimage = qimg.copy()
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"RGB image error: {e}")

    def depth_cb(self, msg: Image):
        try:
            # Convert depth to 8-bit grayscale for display
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            cv_img = cv_img.astype('float32')
            # Clip and normalize
            cv_img[cv_img <= 0] = float('nan')
            max_dist = 4.0  # meters
            norm = cv_img / max_dist
            norm[norm > 1.0] = 1.0
            norm = (1.0 - norm) * 255.0  # nearer is bright
            norm = cv2.cvtColor(norm.astype('uint8'), cv2.COLOR_GRAY2RGB)
            h, w, ch = norm.shape
            bytes_per_line = ch * w
            qimg = QImage(norm.data, w, h, bytes_per_line, QImage.Format_RGB888)
            with self.camera_lock:
                self.depth_qimage = qimg.copy()
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Depth image error: {e}")

    def hazard_cb(self, msg):
        self.hazard_active = msg.data

    def hazard_dist_cb(self, msg):
        self.hazard_min_distance = msg.data

    def ros_spin(self):
        rospy.spin()


    # --------------------
    # UI
    # --------------------
    def init_ui(self):
        self.setWindowTitle("ToolBox Robotics Rover – Mission Control")

        layout = QVBoxLayout(self)
        tabs = QTabWidget(self)
        layout.addWidget(tabs)

        tabs.addTab(self.build_drive_tab(), "Drive")
        tabs.addTab(self.build_arm_tab(), "Arm")
        tabs.addTab(self.build_gripper_tab(), "Gripper")
        tabs.addTab(self.build_nav_tab(), "Navigation")
        tabs.addTab(self.build_camera_tab(), "Camera")
        tabs.addTab(self.build_mission_tab(), "Mission")
        tabs.addTab(self.build_status_tab(), "Status")
        tabs.addTab(self.build_hazard_settings_tab(), "Hazard Settings")


        self.setLayout(layout)
        self.resize(1000, 700)

    # --- Drive tab ---
    def build_drive_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)

        self.drive_enable_checkbox = QCheckBox("Enable Drive Commands")
        self.drive_enable_checkbox.stateChanged.connect(self.on_drive_enable_toggled)
        layout.addWidget(self.drive_enable_checkbox)

        group = QGroupBox("Base Velocity Command")
        g_layout = QFormLayout(group)

        self.slider_lin = QSlider(Qt.Horizontal)
        self.slider_lin.setRange(-100, 100)
        self.slider_lin.setValue(0)
        self.slider_lin.setSingleStep(5)

        self.slider_ang = QSlider(Qt.Horizontal)
        self.slider_ang.setRange(-100, 100)
        self.slider_ang.setValue(0)
        self.slider_ang.setSingleStep(5)

        g_layout.addRow(QLabel("Linear X (m/s) [-1..1]"), self.slider_lin)
        g_layout.addRow(QLabel("Angular Z (rad/s) [-1..1]"), self.slider_ang)
        layout.addWidget(group)

        btn_stop = QPushButton("STOP")
        btn_stop.clicked.connect(self.on_stop_clicked)
        layout.addWidget(btn_stop)

        layout.addStretch(1)
        return w

    def on_drive_enable_toggled(self, state):
        self.drive_enabled = (state == Qt.Checked)

    def publish_drive_cmd(self):
        if not self.drive_enabled or rospy.is_shutdown():
            return
         # If hazard active, prevent forward motion
        if self.hazard_active:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return

        twist = Twist()
        twist.linear.x = self.slider_lin.value() / 100.0
        twist.angular.z = self.slider_ang.value() / 100.0
        self.cmd_vel_pub.publish(twist)


    def on_stop_clicked(self):
        self.slider_lin.setValue(0)
        self.slider_ang.setValue(0)
        self.drive_enabled = False
        self.drive_enable_checkbox.setChecked(False)
        self.cmd_vel_pub.publish(Twist())

    # --- Arm tab ---
    def build_arm_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)

        self.arm_homing_label = QLabel("Homing: UNKNOWN")
        layout.addWidget(self.arm_homing_label)

        group = QGroupBox("Arm Joint Targets (deg)")
        g_layout = QFormLayout(group)

        self.arm_joint_boxes = []
        joint_names = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
        for jn in joint_names:
            box = QDoubleSpinBox()
            box.setRange(-180.0, 180.0)
            box.setSingleStep(5.0)
            box.setValue(0.0)
            self.arm_joint_boxes.append(box)
            g_layout.addRow(QLabel(jn), box)

        layout.addWidget(group)

        btn_send = QPushButton("Send Arm Command (raw)")
        btn_send.clicked.connect(self.send_arm_cmd)
        layout.addWidget(btn_send)

        layout.addStretch(1)
        return w

    def send_arm_cmd(self):
        if not self.homing_done:
            rospy.logwarn("Arm not homed yet, command ignored.")
            return

        js = JointState()
        js.name = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
        js.position = [math.radians(box.value()) for box in self.arm_joint_boxes]
        js.effort = []  # gripper handled separately
        self.arm_cmd_pub.publish(js)

    # --- Gripper tab ---
    def build_gripper_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)

        group = QGroupBox("Gripper")
        g_layout = QFormLayout(group)

        self.gripper_slider = QSlider(Qt.Horizontal)
        self.gripper_slider.setRange(0, 100)
        self.gripper_slider.setValue(0)

        g_layout.addRow(QLabel("Gripper (0=open, 100=closed)"), self.gripper_slider)
        layout.addWidget(group)

        btn_open = QPushButton("Open")
        btn_close = QPushButton("Close")
        btn_send = QPushButton("Send Gripper Command")

        btn_open.clicked.connect(lambda: self.set_gripper(0.0))
        btn_close.clicked.connect(lambda: self.set_gripper(1.0))
        btn_send.clicked.connect(self.send_gripper_from_slider)

        h = QHBoxLayout()
        h.addWidget(btn_open)
        h.addWidget(btn_close)
        h.addWidget(btn_send)
        layout.addLayout(h)

        layout.addStretch(1)
        return w

    def set_gripper(self, cmd_01):
        self.gripper_slider.setValue(int(cmd_01 * 100.0))
        self.send_gripper_cmd(cmd_01)

    def send_gripper_from_slider(self):
        cmd_01 = self.gripper_slider.value() / 100.0
        self.send_gripper_cmd(cmd_01)

    def send_gripper_cmd(self, cmd_01):
        js = JointState()
        js.name = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
        js.position = [float('nan')] * 5
        js.effort = [cmd_01]
        self.arm_cmd_pub.publish(js)
        rospy.loginfo(f"Gripper command: {cmd_01:.2f}")

    # --- Navigation tab ---
    def build_nav_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)

        group = QGroupBox("2D Nav Goal (map frame)")
        g_layout = QFormLayout(group)

        self.nav_x = QDoubleSpinBox()
        self.nav_y = QDoubleSpinBox()
        self.nav_yaw_deg = QDoubleSpinBox()
        self.nav_x.setRange(-1000, 1000)
        self.nav_y.setRange(-1000, 1000)
        self.nav_yaw_deg.setRange(-180, 180)
        self.nav_x.setDecimals(2)
        self.nav_y.setDecimals(2)
        self.nav_yaw_deg.setDecimals(1)

        g_layout.addRow(QLabel("X [m]"), self.nav_x)
        g_layout.addRow(QLabel("Y [m]"), self.nav_y)
        g_layout.addRow(QLabel("Yaw [deg]"), self.nav_yaw_deg)
        layout.addWidget(group)

        btn_here = QPushButton("Set goal to current pose")
        btn_send = QPushButton("Send Goal")

        btn_here.clicked.connect(self.set_goal_to_current_pose)
        btn_send.clicked.connect(self.send_nav_goal)

        h = QHBoxLayout()
        h.addWidget(btn_here)
        h.addWidget(btn_send)
        layout.addLayout(h)

        layout.addStretch(1)
        return w

    def set_goal_to_current_pose(self):
        if not self.last_odom:
            rospy.logwarn("No odom yet")
            return
        pose = self.last_odom.pose.pose
        self.nav_x.setValue(pose.position.x)
        self.nav_y.setValue(pose.position.y)
        q = pose.orientation
        yaw = math.degrees(
            tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        )
        self.nav_yaw_deg.setValue(yaw)

    def send_nav_goal(self):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = self.nav_x.value()
        goal.pose.position.y = self.nav_y.value()
        goal.pose.position.z = 0.0

        yaw_rad = math.radians(self.nav_yaw_deg.value())
        q = tft.quaternion_from_euler(0, 0, yaw_rad)
        goal.pose.orientation = Quaternion(*q)

        self.nav_goal_pub.publish(goal)
        rospy.loginfo("Sent nav goal: (%.2f, %.2f, %.1f deg)",
                      goal.pose.position.x, goal.pose.position.y, self.nav_yaw_deg.value())

    # --- Camera tab ---
    def build_camera_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)

        self.lbl_rgb = QLabel("RGB camera")
        self.lbl_rgb.setAlignment(Qt.AlignCenter)
        self.lbl_rgb.setMinimumSize(320, 240)

        self.lbl_depth = QLabel("Depth camera")
        self.lbl_depth.setAlignment(Qt.AlignCenter)
        self.lbl_depth.setMinimumSize(320, 240)

        layout.addWidget(QLabel(f"RGB topic: {self.rgb_topic}"))
        layout.addWidget(self.lbl_rgb)
        layout.addWidget(QLabel(f"Depth topic: {self.depth_topic}"))
        layout.addWidget(self.lbl_depth)

        layout.addStretch(1)
        return w

    def update_camera_views(self):
        with self.camera_lock:
            if self.rgb_qimage is not None:
                pix = QPixmap.fromImage(self.rgb_qimage).scaled(
                    self.lbl_rgb.width(), self.lbl_rgb.height(),
                    Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.lbl_rgb.setPixmap(pix)
            if self.depth_qimage is not None:
                pix = QPixmap.fromImage(self.depth_qimage).scaled(
                    self.lbl_depth.width(), self.lbl_depth.height(),
                    Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.lbl_depth.setPixmap(pix)



    # --- Mission tab ---
    def build_mission_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)

        # Mission status
        self.mission_status_label = QLabel("Mission: idle")
        layout.addWidget(self.mission_status_label)

        # ROSBAG status
        self.rosbag_status_label = QLabel(self.rosbag_status_text())
        layout.addWidget(self.rosbag_status_label)

        # Mission script listing
        self.mission_text = QTextEdit()
        self.mission_text.setReadOnly(True)
        self.refresh_mission_text()
        layout.addWidget(self.mission_text)

        # Mission control buttons
        btn_start = QPushButton("Start Mission")
        btn_stop = QPushButton("Stop Mission")
        btn_next = QPushButton("Skip to Next Step")

        btn_start.clicked.connect(self.start_mission)
        btn_stop.clicked.connect(self.stop_mission)
        btn_next.clicked.connect(self.skip_step)

        h = QHBoxLayout()
        h.addWidget(btn_start)
        h.addWidget(btn_stop)
        h.addWidget(btn_next)
        layout.addLayout(h)

        # Rosbag manual controls
        bag_group = QGroupBox("Rosbag Recording")
        bg_layout = QHBoxLayout(bag_group)

        self.chk_rosbag_auto = QCheckBox("Auto-record during missions")
        self.chk_rosbag_auto.setChecked(True)
        self.chk_rosbag_auto.stateChanged.connect(
            lambda s: setattr(self, "rosbag_auto", s == Qt.Checked)
        )

        btn_bag_start = QPushButton("Start Recording")
        btn_bag_stop = QPushButton("Stop Recording")

        btn_bag_start.clicked.connect(self.start_rosbag)
        btn_bag_stop.clicked.connect(self.stop_rosbag)

        bg_layout.addWidget(self.chk_rosbag_auto)
        bg_layout.addWidget(btn_bag_start)
        bg_layout.addWidget(btn_bag_stop)

        layout.addWidget(bag_group)

        layout.addStretch(1)
        return w

    def default_mission_script(self):
        # Simple example mission:
        # 1. Drive to a nav goal
        # 2. Lower arm & close gripper
        # 3. Wait, then open gripper and retract arm
        return [
            {"type": "nav", "frame": "map", "x": 1.0, "y": 0.0, "yaw_deg": 0.0,
             "description": "Drive to point (1.0, 0.0)"},
            {"type": "wait", "seconds": 10.0,
             "description": "Wait 10 seconds for navigation"},
            {"type": "arm", "joints_deg": [0, -45, 90, 0, 0],
             "description": "Pose arm down"},
            {"type": "gripper", "cmd_01": 1.0,
             "description": "Close gripper"},
            {"type": "wait", "seconds": 3.0,
             "description": "Hold object"},
            {"type": "gripper", "cmd_01": 0.0,
             "description": "Open gripper"},
            {"type": "arm", "joints_deg": [0, 0, 0, 0, 0],
             "description": "Return arm to neutral"},
        ]

    def refresh_mission_text(self):
        lines = []
        for i, step in enumerate(self.mission_script):
            desc = step.get("description", str(step))
            lines.append(f"{i}: {step['type']} – {desc}")
        self.mission_text.setPlainText("\n".join(lines))

    def start_mission(self):
        if not self.homing_done:
            rospy.logwarn("Cannot start mission, arm not homed.")
            self.mission_status_label.setText("Mission: cannot start (arm not homed)")
            return
        self.mission_active = True
        self.mission_index = 0
        self.mission_step_start_time = rospy.Time.now()
        self.mission_status_label.setText("Mission: running")

        if self.rosbag_auto:
            self.start_rosbag()

        rospy.loginfo("Mission started")

    def stop_mission(self):
        self.mission_active = False
        self.mission_status_label.setText("Mission: stopped")

        if self.rosbag_auto:
            self.stop_rosbag()

        rospy.loginfo("Mission stopped")

    def skip_step(self):
        if not self.mission_active:
            return
        self.mission_index += 1
        self.mission_step_start_time = rospy.Time.now()
        rospy.loginfo("Mission: skipped to step %d", self.mission_index)


    def update_mission(self):
        if not self.mission_active or rospy.is_shutdown():
            return

        # If hazard is active, pause mission progression
        if self.hazard_active:
            self.mission_status_label.setText(
                f"Mission: PAUSED (hazard), min dist={self.hazard_min_distance:.2f} m"
            )
            return

        if self.mission_index >= len(self.mission_script):
            self.mission_active = False
            self.mission_status_label.setText("Mission: completed")
            rospy.loginfo("Mission completed")
            # stop rosbag if auto
            if self.rosbag_auto:
                self.stop_rosbag()
            return

        step = self.mission_script[self.mission_index]
        stype = step.get("type")

        if stype == "nav":
            self.execute_nav_step(step)
            self.next_step()
        elif stype == "arm":
            self.execute_arm_step(step)
            self.next_step()
        elif stype == "gripper":
            self.execute_gripper_step(step)
            self.next_step()
        elif stype == "wait":
            self.process_wait_step(step)
        else:
            rospy.logwarn("Unknown mission step type: %s", stype)
            self.next_step()



    def next_step(self):
        self.mission_index += 1
        self.mission_step_start_time = rospy.Time.now()

    def execute_nav_step(self, step):
        x = step.get("x", 0.0)
        y = step.get("y", 0.0)
        yaw_deg = step.get("yaw_deg", 0.0)
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = step.get("frame", "map")
        goal.pose.position.x = x
        goal.pose.position.y = y
        yaw_rad = math.radians(yaw_deg)
        q = tft.quaternion_from_euler(0, 0, yaw_rad)
        goal.pose.orientation = Quaternion(*q)
        self.nav_goal_pub.publish(goal)
        rospy.loginfo("Mission nav step: (%.2f, %.2f, %.1f deg)", x, y, yaw_deg)
        self.mission_status_label.setText(f"Mission: nav to ({x:.2f}, {y:.2f})")

    def execute_arm_step(self, step):
        joints_deg = step.get("joints_deg", [0, 0, 0, 0, 0])
        if len(joints_deg) < 5:
            joints_deg = joints_deg + [0.0] * (5 - len(joints_deg))
        js = JointState()
        js.name = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
        js.position = [math.radians(d) for d in joints_deg]
        js.effort = []
        self.arm_cmd_pub.publish(js)
        rospy.loginfo("Mission arm step: %s", joints_deg)
        self.mission_status_label.setText(f"Mission: arm -> {joints_deg}")

    def execute_gripper_step(self, step):
        cmd_01 = step.get("cmd_01", 0.0)
        self.send_gripper_cmd(cmd_01)
        self.mission_status_label.setText(f"Mission: gripper -> {cmd_01:.2f}")

    def process_wait_step(self, step):
        seconds = step.get("seconds", 0.0)
        if self.mission_step_start_time is None:
            self.mission_step_start_time = rospy.Time.now()
        elapsed = (rospy.Time.now() - self.mission_step_start_time).to_sec()
        self.mission_status_label.setText(
            f"Mission: waiting {seconds:.1f}s ({elapsed:.1f}s elapsed)"
        )
        if elapsed >= seconds:
            self.next_step()

    # --- Status tab ---
    def build_status_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)

        power_group = QGroupBox("Power")
        pf = QFormLayout(power_group)
        self.lbl_voltage = QLabel("0.0 V")
        self.lbl_current = QLabel("0.0 A")
        self.lbl_power = QLabel("0.0 W")
        self.lbl_batt = QLabel("0 %")
        pf.addRow("Voltage:", self.lbl_voltage)
        pf.addRow("Current:", self.lbl_current)
        pf.addRow("Power:", self.lbl_power)
        pf.addRow("Battery:", self.lbl_batt)
        layout.addWidget(power_group)

        self.lbl_homing = QLabel("Arm homing: UNKNOWN")
        layout.addWidget(self.lbl_homing)

        odom_group = QGroupBox("Odometry (odom frame)")
        of = QFormLayout(odom_group)
        self.lbl_odom_xy = QLabel("x=0.0, y=0.0")
        self.lbl_odom_yaw = QLabel("yaw=0.0 deg")
        of.addRow("Position:", self.lbl_odom_xy)
        of.addRow("Yaw:", self.lbl_odom_yaw)
        layout.addWidget(odom_group)

        hazard_group = QGroupBox("Hazard Monitor")
        hf = QFormLayout(hazard_group)
        self.lbl_hazard = QLabel("No hazard")
        self.lbl_hazard_dist = QLabel("N/A")
        hf.addRow("Hazard:", self.lbl_hazard)
        hf.addRow("Min distance:", self.lbl_hazard_dist)
        layout.addWidget(hazard_group)

        layout.addStretch(1)
        return w

    def update_status_labels(self):
        self.lbl_voltage.setText(f"{self.last_voltage:.2f} V")
        self.lbl_current.setText(f"{self.last_current:.2f} A")
        self.lbl_power.setText(f"{self.last_power:.2f} W")
        self.lbl_batt.setText(f"{self.last_batt_pct:.1f} %")
        self.rosbag_status_label.setText(self.rosbag_status_text())

        if self.homing_done:
            self.lbl_homing.setText("Arm homing: DONE")
            self.arm_homing_label.setText("Homing: DONE")
        else:
            self.lbl_homing.setText("Arm homing: IN PROGRESS / NOT READY")
            self.arm_homing_label.setText("Homing: NOT READY")

        if self.last_odom:
            p = self.last_odom.pose.pose.position
            q = self.last_odom.pose.pose.orientation
            roll, pitch, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.lbl_odom_xy.setText(f"x={p.x:.2f}, y={p.y:.2f}")
            self.lbl_odom_yaw.setText(f"yaw={math.degrees(yaw):.1f} deg")

        if self.hazard_active:
            self.lbl_hazard.setText("ACTIVE")
            self.lbl_hazard_dist.setText(f"{self.hazard_min_distance:.2f} m")
        else:
            self.lbl_hazard.setText("Clear")
            self.lbl_hazard_dist.setText("N/A")

    # --- Hazard Settings tab ---
    def build_hazard_settings_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)

        group = QGroupBox("Hazard Detection Zone (relative to base_link)")
        g = QFormLayout(group)

        self.spin_min_x = QDoubleSpinBox()
        self.spin_min_x.setRange(0.0, 5.0)
        self.spin_min_x.setValue(0.3)

        self.spin_max_x = QDoubleSpinBox()
        self.spin_max_x.setRange(0.0, 5.0)
        self.spin_max_x.setValue(1.0)

        self.spin_max_y = QDoubleSpinBox()
        self.spin_max_y.setRange(0.0, 2.0)
        self.spin_max_y.setValue(0.5)

        self.spin_min_z = QDoubleSpinBox()
        self.spin_min_z.setRange(-2.0, 2.0)
        self.spin_min_z.setValue(-0.2)

        self.spin_max_z = QDoubleSpinBox()
        self.spin_max_z.setRange(-2.0, 2.0)
        self.spin_max_z.setValue(0.5)

        g.addRow("min_x:", self.spin_min_x)
        g.addRow("max_x:", self.spin_max_x)
        g.addRow("max_y:", self.spin_max_y)
        g.addRow("min_z:", self.spin_min_z)
        g.addRow("max_z:", self.spin_max_z)

        layout.addWidget(group)

        btn_apply = QPushButton("Apply Hazard Settings")
        btn_apply.clicked.connect(self.send_hazard_params)
        layout.addWidget(btn_apply)

        layout.addStretch(1)
        return w

    def send_hazard_params(self):
        msg = HazardParams()
        msg.min_x = self.spin_min_x.value()
        msg.max_x = self.spin_max_x.value()
        msg.max_y = self.spin_max_y.value()
        msg.min_z = self.spin_min_z.value()
        msg.max_z = self.spin_max_z.value()

        self.hazard_params_pub.publish(msg)
        rospy.loginfo("[MissionControl] Updated hazard params: %s", msg)



def main():
    app = QApplication(sys.argv)
    win = MissionControl()
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
