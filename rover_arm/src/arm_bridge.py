#!/usr/bin/env python3
import rospy
import serial
import struct

from sensor_msgs.msg import JointState
from rover_msgs.msg import ArmJointStates
from std_msgs.msg import Header

class ArmBridge:
    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyACM1")
        baud = rospy.get_param("~baud", 115200)
        timeout = rospy.get_param("~timeout", 0.05)

        self.port_name = port
        self.baud = baud
        self.timeout = timeout

        self.ser = None
        self.connect_serial()

        self.cmd_sub = rospy.Subscriber("/arm/joint_cmd", JointState, self.cmd_cb, queue_size=1)
        self.state_pub = rospy.Publisher("/arm/joint_states", ArmJointStates, queue_size=10)

        self.rate = rospy.Rate(100)

        # Protocol formats
        self.tx_fmt = struct.Struct("<6f")    # 5 joints + 1 gripper
        self.rx_fmt = struct.Struct("<5f6B")  # 5 pos + 5 homed + 1 gripper flag

    # -------------------------
    # Serial
    # -------------------------
    def connect_serial(self):
        while not rospy.is_shutdown():
            try:
                rospy.loginfo(f"[arm_bridge] Opening serial {self.port_name} @ {self.baud}")
                self.ser = serial.Serial(self.port_name, self.baud, timeout=self.timeout)
                rospy.loginfo("[arm_bridge] Serial connected")
                return
            except serial.SerialException as e:
                rospy.logwarn_throttle(5.0, f"[arm_bridge] Serial open failed: {e}")
                rospy.sleep(1.0)

    def safe_write(self, data: bytes):
        if self.ser is None or not self.ser.is_open:
            self.connect_serial()
        try:
            self.ser.write(data)
        except serial.SerialException as e:
            rospy.logwarn_throttle(1.0, f"[arm_bridge] Serial write error: {e}")
            self.connect_serial()

    def safe_read(self, size: int) -> bytes:
        if self.ser is None or not self.ser.is_open:
            self.connect_serial()
        try:
            return self.ser.read(size)
        except serial.SerialException as e:
            rospy.logwarn_throttle(1.0, f"[arm_bridge] Serial read error: {e}")
            self.connect_serial()
            return b""

    # -------------------------
    # ROS → Arduino
    # -------------------------
    def cmd_cb(self, msg: JointState):
        # Expect up to 5 joint positions
        joints = list(msg.position[:5])
        while len(joints) < 5:
            joints.append(0.0)

        # Gripper command from effort[0] (or position if you prefer)
        if msg.effort:
            gripper_cmd = msg.effort[0]
        else:
            gripper_cmd = 0.0

        # Pack and send
        try:
            payload = self.tx_fmt.pack(*(joints + [gripper_cmd]))
        except struct.error as e:
            rospy.logerr_throttle(1.0, f"[arm_bridge] Pack error: {e}")
            return

        packet = b"A" + payload
        self.safe_write(packet)

    # -------------------------
    # Arduino → ROS
    # -------------------------
    def read_feedback_once(self):
        # Find header 'a'
        header = self.safe_read(1)
        if len(header) == 0 or header != b"a":
            return

        payload = self.safe_read(self.rx_fmt.size)
        if len(payload) != self.rx_fmt.size:
            return

        try:
            unpacked = self.rx_fmt.unpack(payload)
        except struct.error as e:
            rospy.logwarn_throttle(1.0, f"[arm_bridge] Unpack error: {e}")
            return

        pos = unpacked[0:5]
        flags = unpacked[5:11]   # 5 homed + 1 gripper_closed
        homed = [bool(f) for f in flags[0:5]]
        gripper_closed = bool(flags[5])

        msg = ArmJointStates()
        msg.header = Header(stamp=rospy.Time.now())
        msg.joint_position = list(pos)
        msg.joint_velocity = [0.0] * 5  # can be computed later if needed
        msg.joint_homed = homed
        msg.gripper_closed = gripper_closed
        self.state_pub.publish(msg)

    def spin(self):
        while not rospy.is_shutdown():
            self.read_feedback_once()
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("arm_bridge")
    bridge = ArmBridge()
    bridge.spin()
