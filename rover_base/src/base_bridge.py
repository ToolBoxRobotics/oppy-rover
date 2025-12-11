#!/usr/bin/env python3
import rospy
import serial
import struct
import math

from geometry_msgs.msg import Twist
from rover_msgs.msg import WheelStates, PowerStatus
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

class BaseBridge:
    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud = rospy.get_param("~baud", 115200)
        timeout = rospy.get_param("~timeout", 0.05)

        self.port_name = port
        self.baud = baud
        self.timeout = timeout

        self.ser = None
        self.connect_serial()

        self.cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb, queue_size=1)
        self.ws_pub = rospy.Publisher("/rover/wheel_states", WheelStates, queue_size=10)
        self.imu_pub = rospy.Publisher("/rover/imu", Imu, queue_size=10)
        self.power_pub = rospy.Publisher("/rover/power", PowerStatus, queue_size=10)

        self.rate = rospy.Rate(100)  # run loop at 100 Hz

        # Kinematic params (adjust to your rover)
        self.wheel_base = rospy.get_param("~wheel_base", 0.6)
        self.track_width = rospy.get_param("~track_width", 0.6)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.125)

        # Precomputed struct formats
        # 6 wheel speeds + 4 steering angles
        self.tx_fmt = struct.Struct("<6f4f")
        # 6 pos, 6 vel, 4 steer, 4 quat, 3 gyro, 3 accel, 3 power
        self.rx_fmt = struct.Struct("<6f6f4f4f3f3f3f")

    # -------------------------
    # Serial handling
    # -------------------------
    def connect_serial(self):
        while not rospy.is_shutdown():
            try:
                rospy.loginfo(f"[base_bridge] Opening serial {self.port_name} @ {self.baud}")
                self.ser = serial.Serial(self.port_name, self.baud, timeout=self.timeout)
                rospy.loginfo("[base_bridge] Serial connected")
                return
            except serial.SerialException as e:
                rospy.logwarn_throttle(5.0, f"[base_bridge] Serial open failed: {e}")
                rospy.sleep(1.0)

    def safe_write(self, data: bytes):
        if self.ser is None or not self.ser.is_open:
            self.connect_serial()
        try:
            self.ser.write(data)
        except serial.SerialException as e:
            rospy.logwarn_throttle(1.0, f"[base_bridge] Serial write error: {e}")
            self.connect_serial()

    def safe_read(self, size: int) -> bytes:
        if self.ser is None or not self.ser.is_open:
            self.connect_serial()
        try:
            data = self.ser.read(size)
            return data
        except serial.SerialException as e:
            rospy.logwarn_throttle(1.0, f"[base_bridge] Serial read error: {e}")
            self.connect_serial()
            return b""

    # -------------------------
    # ROS callbacks
    # -------------------------
    def cmd_cb(self, msg: Twist):
        vx = msg.linear.x
        wz = msg.angular.z

        # Basic Ackermann-like steering
        if abs(wz) < 1e-3 or abs(vx) < 1e-3:
            steering = [0.0, 0.0, 0.0, 0.0]
        else:
            R = vx / wz
            half_track = self.track_width / 2.0
            # front-left, front-right, rear-left, rear-right
            steering = [
                math.atan2(self.wheel_base, R - half_track),
                math.atan2(self.wheel_base, R + half_track),
                -math.atan2(self.wheel_base, R - half_track),
                -math.atan2(self.wheel_base, R + half_track),
            ]

        # Wheel speeds: simple approximation (all same)
        wheel_speeds = [vx] * 6

        # Build packet: 'C' + 10 floats
        try:
            payload = self.tx_fmt.pack(*(wheel_speeds + steering))
        except struct.error as e:
            rospy.logerr_throttle(1.0, f"[base_bridge] Pack error: {e}")
            return

        packet = b"C" + payload
        self.safe_write(packet)

    # -------------------------
    # Feedback parsing
    # -------------------------
    def read_feedback_once(self):
        # Look for header 'F'
        header = self.safe_read(1)
        if len(header) == 0 or header != b"F":
            return

        payload = self.safe_read(self.rx_fmt.size)
        if len(payload) != self.rx_fmt.size:
            # incomplete packet
            return

        try:
            unpacked = self.rx_fmt.unpack(payload)
        except struct.error as e:
            rospy.logwarn_throttle(1.0, f"[base_bridge] Unpack error: {e}")
            return

        # Slice data
        idx = 0
        wpos = unpacked[idx:idx+6]; idx += 6
        wvel = unpacked[idx:idx+6]; idx += 6
        steering = unpacked[idx:idx+4]; idx += 4
        quat = unpacked[idx:idx+4]; idx += 4
        gyro = unpacked[idx:idx+3]; idx += 3
        accel = unpacked[idx:idx+3]; idx += 3
        V, I, P = unpacked[idx:idx+3]; idx += 3

        now = rospy.Time.now()

        # WheelStates
        ws = WheelStates()
        ws.header = Header(stamp=now)
        ws.wheel_position = list(wpos)
        ws.wheel_velocity = list(wvel)
        ws.steering_angle = list(steering)
        ws.wheel_current = [0.0] * 6
        self.ws_pub.publish(ws)

        # IMU
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "imu_link"
        imu.orientation.w = quat[0]
        imu.orientation.x = quat[1]
        imu.orientation.y = quat[2]
        imu.orientation.z = quat[3]
        imu.angular_velocity.x = gyro[0]
        imu.angular_velocity.y = gyro[1]
        imu.angular_velocity.z = gyro[2]
        imu.linear_acceleration.x = accel[0]
        imu.linear_acceleration.y = accel[1]
        imu.linear_acceleration.z = accel[2]
        self.imu_pub.publish(imu)

        # Power
        ps = PowerStatus()
        ps.header.stamp = now
        ps.bus_voltage = V
        ps.bus_current = I
        ps.bus_power = P
        # Battery % can be estimated on Pi side if desired
        ps.battery_percent = 0.0
        self.power_pub.publish(ps)

    def spin(self):
        while not rospy.is_shutdown():
            self.read_feedback_once()
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("base_bridge")
    bridge = BaseBridge()
    bridge.spin()
