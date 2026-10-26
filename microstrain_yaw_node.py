#!/usr/bin/env python3
"""
Microstrain 3DM-GQ4-45 yaw node.

Reads MIP stream directly from /dev/ttyACM0 (no driver, no handshake).
  - Integrates scaled gyro (0x80/0x05) for instant, lag-free yaw
  - Slowly corrects drift from complementary-filter heading (0x80/0x0C),
    but ONLY while the robot is stationary and the CF has settled
Publishes sensor_msgs/Imu on /imu/data
"""
import math, struct, time
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

PORT       = '/dev/imu'
BAUD       = 115200
CAL_SECS   = 10.0     # stationary calibration window
STILL_RATE = 0.01     # rad/s below which we consider the robot stationary
STILL_TIME = 2.0      # seconds of stillness before trusting CF
CORRECT_K  = 0.0     # fraction of CF/gyro error applied per sample


def checksum_ok(pkt):
    a = b = 0
    for c in pkt[:-2]:
        a = (a + c) & 0xFF
        b = (b + a) & 0xFF
    return bytes([a, b]) == pkt[-2:]


def yaw_to_quat(yaw):
    return (math.cos(yaw / 2.0), math.sin(yaw / 2.0))  # (w, z)


class MicrostrainYaw(Node):
    def __init__(self):
        super().__init__('microstrain_yaw')
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.ser = serial.Serial(PORT, BAUD, timeout=1.0)
        self.buf = b''

        self.bias = 0.0
        self.yaw = 0.0
        self.gz = 0.0
        self.cf_yaw = None
        self.last_t = None
        self.still_since = None
        self.calibrated = False
        self.cal_samples = []
        self.cal_start = time.time()

        self.get_logger().info(f'Calibrating gyro bias — hold still for {CAL_SECS:.0f}s...')
        self.create_timer(0.005, self.spin_serial)

    def spin_serial(self):
        try:
            n = self.ser.in_waiting
            if n > 0:
                self.buf += self.ser.read(n)
        except (serial.SerialException, OSError):
            pass

        while True:
            i = self.buf.find(b'\x75\x65')
            if i < 0 or len(self.buf) < i + 4:
                return
            plen = self.buf[i + 3]
            total = 4 + plen + 2
            if len(self.buf) < i + total:
                return
            pkt = self.buf[i:i + total]
            self.buf = self.buf[i + total:]
            if checksum_ok(pkt):
                self.handle_packet(pkt[2], pkt[4:4 + plen])

    def handle_packet(self, dset, payload):
        if dset != 0x80:
            return
        j = 0
        got_gyro = False
        while j < len(payload) - 1:
            flen, fdesc = payload[j], payload[j + 1]
            if flen < 2:
                break
            fdata = payload[j + 2:j + flen]
            if fdesc == 0x05 and len(fdata) >= 12:          # scaled gyro rad/s
                _, _, self.gz = struct.unpack('>fff', fdata[:12])
                got_gyro = True
            elif fdesc == 0x0C and len(fdata) >= 12:        # CF euler rad
                _, _, self.cf_yaw = struct.unpack('>fff', fdata[:12])
            j += flen
        if got_gyro:
            self.step()

    def step(self):
        now = time.time()

        if not self.calibrated:
            self.cal_samples.append(self.gz)
            if now - self.cal_start >= CAL_SECS:
                self.bias = sum(self.cal_samples) / len(self.cal_samples)
                self.calibrated = True
                self.last_t = now
                if self.cf_yaw is not None:
                    self.yaw = self.cf_yaw
                self.get_logger().info(
                    f'Bias = {self.bias:+.6f} rad/s ({math.degrees(self.bias):+.3f} deg/s), '
                    f'{len(self.cal_samples)} samples. Initial yaw = '
                    f'{math.degrees(self.yaw):+.2f} deg. Publishing.')
            return

        dt = now - self.last_t
        self.last_t = now
        if dt <= 0 or dt > 0.5:
            return

        rate = self.gz - self.bias
        self.yaw -= rate * dt

        # drift correction, only while genuinely still
        if abs(rate) < STILL_RATE:
            if self.still_since is None:
                self.still_since = now
            elif (now - self.still_since) > STILL_TIME and self.cf_yaw is not None:
                err = math.atan2(math.sin(self.cf_yaw - self.yaw),
                                 math.cos(self.cf_yaw - self.yaw))
                self.yaw += CORRECT_K * err
        else:
            self.still_since = None

        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.publish(rate)

    def publish(self, rate):
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'imu_link'
        w, z = yaw_to_quat(self.yaw)
        m.orientation.w, m.orientation.z = w, z
        m.orientation.x = m.orientation.y = 0.0
        m.angular_velocity.z = rate
        m.orientation_covariance[0] = -1.0   # roll/pitch not provided
        m.orientation_covariance[4] = -1.0
        m.orientation_covariance[8] = 0.01
        m.linear_acceleration_covariance[0] = -1.0
        self.pub.publish(m)


def main():
    rclpy.init()
    n = MicrostrainYaw()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.ser.close()
    n.destroy_node()


if __name__ == '__main__':
    main()
