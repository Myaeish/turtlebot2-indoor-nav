#!/usr/bin/env python3
import time
import json
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

CALIB_FILE = '/home/uas/yaw_offset.json'

class ManualYawCalibrator(Node):
    def __init__(self):
        super().__init__('manual_yaw_calibrator')
        self.raw_yaw = None
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.raw_yaw = math.atan2(siny_cosp, cosy_cosp)

def main():
    rclpy.init()
    node = ManualYawCalibrator()

    print("====================================================")
    print("      MANUAL +X AXIS YAW CALIBRATION TOOL          ")
    print("====================================================")
    print("1. Place the robot stationary on the floor.")
    print("2. Point the FRONT BUMPER directly along the lab's +X axis")
    print("   (parallel to floor tiles / reference wall).")
    print("====================================================")
    input("Press Enter once the robot is aligned and stationary...")

    print("\nReading IMU data and settling (averaging 20 samples)...")
    samples = []
    while rclpy.ok() and len(samples) < 20:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.raw_yaw is not None:
            samples.append(node.raw_yaw)
            time.sleep(0.05)

    if not samples:
        print("[ERROR] No IMU data received on /imu/data. Is microstrain_yaw_node running?")
        node.destroy_node()
        rclpy.shutdown()
        return

    # Average angles safely
    sin_sum = sum(math.sin(s) for s in samples)
    cos_sum = sum(math.cos(s) for s in samples)
    avg_raw_yaw = math.atan2(sin_sum, cos_sum)

    # Robot is pointing at theta_room = 0 rad (+X axis)
    # theta_map = theta_imu + offset -> 0 = avg_raw_yaw + offset -> offset = -avg_raw_yaw
    offset = math.atan2(math.sin(-avg_raw_yaw), math.cos(-avg_raw_yaw))

    print("\n----------------------------------------------------")
    print(f"Raw IMU Yaw (facing +X): {math.degrees(avg_raw_yaw):.2f}°")
    print(f"Calculated Yaw Offset:   {math.degrees(offset):.2f}° ({offset:.4f} rad)")
    print("----------------------------------------------------")

    with open(CALIB_FILE, 'w') as f:
        json.dump({'yaw_offset': offset}, f, indent=2)

    print(f"Successfully saved offset to: {CALIB_FILE}")
    print("Restart fusion_node to apply the new offset.")
    print("----------------------------------------------------\n")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
