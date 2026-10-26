#!/usr/bin/env python3
import math
import os
import json
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from marvelmind_ros2_msgs.msg import HedgePosition
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

CALIB_FILE = '/home/uas/yaw_offset.json'

def load_yaw_offset():
    if os.path.exists(CALIB_FILE):
        try:
            with open(CALIB_FILE, 'r') as f:
                data = json.load(f)
                return float(data.get('yaw_offset', 0.0))
        except Exception:
            pass
    return 0.0

class MarvelmindIMUFusionNode(Node):
    def __init__(self):
        super().__init__('marvelmind_fusion')

        # Always read fresh directly from disk on launch
        self.yaw_offset = load_yaw_offset()

        self.raw_imu_yaw = 0.0
        self.have_imu = False

        self.odom_pub = self.create_publisher(Odometry, '/marvelmind_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(HedgePosition, '/hedgehog_pos', self.hedge_callback, 10)

        self.get_logger().info(
            f'IMU Fusion Node active. Loaded offset: {math.degrees(self.yaw_offset):.2f}° ({self.yaw_offset:.4f} rad)'
        )

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.raw_imu_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.have_imu = True

    def hedge_callback(self, msg: HedgePosition):
        if not self.have_imu:
            return

        now_msg = self.get_clock().now().to_msg()

        pos_x = float(msg.x_m)
        pos_y = float(msg.y_m)

        yaw = self.raw_imu_yaw + self.yaw_offset
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = pos_x
        t.transform.translation.y = pos_y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = pos_x
        odom.pose.pose.position.y = pos_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = MarvelmindIMUFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
