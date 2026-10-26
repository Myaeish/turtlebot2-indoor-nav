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

        self.declare_parameter('yaw_offset', load_yaw_offset())
        self.yaw_offset = self.get_parameter('yaw_offset').get_parameter_value().double_value

        self.raw_imu_yaw = 0.0
        self.have_imu = False

        self.last_x = None
        self.last_y = None
        self.last_time = None

        self.odom_pub = self.create_publisher(Odometry, '/marvelmind_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 1. IMU for Heading (orientation)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        # 2. Marvelmind for Position (X, Y)
        self.create_subscription(HedgePosition, '/hedgehog_pos', self.hedge_callback, 10)

        self.get_logger().info(
            f'IMU Fusion Node active. Yaw offset: {math.degrees(self.yaw_offset):.2f} deg'
        )

≈    def imu_callback(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        # Negate yaw so positive CCW turn strictly increases angle
        self.raw_imu_yaw = -math.atan2(siny_cosp, cosy_cosp)
        self.have_imu = True

    def hedge_callback(self, msg: HedgePosition):
        if not self.have_imu:
            self.get_logger().warn('Waiting for /imu/data...', throttle_duration_sec=2.0)
            return

        now = self.get_clock().now()
        now_msg = now.to_msg()

        # Standard calibration offset and wrap to [-pi, pi]
        yaw = self.raw_imu_yaw + self.yaw_offset
        yaw = (yaw + math.pi) % (2.0 * math.pi) - math.pi

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        # Estimate forward velocity for Nav2 controller tracking
        v_linear = 0.0
        if self.last_time is not None:
            dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
            if dt > 0.005:
                vx = (float(msg.x_m) - self.last_x) / dt
                vy = (float(msg.y_m) - self.last_y) / dt
                v_linear = vx * math.cos(yaw) + vy * math.sin(yaw)

        self.last_x = float(msg.x_m)
        self.last_y = float(msg.y_m)
        self.last_time = now

        # Publish Odometry (/marvelmind_odom)
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = float(msg.x_m)
        odom.pose.pose.position.y = float(msg.y_m)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(v_linear)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0
        self.odom_pub.publish(odom)

        # Broadcast map -> base_footprint transform
        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = float(msg.x_m)
        t.transform.translation.y = float(msg.y_m)
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

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
