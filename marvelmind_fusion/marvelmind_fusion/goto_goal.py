#!/usr/bin/env python3
import math
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

class GoToGoalIMU(Node):
    def __init__(self, target_x: float, target_y: float):
        super().__init__('goto_goal')
        self.target_x = float(target_x)
        self.target_y = float(target_y)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_odom = False
        self.reached = False
        self.turn_direction = 0.0  # Direction latch to prevent 180-deg oscillation

        # Publish to /commands/velocity directly with enough torque
        self.cmd_pub = self.create_publisher(Twist, '/commands/velocity', 10)
        self.create_subscription(Odometry, '/marvelmind_odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info(f'Target set: ({self.target_x:.2f}, {self.target_y:.2f})')

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.has_odom = True

    def stop_robot(self):
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass

    def control_loop(self):
        if not self.has_odom or self.reached:
            return

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        dist = math.hypot(dx, dy)

        if dist < 0.15:
            self.stop_robot()
            self.reached = True
            self.get_logger().info('Goal reached!')
            return

        target_yaw = math.atan2(dy, dx)
        err_yaw = normalize_angle(target_yaw - self.current_yaw)

        cmd = Twist()

        # Step 1: Align within 6 degrees before moving forward
        if abs(err_yaw) > math.radians(6.0):
            # Proportional turning speed with minimum friction floor clamp
            turn_speed = max(0.25, min(0.55, 1.2 * abs(err_yaw)))
            cmd.angular.z = float(turn_speed if err_yaw > 0 else -turn_speed)
            cmd.linear.x = 0.0
        else:
            # Step 2: Drive straight with active dynamic heading lock
            # Scale forward speed down if distance is small or if there is heading error
            base_speed = float(min(0.22, max(0.06, 0.4 * dist)))
            
            # Reduce linear speed proportionally if pointing slightly off
            heading_factor = max(0.2, math.cos(err_yaw))
            cmd.linear.x = float(base_speed * heading_factor)

            # High-authority steering gain keeps travel on a straight line
            cmd.angular.z = float(max(-0.4, min(0.4, 1.8 * err_yaw)))

        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'Pos: ({self.current_x:.2f}, {self.current_y:.2f}) | '
            f'Yaw: {math.degrees(self.current_yaw):.1f}° | '
            f'TargetYaw: {math.degrees(target_yaw):.1f}° | '
            f'Err: {math.degrees(err_yaw):.1f}° | '
            f'Dist: {dist:.2f}m',
            throttle_duration_sec=0.25
        )

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 goto_goal.py <x> <y>")
        return
    rclpy.init()
    node = GoToGoalIMU(sys.argv[1], sys.argv[2])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
