#!/usr/bin/env python3
"""
Waypoint Navigation with Automated Bounding-Box Obstacle Avoidance
Routes around measured static obstacles using geometry-based detour waypoints.
"""
import sys
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

def line_intersects_box(p1, p2, box):
    """
    Checks if a line segment between (x1, y1) and (x2, y2) crosses
    the buffered obstacle box [x_min, x_max, y_min, y_max].
    """
    x1, y1 = p1
    x2, y2 = p2
    bx_min, bx_max, by_min, by_max = box

    # Sample 50 points along the straight-line trajectory
    steps = 50
    for i in range(steps + 1):
        t = i / float(steps)
        xi = x1 + t * (x2 - x1)
        yi = y1 + t * (y2 - y1)
        if bx_min <= xi <= bx_max and by_min <= yi <= by_max:
            return True
    return False

class ObstacleAvoidanceNavigator(Node):
    def __init__(self, final_x: float, final_y: float):
        super().__init__('obstacle_avoidance_navigator')
        
        self.final_goal = (float(final_x), float(final_y))
        self.waypoints = []
        self.current_wp_idx = 0
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_odom = False
        self.route_planned = False
        self.all_reached = False

        # =========================================================================
        # 1. DEFINE YOUR PHYSICAL OBSTACLE & DIMENSIONS HERE (IN METERS)
        # =========================================================================
        # Center of the obstacle in the Marvelmind room frame:
        self.obs_center_x = 2.8
        self.obs_center_y = -2.95

        # Physical size of the object (Length along X, Width along Y):
        self.obs_length_x = 0.5  # e.g., 50 cm wide box
        self.obs_width_y  = 0.5  # e.g., 50 cm long box

        # Safety inflation (robot base radius ~0.18m + safe clearance ~0.17m):
        self.safety_margin = 0.50 

        # Calculate the buffered keep-out boundary
        half_x = (self.obs_length_x / 2.0) + self.safety_margin
        half_y = (self.obs_width_y / 2.0) + self.safety_margin
        self.box = [
            self.obs_center_x - half_x,  # x_min
            self.obs_center_x + half_x,  # x_max
            self.obs_center_y - half_y,  # y_min
            self.obs_center_y + half_y   # y_max
        ]
        # =========================================================================

        self.cmd_pub = self.create_publisher(Twist, '/commands/velocity', 10)
        self.create_subscription(Odometry, '/marvelmind_odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("==================================================")
        self.get_logger().info("     GEOMETRIC OBSTACLE AVOIDANCE INITIALIZED     ")
        self.get_logger().info(f"Target Destination: ({self.final_goal[0]:.2f}, {self.final_goal[1]:.2f})")
        self.get_logger().info(
            f"Obstacle at ({self.obs_center_x:.2f}, {self.obs_center_y:.2f}) | "
            f"Size: {self.obs_length_x:.2f}m x {self.obs_width_y:.2f}m"
        )
        self.get_logger().info(
            f"Buffered Keep-Out Zone: X: [{self.box[0]:.2f}, {self.box[1]:.2f}], "
            f"Y: [{self.box[2]:.2f}, {self.box[3]:.2f}]"
        )
        self.get_logger().info("==================================================")

    def plan_corridor(self):
        """Checks for intersection and plans an automated detour if blocked."""
        start = (self.current_x, self.current_y)
        goal = self.final_goal

        if not line_intersects_box(start, goal, self.box):
            self.get_logger().info("Direct path is clear! Driving straight to goal.")
            self.waypoints = [goal]
        else:
            self.get_logger().warn("DIRECT PATH INTERSECTS OBSTACLE! Computing detour...")
            
            # Candidate detour waypoints (corners outside the buffered box)
            x_min, x_max, y_min, y_max = self.box
            
            # Pick side with shortest total distance
            # Detour Option A: Pass around the lower Y side
            detour_a = (self.obs_center_x, y_min - 0.25)
            # Detour Option B: Pass around the upper Y side
            detour_b = (self.obs_center_x, y_max + 0.25)

            dist_a = math.hypot(detour_a[0] - start[0], detour_a[1] - start[1]) + \
                     math.hypot(goal[0] - detour_a[0], goal[1] - detour_a[1])
            dist_b = math.hypot(detour_b[0] - start[0], detour_b[1] - start[1]) + \
                     math.hypot(goal[0] - detour_b[0], goal[1] - detour_b[1])

            chosen_detour = detour_a if dist_a < dist_b else detour_b
            self.waypoints = [chosen_detour, goal]
            
            self.get_logger().info(
                f"Generated 2-Leg Route: Detour -> ({chosen_detour[0]:.2f}, {chosen_detour[1]:.2f}) "
                f"then Goal -> ({goal[0]:.2f}, {goal[1]:.2f})"
            )

        self.route_planned = True

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
        if not self.has_odom or self.all_reached:
            return

        if not self.route_planned:
            self.plan_corridor()
            return

        target_x, target_y = self.waypoints[self.current_wp_idx]
        is_final_goal = (self.current_wp_idx == len(self.waypoints) - 1)

        dx = target_x - self.current_x
        dy = target_y - self.current_y
        dist = math.hypot(dx, dy)

        # Handoff threshold: 0.25m for intermediate detour, 0.15m for final target
        threshold = 0.15 if is_final_goal else 0.25

        if dist < threshold:
            if is_final_goal:
                self.stop_robot()
                self.all_reached = True
                self.get_logger().info(
                    f"MISSION COMPLETE! Final Pos: ({self.current_x:.2f}, {self.current_y:.2f}) | Error: {dist:.2f}m"
                )
                return
            else:
                self.get_logger().info(
                    f"--> Cleared Detour Waypoint {self.current_wp_idx + 1}. Turning toward final destination..."
                )
                self.current_wp_idx += 1
                return

        target_yaw = math.atan2(dy, dx)
        err_yaw = normalize_angle(target_yaw - self.current_yaw)

        cmd = Twist()

        # Pure turn-in-place until aligned within 10 degrees (prevents arcing)
        if abs(err_yaw) > math.radians(10.0):
            turn_speed = max(0.24, min(0.42, 1.1 * abs(err_yaw)))
            cmd.angular.z = float(turn_speed if err_yaw > 0 else -turn_speed)
            cmd.linear.x = 0.0
        else:
            base_speed = float(min(0.20, max(0.09, 0.45 * dist)))
            cmd.linear.x = float(base_speed * max(0.2, math.cos(err_yaw)))
            cmd.angular.z = float(max(-0.30, min(0.30, 1.4 * err_yaw)))

        self.cmd_pub.publish(cmd)

        wp_label = "FINAL GOAL" if is_final_goal else "DETOUR"
        self.get_logger().info(
            f"[{wp_label}] Target: ({target_x:.2f}, {target_y:.2f}) | "
            f"Pos: ({self.current_x:.2f}, {self.current_y:.2f}) | "
            f"Err: {math.degrees(err_yaw):.1f}° | Dist: {dist:.2f}m",
            throttle_duration_sec=0.3
        )

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 goto_goal_obstacle.py <goal_x> <goal_y>")
        return
    rclpy.init()
    node = ObstacleAvoidanceNavigator(float(sys.argv[1]), float(sys.argv[2]))
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
