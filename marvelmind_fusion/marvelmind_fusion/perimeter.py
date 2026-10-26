import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from marvelmind_ros2_msgs.msg import BeaconPositionAddressed
import math

class PerimeterRun(Node):
    def __init__(self):
        super().__init__('perimeter_run')
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.has_position = False
        self.x_min = float('inf')
        self.x_max = float('-inf')
        self.y_min = float('inf')
        self.y_max = float('-inf')
        self.beacons_seen = set()
        self.bounds_ready = False
        self.margin = 0.4
        self.waypoints = []
        self.current_waypoint = 0
        self.running = False
        self.last_check_x = 0.0
        self.last_check_y = 0.0
        self.stuck_timer = 0
        self.stuck_check_interval = 50
        self.stuck_counter = 0
        self.escaping = False
        self.escape_timer = 0
        self.cmd_pub = self.create_publisher(Twist, '/commands/velocity', 10)
        self.create_subscription(Odometry, '/marvelmind_odom', self.odom_callback, 10)
        self.create_subscription(BeaconPositionAddressed, '/beacons_pos_addressed', self.beacon_callback, 10)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Perimeter node started! Waiting for beacon positions...')

    def beacon_callback(self, msg):
        if self.bounds_ready:
            return
        self.beacons_seen.add(msg.address)
        self.x_min = min(self.x_min, msg.x_m)
        self.x_max = max(self.x_max, msg.x_m)
        self.y_min = min(self.y_min, msg.y_m)
        self.y_max = max(self.y_max, msg.y_m)
        if len(self.beacons_seen) >= 4:
            self.bounds_ready = True
            mx = self.margin
            self.waypoints = [
                (self.x_min + mx, self.y_max - mx),
                (self.x_max - mx, self.y_max - mx),
                (self.x_max - mx, self.y_min + mx),
                (self.x_min + mx, self.y_min + mx),
                (self.x_min + mx, self.y_max - mx),
            ]
            self.last_check_x = self.x
            self.last_check_y = self.y
            self.get_logger().info(f'Bounds detected! x:[{self.x_min:.2f}, {self.x_max:.2f}] y:[{self.y_min:.2f}, {self.y_max:.2f}]')
            self.get_logger().info(f'Waypoints: {self.waypoints}')
            self.running = True

    def odom_callback(self, msg):
        self.has_position = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if not self.has_position or not self.running:
            return

        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('Perimeter complete!')
            self.stop()
            self.running = False
            return

        # Escape maneuver
        if self.escaping:
            self.escape_timer += 1
            cmd = Twist()
            if self.escape_timer < 10:
                cmd.linear.x = -0.15
                cmd.angular.z = 0.3
            elif self.escape_timer < 20:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5
            else:
                self.escaping = False
                self.escape_timer = 0
                self.stuck_counter = 0
                self.last_check_x = self.x
                self.last_check_y = self.y
                self.get_logger().info('Escape complete, resuming...')
            self.cmd_pub.publish(cmd)
            return

        # Stuck detection
        self.stuck_timer += 1
        if self.stuck_timer >= self.stuck_check_interval:
            self.stuck_timer = 0
            dist_moved = math.sqrt(
                (self.x - self.last_check_x)**2 +
                (self.y - self.last_check_y)**2
            )
            if dist_moved < 0.15:
                self.stuck_counter += 1
                self.get_logger().warn(f'Possible stuck! Moved {dist_moved:.3f}m (count: {self.stuck_counter})')
                if self.stuck_counter >= 2:
                    self.get_logger().warn('STUCK! Escaping...')
                    self.escaping = True
                    self.escape_timer = 0
            else:
                self.stuck_counter = 0
            self.last_check_x = self.x
            self.last_check_y = self.y

        goal_x, goal_y = self.waypoints[self.current_waypoint]
        dx = goal_x - self.x
        dy = goal_y - self.y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.2:
            self.get_logger().info(f'Waypoint {self.current_waypoint + 1}/{len(self.waypoints)} reached!')
            self.current_waypoint += 1
            self.stuck_counter = 0
            return

        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.yaw
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        cmd = Twist()
        if abs(angle_error) > 0.2:
            cmd.angular.z = 0.4 * angle_error
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = min(0.2, 0.3 * distance)
            cmd.angular.z = 0.3 * angle_error
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = PerimeterRun()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
