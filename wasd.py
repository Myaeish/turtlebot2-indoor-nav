#!/usr/bin/env python3
"""WASD teleop for Kobuki. Publishes to /commands/velocity at 20Hz."""
import sys, tty, termios, select, threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

LIN_STEP = 0.05
ANG_STEP = 0.2
LIN_MAX  = 0.5
ANG_MAX  = 1.5
TIMEOUT  = 0.4   # stop if no key held this long

HELP = """
  W / S : forward / back      A / D : turn left / right
  SPACE : stop                Q     : quit
  - / = : slower / faster
"""

class Wasd(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.pub = self.create_publisher(Twist, '/commands/velocity', 10)
        self.lin = self.ang = 0.0
        self.lin_sp, self.ang_sp = 0.15, 0.5
        self.create_timer(0.05, self.tick)   # 20 Hz keeps watchdog fed

    def tick(self):
        t = Twist()
        t.linear.x, t.angular.z = self.lin, self.ang
        self.pub.publish(t)

    def stop(self):
        self.lin = self.ang = 0.0
        self.tick()

def main():
    rclpy.init()
    n = Wasd()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    old = termios.tcgetattr(sys.stdin)
    print(HELP)
    print(f"speed: lin={n.lin_sp:.2f} ang={n.ang_sp:.2f}\r")
    try:
        tty.setraw(sys.stdin.fileno())
        import time
        last_key = 0.0
        while True:
            if select.select([sys.stdin], [], [], 0.05)[0]:
                k = sys.stdin.read(1).lower()
                last_key = time.time()
                if   k == 'w': n.lin, n.ang = -n.lin_sp, 0.0
                elif k == 's': n.lin, n.ang =  n.lin_sp, 0.0
                elif k == 'a': n.lin, n.ang =  0.0,  n.ang_sp
                elif k == 'd': n.lin, n.ang =  0.0, -n.ang_sp
                elif k == ' ': n.stop()
                elif k == '=':
                    n.lin_sp = min(LIN_MAX, n.lin_sp + LIN_STEP)
                    n.ang_sp = min(ANG_MAX, n.ang_sp + ANG_STEP)
                    print(f"speed: lin={n.lin_sp:.2f} ang={n.ang_sp:.2f}\r")
                elif k == '-':
                    n.lin_sp = max(0.05, n.lin_sp - LIN_STEP)
                    n.ang_sp = max(0.10, n.ang_sp - ANG_STEP)
                    print(f"speed: lin={n.lin_sp:.2f} ang={n.ang_sp:.2f}\r")
                elif k == 'q': break
                elif k == '\x03': break     # Ctrl-C
            elif time.time() - last_key > TIMEOUT:
                n.stop()                    # dead-man: release key = stop
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        n.stop()
        print("\r\nstopped")

if __name__ == '__main__':
    main()
