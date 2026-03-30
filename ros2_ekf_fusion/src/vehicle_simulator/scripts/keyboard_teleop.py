#!/usr/bin/env python3
"""
Keyboard teleoperation for vehicle simulator.
  W/S : increase/decrease speed
  A/D : steer left/right
  Q/E : max steer left/right
  SPACE : brake (speed → 0)
  R   : reset (speed=0, steer=0)
  X   : reverse
  +/- : adjust speed step
  1-5 : preset speeds (2/4/6/10/14 m/s)
  ESC / Ctrl-C : quit
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, tty, termios, threading, math

SPEED_PRESETS = [2.0, 4.0, 6.0, 10.0, 14.0]

BANNER = """
╔══════════════════════════════════════════════════╗
║         ARAÇ KEYBOARD KONTROLÜ                   ║
╠══════════════════════════════════════════════════╣
║  W / S   : Hız artır / azalt                    ║
║  A / D   : Sol / Sağ direksiyon                 ║
║  Q / E   : Maksimum direksiyon sol/sağ           ║
║  SPACE   : Fren (hız → 0)                       ║
║  R       : Reset (hız=0, direksiyon=0)          ║
║  X       : Geri vites                           ║
║  +/-     : Hız adımını değiştir                 ║
║  1-5     : Preset hızlar (2/4/6/10/14 m/s)      ║
║  ESC     : Çıkış                                ║
╚══════════════════════════════════════════════════╝
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('max_speed', 15.0)
        self.declare_parameter('max_steering', 0.52)
        self.declare_parameter('speed_step', 0.5)
        self.declare_parameter('steer_step', 0.05)
        self.declare_parameter('publish_rate', 20.0)

        self.max_speed    = self.get_parameter('max_speed').value
        self.max_steer    = self.get_parameter('max_steering').value
        self.speed_step   = self.get_parameter('speed_step').value
        self.steer_step   = self.get_parameter('steer_step').value
        self.pub_rate     = self.get_parameter('publish_rate').value

        self.target_speed = 0.0
        self.target_steer = 0.0
        self._lock = threading.Lock()
        self._running = True

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0 / self.pub_rate, self.publish_cmd)

        print(BANNER)
        self._key_thread = threading.Thread(target=self._key_loop, daemon=True)
        self._key_thread.start()

    def publish_cmd(self):
        msg = Twist()
        with self._lock:
            msg.linear.x  = self.target_speed
            msg.angular.z = self.target_steer
        self.pub.publish(msg)
        self._print_status()

    def _print_status(self):
        with self._lock:
            speed = self.target_speed
            steer = self.target_steer
        steer_deg = math.degrees(steer)
        bar_len = 20
        # Speed bar
        s_frac = abs(speed) / self.max_speed
        s_filled = int(s_frac * bar_len)
        s_bar = ('█' * s_filled).ljust(bar_len)
        # Steer indicator
        center = bar_len // 2
        st_frac = steer / self.max_steer
        st_pos = int(center + st_frac * center)
        st_bar = [' '] * (bar_len + 1)
        st_bar[center] = '│'
        st_pos = max(0, min(bar_len, st_pos))
        st_bar[st_pos] = '▼'
        st_str = ''.join(st_bar)

        direction = "→ İLERİ" if speed > 0 else ("← GERİ" if speed < 0 else "■ DURUYOR")
        sys.stdout.write(
            f"\r  Hız: {speed:+6.2f} m/s ({speed*3.6:+6.1f} km/h) [{s_bar}] {direction}"
            f"  |  Direksiyon: {steer_deg:+6.1f}°  [{st_str}]    "
        )
        sys.stdout.flush()

    def _getkey(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch

    def _key_loop(self):
        while self._running:
            key = self._getkey()
            with self._lock:
                if key == 'w' or key == 'W':
                    self.target_speed = min(self.target_speed + self.speed_step, self.max_speed)
                elif key == 's' or key == 'S':
                    self.target_speed = max(self.target_speed - self.speed_step, -self.max_speed)
                elif key == 'a' or key == 'A':
                    self.target_steer = min(self.target_steer + self.steer_step, self.max_steer)
                elif key == 'd' or key == 'D':
                    self.target_steer = max(self.target_steer - self.steer_step, -self.max_steer)
                elif key == 'q' or key == 'Q':
                    self.target_steer = self.max_steer
                elif key == 'e' or key == 'E':
                    self.target_steer = -self.max_steer
                elif key == ' ':
                    self.target_speed = 0.0
                elif key == 'r' or key == 'R':
                    self.target_speed = 0.0
                    self.target_steer = 0.0
                elif key == 'x' or key == 'X':
                    self.target_speed = -abs(self.target_speed) if self.target_speed != 0 else -2.0
                elif key == '+' or key == '=':
                    self.speed_step = min(self.speed_step + 0.1, 2.0)
                    print(f"\n  Hız adımı: {self.speed_step:.1f} m/s")
                elif key == '-' or key == '_':
                    self.speed_step = max(self.speed_step - 0.1, 0.1)
                    print(f"\n  Hız adımı: {self.speed_step:.1f} m/s")
                elif key in '12345':
                    idx = int(key) - 1
                    self.target_speed = SPEED_PRESETS[idx]
                elif key == '\x1b' or key == '\x03':
                    self._running = False
                    print("\n\nÇıkılıyor...")
                    rclpy.shutdown()
                    return


def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()


if __name__ == '__main__':
    main()
