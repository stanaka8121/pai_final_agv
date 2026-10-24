"""キーボードで AGV を操作するノード（手動モード ⇔ 自動モード切替）

サンプル記事では外部パッケージ key_event を使っていたが、
本プロジェクトでは依存を減らすため termios で直接キー入力を読む。

  m        : 自動運転（AI搬送）を開始・再開
  移動キー : 手動操作に切り替えて移動（自動運転中でも人が割り込める）
  space    : 停止して手動モードへ（非常停止）
"""
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

HELP = """
---------------------------------------
  AGV 操作 (このウィンドウを選択して入力)
---------------------------------------
   m         : 自動運転 開始 / 再開
     q   w   e        w/s : 前進 / 後退
     a   s   d        a/d : 左旋回 / 右旋回
                      q/e : 左前 / 右前
   移動キー  : 手動操作に切替えて移動
   space / x : 停止（手動モードへ）
   + / -     : 手動の速度アップ / ダウン
   Ctrl-C    : 終了
---------------------------------------
"""

# キー -> (直進方向, 旋回方向)
MOVES = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    'q': (1.0, 1.0),
    'e': (1.0, -1.0),
    ' ': (0.0, 0.0),
    'x': (0.0, 0.0),
}


class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_mode = self.create_publisher(String, '/agv/mode', latched)
        self.linear_speed = 0.4   # m/s
        self.angular_speed = 0.8  # rad/s
        self.mode = None
        self.set_mode('manual')

    def set_mode(self, mode):
        if mode != self.mode:
            self.mode = mode
            self.pub_mode.publish(String(data=mode))
            print(f'\r\n [MODE] {mode.upper()}\r\n', end='', flush=True)

    def send(self, lin_dir, ang_dir):
        msg = Twist()
        msg.linear.x = lin_dir * self.linear_speed
        msg.angular.z = ang_dir * self.angular_speed
        self.pub.publish(msg)
        return msg


def read_key(timeout=0.1):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if ready else ''


def main():
    rclpy.init()
    node = TeleopKey()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP.replace('\n', '\r\n'))
    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key()
            if key == '\x03':  # Ctrl-C
                break
            if key == 'm':
                node.set_mode('auto')
            elif key in MOVES:
                node.set_mode('manual')  # 手動の割り込みが常に優先
                msg = node.send(*MOVES[key])
                print(f'\r linear={msg.linear.x:+.2f}  angular={msg.angular.z:+.2f}   ',
                      end='', flush=True)
            elif key in ('+', '-'):
                scale = 1.2 if key == '+' else 1 / 1.2
                node.linear_speed = min(0.6, node.linear_speed * scale)
                node.angular_speed = min(1.5, node.angular_speed * scale)
                print(f'\r speed: {node.linear_speed:.2f} m/s, '
                      f'{node.angular_speed:.2f} rad/s   ', end='', flush=True)
            rclpy.spin_once(node, timeout_sec=0)
    finally:
        node.set_mode('manual')
        node.send(0.0, 0.0)  # 終了時は必ず停止
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
