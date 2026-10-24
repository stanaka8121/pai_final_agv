"""搬送工程の管理と自律走行ノード

工程: HOME → PARTS(積み込み) → PROCESS(加工へ受け渡し) → SHIP(荷降ろし) → HOME(待機)
  - /odom で自己位置を知り、ステーションへ向かう（その場旋回 → 直進）
  - /perception/persons（YOLOの結果）で安全レベルを判定し速度を制限
      CLEAR : 通常走行
      SLOW  : 進路上に作業者（遠い）→ 減速
      STOP  : 進路上に作業者（近い）→ 保護停止。いなくなって一定時間後に再開
      NO_AI : AIの結果が途絶えた → フェイルセーフで停止
  - /agv/mode が "auto" のときだけ /cmd_vel を出す（手動操作と共存）
"""
import json
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

# (名前, ワールド座標x, y, 到着時の作業内容, 作業ログ)
ROUTE = [
    ('PARTS', 4.0, 3.0, 'LOADING', '部品置き場で部品を積み込み'),
    ('PROCESS', 4.0, -3.0, 'HANDOVER', '加工ステーションへ部品を受け渡し'),
    ('SHIP', -4.0, -3.0, 'UNLOADING', '出荷エリアで製品を荷降ろし'),
    ('HOME', -4.0, 3.0, 'STANDBY', '待機位置に帰還'),
]
SPEED_SCALE = {'CLEAR': 1.0, 'SLOW': 0.4, 'STOP': 0.0, 'NO_AI': 0.0}
SAFETY_LOG = {
    'CLEAR': '進路クリア：通常走行',
    'SLOW': '進路上に作業者を検知：減速',
    'STOP': '作業者が接近：保護停止',
    'NO_AI': 'AI認識が途絶：フェイルセーフ停止',
}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Mission(Node):
    def __init__(self):
        super().__init__('mission')
        defaults = {
            'start_x': -4.0, 'start_y': 3.0, 'start_yaw': 0.0,     # AGVの初期位置（SDFと一致）
            'max_linear': 0.4, 'max_angular': 0.8, 'goal_tolerance': 0.15,
            'dwell_time': 3.0,                                     # 各ステーションでの作業時間 [s]
            'slow_distance': 4.5, 'stop_distance': 2.5,            # 減速・停止の距離 [m]
            'perception_timeout': 1.5, 'resume_delay': 1.0,
            'cycles': 0,                                           # 0 = 無限に繰り返す
        }
        for name, val in defaults.items():
            self.declare_parameter(name, val)
        self.p = {n: self.get_parameter(n).value for n in defaults}

        self.mode = 'manual'
        self.pose = None              # (x, y, yaw) ワールド座標
        self.idx = 0                  # 次に向かうステーション
        self.state = 'DRIVING'        # DRIVING / WORKING / DONE
        self.work_end = None
        self.cycle = 0
        self.safety = 'NO_AI'
        self.last_perception = None
        self.persons = []
        self.last_stop_time = 0.0
        self.nearest = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/agv/mode', self.on_mode, latched)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(String, '/perception/persons', self.on_persons, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/agv/status', 10)
        self.create_timer(0.1, self.control)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info('工程管理ノード起動。キー操作ウィンドウで m を押すと自動運転を開始')

    # ---------- 入力 ----------
    def on_mode(self, msg):
        if msg.data == self.mode:
            return
        self.mode = msg.data
        self.get_logger().info(f'モード切替: {self.mode}')
        if self.mode == 'auto':
            self.get_logger().info(f'自動運転開始 → 次の目的地 {ROUTE[self.idx][0]}')

    def on_odom(self, msg):
        # DiffDrive のオドメトリは起動位置が原点なので、ワールド座標に変換する
        ox, oy = msg.pose.pose.position.x, msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        oyaw = 2.0 * math.atan2(q.z, q.w)
        c, s = math.cos(self.p['start_yaw']), math.sin(self.p['start_yaw'])
        self.pose = (self.p['start_x'] + c * ox - s * oy,
                     self.p['start_y'] + s * ox + c * oy,
                     wrap(self.p['start_yaw'] + oyaw))

    def on_persons(self, msg):
        self.last_perception = self.now()
        self.persons = json.loads(msg.data).get('persons', [])

    # ---------- 判断 ----------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def evaluate_safety(self):
        t = self.now()
        if self.last_perception is None or t - self.last_perception > self.p['perception_timeout']:
            raw = 'NO_AI'
        else:
            in_path = [q for q in self.persons if q['in_path']]
            self.nearest = min((q['dist'] for q in in_path), default=None)
            close = any(q['truncated'] or q['dist'] < self.p['stop_distance'] for q in in_path)
            if close:
                raw = 'STOP'
            elif self.nearest is not None and self.nearest < self.p['slow_distance']:
                raw = 'SLOW'
            else:
                raw = 'CLEAR'
        if raw in ('STOP', 'NO_AI'):
            self.last_stop_time = t
        elif self.safety in ('STOP', 'NO_AI') and t - self.last_stop_time < self.p['resume_delay']:
            raw = self.safety  # 安全側に倒すため、少し待ってから再開
        if raw != self.safety:
            dist = f'（{self.nearest:.1f}m）' if raw in ('SLOW', 'STOP') and self.nearest else ''
            self.get_logger().info(f'[安全] {SAFETY_LOG[raw]}{dist}')
            self.safety = raw

    # ---------- 制御ループ ----------
    def control(self):
        self.evaluate_safety()
        if self.mode != 'auto' or self.pose is None or self.state == 'DONE':
            return
        cmd = Twist()
        name, gx, gy, task, log = ROUTE[self.idx]

        if self.state == 'WORKING':
            if self.now() >= self.work_end:
                self.idx = (self.idx + 1) % len(ROUTE)
                if self.idx == 0:
                    self.cycle += 1
                    self.get_logger().info(f'=== 搬送サイクル {self.cycle} 回完了 ===')
                    if self.p['cycles'] and self.cycle >= self.p['cycles']:
                        self.state = 'DONE'
                        self.get_logger().info('指定回数の搬送が完了しました')
                        self.pub_cmd.publish(cmd)
                        return
                self.state = 'DRIVING'
                self.get_logger().info(f'出発 → {ROUTE[self.idx][0]}')
            self.pub_cmd.publish(cmd)
            return

        x, y, yaw = self.pose
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        if dist < self.p['goal_tolerance']:
            self.state = 'WORKING'
            self.work_end = self.now() + self.p['dwell_time']
            self.get_logger().info(f'到着 {name}：{log}')
            self.pub_cmd.publish(cmd)
            return

        err = wrap(math.atan2(dy, dx) - yaw)
        if abs(err) > 0.35:      # 向きが大きくずれている → その場旋回
            lin, ang = 0.0, 1.5 * err
        else:                    # ほぼ正面 → 進みながら向きを補正
            lin, ang = min(self.p['max_linear'], 0.5 * dist + 0.05), 2.0 * err
        ang = max(-self.p['max_angular'], min(self.p['max_angular'], ang))

        scale = SPEED_SCALE[self.safety]
        cmd.linear.x = lin * scale
        cmd.angular.z = ang if scale > 0 else 0.0
        self.pub_cmd.publish(cmd)

    def publish_status(self):
        name, _, _, task, _ = ROUTE[self.idx]
        if self.state == 'WORKING':
            step = f'{task}@{name}'
        elif self.state == 'DONE':
            step = 'DONE'
        else:
            step = f'-> {name}'
        near = f' {self.nearest:.1f}m' if self.safety in ('SLOW', 'STOP') and self.nearest else ''
        text = f'{self.mode.upper()} | {step} | {self.safety}{near} | cycle {self.cycle}'
        self.pub_status.publish(String(data=text))


def main():
    rclpy.init()
    node = Mission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
