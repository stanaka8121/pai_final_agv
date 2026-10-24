"""YOLO による作業者検知ノード（AI推論）

入力 : /camera/image (sensor_msgs/Image)
出力 : /perception/persons (std_msgs/String, JSON)
         検出した人ごとに 推定距離[m]・横方向のずれ[m]・区分(path/warn/side)
       /perception/image   (sensor_msgs/Image) 検出枠を描いた確認・動画用の画像

距離推定（主）：足元（検出枠の下端）と地面の交点から求める
          足元への俯角 = カメラの下向き角 + atan((枠下端y - 画像中心y) / 焦点距離)
          距離 = カメラ高さ / tan(俯角)
          → 近づくと頭が画面上にはみ出しても、足元が見えていれば正確に測れる
距離推定（予備）：足元が見えないときは身長1.7mを仮定し
          距離 = 身長 × 焦点距離[px] / 検出枠の高さ[px]
進路判定：検出枠の中心の横ずれを実距離に換算し、AGVの走行幅(±corridor)に入るか
"""
import json
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class Detector(Node):
    def __init__(self):
        super().__init__('detector')
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('conf', 0.4)
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('rate', 5.0)               # 推論の最大頻度 [Hz]
        self.declare_parameter('person_height', 1.7)      # [m]
        self.declare_parameter('hfov', 1.2)               # カメラ水平画角 [rad]（SDFと一致させる）
        self.declare_parameter('corridor_half_width', 0.6)  # 進路とみなす横幅の半分 [m]
        self.declare_parameter('warning_half_width', 1.5)  # 警告エリアの横幅の半分 [m]
        self.declare_parameter('camera_height', 0.38)    # 床からカメラまでの高さ [m]（SDFと一致）
        self.declare_parameter('camera_pitch', 0.15)      # カメラの下向き角 [rad]（SDFと一致）

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.conf = gp('conf')
        self.imgsz = gp('imgsz')
        self.person_h = gp('person_height')
        self.hfov = gp('hfov')
        self.corridor = gp('corridor_half_width')
        self.warn_width = gp('warning_half_width')
        self.cam_h = gp('camera_height')
        self.pitch = gp('camera_pitch')

        self.get_logger().info(f'YOLO モデル読み込み中: {gp("model")}（初回は自動ダウンロード）')
        self.model = YOLO(gp('model'))
        self.model.predict(np.zeros((240, 320, 3), np.uint8), imgsz=self.imgsz, verbose=False)
        self.get_logger().info('YOLO 準備完了')

        self.latest = None
        self.status_text = ''
        self.create_subscription(Image, '/camera/image', self.on_image, qos_profile_sensor_data)
        self.create_subscription(String, '/agv/status', self.on_status, 10)
        self.pub_persons = self.create_publisher(String, '/perception/persons', 10)
        self.pub_image = self.create_publisher(Image, '/perception/image', 10)
        self.create_timer(1.0 / gp('rate'), self.process)

    def on_image(self, msg):
        self.latest = msg  # 最新フレームだけ保持（推論が遅くても遅延が溜まらない）

    def on_status(self, msg):
        self.status_text = msg.data

    def process(self):
        msg = self.latest
        if msg is None:
            return
        self.latest = None
        w, h = msg.width, msg.height
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, msg.step)[:, :w * 3]
        img = img.reshape(h, w, 3)
        bgr = np.ascontiguousarray(img[:, :, ::-1] if msg.encoding == 'rgb8' else img)

        t0 = time.time()
        result = self.model.predict(bgr, imgsz=self.imgsz, conf=self.conf, verbose=False)[0]
        infer_ms = (time.time() - t0) * 1000

        f = (w / 2) / math.tan(self.hfov / 2)  # 焦点距離 [px]（正方画素）
        persons = []
        for box in result.boxes:
            name = result.names[int(box.cls)]
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            conf = float(box.conf)
            if name != 'person':
                self.draw(bgr, x1, y1, x2, y2, f'{name} {conf:.2f}', (180, 180, 180))
                continue
            feet_visible = y2 < h - 2
            depression = self.pitch + math.atan((y2 - h / 2) / f)  # 足元への俯角
            if feet_visible and depression > 0.02:
                dist = self.cam_h / math.tan(depression)          # 地面との交点
            else:
                dist = self.person_h * f / max(y2 - y1, 1.0)       # 予備：身長から
            truncated = not feet_visible   # 足元まではみ出す＝すぐ目の前
            lateral = ((x1 + x2) / 2 - w / 2) * dist / f
            in_path = abs(lateral) < self.corridor or (x1 < w / 2 < x2)
            # 区分: path=進路上 / warn=進路のすぐ脇（入ってくる可能性） / side=十分離れている
            if in_path:
                zone = 'path'
            elif abs(lateral) < self.warn_width:
                zone = 'warn'
            else:
                zone = 'side'
            persons.append({
                'conf': round(conf, 2), 'dist': round(dist, 2),
                'lateral': round(lateral, 2), 'in_path': in_path, 'zone': zone,
                'truncated': truncated,
            })
            color = {'path': (0, 0, 255), 'warn': (0, 165, 255), 'side': (0, 200, 0)}[zone]
            tag = {'path': 'IN PATH', 'warn': 'NEAR', 'side': 'side'}[zone]
            self.draw(bgr, x1, y1, x2, y2, f'person {dist:.1f}m {tag}', color)

        self.pub_persons.publish(String(data=json.dumps({'infer_ms': round(infer_ms, 1),
                                                          'persons': persons})))
        self.publish_overlay(bgr, msg, infer_ms)

    @staticmethod
    def draw(img, x1, y1, x2, y2, label, color):
        p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
        cv2.rectangle(img, p1, p2, color, 2)
        cv2.putText(img, label, (p1[0], max(p1[1] - 4, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)

    def publish_overlay(self, bgr, src, infer_ms):
        h, w = bgr.shape[:2]
        cv2.rectangle(bgr, (0, 0), (w, 16), (0, 0, 0), -1)
        cv2.putText(bgr, f'{self.status_text}  [{infer_ms:.0f}ms]', (3, 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1, cv2.LINE_AA)
        out = Image()
        out.header = src.header
        out.height, out.width = h, w
        out.encoding = 'rgb8'
        out.step = w * 3
        out.data = np.ascontiguousarray(bgr[:, :, ::-1]).tobytes()
        self.pub_image.publish(out)


def main():
    rclpy.init()
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
