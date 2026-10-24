# agv_factory — AIで安全を判断する工場内搬送AGV

ROS2 Humble + Ignition Gazebo Fortress で動く、差動二輪の工場内搬送ロボット（AGV）です。
ステーションを順に回って部品を搬送し、カメラ画像を YOLO で解析して、進路上の作業者を検知すると減速・保護停止します。

ベースライン：「gazeboとROS2を連携してロボットを操作する」（Qiita, @N622）の
差動二輪・ros_ign_bridge・launch 構成。

## 工程

| 順 | 場所 | 色 | 座標 | 作業 |
|---|---|---|---|---|
| 1 | PARTS | 青 | (4, 3) | 部品を積み込み |
| 2 | PROCESS | 黄 | (4, -3) | 加工ステーションへ受け渡し |
| 3 | SHIP | 緑 | (-4, -3) | 製品を荷降ろし |
| 4 | HOME | 灰 | (-4, 3) | 待機位置に帰還（初期位置） |

## ノード構成

```
/camera/image ─▶ detector (YOLOv8n) ─▶ /perception/persons ─▶ mission ─▶ /cmd_vel ─▶ Gazebo
                        └─▶ /perception/image（検出枠つき画像）     ▲
/odom ─────────────────────────────────────────────────────────┘
teleop_key ─▶ /agv/mode (manual/auto), /cmd_vel（手動時）
```

- **detector**：人の検出枠の下端（足元）と地面の交点から距離を推定（距離 = カメラ高さ / tan(足元への俯角)。足元が見えないときは身長1.7mから推定）し、横ずれから 進路上 / 警告エリア / 進路外 に区分
- **mission**：工程の状態管理と自律走行。安全レベル CLEAR / CAUTION / SLOW / STOP / NO_AI で速度を制限
  - CAUTION：進路のすぐ脇（警告エリア ±1.5m）の3m以内に作業者 → 通路に入ってくる前に減速
  - NO_AI：AIの結果が途絶えたらフェイルセーフで停止
- **teleop_key**：`m` で自動運転、移動キーで手動に割り込み、`space` で停止

## セットアップ

```bash
sudo apt install -y xterm ros-humble-ros-ign-gazebo ros-humble-ros-ign-bridge ros-humble-rqt-image-view
python3 -m pip install --user torch torchvision --index-url https://download.pytorch.org/whl/cpu
python3 -m pip install --user ultralytics "numpy<2"
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

`numpy<2` は ROS2 Humble のメッセージ処理との互換性のため。

## 起動

```bash
ros2 launch agv_factory factory.launch.py           # 全機能
ros2 launch agv_factory factory.launch.py ai:=false # 手動操作のみ
ros2 run rqt_image_view rqt_image_view /perception/image   # AIの認識結果を表示
```

## うまくいかないとき

- カメラ画像が来ない・真っ黒：`worlds/factory.sdf` の `ogre2` を `ogre` に変更して再ビルド
- モデルが2台出る・変更が反映されない：`pkill -f "ign gazebo"` してから再起動
- `numpy` 関連のエラー：`python3 -m pip install --user "numpy<2"` を再実行
