# agv_factory — AIで判断する工場内搬送AGV

ROS2 Humble + Ignition Gazebo Fortress で動く、差動二輪の工場内搬送ロボット（AGV）です。
ベースライン：「gazeboとROS2を連携してロボットを操作する」（Qiita, @N622）の
差動二輪・ros_ign_bridge・launch 構成。

## 現在の段階：第1段階（ワールド・AGV・キー操作）

| 場所 | 色 | 座標 | 役割 |
|---|---|---|---|
| HOME | 灰 | (-4, 3) | 待機・充電（初期位置） |
| PARTS | 青 | (4, 3) | 部品置き場 |
| PROCESS | 黄 | (4, -3) | 加工ステーション |
| SHIP | 緑 | (-4, -3) | 出荷エリア |

## ビルドと起動

```bash
sudo apt install -y xterm ros-humble-ros-ign-gazebo ros-humble-ros-ign-bridge
mkdir -p ~/ros2_ws/src && cp -r agv_factory ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --symlink-install
source install/setup.bash
ros2 launch agv_factory factory.launch.py
```

xterm のウィンドウを選択して w/a/s/d で操作します。

## 動作確認

```bash
ros2 topic hz /odom            # 約20Hz
ros2 topic hz /camera/image    # 約10Hz（CPUでは下回ることあり）
ros2 run rqt_image_view rqt_image_view /camera/image
```

## うまくいかないとき

- カメラ画像が来ない・真っ黒：`worlds/factory.sdf` の `<render_engine>ogre2</render_engine>` を `ogre` に変更して再ビルド
- モデルが2台出る・変更が反映されない：Gazebo のゾンビプロセス。`pkill -f "ign gazebo"` してから再起動
