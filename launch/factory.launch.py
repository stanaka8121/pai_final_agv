"""工場ワールド + AGV + bridge + キー操作 を一括起動する launch ファイル

使い方:
  ros2 launch agv_factory factory.launch.py            # 全部起動（キー操作 + AI + 自動搬送）
  ros2 launch agv_factory factory.launch.py ai:=false  # 第1段階と同じ（手動操作のみ）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('agv_factory')
    world_path = os.path.join(pkg_share, 'worlds', 'factory.sdf')
    bridge_cfg = os.path.join(pkg_share, 'config', 'bridge.yaml')

    use_teleop = LaunchConfiguration('teleop')

    # Gazebo がワールド内の資源を探せるようにする
    resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.join(pkg_share, 'worlds'),
    )

    # Ignition Gazebo (Fortress) を起動。-r で開始ボタンを押さずに再生
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_ign_gazebo'),
                         'launch', 'ign_gazebo.launch.py')),
        launch_arguments=[('ign_args', f'-r -v 3 {world_path}')],
    )

    # ROS2 <-> Gazebo のメッセージ変換
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_cfg, 'use_sim_time': True}],
        output='screen',
    )

    # キー操作ノード（ログと混ざらないよう xterm の別ウィンドウで起動）
    teleop = Node(
        package='agv_factory',
        executable='teleop_key',
        prefix='xterm -fa Monospace -fs 12 -geometry 60x20 -e',
        output='screen',
        condition=IfCondition(use_teleop),
    )

    use_ai = LaunchConfiguration('ai')

    # YOLO 作業者検知（AI推論）
    detector = Node(
        package='agv_factory',
        executable='detector',
        output='screen',
        condition=IfCondition(use_ai),
    )

    # 搬送工程の管理と自律走行
    mission = Node(
        package='agv_factory',
        executable='mission',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_ai),
    )

    return LaunchDescription([
        DeclareLaunchArgument('teleop', default_value='true',
                              description='キー操作ノードを起動するか'),
        DeclareLaunchArgument('ai', default_value='true',
                              description='AI検知と自動搬送ノードを起動するか'),
        resource_path,
        gazebo,
        bridge,
        teleop,
        detector,
        mission,
    ])
