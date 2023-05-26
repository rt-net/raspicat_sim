# raspicat_sim

Gazebo上でシミュレートできるRaspberry Pi CatのROS 2パッケージ一式です。

![raspicat_sim](https://rt-net.github.io/images/raspberry-pi-cat/raspicat_gazebo_with_iscas_museum.gif)

## Requirements

- Linux OS
  - [Ubuntu Desktop 22.04](https://ubuntu.com/download/desktop)
- ROS 2
  - [Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html)
- Gazebo
  - Gazebo 11.x

## Installation
### Source Build

```sh
# パッケージのダウンロード
cd ~/catkin_ws/src
git clone -b ros2 https://github.com/rt-net/raspicat_sim.git
git clone -b ros2 https://github.com/rt-net/raspicat_description.git
git clone -b ros2 https://github.com/rt-net/raspicat_ros.git
git clone -b $ROS_DISTRO-devel https://github.com/rt-net/raspimouse2

# 依存パッケージのインストール
rosdep update
rosdep install -r -y -i --from-paths raspicat* raspimouse*

# ビルド＆インストール
cd ~/catkin_ws
colcon build --symlink-install
source ~/catkin_ws/install/setup.bash
```

## Package Overview
### raspicat_gazebo

Gazeboシミュレータを立ち上げるためのパッケージです。

## Usage

### raspicat_gazebo

* Gazeboのみ立ち上げ
```
roslaunch raspicat_gazebo raspicat_with_iscas_museum.launch rviz:=false
```

* GazeboとRVizの立ち上げ
```
roslaunch raspicat_gazebo raspicat_with_iscas_museum.launch
```

__SLAMやNavigationのソフトウェアと組み合わせる方法など、より詳しい使い方については[RT Software Tutorials](https://rt-net.github.io/tutorials/raspicat/)を参照してください。__



## License

(C) 2020 RT Corporation \<support@rt-net.jp\>

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。  
ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。

※このソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。本ソフトウェアに関する無償サポートはありません。  
バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。
