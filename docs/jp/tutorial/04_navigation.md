# Navigation
このチュートリアルではNavigation2を用いて2Dの自動運転を行います

以下の内容が学べます
- Navigation2の具体的な動き
- Navigation2の使い方

## Navigation2とは
Navigation2（Nav2）は、ROS 2でモバイルロボットに自律走行（Navigation）機能を与えるためのパッケージ群です。
自己位置推定、経路計画、障害物回避、経路追従などを統合して、ロボットを指定した目的地まで自動的に移動させることができます。

主な用途：
- 室内の移動ロボット（配送ロボット、掃除ロボットなど）
- 研究用ロボットの自律移動
- ロボコンや実験プロジェクトでの自律走行
- マイクラのプレーヤーの自動操作

## Navigation2の仕組み
Navigation2は主に以下のレイヤーに分かれています。

### Costmap
マップやLiDARなどから周囲の環境をコストマップとして表現します。
コストマップがあることで障害物までの距離などがわかり、障害物回避や適切な経路生成ができるようになります。

### Planner
コストマップやロボットのあたり判定などから経路を生成します。
スタートからゴールまでどういう道を通るかというのがわかるようになります。

### Controller
プランナーが生成した経路を追従します。
経路情報や自己位置から速度司令を計算し出力することで、経路に従ってロボットなどを走らせることができます

### Behavior Tree
目的地まで移動するための全体フローを制御します。
タスクをxml形式のツリー構造で記述することで運転以外のこともできるようになります。

## 前提

* **ROS 2 Humble**（Ubuntu 22.04 を想定）
* `git`, `vcstool`, `colcon`, `rosdep` などのビルド/取得ツール
* リポジトリ：`minecraft_ros2_example`
* パッケージ群：`minecraft_ros2` 系（`.repos` でまとめて取得）
* Minecraft 用クライアントはスクリプトから起動します（`runClient.sh`）


## セットアップ手順

### 0. 開発ツールのインストール（未導入なら）

```bash
sudo apt update
sudo apt install -y git python3-vcstool python3-colcon-common-extensions python3-rosdep
# rosdep の初期化（初回のみ）
sudo rosdep init || true
rosdep update
```

### 1. ROS 2 環境を読み込む

```bash
source /opt/ros/humble/setup.bash
```

（これで `ROS_DISTRO=humble` が有効になります）

### 2. ワークスペース作成＆サンプル取得

[サンプルコードのセットアップ](/jp/documentation/setup_sample) を参考にサンプルコードの準備をしてください


## 実行手順

### Terminal 1： Nav2の起動

```bash
source ~/ros2_java_ws/install/setup.bash
source ~/minecraft_ros2_example_ws/install/setup.bash
ros2 launch minecraft_ros2_example nav2_bringup.launch.xml
```
- Nav2関連のノードが立ち上がります
- world -> player のTFを参照し自動運転ができるようになっています

### Terminal 2：Minecraft クライアントの起動

```bash
cd ~/minecraft_ros2
./runClient.sh
```

- Minecraftが起動したらスーパーフラットのワールドで壁を作成、LiDARを装備してください
- 正しくcostmapが出ていれば成功です
- Rviz2の `2D Goal Pose` で目標位置を送信することで自動運転を開始できます