# 自己位置推定とマッピング

このチュートリアルは、**Minecraft の世界を舞台に ROS 2（Humble）で LiDAR-SLAM を動かし、ロボットの「自己位置推定」を体験**するためのものです。初心者でも迷わないように、「自己位置推定」「SLAM」の意味から、実際のコマンド実行まで順を追って説明します。


## 自己位置推定（Localization）とは？

ロボットが「**いま自分がどこにいるか**」を推定する技術です。
車輪の回転（オドメトリ）、IMU、LiDAR、カメラなどのセンサ情報を使い、**ロボットの姿勢（位置 x, y, z と向き yaw/pitch/roll）**を計算します。
地図（マップ）が**すでにある**場合は、その地図とセンサの観測を照らし合わせて自分の現在地を推定します。

> 身近なたとえ：スマホの地図アプリが、GPSやセンサを使って地図上で自分のアイコンを動かすのと同じイメージです。


## SLAM（Simultaneous Localization And Mapping）とは？

**地図がない環境**で、**地図の作成（Mapping）**と**自己位置推定（Localization）**を**同時に行う**技術です。
LiDAR-SLAM なら、LiDAR が見た周囲の形（壁や障害物）から\*\*占有グリッド地図（map）**を作りつつ、その地図上で**自分の現在地（/tf: map→base\_link）\*\*を推定し続けます。

* **Localization**：既存地図がある前提。自分の位置だけを推定。
* **SLAM**：地図がない or 変化する前提。地図と自己位置を同時に更新。

このチュートリアルでは LiDAR を使った SLAM を Minecraft の世界で体験します。


## このチュートリアルでやること（全体像）

1. ROS 2 Humble と開発ツールを整える
2. ワークスペースにサンプル群を取得・依存関係を解決してビルド
3. **Terminal 1** で SLAM を起動
4. **Terminal 2** で Minecraft クライアント（ROS 連携）を起動
5. ロボットが仮想世界を動き回ると、**/map（地図）**と**自己位置**が更新されます


## 前提（Prerequisites）

* **ROS 2 Humble**（Ubuntu 22.04 を想定）
* `git`, `vcstool`, `colcon`, `rosdep` などのビルド/取得ツール
* リポジトリ：`minecraft_ros2_example`
* パッケージ群：`minecraft_ros2` 系（`.repos` でまとめて取得）
* Minecraft 用クライアントはスクリプトから起動します（`runClient.sh`）


## セットアップ手順（Step by Step）

### 0) 開発ツールのインストール（未導入なら）

```bash
sudo apt update
sudo apt install -y git python3-vcstool python3-colcon-common-extensions python3-rosdep
# rosdep の初期化（初回のみ）
sudo rosdep init || true
rosdep update
```

### 1) ROS 2 環境を読み込む

```bash
source /opt/ros/humble/setup.bash
```

（これで `ROS_DISTRO=humble` が有効になります）

### 2) ワークスペース作成＆サンプル取得

以下のように **`~/minecraft_ros2` をワークスペースのルート**にします。

```bash
mkdir -p ~/minecraft_ros2/src
cd ~/minecraft_ros2
# サンプルの例リポジトリを取得
git clone https://github.com/minecraft-ros2/minecraft_ros2_example.git
# .repos で依存パッケージ群を src/ に一括取得
vcs import src < minecraft_ros2_example/example.repos
```

### 3) 依存関係の解決

```bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

### 4) ビルド

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```


## 実行手順（Run）

### Terminal 1：SLAM の起動

```bash
cd ~/minecraft_ros2
source ./install/setup.bash
ros2 launch minecraft_ros2_example lidarslam_ros2.launch.xml
```

* LiDAR-SLAM ノードや橋渡しノードが立ち上がります。
* `/scan`（LiDAR スキャン）, `/map`（占有グリッド地図）, `/tf`（座標変換）などが流れます。

### Terminal 2：Minecraft クライアントの起動

```bash
cd ~/minecraft_ros2
./runClient.sh
```

* Minecraft のクライアント（ROS 連携モード）が起動します。



## 動作確認のコツ

* トピック一覧：

  ```bash
  source ~/minecraft_ros2/install/setup.bash
  ros2 topic list
  ```
* 地図の更新があるか：

  ```bash
  ros2 topic echo /map --qos-durability=transient_local
  ```
* 位置推定（tf）が出ているか：

  ```bash
  ros2 run tf2_tools view_frames
  # 生成された frames.pdf を確認（map→odom→base_link 等が見えるはず）
  ```
* 可視化（任意）：`rviz2` を起動し、**Map**（/map）、**LaserScan/PointCloud**（/scan など）、**TF** を表示すると理解しやすいです。

  ```bash
  rviz2
  ```


## うまくいかないとき（よくあるつまずき）

* **`command not found: ros2/colcon`**
  → ROS 2 をインストールしていない or ターミナルで `source /opt/ros/humble/setup.bash` を忘れている。
* **`rosdep` でエラー**
  → `sudo rosdep init` と `rosdep update` を実行。プロキシ環境ならネットワーク設定を確認。
* **`runClient.sh` が無い/実行権がない**
  → 位置を確認し、`chmod +x runClient.sh` を付与。
* **ビルド失敗（依存欠如）**
  → `rosdep install ...` を再実行。`apt` のエラーは `sudo apt -f install` で修復を試す。
* **トピックが来ない**
  → Terminal 1（SLAM 側）が正常起動しているか、クライアント（Terminal 2）が接続できているかを確認。


## まとめ

* **自己位置推定**は「既存の地図の上で自分の位置を推定」
* **SLAM**は「地図がない環境で地図作成と位置推定を同時に実行」
* このチュートリアルでは、**Minecraft の仮想世界**で LiDAR-SLAM を動かし、/map と /tf を通じて **地図と自己位置**が更新される様子を体験できます。

そのまま上の手順を実行すれば動作する構成にしてあります。必要なら、RViz で可視化しながら動きを確認してみてください。
