# Minecraft\_ros2（日本語版）
[![build mod](https://github.com/minecraft-ros2/minecraft_ros2/actions/workflows/build_test.yaml/badge.svg)](https://github.com/minecraft-ros2/minecraft_ros2/actions/workflows/build_test.yaml)
[![build docker image](https://github.com/minecraft-ros2/minecraft_ros2/actions/workflows/docker_build_main.yaml/badge.svg)](https://github.com/minecraft-ros2/minecraft_ros2/actions/workflows/docker_build_main.yaml)

**minecraft_ros2** は、Minecraft と ROS 2 を統合する MOD です。Minecraft のワールドデータ（例：プレイヤーの位置やブロックの状態など）を ROS 2 にパブリッシュすることができ、RViz2 上での可視化や ROS 2 ノードとの連携が可能になります。

このリポジトリには MOD のソースコードが含まれており、Minecraft とロボットミドルウェアの研究・実験の基盤として利用できます。

---

## 動作環境

* Ubuntu 22.04
* Minecraft（Java Edition）
* ROS 2 Humble 以降
* [ros2\_java](https://github.com/minecraft-ros2/ros2_java)

---

## 🐳 Docker で簡易的に試す方法

このリポジトリの環境を手軽に試すには、Docker を使った実行が便利です。以下の手順に従ってください。
> **⚠️ 事前準備**
> Docker および GUI アプリをホストと共有するための設定（`xhost` など）が必要です。

### 手順

1. **GUI アクセスを許可**

   ```bash
   xhost +local:root
   ```
2. **リポジトリのクローン**

   ```bash
   git clone https://github.com/minecraft-ros2/minecraft_ros2.git
   cd minecraft_ros2
   ```

3. **Docker コンテナを起動**

   ```bash
   docker compose up
   ```

   このコマンドを実行すると、MinecraftとRVizが同時に起動します。RVizで点群を表示する方法については、[センサーの使い方](#センサーの使い方)をご覧ください。

4. 権限の復元
   ```bash
   xhost -local:root
   ```

## ソースインストール手順

### 1. ros2\_java のビルド

以下のリポジトリの手順に従って `ros2_java` をビルドしてください：

👉 [https://github.com/minecraft-ros2/ros2\_java](https://github.com/minecraft-ros2/ros2_java)

### 2. 環境変数の設定

`.bashrc` などのシェル設定ファイルに以下のように `ros2_java` の install ディレクトリを指定してください：

```bash
export ROS2JAVA_INSTALL_PATH=/home/USERNAME/ros2_java_ws/install
```

編集後は次のコマンドで反映させます：

```bash
source ~/.bashrc
```

### 3. Minecraft の起動

本リポジトリに含まれる以下のスクリプトを実行して、MOD付きの Minecraft を起動します：

```bash
./runClient.sh
```

### 4. RViz2 での可視化

RViz2 を起動し、`minecraft.rviz` を読み込んで Minecraft のデータを可視化します：

```bash
rviz2 -d minecraft.rviz
```

---

## センサーの使い方

「minecraft\_ros2」で提供されているLiDARセンサーは、防具のヘルメットとして実装されています。クリエイティブモードでは、「戦闘」タブの一番最後に追加されており、プレイヤーがこれを装備することで、点群データのパブリッシュが自動的に開始されます。

![lidar_2](/docs/images/lidar_2.png)

![lidar_1](/docs/images/lidar_1.png)

---

## ライセンス

このプロジェクトは Apache License 2.0　でライセンスされています。

---

## コントリビューション

貢献は大歓迎です！Issue や Pull Request を自由に送ってください。

---

## 作者

* [kazu-321](https://github.com/kazu-321)

---

Velodyne VLP-16 is a registered trademark of Ouster Lidar, Inc

Hesai and XT-32 is a registered trademark of Hesai Technology, Inc
