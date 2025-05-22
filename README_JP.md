# Minecraft\_ros2（日本語版）
[![build mod](https://github.com/minecraft-ros2/minecraft_ros2/actions/workflows/build_test.yaml/badge.svg)](https://github.com/minecraft-ros2/minecraft_ros2/actions/workflows/build_test.yaml)
[![build docker image](https://github.com/minecraft-ros2/minecraft_ros2/actions/workflows/docker_build.yaml/badge.svg)](https://github.com/minecraft-ros2/minecraft_ros2/actions/workflows/docker_build.yaml)

**minecraft_ros2** は、Minecraft と ROS 2 を統合する MOD です。Minecraft のワールドデータ（例：プレイヤーの位置やブロックの状態など）を ROS 2 にパブリッシュすることができ、RViz2 上での可視化や ROS 2 ノードとの連携が可能になります。

このリポジトリには MOD のソースコードが含まれており、Minecraft とロボットミドルウェアの研究・実験の基盤として利用できます。

---

## 動作環境

* Ubuntu 22.04
* Minecraft（Java Edition）
* ROS 2 Humble 以降
* [ros2\_java](https://github.com/minecraft-ros2/ros2_java)

---

## インストール手順

### 1. Minecraft のインストール

Ubuntu 22.04 に Minecraft Java Edition をインストールし、正常に動作することを確認してください。

### 2. ros2\_java のビルド

以下のリポジトリの手順に従って `ros2_java` をビルドしてください：

👉 [https://github.com/minecraft-ros2/ros2\_java](https://github.com/minecraft-ros2/ros2_java)

### 3. 環境変数の設定

`.bashrc` などのシェル設定ファイルに以下のように `ros2_java` の install ディレクトリを指定してください：

```bash
export ROS2JAVA_INSTALL_PATH=/home/USERNAME/ros2_java_ws/install
```

編集後は次のコマンドで反映させます：

```bash
source ~/.bashrc
```

### 4. Minecraft の起動

本リポジトリに含まれる以下のスクリプトを実行して、MOD付きの Minecraft を起動します：

```bash
./runClient.sh
```

### 5. RViz2 での可視化

RViz2 を起動し、`minecraft.rviz` を読み込んで Minecraft のデータを可視化します：

```bash
rviz2 -d minecraft.rviz
```

---

## ライセンス

このプロジェクトは GNU Lesser General Public License v2.1（LGPL-2.1）でライセンスされています。

---

## コントリビューション

貢献は大歓迎です！Issue や Pull Request を自由に送ってください。

---

## 作者

* [kazu-321](https://github.com/kazu-321)

---
