# ソースインストール

## 動作環境

- Ubuntu 22.04
- ROS 2 Humble

## ビルドツールのインストール

1. **Gradle をインストールする**

    ```bash
    sudo apt install gradle
    ```
    ::: info
    Gradle 3.2 以降がインストールされていることを確認してください。
    :::

2. **必要なツールをインストールする**

    ```bash
    sudo apt install curl python3-colcon-common-extensions python3-pip python3-vcstool
    ```

3. **colcon 用の Gradle 拡張をインストールする**

    ```bash
    python3 -m pip install -U git+https://github.com/colcon/colcon-gradle
    python3 -m pip install --no-deps -U git+https://github.com/colcon/colcon-ros-gradle
    ```

## ros2_javaのセットアップ

1. **ROS 2 Java リポジトリをワークスペースにダウンロードする**

    ```bash
    mkdir -p ros2_java_ws/src
    cd ros2_java_ws
    curl -skL https://raw.githubusercontent.com/minecraft-ros2/ros2_java/main/ros2_java_desktop.repos | vcs import src
    ```

2. **ROS の依存関係をインストールする**

    ```bash
    rosdep install --from-paths src -y -i --skip-keys "ament_tools"
    ```

3. **デスクトップパッケージをビルドする**

    ```bash
    colcon build
    ```
    ::: warning
    `--symlink-install` オプションは使用しないでください。
    :::

## minecraft_ros2のインストール

1. **環境変数の設定**

    `.bashrc` などのシェル設定ファイルに以下のように `ros2_java` の install ディレクトリを指定してください

    ```bash
    export ROS2JAVA_INSTALL_PATH=/home/USERNAME/ros2_java_ws/install
    ```

    ::: info
    `USERNAME`の部分は環境に合わせて変更してください。`pwd`コマンドで確認できます。
    :::

    編集後は次のコマンドで反映させます

    ```bash
    source ~/.bashrc
    ```

2. **Minecraft の起動**

    本リポジトリに含まれる以下のスクリプトを実行して、MOD付きの Minecraft を起動します

    ```bash
    git clone https://github.com/minecraft-ros2/minecraft_ros2.git
    ./runClient.sh
    ```

3. **RViz2 での可視化**

    RViz2 を起動し、`minecraft.rviz` を読み込んで Minecraft のデータを可視化します

    ```bash
    rviz2 -d minecraft.rviz
    ```

