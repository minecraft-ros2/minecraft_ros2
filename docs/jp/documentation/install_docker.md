# Dockerを使ったインストール

## 動作環境

- Ubuntu 22.04
- Docker
- NVIDIA Container Toolkit

## 環境構築方法

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
    このコマンドを実行すると、MinecraftとRVizが同時に起動します。
    <!-- RVizで点群を表示する方法については、[センサーの使い方](#センサーの使い方)をご覧ください。 -->


4. **権限の復元**

    ```bash
    xhost -local:root
    ```