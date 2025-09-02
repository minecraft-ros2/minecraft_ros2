# サンプルコードのセットアップ

## 1. ダウンロード
以下のように **`~/minecraft_ros2_example_ws` をワークスペースのルート**にします。

```bash
mkdir -p ~/minecraft_ros2_example_ws/src
cd ~/minecraft_ros2_example_ws
# サンプルの例リポジトリを取得
git clone https://github.com/minecraft-ros2/minecraft_ros2_example.git
# .repos で依存パッケージ群を src/ に一括取得
vcs import src < minecraft_ros2_example/example.repos
```

## 2. 依存関係の解決

```bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

## 3. ビルド

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
