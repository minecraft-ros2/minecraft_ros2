# Control Player
Pythonを使いプレーヤーを遠隔操縦してみましょう。

このページでは以下の内容を学びます。
- パッケージ、ノード作成
- Pythonパブリッシャー作成

## 1. パッケージを作成する
適当な作業場所に移動します。

```bash
mkdir -p ~/minecraft_ros2_tutorial_ws/src
cd ~/minecraft_ros2_tutorial_ws/src
```

以下コマンドを実行し、Pythonノードを含むパッケージのテンプレートを作成します。

```bash
ros2 pkg create --build-type ament_python minecraft_ros2_tutorial
```

テンプレートをもとに変更が必要な点を直します。

### package.xml
package.xmlにはパッケージの情報を記入します。

今回は依存関係で geometry_msgs が必要なのでそれを追記します

`<test_depend>` と `<export>` の間に以下を書き足してください。

```xml
<depend>geometry_msgs</depend>
```

また、descriptionやmaintainer、licenseタグも編集が必要なので編集します。

### setup.py
setup.py はPythonに関する設定を記入します。

今回はノードの登録が必要なので最後の方にある `console_scripts` を編集します

```py
entry_points={
        'console_scripts': [
                'control_player = minecraft_ros2_tutorial.control_player:main',
        ],
},
```

## 2. Pythonコードを書く
`minecraft_ros2_tutorial/control_player.py` を作成し、以下のコードを記入します。

```py
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class ControlPlayer(Node):
    def __init__(self):
        super().__init__('control_player') # control_playerという名前のNodeを作成
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10) # /cmd_vel のパブリッシャーを作成
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback) # 0.01秒ごとに関数 timer_callback を実行
        self.speed = 1.0
        print("Control Player Node Initialized. Use wasd keys to move, Space to jump, WASD for rotation.")

    def timer_callback(self):
        key = input("enter key (wasd,space,WASD) >>>")
        twist = Twist() # メッセージ作成

        # 値を格納
        if 'w' in key:
            twist.linear.x += self.speed
        if 's' in key:
            twist.linear.x -= self.speed
        if 'a' in key:
            twist.linear.y += self.speed
        if 'd' in key:
            twist.linear.y -= self.speed
        if ' ' in key:
            twist.linear.z = 1.0
        if 'W' in key:
            twist.angular.y += self.speed
        if 'S' in key:
            twist.angular.y -= self.speed
        if 'A' in key:
            twist.angular.z += self.speed
        if 'D' in key:
            twist.angular.z -= self.speed

        self.publisher_.publish(twist) # メッセージをパブリッシュ


def main(args=None):
    rclpy.init(args=args) # ROS 2 初期化

    control_player = ControlPlayer()

    rclpy.spin(control_player) # spinで無限ループスタート

    # 終了時に以下実行
    control_player.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

このコードでは wasd で平行移動、 スペースキーでジャンプ、 WASD で視点移動ができるようなメッセージを送信します。

試しに実行してみましょう！ (minecraft_ros2を起動して適当なワールドに参加しておいてください)

`cd ~/ros2_tutorial_ws` で作業場所に移動し、 `colcon build` でビルドしてみましょう！

初回のみ `source install/setup.bash` の実行が必要です。

```bash
ros2 run minecraft_ros2_tutorial control_player
```

これでマイクラ内のプレーヤーの移動がROS 2から遠隔でできるようになりました！
