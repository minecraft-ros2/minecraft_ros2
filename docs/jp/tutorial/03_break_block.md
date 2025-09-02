# Break Block
Pythonを使いプレーヤーの操作を通して周囲のブロックを破壊してみましょう。

このページでは以下の内容を学びます。
- クオータニオン
- サービス通信
- 位置のP制御

## 1. サービス通信をやってみる
サービス通信とは同期、非同期型の通信方法で、serverとclientが処理のリクエストと処理結果のレスポンスをやり取りします。

minecraft_ros2ではブロックの破壊やゲーム内コマンドの実行、エンティティ召喚などで利用することができます。

runClientを実行しワールドに入ってください。

以下コマンドでサービス一覧を確認してみましょう。
```bash
ros2 service list
```

今回は `/dig_block` というサービスを利用します。

試しにプレーヤー視点をブロックに向け、以下コマンドを実行してみましょう。

```bash
ros2 service call /dig_block minecraft_msgs/srv/DigBlock 
```

ブロックが破壊されたら成功です！

:::info
もし破壊されない場合は minecraft_msgs を再読み込み(source)してください。
:::

## 2. プレーヤーの角度を取得する
ブロックの破壊はできるようになりましたが視点を操作し、照準を合わせる必要があります。

せっかくなのでこれも自動でやりたいですよね？

今回はチュートリアルなのでデバッグ用メッセージを用いて視点情報を取得します。

`/player/ground_truth` というトピックにプレーヤーの位置情報が流れているので確認してみましょう。

```bash
ros2 topic echo /player/ground_truth
```

```bash
---
position:
  x: 14.988304348096932
  y: -589.1905229054375
  z: 74.0
orientation:
  x: 0.025652511045336723
  y: 0.03142489492893219
  z: -0.6318491101264954
  w: 0.7740291357040405
---
```

このように x,y,zの位置と x,y,z,wの角度が得られます。

w ?!? と思った方もいると思います。

これはクオータニオンといい、回転情報を正確に伝えることができます

:::info
ジンバルロックという現象を起こさない方式です。

:::

クオータニオンからxyzに変換することもできます。


## 2. 新規コードを追加する
`~/minecraft_ros2_tutorial_ws/src/minecraft_ros2_tutorial/minecraft_ros2_tutorial/break_block.py` を追加し、以下のコードを記入してください


```py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
import math

class BreakBlock(Node):
    def __init__(self):
        super().__init__('break_block')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose, '/player/ground_truth', self.pose_callback, 10
        ) # /player/ground_truth を受信ごpose_callbackを呼ぶ

        self.player_pose = None
        self.Kp = 2.0
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info("Break Block Node Initialized.")

    def pose_callback(self, msg: Pose):
        self.player_pose = msg # メッセージを格納

    def quaternion_to_yaw_pitch(self, q): # 値変換
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = -math.atan2(siny_cosp, cosy_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        return yaw, pitch

    def timer_callback(self):
        if self.player_pose is None:
            return

        yaw, pitch = self.quaternion_to_yaw_pitch(self.player_pose.orientation)

        self.target_block_position = [1, 0, -1] # x:前, y:左, z:下
        dx, dy, dz = self.target_block_position
        dz = dz-1 # プレーヤーの頭は原点からずれている
        target_yaw = math.atan2(-dy, dx)
        target_pitch = math.atan2(-dz, math.sqrt(dx**2 + (-dy)**2))

        # 誤差計算
        yaw_error = target_yaw - yaw
        pitch_error = target_pitch - pitch

        twist = Twist()
        # 定数倍
        twist.angular.y = -self.Kp * pitch_error
        twist.angular.z = -self.Kp * yaw_error
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = BreakBlock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

`setup.py`の`console_scripts` に以下を追加します
```py
'break_block = minecraft_ros2_tutorial.break_block:main',
```

```bash
colcon build
source install/setup.bash
```
をwsで実行後 `ros2 run minecraft_ros2_tutorial break_block` で起動してみてください。

target_block_position　の方に視点が勝手に移動したと思います。

これは座標変換とP制御をすることで実現しています。

target_block_position はプレーヤーから見たブロックの相対座標で、今回は前斜め下に相当するブロックが対象です。

P制御とは目標値と現在地の差の定数倍を司令値にすることで、差が大きいときは大きな動き、小さいときは小さな動きができます。

target_block_positionを変更して動作確認をしてみてください。

## 3. 指定したブロックを破壊する
今回の例では斜め破壊＋前移動で階段を自動で作成したいと思います。

現在ブロックの破壊でゲームが一時停止してしまう問題があるため、特別にフラグを建てることで対応しています。

```py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from minecraft_msgs.srv import DigBlock
import math

class BreakBlock(Node):
    def __init__(self):
        super().__init__('break_block')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose, '/player/ground_truth', self.pose_callback, 10
        ) # /player/ground_truth を受信ごpose_callbackを呼ぶ

        # DigBlock サービスクライアント
        self.dig_client = self.create_client(DigBlock, '/dig_block')
        while not self.dig_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /dig_block service...')

        self.player_pose = None
        self.in_progress = False
        self.Kp = 2.0
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Break Block Node Initialized.")


    def pose_callback(self, msg: Pose):
        self.player_pose = msg # メッセージを格納

    def quaternion_to_yaw_pitch(self, q): # 値変換
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = -math.atan2(siny_cosp, cosy_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        return yaw, pitch

    def timer_callback(self):
        if self.player_pose is None:
            return

        yaw, pitch = self.quaternion_to_yaw_pitch(self.player_pose.orientation)

        self.target_block_position = [1, 0, -1] # x:前, y:左, z:下
        dx, dy, dz = self.target_block_position
        # dz = dz-1 # プレーヤーの頭は原点からずれている
        target_yaw = math.atan2(-dy, dx)
        target_pitch = math.atan2(-dz, math.sqrt(dx**2 + (-dy)**2))

        # 誤差計算
        yaw_error = target_yaw - yaw
        pitch_error = target_pitch - pitch

        twist = Twist()
        # 定数倍
        twist.angular.y = -self.Kp * pitch_error
        twist.angular.z = -self.Kp * yaw_error

        self.publisher_.publish(twist)

        if abs(yaw_error) < 0.05 and abs(pitch_error) < 0.05:
            if not self.in_progress:
                self.in_progress = True
                self.get_logger().info('Target reached, sending dig command...')
                twist = Twist()
                self.publisher_.publish(twist)
                req = DigBlock.Request()
                future = self.dig_client.call_async(req)
                future.add_done_callback(self.dig_response_callback)

    def dig_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'DigBlock response received: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        self.in_progress = False
        
        twist = Twist()
        twist.linear.x = 1.0
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = BreakBlock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

`package.xml` のdependに `minecraft_msgs`を追加しましょう

```xml
<depend>minecraft_msgs</depend>
```


これを実行すると自動で階段を生成してくれます！
