# topic
topicはROS 2をやる上で欠かせない通信方法です。

このページでは以下の内容を学びます。
- message
- publisher, subsctiber
- `ros2 topic`コマンド
- `ros2 interface`コマンド

## message
トピックでやり取りされる情報をメッセージ(message)と呼びます。

メッセージの型は自由に定義することができます。

例えば`std_msgs`というパッケージに含まれる`msg/Bool`を使えばbool型のトピック通信を行うことができます。

minecraft_ros2では`minecraft_msgs`というパッケージで独自の型を定義しています。

## publisher, subsctiber
送信者をパブリッシャー(publisher)、受診者をサブスクライバー(subscriber)と呼びます。

つまり、パブリッシャーがあるトピックにメッセージを送信すると同じトピックを見ているサブスクライバーが受信し、何らかの動作が実行される　というわけです。

## `ros2 topic`コマンド
`ros2 topic`コマンドを使うとトピック通信に関する情報を確認したりすることができます。

ここではよく使用するサブコマンドのlist, echo, info, pub を紹介します。

### list
`ros2 topic list`と実行することで現在存在しているトピックの一覧を確認できます。

また`-v`というオプションをつけることで詳細情報を確認できます。

試しにminecraft_ros2を起動し、ワールドに参加すると以下の様な出力になるはずです。

```bash
    $ ros2 topic list
/cmd_vel
/parameter_events
/player/image_raw
/player/imu
/player/nearby_living_entities
/player/pointcloud
/player/status
/rosout
/tf
```

### echo
`ros2 topic echo トピック名`と実行するとtopicに流れてきているデータを確認できます。

minecraft_ros2を起動した状態で `ros2 topic echo /player/status` を実行するとプレーヤーの状態を確認できるはずです。

```bash
    $ ros2 topic echo /player/status 
name: Dev
dimension: minecraft:overworld
food_level: 20
score: 0
.....
```


### info
`ros2 topic info トピック名`と実行するとそのトピックのパブリッシャーとサブスクライバーなどの情報がわかります。

```bash
    $ ros2 topic info /player/status 
Type: minecraft_msgs/msg/PlayerStatus
Publisher count: 1
Subscription count: 0
```

### pub
`ros2 topic pub トピック名 型 メッセージ`と実行するとそのトピックにメッセージをパブリッシュすることができます。

例えば `/cmd_vel` に対して前に1m/s、左に0.3m/sの司令を送るときは、

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.3}}"
```
を実行します。

geometry_msgs/msg/Twist は3次元の平行移動と回転に関するメッセージ型で、linear.xが前に平行移動、linear.yが左に平行移動という意味を持ちます。

コマンドを実行すると止まらなくなると思うので一度 Ctrl + C で停止し、空のメッセージを送信しましょう。

メッセージを省略すると空のメッセージを送信することができます。

## `ros2 interface`コマンド
`ros2 interface`コマンドを使うとメッセージなどの情報を取得することができます。
主に package, showを使います。

### package
`ros2 interface package パッケージ名`を実行するとそのパッケージに含まれているメッセージを取得することができます。

```bash
    $ ros2 interface package minecraft_msgs 
minecraft_msgs/srv/Command
minecraft_msgs/msg/BlockArray
minecraft_msgs/msg/LivingEntity
minecraft_msgs/msg/LivingEntityArray
minecraft_msgs/msg/PlayerStatus
minecraft_msgs/msg/Item
minecraft_msgs/msg/Block
minecraft_msgs/msg/MobCategory
```

### show
`ros2 interface show メッセージ型`を実行するとその型の中身を調べることができます。

```bash
    $ ros2 interface show minecraft_msgs/msg/Item 
string name
uint8 count
int16 damage
int16 max_damage
```