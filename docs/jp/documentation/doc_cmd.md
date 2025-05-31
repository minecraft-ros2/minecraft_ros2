# コマンドの実行

プレイヤーが存在するワールド内で、受信した任意のコマンドを実行します。コマンドの実行には、権限レベル2（ゲームマスター）が付与されています。

::: tip
権限レベル2（ゲームマスター）では、`gamemode`、`weather`、`item`、`time` など、ゲーム環境に関する多くのコマンドを使用できます。
:::

### ROS 2 インターフェース

| Type                         | Topic Name           |
| ---------------------------- | -------------------- |
| `minecraft_msgs/srv/Command` | `/minecraft/command` |

### 使用例

以下は、座標 `X: -24, Y: 160, Z: 18` にアイアンゴーレムを召喚するコマンドの例です。通常ゲーム内で実行する際に付ける先頭の `/` は不要です。

```bash
ros2 service call /minecraft/command minecraft_msgs/srv/Command "{command: 'summon minecraft:iron_golem -24 160 18'}"
```
