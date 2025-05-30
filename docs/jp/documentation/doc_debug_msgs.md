# デバッグメッセージ

minecraft-ros2では、デバッグ用のメッセージを提供しています。

Minecraft内のデータを直接Publishするため、センサーデータとともに正解データの収集や不具合の調査に役立てることができます。

![](/images/debug_msgs.png)

::: info
デバッグメッセージはデフォルトでは無効になっています。有効にするには、 `run/config/minecraft_ros2-common.toml` ファイル内の `enableDebugDataStreaming` を `true` に設定してください。
:::

## プレイヤーステータス
プレイヤーの状態を取得します。

### ROS 2 Interface

| Type                          | Topic Name       |
| ----------------------------- | ---------------- |
| `minecraft_msgs/PlayerStatus` | `/player_status` |

### メッセージ詳細

**minecraft_msgs/PlayerStatus**

| Field Name | Type   | Description                     |
| ---------- | ------ | ------------------------------- |
| `name` | string | プレイヤーの名前 |
| `dimension` | string | プレイヤーがいる次元 |
| `food_level` | uint8  | プレイヤーの空腹度 |
| `score` | uint32 | プレイヤーのスコア |
| `sleep_timer` | uint8  | プレイヤーの睡眠時間 |
| `xp_level` | uint32 | プレイヤーのレベル |
| `xp_progress` | float32 | 次のレベルまでの残り経験値（％） |
| `total_xp` | uint32 | プレイヤーの総経験値 |
| `health` | float32 | プレイヤーの現在の体力 |
| `max_health` | float32 | プレイヤーの最大体力 |
| `air` | uint16 | プレイヤーの現在の空気確保量 |
| `max_air` | uint16 | プレイヤーの最大空気確保量 |
| `active_effects` | string[] | プレイヤーが受けている効果のリスト |
| `inventory_items` | minecraft_msgs/Item[] | プレイヤーのインベントリ内のアイテムのリスト |
| `main_hand_item` | minecraft_msgs/Item | プレイヤーのメインハンドに持っているアイテム |
| `off_hand_item` | minecraft_msgs/Item | プレイヤーのオフハンドに持っているアイテム |

## Mobとの相対距離
プレイヤーとMobとの相対距離を取得します。

### ROS 2 Interface
| Type                          | Topic Name       |
| ----------------------------- | ---------------- |
| `minecraft_msgs/LivingEntityArray` | `/player/nearby_living_entities` |

### メッセージ詳細

**minecraft_msgs/LivingEntityArray**
| Field Name | Type   | Description                     |
| ---------- | ------ | ------------------------------- |
| `header` | std_msgs/Header | メッセージのヘッダー情報 |
| `entities` | minecraft_msgs/LivingEntity[] | 近くのMobのリスト |

**minecraft_msgs/LivingEntity**
| Field Name | Type   | Description                     |
| ---------- | ------ | ------------------------------- |
| `description_id` | string | Mobの説明ID |
| `name` | string | Mobの名前 |
| `pose` | geometry_msgs/Pose | Mobの位置と向き |
| `id` | uint32 | Mobの一意のID |
| `health` | float32 | Mobの現在のヘルス |
| `max_health` | float32 | Mobの最大ヘルス |
| `hit_box` | geometry_msgs/Vector3 | Mobのヒットボックスのサイズ |
| `category` | minecraft_msgs/MobCategory | Mobのカテゴリ |

**minecraft_msgs/MobCategory**
| Field Name | Type   | Description                     |
| ---------- | ------ | ------------------------------- |
| `mob_category` | uint8 | Mobのカテゴリ |

次のカテゴリが定義されています。

| enum Value | Value | Description |
| ---------- | ----- | ----------- |
| `MONSTER` | 0 | モンスター (敵対Mob) |
| `CREATURE` | 1 | 動物 (ほとんどの場合友好Mob) |
| `AMBIENT` | 2 | 環境音を発するMob |
| `AXOLOTLS` | 3 | ウーパールーパー |
| `UNDERGROUND_WATER_CREATURE` | 4 | 地下水棲生物 |
| `WATER_CREATURE` | 5 | 水棲生物 |
| `WATER_AMBIENT` | 6 | 水中の環境音を発するMob |
| `MISC` | 7 | その他のMob |