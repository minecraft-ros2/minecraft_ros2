# Debug Messages

minecraft-ros2 provides debug messages.

By directly publishing data from Minecraft, these messages allow for collecting ground truth data along with sensor data, which is useful for both data collection and troubleshooting.

![](/images/debug_msgs.png)

:::info
There are disabled by default. To enable them, set `enableDebugDataStreaming` to `true` in the `run/config/minecraft_ros2-common.toml` file.
:::

## Player Status
Retrieves the player's current status information.

### ROS 2 Interface

| Type                          | Topic Name       |
| ----------------------------- | ---------------- |
| `minecraft_msgs/PlayerStatus` | `/player_status` |

### Message Details

**minecraft_msgs/PlayerStatus**

| Field Name | Type   | Description                     |
| ---------- | ------ | ------------------------------- |
| `name` | string | Player's name |
| `dimension` | string | Player's current dimension name |
| `food_level` | uint8  | Player's hunger level |
| `score` | uint32 | Player's score |
| `sleep_timer` | uint8  | Player's sleep timer |
| `xp_level` | uint32 | Player's experience level |
| `xp_progress` | float32 | Player's experience progress |
| `total_xp` | uint32 | Player's total experience |
| `health` | float32 | Player's current health |
| `max_health` | float32 | Player's maximum health |
| `air` | uint16 | Player's current air supply |
| `max_air` | uint16 | Player's maximum air supply |
| `active_effects` | string[] | List of effects currently applied to the player |
| `inventory_items` | minecraft_msgs/Item[] | List of items in the player's inventory |
| `main_hand_item` | minecraft_msgs/Item | Item held in the player's main hand |
| `off_hand_item` | minecraft_msgs/Item | Item held in the player's off hand |

## Relative Distance to Mobs
Retrieves the relative distance between the player and mobs.

### ROS 2 Interface
| Type                          | Topic Name       |
| ----------------------------- | ---------------- |
| `minecraft_msgs/LivingEntityArray` | `/player/nearby_living_entities` |

### Message Details

**minecraft_msgs/LivingEntityArray**
| Field Name | Type   | Description                     |
| ---------- | ------ | ------------------------------- |
| `header` | std_msgs/Header | Message's header information |
| `entities` | minecraft_msgs/LivingEntity[] | List of nearby mobs |

**minecraft_msgs/LivingEntity**
| Field Name | Type   | Description                     |
| ---------- | ------ | ------------------------------- |
| `description_id` | string | Mob's description ID |
| `name` | string | Mob's name |
| `pose` | geometry_msgs/Pose | Mob's position and orientation |
| `id` | uint32 | Mob's unique ID |
| `health` | float32 | Mob's current health |
| `max_health` | float32 | Mob's maximum health |
| `hit_box` | geometry_msgs/Vector3 | Size of the mob's hit box |
| `category` | minecraft_msgs/MobCategory | Mob's category |

**minecraft_msgs/MobCategory**
| Field Name | Type   | Description                     |
| ---------- | ------ | ------------------------------- |
| `MONSTER` | uint8 | Monster category |

The following categories are defined:

| enum Value | Value |
| ---------- | ----- |
| `MONSTER` | 0 |
| `CREATURE` | 1 |
| `AMBIENT` | 2 |
| `AXOLOTLS` | 3 |
| `UNDERGROUND_WATER_CREATURE` | 4 |
| `WATER_CREATURE` | 5 |
| `WATER_AMBIENT` | 6 |
| `MISC` | 7 |
