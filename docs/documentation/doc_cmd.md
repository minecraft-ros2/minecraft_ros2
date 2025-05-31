# Command Execution

Executes any received command within the world where the player exists. The command is executed with permission level 2 (Game Master).

::: tip
Permission level 2 (Game Master) allows the use of many commands related to the game environment, such as `gamemode`, `weather`, `item`, and `time`.
:::

### ROS 2 Interface

| Type                         | Topic Name           |
| ---------------------------- | -------------------- |
| `minecraft_msgs/srv/Command` | `/minecraft/command` |

### Example

The following is an example command to summon an iron golem at coordinates `X: -24, Y: 160, Z: 18`. Note that the leading `/` used in in-game commands is not required here.

```bash
ros2 service call /minecraft/command minecraft_msgs/srv/Command "{command: 'summon minecraft:iron_golem -24 160 18'}"
```
