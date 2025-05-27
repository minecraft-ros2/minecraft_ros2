# Installation Using Docker

## System Requirements

* Ubuntu 22.04
* Docker

## How to Set Up the Environment

1. **Allow GUI access**

   ```bash
   xhost +local:root
   ```

2. **Clone the repository**

   ```bash
   git clone https://github.com/minecraft-ros2/minecraft_ros2.git
   cd minecraft_ros2
   ```

3. **Launch the Docker containers**

   ```bash
   docker compose up
   ```

   Running this command will start both Minecraft and RViz at the same time.

    <!-- For instructions on displaying point clouds in RViz, see [Using the Sensors](#using-the-sensors). -->

4. **Revoke permissions**

   ```bash
   xhost -local:root
   ```
