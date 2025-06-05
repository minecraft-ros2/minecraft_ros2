package com.kazusa.minecraft_ros2.ros2;

import net.minecraft.core.BlockPos;
import net.minecraft.core.Direction;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.block.state.properties.BlockStateProperties;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.server.MinecraftServer;
import net.minecraftforge.server.ServerLifecycleHooks;
import net.minecraft.world.level.Level;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.publisher.Publisher;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import std_msgs.msg.Int32;
import java.util.concurrent.TimeUnit;
import java.lang.Boolean;

public class BlockIntPublisher extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(BlockIntPublisher.class);

    private final Publisher<Int32> publisher;

    private BlockPos targetPos;

    private long delta_time = 10;

    public BlockIntPublisher(BlockPos pos, String namespace) {
        super("minecraft_block_int_subscriber");
        this.targetPos = pos;
        String topicName = "output";
        if (namespace != null && !namespace.isEmpty()) {
            if (!namespace.startsWith("/")) {
                namespace = "/" + namespace;
            }
            if (!namespace.endsWith("/")) {
                namespace += "/";
            }
            topicName = namespace + topicName;
        }
        this.publisher = this.node.<Int32>createPublisher(
            Int32.class, topicName);
        node.createWallTimer(delta_time, TimeUnit.MILLISECONDS, this::publishData);
        LOGGER.info("BlockIntPublisher initialized and publishing to '{}' topic", topicName);
    }

    private void publishData() {
        // MinecraftServer を取得して「サーバースレッド」に作業を委譲
        MinecraftServer server = ServerLifecycleHooks.getCurrentServer();
        if (server == null) {
            LOGGER.warn("Server instance is null, cannot update block");
            return;
        }

        server.submit(() -> {
            ServerLevel world = server.getLevel(Level.OVERWORLD);
            if (world == null) {
                LOGGER.warn("Target world is null");
                return;
            }

            BlockState blockState = world.getBlockState(targetPos);
            int maxSignal = blockState.getValue(BlockStateProperties.POWERED) ? 15 : 0;
            for (Direction direction : Direction.values()) {
                BlockPos adjacentPos = targetPos.relative(direction);
                BlockState adjacentState = world.getBlockState(adjacentPos);
                int signal = adjacentState.getSignal(world, adjacentPos, direction);
                if (signal > maxSignal) {
                    maxSignal = signal;
                }
            }

            Int32 msg = new Int32();
            msg.setData(maxSignal);
            publisher.publish(msg);
            LOGGER.debug("Published signal strength {}", maxSignal);
        });
    }

    public void shutdown() {
        if (this.publisher != null) {
            this.publisher.dispose();          // パブリッシャー破棄
        }
        node.dispose();                 // ノード破棄
    }
}