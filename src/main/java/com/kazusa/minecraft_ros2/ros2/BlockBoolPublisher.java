package com.kazusa.minecraft_ros2.ros2;

import net.minecraft.client.Minecraft;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.block.state.properties.BlockStateProperties;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.server.MinecraftServer;
import net.minecraftforge.server.ServerLifecycleHooks;
import net.minecraft.world.level.Level;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.Time;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import std_msgs.msg.Bool;
import java.util.concurrent.TimeUnit;
import java.lang.Boolean;

public class BlockBoolPublisher extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(BlockBoolPublisher.class);

    private final Publisher<Bool> publisher;

    private Minecraft minecraft;

    private BlockPos targetPos;

    private boolean state;

    private long delta_time = 10;

    public BlockBoolPublisher(BlockPos pos, String namespace) {
        super("minecraft_block_bool_subscriber");
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
        this.publisher = this.node.<Bool>createPublisher(
            Bool.class, topicName);
        node.createWallTimer(delta_time, TimeUnit.MILLISECONDS, this::publishData);
        LOGGER.info("BlockBoolSubscriber initialized and listening on 'cmd_vel' topic");
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
            var powered = blockState.getValue(BlockStateProperties.POWERED);
            Bool msg = new Bool();
            msg.setData((Boolean)powered);
            publisher.publish(msg);
        });
    }

    public void shutdown() {
        if (this.publisher != null) {
            this.publisher.dispose();          // パブリッシャー破棄
        }
        node.dispose();                 // ノード破棄
    }
}