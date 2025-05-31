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
import org.ros2.rcljava.subscription.Subscription;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import std_msgs.msg.Bool;

public class BlockBoolSubscriber extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(BlockBoolSubscriber.class);

    private final Subscription<Bool> subscription;

    private Minecraft minecraft;

    private BlockPos targetPos;

    private boolean state;

    public BlockBoolSubscriber(BlockPos pos, String namespace) {
        super("minecraft_block_bool_subscriber");
        this.targetPos = pos;
        String topicName = "input";
        if (namespace != null && !namespace.isEmpty()) {
            if (!namespace.startsWith("/")) {
                namespace = "/" + namespace;
            }
            if (!namespace.endsWith("/")) {
                namespace += "/";
            }
            topicName = namespace + topicName;
        }
        this.subscription = this.node.<Bool>createSubscription(
            Bool.class, topicName, this::boolCallback);
        LOGGER.info("BlockBoolSubscriber initialized and listening on '{}' topic", topicName);
    }

    private void boolCallback(final Bool msg) {
        boolean powered = msg.getData();

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

            BlockState oldState = world.getBlockState(targetPos);
            BlockState newState = oldState.setValue(BlockStateProperties.POWERED, powered);

            world.setBlock(targetPos, newState, 3);
            world.updateNeighborsAt(targetPos, newState.getBlock());
        });
    }

    public void shutdown() {
        if (this.subscription != null) {
            this.subscription.dispose();      // 購読解除
        }
        this.node.dispose();                 // ノード破棄
    }
}