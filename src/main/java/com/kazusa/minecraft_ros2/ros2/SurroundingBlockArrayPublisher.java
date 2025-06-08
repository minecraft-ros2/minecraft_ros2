package com.kazusa.minecraft_ros2.ros2;

import net.minecraft.core.BlockPos;
import net.minecraft.client.Minecraft;
import net.minecraft.world.level.Level;
import net.minecraft.world.entity.player.Player;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.shapes.VoxelShape;
import net.minecraft.core.registries.Registries;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.Time;
import org.ros2.rcljava.publisher.Publisher;
import minecraft_msgs.msg.Block;
import minecraft_msgs.msg.BlockArray;
import geometry_msgs.msg.Point;
import geometry_msgs.msg.Vector3;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;

public class SurroundingBlockArrayPublisher extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(SurroundingBlockArrayPublisher.class);

    private final Publisher<BlockArray> publisher;
    private final Minecraft minecraft;
    private Player player;

    private long delta_time = 200;

    private int checkBlockRange = 3;

    public SurroundingBlockArrayPublisher() {
        super("minecraft_surrounding_block_array_publisher");
        publisher = this.node.createPublisher(BlockArray.class, "/player/surrounding_block_array");
        minecraft = Minecraft.getInstance();
        this.node.createWallTimer(delta_time, TimeUnit.MILLISECONDS, this::publishSurroundingBlockArray);
        LOGGER.info("SurroundingBlockArrayPublisher initialized and publishing to '/player/surrounding_block_array'");
    }

    public void publishSurroundingBlockArray() {
        CompletableFuture.runAsync(() -> {
            player = minecraft.player;
            if (player == null) {
                LOGGER.warn("Player is null, cannot get surrounding block array");
                return;
            }

            Level world = minecraft.level;
            if (world == null) {
                LOGGER.warn("current world is null, cannot publish surrounding block array");
                return;
            }

            float playerX = (float) player.getX();
            float playerY = (float) player.getY();
            float playerZ = (float) player.getZ();
            float playerYaw = -player.getYRot();

            BlockArray blockArray = new BlockArray();
            for (int x = -checkBlockRange; x <= checkBlockRange; x++) {
                for (int y = -checkBlockRange; y <= checkBlockRange; y++) {
                    for (int z = -checkBlockRange; z <= checkBlockRange; z++) {
                        Block blockMsg = new Block();

                        // ブロックの位置を計算
                        double blockX = playerX + x;
                        double blockY = playerY + y;
                        double blockZ = playerZ + z;

                        BlockPos pos = new BlockPos((int) blockX, (int) blockY, (int) blockZ);
                        BlockState state = world.getBlockState(pos);
                        var block = state.getBlock();

                        // 1) name
                        String name = world.registryAccess()
                                            .registryOrThrow(Registries.BLOCK)
                                            .getKey(block)
                                            .toString();
                        if (name.contains("air")) {
                            // 空気ブロックはスキップ
                            continue;
                        }
                        blockMsg.setName(name);

                        // 2) position
                        Point p = blockMsg.getPosition();
                        p.setX(z * Math.cos(Math.toRadians(playerYaw)) - x * Math.sin(Math.toRadians(playerYaw)));
                        p.setY(z * Math.sin(Math.toRadians(playerYaw)) + x * Math.cos(Math.toRadians(playerYaw)));
                        p.setZ(y);

                        // 3) collision ボックス
                        try {
                            VoxelShape shape = state.getCollisionShape(world, pos);
                            AABB box = shape.bounds();
                            double sizeX = box.maxX - box.minX;
                            double sizeY = box.maxY - box.minY;
                            double sizeZ = box.maxZ - box.minZ;
                            Vector3 coll = blockMsg.getCollision();
                            coll.setX(sizeZ);
                            coll.setY(sizeX);
                            coll.setZ(sizeY);
                        } catch (Exception e) {
                            Vector3 coll = blockMsg.getCollision();
                            coll.setX(0.0);
                            coll.setY(0.0);
                            coll.setZ(0.0);
                        }

                        // 4) can_occlude
                        blockMsg.setCanOcclude(state.canOcclude());

                        blockArray.getBlocks().add(blockMsg);
                    }
                }
            }

            blockArray.getHeader().setStamp(Time.now());
            String playerName = minecraft
                .getConnection()
                .getPlayerInfo(player.getUUID())
                .getProfile()
                .getName();
            blockArray.getHeader().setFrameId(playerName);

            publisher.publish(blockArray);
        });
    }
}
