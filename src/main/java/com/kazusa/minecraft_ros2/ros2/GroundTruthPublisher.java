package com.kazusa.minecraft_ros2.ros2;

import net.minecraft.client.Minecraft;
import net.minecraft.world.entity.player.Player;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.publisher.Publisher;
import geometry_msgs.msg.Pose;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;

public class GroundTruthPublisher extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(GroundTruthPublisher.class);

    private final Publisher<Pose> publisher;
    private final Minecraft minecraft;
    private Player player;

    private long delta_time = 100;

    public GroundTruthPublisher() {
        super("minecraft_ground_truth_publisher");
        publisher = this.node.createPublisher(Pose.class, "/player/ground_truth");
        minecraft = Minecraft.getInstance();
        this.node.createWallTimer(delta_time, TimeUnit.MILLISECONDS, this::publishGroundTruth);
        LOGGER.info("GroundTruthPublisher initialized and publishing to '/player/ground_truth'");
    }

    public void publishGroundTruth() {
        CompletableFuture.runAsync(() -> {
            player = minecraft.player;

            if (player == null) {
                LOGGER.warn("Player is null, cannot get ground truth");
                return;
            }

            Pose pose = new Pose();

            pose.getPosition().setX(player.getX());
            pose.getPosition().setY(player.getY());
            pose.getPosition().setZ(player.getZ());

            float yaw = -player.getYRot();
            float pitch = player.getXRot();
            float roll = 0.0f; // Minecraftではロールは通常使用しないため0で初期化

            float cy = (float) Math.cos(Math.toRadians(yaw * 0.5));
            float sy = (float) Math.sin(Math.toRadians(yaw * 0.5));
            float cp = (float) Math.cos(Math.toRadians(pitch * 0.5));
            float sp = (float) Math.sin(Math.toRadians(pitch * 0.5));
            float cr = (float) Math.cos(Math.toRadians(roll * 0.5));
            float sr = (float) Math.sin(Math.toRadians(roll * 0.5));
            pose.getOrientation().setX(sr * cp * cy - cr * sp * sy);
            pose.getOrientation().setY(cr * sp * cy + sr * cp * sy);
            pose.getOrientation().setZ(cr * cp * sy - sr * sp * cy);
            pose.getOrientation().setW(cr * cp * cy + sr * sp * sy);

            publisher.publish(pose);
        });
    }
}
