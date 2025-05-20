package com.kazusa.ros2mc.ros2;

import net.minecraft.client.Minecraft;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.phys.shapes.VoxelShape;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.phys.AABB;

import org.ros2.rcljava.Time;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.timer.WallTimer;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import std_msgs.msg.Header;
import geometry_msgs.msg.Point32;
import sensor_msgs.msg.PointCloud;
import sensor_msgs.msg.ChannelFloat32;
import tf2_msgs.msg.TFMessage;
import geometry_msgs.msg.TransformStamped;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;

public class PointCloudPublisher extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(PointCloudPublisher.class);

    private final Publisher<PointCloud> publisher;
    private final Publisher<TFMessage> tfPublisher;
    private final WallTimer timer;
    private final Minecraft minecraft;

    public PointCloudPublisher() {
        super("minecraft_pointcloud_publisher");
        publisher = this.node.createPublisher(PointCloud.class, "/player/pointcloud");
        tfPublisher = this.node.createPublisher(TFMessage.class, "/tf");
        minecraft = Minecraft.getInstance();
        timer = this.node.createWallTimer(100, TimeUnit.MILLISECONDS, this::publishAsyncLidarScan);
        LOGGER.info("PointCloudPublisher initialized and publishing to '/player/pointcloud'");
    }

    private void publishAsyncLidarScan() {
        if (minecraft.player == null || minecraft.level == null) return;

        double px = minecraft.player.getX();
        double py = minecraft.player.getY() + minecraft.player.getEyeHeight();
        double pz = minecraft.player.getZ();
        double yawRadPlayer = Math.toRadians(minecraft.player.getYRot());
        double pitchRadPlayer = 0.0;

        var level = minecraft.level;
        var player = minecraft.player;

        CompletableFuture.runAsync(() -> {
            int verticalSteps = 90;
            int horizontalSteps = 90;
            double maxDistance = 5.0;
            double stepSize = 0.05;
            List<Point32> points = new ArrayList<>();
            List<Float> rList = new ArrayList<>();
            List<Float> gList = new ArrayList<>();
            List<Float> bList = new ArrayList<>();

            List<Entity> entities = level.getEntities(
                    player,
                    new AABB(px - maxDistance, py - maxDistance, pz - maxDistance,
                             px + maxDistance, py + maxDistance, pz + maxDistance),
                    entity -> !entity.is(player)
            );

            for (int d = 0; d < verticalSteps; d++) {
                for (int h = 0; h < horizontalSteps; h++) {
                    double pitch = Math.PI * ((double) d / verticalSteps - 0.5);
                    double yaw = 2 * Math.PI * h / horizontalSteps;

                    double dx = Math.cos(pitch) * Math.sin(yaw);
                    double dy = Math.sin(pitch);
                    double dz = Math.cos(pitch) * Math.cos(yaw);

                    for (double dist = 0.0; dist <= maxDistance; dist += stepSize) {
                        double tx = px + dx * dist;
                        double ty = py + dy * dist;
                        double tz = pz + dz * dist;

                        BlockPos blockPos = new BlockPos((int) Math.floor(tx), (int) Math.floor(ty), (int) Math.floor(tz));
                        BlockState blockState = level.getBlockState(blockPos);
                        String blockName = blockState.getBlock().toString().toLowerCase();
                        if (blockName.contains("glass")) continue;

                        VoxelShape shape = blockState.getCollisionShape(level, blockPos);
                        boolean isBlockHit = false;
                        if (!shape.isEmpty()) {
                            for (AABB shapeAABB : shape.toAabbs()) {
                                AABB movedAABB = shapeAABB.move(blockPos);
                                if (movedAABB.contains(tx, ty, tz)) {
                                    isBlockHit = true;
                                    break;
                                }
                            }
                        }

                        boolean isEntityHit = false;
                        if (!isBlockHit) {
                            for (Entity entity : entities) {
                                if (entity.getBoundingBox().contains(tx, ty, tz)) {
                                    isEntityHit = true;
                                    break;
                                }
                            }
                        }

                        if (isBlockHit || isEntityHit) {
                            double rx = tx - px;
                            double ry = ty - py;
                            double rz = tz - pz;

                            double cosPitch = Math.cos(pitchRadPlayer);
                            double sinPitch = Math.sin(pitchRadPlayer);
                            double ryPitch = ry * cosPitch - rz * sinPitch;
                            double rzPitch = ry * sinPitch + rz * cosPitch;

                            double cosYaw = Math.cos(yawRadPlayer);
                            double sinYaw = Math.sin(yawRadPlayer);
                            double rxYaw = rx * cosYaw + rzPitch * sinYaw;
                            double rzYaw = -rx * sinYaw + rzPitch * cosYaw;

                            float x_ros = (float) rzYaw;
                            float y_ros = (float) rxYaw;
                            float z_ros = (float) ryPitch;

                            Point32 point = new Point32();
                            point.setX(x_ros);
                            point.setY(y_ros);
                            point.setZ(z_ros);
                            points.add(point);

                            int color = blockState.getMapColor(level, blockPos).col;
                            float r = ((color >> 16) & 0xFF) / 255.0f;
                            float g = ((color >> 8) & 0xFF) / 255.0f;
                            float b = (color & 0xFF) / 255.0f;
                            rList.add(r);
                            gList.add(g);
                            bList.add(b);

                            break;
                        }
                    }
                }
            }

            if (points.isEmpty()) {
                LOGGER.warn("No points detected in pointcloud scan, skipping publish.");
                return;
            }

            // --- TF transform ---
            TransformStamped transform = new TransformStamped();
            Header tfHeader = new Header();
            tfHeader.setStamp(Time.now());
            tfHeader.setFrameId("world");
            transform.setHeader(tfHeader);
            transform.setChildFrameId("player");

            double rosX = pz;
            double rosY = px;
            double rosZ = py - 63.0;

            transform.getTransform().getTranslation().setX(rosX);
            transform.getTransform().getTranslation().setY(rosY);
            transform.getTransform().getTranslation().setZ(rosZ);

            double roll = 0.0;
            double pitch = pitchRadPlayer;
            double yaw = -yawRadPlayer;

            double cy = Math.cos(yaw * 0.5);
            double sy = Math.sin(yaw * 0.5);
            double cp = Math.cos(pitch * 0.5);
            double sp = Math.sin(pitch * 0.5);
            double cr = Math.cos(roll * 0.5);
            double sr = Math.sin(roll * 0.5);

            double qw = cr * cp * cy + sr * sp * sy;
            double qx = sr * cp * cy - cr * sp * sy;
            double qy = cr * sp * cy + sr * cp * sy;
            double qz = cr * cp * sy - sr * sp * cy;

            transform.getTransform().getRotation().setX(qx);
            transform.getTransform().getRotation().setY(qy);
            transform.getTransform().getRotation().setZ(qz);
            transform.getTransform().getRotation().setW(qw);

            TFMessage tfMessage = new TFMessage();
            tfMessage.setTransforms(List.of(transform));
            tfPublisher.publish(tfMessage);

            // --- PointCloud message ---
            PointCloud msg = new PointCloud();
            Header header = new Header();
            header.setStamp(Time.now());
            header.setFrameId("player");
            msg.setHeader(header);
            msg.setPoints(points);

            List<ChannelFloat32> channels = new ArrayList<>();
            channels.add(createChannel("r", rList));
            channels.add(createChannel("g", gList));
            channels.add(createChannel("b", bList));
            msg.setChannels(channels);

            publisher.publish(msg);
        });
    }

    private ChannelFloat32 createChannel(String name, List<Float> values) {
        ChannelFloat32 channel = new ChannelFloat32();
        channel.setName(name);
        channel.setValues(values);
        return channel;
    }
}
