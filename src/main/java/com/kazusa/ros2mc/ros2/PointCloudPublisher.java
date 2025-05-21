package com.kazusa.ros2mc.ros2;

import net.minecraft.client.Minecraft;
import net.minecraft.client.renderer.entity.EntityRenderDispatcher;
import net.minecraft.core.BlockPos;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.shapes.VoxelShape;

import org.ros2.rcljava.Time;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.timer.WallTimer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import geometry_msgs.msg.Point32;
import geometry_msgs.msg.TransformStamped;
import sensor_msgs.msg.ChannelFloat32;
import sensor_msgs.msg.PointCloud;
import std_msgs.msg.Header;
import tf2_msgs.msg.TFMessage;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.InputStream;
import java.util.*;
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
            int verticalSteps = 180;
            int horizontalSteps = 180;
            double maxDistance = 5.0;
            double stepSize = 0.1;
            List<Point32> points = new ArrayList<>();
            List<Float> rList = new ArrayList<>();
            List<Float> gList = new ArrayList<>();
            List<Float> bList = new ArrayList<>();

            // --- 事前にエンティティの色情報を取得 ---
            List<Entity> entities = level.getEntities(
                    player,
                    new AABB(px - maxDistance, py - maxDistance, pz - maxDistance,
                             px + maxDistance, py + maxDistance, pz + maxDistance),
                    entity -> !entity.is(player)
            );

            Map<Entity, float[]> entityColors = new HashMap<>();
            for (Entity entity : entities) {
                float[] rgb = getEntityColor(entity);
                entityColors.put(entity, rgb);
            }

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

                        Entity hitEntity = null;
                        if (!isBlockHit) {
                            for (Entity entity : entities) {
                                if (entity.getBoundingBox().contains(tx, ty, tz)) {
                                    hitEntity = entity;
                                    break;
                                }
                            }
                        }

                        if (isBlockHit || hitEntity != null) {
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

                            if (isBlockHit) {
                                int color = blockState.getMapColor(level, blockPos).col;
                                float r = ((color >> 16) & 0xFF) / 255.0f;
                                float g = ((color >> 8) & 0xFF) / 255.0f;
                                float b = (color & 0xFF) / 255.0f;
                                rList.add(r);
                                gList.add(g);
                                bList.add(b);
                            } else if (hitEntity != null && entityColors.containsKey(hitEntity)) {
                                float[] rgb = entityColors.get(hitEntity);
                                if (rgb == null) {
                                    rList.add(1.0f);
                                    gList.add(1.0f);
                                    bList.add(1.0f);
                                } else {
                                    rList.add(rgb[0]);
                                    gList.add(rgb[1]);
                                    bList.add(rgb[2]);
                                }
                            }

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

    private float[] getEntityColor(Entity entity) {
        try {
            Minecraft mc = Minecraft.getInstance();
            EntityRenderDispatcher dispatcher = mc.getEntityRenderDispatcher();
            ResourceLocation texLocation = dispatcher.getRenderer(entity).getTextureLocation(entity);

            if (texLocation == null || texLocation.getPath().startsWith("textures/atlas/")) {
                LOGGER.warn("Skipping texture for entity: {}", texLocation);
                return null;
            }

            var resourceOpt = mc.getResourceManager().getResource(texLocation);  // ← ここ変更
            if (resourceOpt.isEmpty()) {
                LOGGER.warn("Texture not found for entity: {}", texLocation);
                return null;
            }

            InputStream stream = resourceOpt.get().open();
            BufferedImage img = ImageIO.read(stream);

            int width = img.getWidth();
            int height = img.getHeight();
            long rSum = 0, gSum = 0, bSum = 0, count = 0;
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    int argb = img.getRGB(x, y);
                    int alpha = (argb >> 24) & 0xFF;
                    if (alpha < 16) continue;
                    int r = (argb >> 16) & 0xFF;
                    int g = (argb >> 8) & 0xFF;
                    int b = argb & 0xFF;
                    rSum += r;
                    gSum += g;
                    bSum += b;
                    count++;
                }
            }
            if (count == 0) return null;
            return new float[]{
                rSum / (float) count / 255.0f,
                gSum / (float) count / 255.0f,
                bSum / (float) count / 255.0f
            };
        } catch (Exception e) {
            LOGGER.warn("Failed to load texture for entity {}: {}", entity.getType(), e.toString());
            return null;
        }
    }

    private ChannelFloat32 createChannel(String name, List<Float> values) {
        ChannelFloat32 channel = new ChannelFloat32();
        channel.setName(name);
        channel.setValues(values);
        return channel;
    }
}
