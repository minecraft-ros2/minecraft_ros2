package com.kazusa.ros2mc.ros2;

import net.minecraft.client.Minecraft;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.phys.shapes.VoxelShape;
import org.ros2.rcljava.Time;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.timer.WallTimer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import sensor_msgs.msg.PointCloud2;
import sensor_msgs.msg.PointField;
import std_msgs.msg.Header;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.phys.AABB;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class PointCloudPublisher extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(PointCloudPublisher.class);

    private final Publisher<PointCloud2> publisher;
    private final WallTimer timer;
    private final Minecraft minecraft;

    public PointCloudPublisher() {
        super("minecraft_pointcloud_publisher");
        publisher = this.node.createPublisher(PointCloud2.class, "/player/pointcloud");
        minecraft = Minecraft.getInstance();
        timer = this.node.createWallTimer(200, TimeUnit.MILLISECONDS, this::publishLidarScan);
        LOGGER.info("PointCloudPublisher initialized and publishing to '/player/pointcloud'");
    }

    private void publishLidarScan() {
        if (minecraft.player == null || minecraft.level == null) return;

        double px = minecraft.player.getX();
        double py = minecraft.player.getY() + minecraft.player.getEyeHeight();
        double pz = minecraft.player.getZ();

        double yawRadPlayer = Math.toRadians(minecraft.player.getYRot());
        double pitchRadPlayer = 0.0;
        int verticalSteps = 90;
        int horizontalSteps = 90;
        double maxDistance = 15.0;
        List<Float> points = new ArrayList<>();

        List<Entity> entities = minecraft.level.getEntities(
            minecraft.player,
            new AABB(px - maxDistance, py - maxDistance, pz - maxDistance,
                     px + maxDistance, py + maxDistance, pz + maxDistance),
            entity -> !entity.is(minecraft.player)
        );

        for (int d = 0; d < verticalSteps; d++) {
            for (int h = 0; h < horizontalSteps; h++) {
                double pitch = Math.PI * ((double) d / verticalSteps - 0.5);
                double yaw = 2 * Math.PI * h / horizontalSteps;

                double dx = Math.cos(pitch) * Math.sin(yaw);
                double dy = Math.sin(pitch);
                double dz = Math.cos(pitch) * Math.cos(yaw);

                for (double dist = 0.0; dist <= maxDistance; dist += 0.02) {
                    double tx = px + dx * dist;
                    double ty = py + dy * dist;
                    double tz = pz + dz * dist;

                    BlockPos blockPos = new BlockPos((int) Math.floor(tx), (int) Math.floor(ty), (int) Math.floor(tz));
                    BlockState blockState = minecraft.level.getBlockState(blockPos);
                    VoxelShape shape = blockState.getCollisionShape(minecraft.level, blockPos);
                    boolean isBlockHit = false;

                    String blockName = blockState.getBlock().toString().toLowerCase();
                    if (blockName.contains("glass")) {
                        continue; // Skip transparent blocks
                    }

                    if (!shape.isEmpty()) {
                        for (AABB aabb : shape.toAabbs()) {
                            AABB worldAabb = aabb.move(blockPos);
                            if (worldAabb.contains(tx, ty, tz)) {
                                isBlockHit = true;
                                break;
                            }
                        }
                    }

                    boolean isEntityHit = false;
                    for (Entity entity : entities) {
                        if (entity.getBoundingBox().contains(tx, ty, tz)) {
                            isEntityHit = true;
                            break;
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

                        points.add(x_ros);
                        points.add(y_ros);
                        points.add(z_ros);
                        break; // Ray hit something
                    }
                }
            }
        }

        int pointCount = points.size() / 3;
        if (pointCount == 0) {
            LOGGER.warn("No points detected in pointcloud scan, skipping publish.");
            return;
        }

        ByteBuffer buffer = ByteBuffer.allocate(pointCount * 12).order(ByteOrder.LITTLE_ENDIAN);
        for (Float f : points) buffer.putFloat(f);

        PointCloud2 msg = new PointCloud2();
        Header header = new Header();
        header.setStamp(Time.now());
        header.setFrameId("player");
        msg.setHeader(header);

        msg.setHeight(1);
        msg.setWidth(pointCount);
        msg.setPointStep(12);
        msg.setRowStep(pointCount * 12);
        msg.setIsDense(true);
        msg.setIsBigendian(false);

        List<PointField> fields = new ArrayList<>();
        fields.add(createPointField("x", 0));
        fields.add(createPointField("y", 4));
        fields.add(createPointField("z", 8));
        msg.setFields(fields);
        msg.setData(buffer.array());

        publisher.publish(msg);
        // LOGGER.info("Published improved pointcloud with {} points", pointCount);
    }

    private PointField createPointField(String name, int offset) {
        PointField field = new PointField();
        field.setName(name);
        field.setOffset(offset);
        field.setDatatype(PointField.FLOAT32);
        field.setCount(1);
        return field;
    }
}
