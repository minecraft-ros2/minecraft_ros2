package com.kazusa.minecraft_ros2.ros2;

import net.minecraft.client.Minecraft;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.entity.EquipmentSlot;
import net.minecraft.world.item.ItemStack;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.ClipContext;
import net.minecraft.world.level.ClipContext.Block;
import net.minecraft.world.level.ClipContext.Fluid;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.BlockHitResult;
import net.minecraft.world.phys.HitResult;
import net.minecraft.world.phys.Vec3;
import net.minecraftforge.registries.ForgeRegistries;
import org.ros2.rcljava.Time;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.publisher.Publisher;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import geometry_msgs.msg.Point32;
import geometry_msgs.msg.TransformStamped;
import sensor_msgs.msg.PointField;
import sensor_msgs.msg.PointCloud2;
import std_msgs.msg.Header;
import tf2_msgs.msg.TFMessage;
import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.InputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Collectors;


public class PointCloudPublisher extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(PointCloudPublisher.class);

    private final Publisher<PointCloud2> publisher;
    private final Publisher<TFMessage> tfPublisher;
    private final Minecraft minecraft;
    private final List<Point3D> baseVector = new ArrayList<>();
    private PointCloud2 msg;
    private TFMessage tfMsg;
    private String lidarName;
    double halfHoriz;
    private final Map<ResourceLocation, float[]> textureColorCache = new HashMap<>();
    private final AtomicBoolean publishPC2 = new AtomicBoolean(false);

    // Parameters
    private double horizontalFovDeg = 360.0;
    private double horizontalResDeg = 0.18;
    private double verticalResDeg   = 1.0;
    private double verticalFovDeg   = 31.0;
    private double minDistance      = 0.05;
    private double maxDistance      = 120.0;

    private final boolean publishTF       = true;


    public PointCloudPublisher() {
        super("minecraft_pointcloud_publisher");
        publisher = node.createPublisher(PointCloud2.class, "/player/pointcloud");
        tfPublisher = node.createPublisher(TFMessage.class, "/tf");
        minecraft = Minecraft.getInstance();
        initBaseVector();
        preloadAllEntityTextures();
        node.createWallTimer(100, TimeUnit.MILLISECONDS, this::publishAsyncLidarScan);
    }

    private void initBaseVector() {
        baseVector.clear();
        int horizSteps = (int)(360.0 / horizontalResDeg) + 1;

        if (verticalFovDeg < 0.001) {
            for (int ih = 0; ih < horizSteps; ih++) {
                double yaw = Math.toRadians(-180 + ih * horizontalResDeg);
                baseVector.add(new Point3D(
                    Math.sin(yaw),  // x
                    0.0,          // y = 0
                    Math.cos(yaw)   // z
                ));
            }
        } else {
            int vertSteps = (int)(verticalFovDeg / verticalResDeg) + 1;
            double minPitch = Math.toRadians(-verticalFovDeg / 2);
            double maxPitch = Math.toRadians(verticalFovDeg / 2);

            for (int iv = 0; iv < vertSteps; iv++) {
                double pitch = minPitch + (maxPitch - minPitch) * iv / (vertSteps - 1);
                for (int ih = 0; ih < horizSteps; ih++) {
                    double yaw = Math.toRadians(-180 + ih * horizontalResDeg);
                    baseVector.add(new Point3D(
                        Math.cos(pitch) * Math.sin(yaw),  // x
                        Math.sin(pitch),                  // y
                        Math.cos(pitch) * Math.cos(yaw)   // z
                    ));
                }
            }
        }

        LOGGER.info("Initialized in {}-mode: horizRes={}° vertRes={}° vertFOV={}°",
            (verticalFovDeg < 0.001) ? "2D" : "3D",
            horizontalResDeg,
            verticalResDeg,
            verticalFovDeg
        );
    }

    private void playerHaveLiDAR() {
        @SuppressWarnings("null")
        ItemStack helmet = minecraft.player.getItemBySlot(EquipmentSlot.HEAD);
        ResourceLocation key = ForgeRegistries.ITEMS.getKey(helmet.getItem());
        String objectKeyName = key.toString();
        boolean isRos2 = objectKeyName.contains("minecraft_ros2");

        if (isRos2) {
            if (lidarName == null || !lidarName.equals(objectKeyName)) {
                lidarName = objectKeyName;
                setLiDARParameter(lidarName);
                initBaseVector();
            }
            publishPC2.set(true);

        } else {
            if (publishPC2.get()) {
                baseVector.clear();
            }
            lidarName = null;
            publishPC2.set(false);
        }
    }

    private void setLiDARParameter(String name) {
        switch (name) {
            case "minecraft_ros2:velodyne_vlp16":
                horizontalFovDeg = 360.0;
                horizontalResDeg = 0.2;
                verticalFovDeg   = 31.0;
                verticalResDeg   = 2.0;
                minDistance      = 0.1;
                maxDistance      = 100.0;
                break;
            case "minecraft_ros2:hesai_xt32":
                horizontalFovDeg = 360.0;
                horizontalResDeg = 0.18;
                verticalFovDeg   = 31.0;
                verticalResDeg   = 1.0;
                minDistance      = 0.05;
                maxDistance      = 120.0;
                break;
            case "minecraft_ros2:hesai_ft120":
                horizontalFovDeg = 100.0;
                horizontalResDeg = 0.625;
                verticalFovDeg   = 75.0;
                verticalResDeg   = 0.625;
                minDistance      = 0.1;
                maxDistance      = 22.0;
                break;
            case "minecraft_ros2:rs_lidar_m1":
                horizontalFovDeg = 120.0;
                horizontalResDeg = 0.2;
                verticalFovDeg   = 25.0;
                verticalResDeg   = 0.2;
                minDistance      = 0.05;
                maxDistance      = 120.0;
                break;
            case "minecraft_ros2:utm_30ln":
                horizontalFovDeg = 270.0;
                horizontalResDeg = 0.25;
                verticalFovDeg   = 0.0;
                verticalResDeg   = 1.0;
                minDistance      = 0.05;
                maxDistance      = 30.0;
                break;
            default:
                horizontalFovDeg = 360.0;
                horizontalResDeg = 1.0;
                verticalFovDeg   = 31.0;
                verticalResDeg   = 2.0;
                minDistance      = 0.1;
                maxDistance      = 10.0;
                break;
        }
        halfHoriz = Math.toRadians(horizontalFovDeg / 2.0);
        LOGGER.info("LiDAR '{}' parameters → horiz={}° vert={}° FOV={}° dist=[{},{}]", 
                    name, horizontalResDeg, verticalResDeg, verticalFovDeg, minDistance, maxDistance);
    }

    private static record ScanResult(Point32 pt, float r, float g, float b) {}

    private void publishAsyncLidarScan() {
        Minecraft mc = minecraft;
        Entity player = mc.player;
        Level level = mc.level;

        if (player == null || level == null) return;
        playerHaveLiDAR();
        if (publishPC2.get() == false) return;

        if (textureColorCache.isEmpty()) {
            preloadAllEntityTextures();
        }

        double px = player.getX();
        double py = player.getY() + player.getEyeHeight();
        double pz = player.getZ();
        double yawRad   = Math.toRadians(player.getYRot());
        double pitchRad = 0.0;

        CompletableFuture.runAsync(() -> {
            long start = System.nanoTime();

            List<Entity> entities = gatherEntities(level, player, px, py, pz);
            Map<Entity, float[]> colors = computeEntityColors(entities);
            double cosP = Math.cos(pitchRad), sinP = Math.sin(pitchRad);
            double cosY = Math.cos(yawRad),   sinY = Math.sin(yawRad);

            double elapsed_1 = (System.nanoTime() - start) / 1_000_000.0;
            start = System.nanoTime();

            List<ScanResult> results = performLidarScan(px, py, pz, cosP, sinP, cosY, sinY, level, entities, colors);
            if (results.isEmpty()) {
                LOGGER.warn("No points detected, skipping publish.");
                return;
            }

            double elapsed_2 = (System.nanoTime() - start) / 1_000_000.0;
            start = System.nanoTime();

            if (publishTF) {
                try {
                    publishTransform(px, py, pz, yawRad, pitchRad);
                } catch (Exception e) {
                    LOGGER.error("Failed to publish transform", e);
                }
            }

            try {
                publishPointCloud(results);
            } catch (Exception e) {
                LOGGER.error("Failed to publish point cloud", e);
            }

            double elapsed_3 = (System.nanoTime() - start) / 1_000_000.0;

            // LOGGER.info("Color: {} ms, LiDAR: {} ms, ROS2: {} ms", String.format("%.2f", elapsed_1), String.format("%.2f", elapsed_2), String.format("%.2f", elapsed_3));
        });
    }

    private List<Entity> gatherEntities(Level level, Entity player, double px, double py, double pz) {
        AABB area = new AABB(px - maxDistance, py - maxDistance, pz - maxDistance,
                             px + maxDistance, py + maxDistance, pz + maxDistance);
        return level.getEntities(player, area, e -> !e.is(player));
    }

    private List<ScanResult> performLidarScan(double px, double py, double pz,
                                              double cosP, double sinP,
                                              double cosY, double sinY,
                                              Level level,
                                              List<Entity> entities,
                                              Map<Entity, float[]> colors) {
        return baseVector.parallelStream()
            .map(dir -> scanDirection(dir, px, py, pz, cosP, sinP, cosY, sinY, level, entities, colors))
            .filter(Objects::nonNull)
            .collect(Collectors.toList());
    }

    private ScanResult scanDirection(Point3D dir,
                                     double px, double py, double pz,
                                     double cosP, double sinP,
                                     double cosY, double sinY,
                                     Level level,
                                     List<Entity> entities,
                                     Map<Entity, float[]> colors) {
        try {
            Vec3 start = new Vec3(px + dir.x * minDistance,
                                py + dir.y * minDistance,
                                pz + dir.z * minDistance);
            Vec3 end = start.add(dir.x * maxDistance,
                                dir.y * maxDistance,
                                dir.z * maxDistance);
            @SuppressWarnings("null")
            BlockHitResult bhr =
                level.clip(new ClipContext(start, end, Block.OUTLINE, Fluid.NONE, minecraft.player));
            String blockName = level.getBlockState(bhr.getBlockPos()).getBlock().toString().toLowerCase();
            if (blockName.contains("glass"))return null;

            HitData hd = findClosestHit(start, bhr, level, entities);
            if (hd == null) return null;

            Point32 pt = rotateToSensorFrame(hd.location, px, py, pz, cosP, sinP, cosY, sinY);
            float[] col = hd.entity != null ? colors.get(hd.entity) : hd.blockColor;
            double azimuth = Math.atan2(pt.getY(), pt.getX());
            if (azimuth < -halfHoriz || azimuth > halfHoriz) {
                return null;
            }
            return new ScanResult(pt, col[0], col[1], col[2]);
        } catch (Exception e) {
            LOGGER.error("Error scanning direction {}", dir, e);
            return null;
        }
    }

    private static class HitData {
        Vec3    location;
        Entity  entity;
        float[] blockColor;
    }

    private HitData findClosestHit(Vec3 start,
                                   BlockHitResult bhr,
                                   Level level,
                                   List<Entity> entities) {
        double blockDist = Double.POSITIVE_INFINITY;
        Vec3 blockHit = null;
        if (bhr.getType() == HitResult.Type.BLOCK) {
            blockHit = bhr.getLocation();
            blockDist = start.distanceTo(blockHit);
        }

        Entity closestE = null;
        Vec3 entityHit = null;
        double entityDist = Double.POSITIVE_INFINITY;
        for (Entity e : entities) {
            Vec3 hit = e.getBoundingBox().clip(start, bhr.getLocation()).orElse(null);
            if (hit != null) {
                double d = hit.distanceTo(start);
                if (d < entityDist) {
                    entityDist = d;
                    entityHit = hit;
                    closestE = e;
                }
            }
        }

        if (entityHit != null && entityDist < blockDist) {
            HitData hd = new HitData();
            hd.location   = entityHit;
            hd.entity     = closestE;
            return hd;
        } else if (blockHit != null) {
            HitData hd = new HitData();
            hd.location   = blockHit;
            int c = level.getBlockState(bhr.getBlockPos())
                         .getMapColor(level, bhr.getBlockPos()).col;
            
            hd.blockColor = new float[]{((c >> 16) & 0xFF) / 255f,
                                        ((c >> 8)  & 0xFF) / 255f,
                                        (c & 0xFF)        / 255f};
            return hd;
        }
        return null;
    }

    private Point32 rotateToSensorFrame(Vec3 hit,
                                        double px, double py, double pz,
                                        double cosP, double sinP,
                                        double cosY, double sinY) {
        double rx = hit.x - px;
        double ry = hit.y - py;
        double rz = hit.z - pz;
        double ryP = ry * cosP - rz * sinP;
        double rzP = ry * sinP + rz * cosP;
        double rxY = rx * cosY + rzP * sinY;
        double rzY = -rx * sinY + rzP * cosY;

        Point32 pt = new Point32();
        pt.setX((float) rzY);
        pt.setY((float) rxY);
        pt.setZ((float) ryP);
        return pt;
    }

    private void publishTransform(double px, double py, double pz,
                                  double yawRad, double pitchRad) {
        TransformStamped transform = new TransformStamped();
        Header tfHeader = new Header();
        tfHeader.setStamp(Time.now());
        tfHeader.setFrameId("world");
        transform.setHeader(tfHeader);
        transform.setChildFrameId("player");

        transform.getTransform().getTranslation().setX(pz);
        transform.getTransform().getTranslation().setY(px);
        transform.getTransform().getTranslation().setZ(py - 63.0);

        double cy = Math.cos(-yawRad * 0.5);
        double sy = Math.sin(-yawRad * 0.5);
        double cp = Math.cos(pitchRad * 0.5);
        double sp = Math.sin(pitchRad * 0.5);
        double cr = 1.0;
        double sr = 0.0;

        double qw = cr * cp * cy + sr * sp * sy;
        double qx = sr * cp * cy - cr * sp * sy;
        double qy = cr * sp * cy + sr * cp * sy;
        double qz = cr * cp * sy - sr * sp * cy;

        transform.getTransform().getRotation().setX(qx);
        transform.getTransform().getRotation().setY(qy);
        transform.getTransform().getRotation().setZ(qz);
        transform.getTransform().getRotation().setW(qw);

        if (tfMsg == null) {
            tfMsg = new TFMessage();
        }
        tfMsg.setTransforms(List.of(transform));
        tfPublisher.publish(tfMsg);
    }

    private void publishPointCloud(List<ScanResult> results) {
        if (msg == null) {
            msg = new PointCloud2();
        }
        msg.getHeader().setStamp(Time.now());
        msg.getHeader().setFrameId("player");

        int pointCount = results.size();
        msg.setHeight(1);
        msg.setWidth(pointCount);

        List<PointField> fields = new ArrayList<>();
        fields.add(createField("x", 0, PointField.FLOAT32, 1));
        fields.add(createField("y", 4, PointField.FLOAT32, 1));
        fields.add(createField("z", 8, PointField.FLOAT32, 1));
        fields.add(createField("rgb", 12, PointField.FLOAT32, 1));
        msg.setFields(fields);

        msg.setIsBigendian(false);
        int pointStep = 16;
        msg.setPointStep(pointStep);
        msg.setRowStep(pointStep * pointCount);
        msg.setIsDense(true);

        ByteBuffer buf = ByteBuffer.allocate(pointCount * pointStep).order(ByteOrder.LITTLE_ENDIAN);
        for (ScanResult r : results) {
            buf.putFloat(r.pt.getX());
            buf.putFloat(r.pt.getY());
            buf.putFloat(r.pt.getZ());
            int ir = (int)(r.r * 255) & 0xFF;
            int ig = (int)(r.g * 255) & 0xFF;
            int ib = (int)(r.b * 255) & 0xFF;
            int rgbInt = (ir << 16) | (ig << 8) | ib;
            buf.putFloat(Float.intBitsToFloat(rgbInt));
        }
        buf.rewind();
        List<Byte> data = new ArrayList<>(buf.capacity());
        while (buf.hasRemaining()) data.add(buf.get());
        msg.setData(data);

        publisher.publish(msg);
    }

    /**
     * Get average color of Entity
     */

    private void preloadAllEntityTextures() {
        var dispatcher = minecraft.getEntityRenderDispatcher();
        dispatcher.renderers.values().forEach(renderer -> {
            try {
                @SuppressWarnings("null")
                ResourceLocation loc = renderer.getTextureLocation(null);
                if (loc == null || loc.getPath().startsWith("textures/atlas/")) return;
                textureColorCache.computeIfAbsent(loc, this::computeAverageColor);
            } catch (Exception e) {
                LOGGER.warn("Failed to preload: {}", e.toString());
            }
        });
    }

    private float[] computeAverageColor(ResourceLocation loc) {
        try (InputStream in = minecraft.getResourceManager().getResource(loc).get().open()) {
            BufferedImage img = ImageIO.read(in);
            long r=0, g=0, b=0, count=0;
            for (int y = 0; y < img.getHeight(); y++) {
                for (int x = 0; x < img.getWidth(); x++) {
                    int argb = img.getRGB(x, y);
                    if (((argb >>> 24) & 0xFF) < 16) continue;
                    r += (argb >> 16) & 0xFF;
                    g += (argb >> 8) & 0xFF;
                    b += argb & 0xFF;
                    count++;
                }
            }
            if (count == 0) return new float[]{1f,1f,1f};
            return new float[]{r / (float)(count * 255), g / (float)(count * 255), b / (float)(count * 255)};
        } catch (IOException e) {
            LOGGER.warn("Failed to load texture {}: {}", loc, e);
            return new float[]{1f,1f,1f};
        }
    }

    private Map<Entity, float[]> computeEntityColors(List<Entity> entities) {
        var map = new HashMap<Entity, float[]>();
        for (Entity e : entities) {
            try {
                ResourceLocation loc = minecraft.getEntityRenderDispatcher()
                                                 .getRenderer(e)
                                                 .getTextureLocation(e);
                float[] col = (loc != null)
                    ? textureColorCache.getOrDefault(loc, new float[]{1f,1f,1f})
                    : new float[]{1f,1f,1f};
                map.put(e, col);
            } catch (Exception ex) {
                map.put(e, new float[]{1f,1f,1f});
            }
        }
        return map;
    }

    private PointField createField(String name, int offset, byte datatype, int count) {
        PointField f = new PointField();
        f.setName(name);
        f.setOffset(offset);
        f.setDatatype(datatype);
        f.setCount(count);
        return f;
    }
}
