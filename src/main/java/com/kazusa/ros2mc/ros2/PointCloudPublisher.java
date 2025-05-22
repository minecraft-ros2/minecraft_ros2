package com.kazusa.ros2mc.ros2;

import com.kazusa.ros2mc.ros2.Point3D;
import net.minecraft.client.Minecraft;
import net.minecraft.client.renderer.entity.EntityRenderDispatcher;
import net.minecraft.core.BlockPos;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.ClipContext;
import net.minecraft.world.level.ClipContext.Block;
import net.minecraft.world.level.ClipContext.Fluid;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.BlockHitResult;
import net.minecraft.world.phys.HitResult;
import net.minecraft.world.phys.Vec3;
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
import java.util.stream.Collectors;

public class PointCloudPublisher extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(PointCloudPublisher.class);

    private final Publisher<PointCloud> publisher;
    private final Publisher<TFMessage> tfPublisher;
    private final WallTimer timer;
    private final Minecraft minecraft;
    private final List<Point3D> baseVector = new ArrayList<>();

    // Parameters
    private final double horizontalResDeg = 0.18;
    private final double verticalResDeg   = 1.0;
    private final double verticalFovDeg   = 31.0;
    private final double minDistance      = 0.05;
    private final double maxDistance      = 120.0;

    public PointCloudPublisher() {
        super("minecraft_pointcloud_publisher");
        publisher = node.createPublisher(PointCloud.class, "/player/pointcloud");
        tfPublisher = node.createPublisher(TFMessage.class, "/tf");
        minecraft = Minecraft.getInstance();
        initBaseVector();
        timer = node.createWallTimer(100, TimeUnit.MILLISECONDS, this::publishAsyncLidarScan);
        LOGGER.info("Initialized with horizRes={}° vertRes={}° vertFOV={}°", horizontalResDeg, verticalResDeg, verticalFovDeg);
    }

    private void initBaseVector() {
        int vertSteps = (int)(verticalFovDeg / verticalResDeg) + 1;
        int horizSteps = (int)(360.0 / horizontalResDeg) + 1;
        double minPitch = Math.toRadians(-verticalFovDeg / 2);
        double maxPitch = Math.toRadians(verticalFovDeg / 2);

        for (int iv = 0; iv < vertSteps; iv++) {
            double pitch = minPitch + (maxPitch - minPitch) * iv / (vertSteps - 1);
            for (int ih = 0; ih < horizSteps; ih++) {
                double yaw = Math.toRadians(-180 + ih * horizontalResDeg);
                baseVector.add(new Point3D(
                    Math.cos(pitch) * Math.sin(yaw),
                    Math.sin(pitch),
                    Math.cos(pitch) * Math.cos(yaw)
                ));
            }
        }
    }

    private static record ScanResult(Point32 pt, float r, float g, float b) {}

    private void publishAsyncLidarScan() {
        if (minecraft.player == null || minecraft.level == null) return;
        double px = minecraft.player.getX();
        double py = minecraft.player.getY() + minecraft.player.getEyeHeight();
        double pz = minecraft.player.getZ();
        double yawRad   = Math.toRadians(minecraft.player.getYRot());
        double pitchRad = 0.0;

        Level level = minecraft.level;
        Entity player = minecraft.player;

        CompletableFuture.runAsync(() -> {
            long start = System.nanoTime();

            List<Entity> entities = gatherEntities(level, player, px, py, pz);
            Map<Entity, float[]> colors = computeEntityColors(entities);
            double cosP = Math.cos(pitchRad), sinP = Math.sin(pitchRad);
            double cosY = Math.cos(yawRad),   sinY = Math.sin(yawRad);

            List<ScanResult> results = performLidarScan(px, py, pz, cosP, sinP, cosY, sinY, level, entities, colors);
            if (results.isEmpty()) {
                LOGGER.warn("No points detected, skipping publish.");
                return;
            }

            publishTransform(px, py, pz, yawRad, pitchRad);
            publishPointCloud(results);

            double elapsed = (System.nanoTime() - start) / 1_000_000.0;
            LOGGER.info("LIDAR scan compute time: {} ms", String.format("%.2f", elapsed));
        });
    }

    private List<Entity> gatherEntities(Level level, Entity player, double px, double py, double pz) {
        AABB area = new AABB(px - maxDistance, py - maxDistance, pz - maxDistance,
                             px + maxDistance, py + maxDistance, pz + maxDistance);
        return level.getEntities(player, area, e -> !e.is(player));
    }

    private Map<Entity, float[]> computeEntityColors(List<Entity> entities) {
        Map<Entity, float[]> map = new HashMap<>();
        for (Entity e : entities) {
            float[] c = getEntityColor(e);
            map.put(e, c != null ? c : new float[]{1f, 0f, 0f});
        }
        return map;
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
        Vec3 start = new Vec3(px + dir.x * minDistance,
                              py + dir.y * minDistance,
                              pz + dir.z * minDistance);
        Vec3 end = start.add(dir.x * maxDistance,
                             dir.y * maxDistance,
                             dir.z * maxDistance);
        BlockHitResult bhr =
            level.clip(new ClipContext(start, end, Block.OUTLINE, Fluid.NONE, minecraft.player));

        HitData hd = findClosestHit(start, bhr, level, entities);
        if (hd == null) return null;

        Point32 pt = rotateToSensorFrame(hd.location, px, py, pz, cosP, sinP, cosY, sinY);
        float[] col = hd.entity != null ? colors.get(hd.entity) : hd.blockColor;
        return new ScanResult(pt, col[0], col[1], col[2]);
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

        TFMessage msg = new TFMessage();
        msg.setTransforms(List.of(transform));
        tfPublisher.publish(msg);
    }

    private void publishPointCloud(List<ScanResult> results) {
        PointCloud msg = new PointCloud();
        Header header = new Header();
        header.setStamp(Time.now());
        header.setFrameId("player");
        msg.setHeader(header);
        msg.setPoints(results.stream().map(ScanResult::pt).collect(Collectors.toList()));

        List<ChannelFloat32> channels = new ArrayList<>();
        channels.add(createChannel("r", results.stream().map(r -> r.r).collect(Collectors.toList())));
        channels.add(createChannel("g", results.stream().map(r -> r.g).collect(Collectors.toList())));
        channels.add(createChannel("b", results.stream().map(r -> r.b).collect(Collectors.toList())));
        msg.setChannels(channels);

        publisher.publish(msg);
    }

    /**
     * Entity の平均色を取得
     */
    private float[] getEntityColor(Entity entity) {
        try {
            ResourceLocation loc = Minecraft.getInstance()
                .getEntityRenderDispatcher().getRenderer(entity)
                .getTextureLocation(entity);
            if (loc == null || loc.getPath().startsWith("textures/atlas/")) return null;
            var resOpt = Minecraft.getInstance().getResourceManager().getResource(loc);
            if (resOpt.isEmpty()) return null;
            BufferedImage img = ImageIO.read(resOpt.get().open());
            long r=0,g=0,b=0,c=0;
            for (int y=0;y<img.getHeight();y++) for (int x=0;x<img.getWidth();x++) {
                int argb=img.getRGB(x,y);
                if (((argb>>>24)&0xFF)<16) continue;
                r+=(argb>>16)&0xFF;
                g+=(argb>>8)&0xFF;
                b+=argb&0xFF;
                c++;
            }
            if (c==0) return null;
            return new float[]{r/(float)c/255f, g/(float)c/255f, b/(float)c/255f};
        } catch (Exception e) {
            LOGGER.warn("Failed to load texture: {}", e.toString());
            return null;
        }
    }

    private ChannelFloat32 createChannel(String name, List<Float> vals) {
        ChannelFloat32 ch = new ChannelFloat32();
        ch.setName(name);
        ch.setValues(vals);
        return ch;
    }
}
