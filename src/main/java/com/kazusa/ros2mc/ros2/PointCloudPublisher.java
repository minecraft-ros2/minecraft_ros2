package com.kazusa.ros2mc.ros2;
import com.kazusa.ros2mc.ros2.Point3D;

import net.minecraft.client.Minecraft;
import net.minecraft.client.renderer.entity.EntityRenderDispatcher;
import net.minecraft.core.BlockPos;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.shapes.VoxelShape;
import net.minecraft.world.phys.Vec3;
import net.minecraft.world.phys.HitResult;
import net.minecraft.world.phys.BlockHitResult;
import net.minecraft.world.level.ClipContext;
import net.minecraft.world.level.ClipContext.Block;
import net.minecraft.world.level.ClipContext.Fluid;
import net.minecraft.world.phys.EntityHitResult;
import net.minecraft.world.entity.projectile.ProjectileUtil;

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
import java.util.Objects;
import java.util.Optional;


public class PointCloudPublisher extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(PointCloudPublisher.class);

    private final Publisher<PointCloud> publisher;
    private final Publisher<TFMessage> tfPublisher;
    private final WallTimer timer;
    private final Minecraft minecraft;

    private final List<Point3D> baseVector = new ArrayList<>();

    // Exmaple parameter: Hesai XT32
    private final double horizontalResolutionDeg = 0.18;  // 水平角解像度[°]
    private final double verticalResolutionDeg   = 1.0;   // 垂直角解像度[°]
    private final double verticalFovDeg          = 31.0;  // 垂直視野角[°]
    private final double minimumDistance         = 0.05;  // 最低測定距離[m]
    private final double maxmiumDistance         = 120;   // 最大測定距離[m]

    public PointCloudPublisher() {
        super("minecraft_pointcloud_publisher");
        publisher = this.node.createPublisher(PointCloud.class, "/player/pointcloud");
        tfPublisher = this.node.createPublisher(TFMessage.class, "/tf");
        minecraft = Minecraft.getInstance();
        timer = node.createWallTimer(100, TimeUnit.MILLISECONDS, this::publishAsyncLidarScan);
        LOGGER.info("Initialized with horizRes={}° vertRes={}° vertFOV={}°",
            horizontalResolutionDeg, verticalResolutionDeg, verticalFovDeg);

        int vertSteps = (int)(verticalFovDeg / verticalResolutionDeg) + 1;
        int horizSteps = (int)(360.0 / horizontalResolutionDeg) + 1;

        double minPitch = Math.toRadians(-verticalFovDeg/2);
        double maxPitch = Math.toRadians(verticalFovDeg/2);
        for (int iv = 0; iv < vertSteps; iv++) {
            double pitch = minPitch + (maxPitch-minPitch) * iv/(vertSteps-1);
            for (int ih = 0; ih < horizSteps; ih++){
                double yaw = Math.toRadians(-180 + ih*horizontalResolutionDeg);
                double dx = Math.cos(pitch)*Math.sin(yaw);
                double dy = Math.sin(pitch);
                double dz = Math.cos(pitch)*Math.cos(yaw);
                baseVector.add(new Point3D(dx, dy, dz));
            }
        }
    }

    private static record ScanResult(Point32 pt, float r, float g, float b) {}

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
            long startNano = System.nanoTime();

            List<Entity> entities = level.getEntities(
                player,
                new AABB(px - maxmiumDistance, py - maxmiumDistance, pz - maxmiumDistance,
                        px + maxmiumDistance, py + maxmiumDistance, pz + maxmiumDistance),
                e -> !e.is(player)
            );
            Map<Entity, float[]> entityColors = new HashMap<>();
            for (Entity e : entities) {
                entityColors.put(e, getEntityColor(e));
            }

            double cosP = Math.cos(pitchRadPlayer), sinP = Math.sin(pitchRadPlayer);
            double cosY = Math.cos(yawRadPlayer),   sinY = Math.sin(yawRadPlayer);

            List<ScanResult> results = baseVector.parallelStream()
                .map(dir -> {
                    Vec3 start = new Vec3(
                        px + dir.x * minimumDistance,
                        py + dir.y * minimumDistance,
                        pz + dir.z * minimumDistance
                    );
                    Vec3 end = start.add(
                        dir.x * maxmiumDistance,
                        dir.y * maxmiumDistance,
                        dir.z * maxmiumDistance
                    );

                    BlockHitResult bhr = level.clip(
                        new ClipContext(start, end,
                                        ClipContext.Block.OUTLINE,
                                        ClipContext.Fluid.NONE,
                                        player)
                    );
                    double blockDist = Double.POSITIVE_INFINITY;
                    Vec3   blockHit  = null;
                    if (bhr.getType() == HitResult.Type.BLOCK) {
                        blockHit  = bhr.getLocation();
                        blockDist = blockHit.distanceTo(start);
                    }

                    Entity closestE = null;
                    Vec3   entityHit = null;
                    double entityDist = Double.POSITIVE_INFINITY;
                    for (Entity e : entities) {
                        var bb = e.getBoundingBox();
                        Vec3 hit = bb.clip(start, end).orElse(null);
                        if (hit != null) {
                            double d = hit.distanceTo(start);
                            if (d < entityDist) {
                                entityDist = d;
                                entityHit  = hit;
                                closestE   = e;
                            }
                        }
                    }

                    if (entityHit != null && entityDist < blockDist) {
                        Vec3 hit = entityHit;
                        double rx = hit.x - px, ry = hit.y - py, rz = hit.z - pz;
                        double ryP = ry * cosP - rz * sinP;
                        double rzP = ry * sinP + rz * cosP;
                        double rxY = rx * cosY + rzP * sinY;
                        double rzY = -rx * sinY + rzP * cosY;

                        Point32 pt = new Point32();
                        pt.setX((float) rzY);
                        pt.setY((float) rxY);
                        pt.setZ((float) ryP);

                        float[] col = entityColors.getOrDefault(closestE, new float[]{1f,0f,0f});
                        return new ScanResult(pt, col[0], col[1], col[2]);
                    } else if (blockHit != null) {
                        Vec3 hit = blockHit;
                        double rx = hit.x - px, ry = hit.y - py, rz = hit.z - pz;
                        double ryP = ry * cosP - rz * sinP;
                        double rzP = ry * sinP + rz * cosP;
                        double rxY = rx * cosY + rzP * sinY;
                        double rzY = -rx * sinY + rzP * cosY;

                        Point32 pt = new Point32();
                        pt.setX((float) rzY);
                        pt.setY((float) rxY);
                        pt.setZ((float) ryP);

                        BlockPos bp = bhr.getBlockPos();
                        int c = level.getBlockState(bp).getMapColor(level, bp).col;
                        float rf = ((c >> 16) & 0xFF) / 255f;
                        float gf = ((c >>  8) & 0xFF) / 255f;
                        float bf = ( c        & 0xFF) / 255f;

                        return new ScanResult(pt, rf, gf, bf);
                    }
                    return null;
                })
                .filter(Objects::nonNull)
                .collect(Collectors.toList());

            if (results.isEmpty()) {
                LOGGER.warn("No points detected in pointcloud scan, skipping publish.");
                return;
            }

            List<Point32> points = results.stream()
                                        .map(ScanResult::pt)
                                        .collect(Collectors.toList());
            List<Float>   rList   = results.stream()
                                        .map(ScanResult::r)
                                        .collect(Collectors.toList());
            List<Float>   gList   = results.stream()
                                        .map(ScanResult::g)
                                        .collect(Collectors.toList());
            List<Float>   bList   = results.stream()
                                        .map(ScanResult::b)
                                        .collect(Collectors.toList());

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

            // --- PointCloud メッセージ作成・送信 ---
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

            long endNano = System.nanoTime();
            double elapsedMs = (endNano - startNano) / 1_000_000.0;
            LOGGER.info("LIDAR scan compute time: {} ms", String.format("%.2f", elapsedMs));
        });
    }


    /**
     * エンティティのテクスチャから平均色を計算して返す
     * @return RGB(float[3]) or null
     */
    private float[] getEntityColor(Entity entity) {
        try {
            Minecraft mc = Minecraft.getInstance();
            EntityRenderDispatcher dispatcher = mc.getEntityRenderDispatcher();
            ResourceLocation texLocation = dispatcher.getRenderer(entity).getTextureLocation(entity);

            // テクスチャなし、またはアトラスの場合はスキップ
            if (texLocation == null || texLocation.getPath().startsWith("textures/atlas/")) {
                // LOGGER.warn("Skipping texture for entity: {}", texLocation);
                return null;
            }

            var resourceOpt = mc.getResourceManager().getResource(texLocation);
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
                    if (alpha < 16) continue;  // 透明度の低いピクセルは無視
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

    /**
     * PointCloud の ChannelFloat32 を生成するヘルパー
     */
    private ChannelFloat32 createChannel(String name, List<Float> values) {
        ChannelFloat32 channel = new ChannelFloat32();
        channel.setName(name);
        channel.setValues(values);
        return channel;
    }
}
