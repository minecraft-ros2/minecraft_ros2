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

    // Exmaple parameter: Hesai XT32
    private final double horizontalResolutionDeg = 1.8;  // 水平角解像度
    private final double verticalResolutionDeg   = 1.0;  // 垂直角解像度
    private final double verticalFovDeg          = 31.0; // 垂直視野角

    public PointCloudPublisher() {
        super("minecraft_pointcloud_publisher");
        publisher = this.node.createPublisher(PointCloud.class, "/player/pointcloud");
        tfPublisher = this.node.createPublisher(TFMessage.class, "/tf");
        minecraft = Minecraft.getInstance();
        timer = node.createWallTimer(100, TimeUnit.MILLISECONDS, this::publishAsyncLidarScan);
        LOGGER.info("Initialized with horizRes={}° vertRes={}° vertFOV={}°",
            horizontalResolutionDeg, verticalResolutionDeg, verticalFovDeg);
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
            double maxDistance = 20.0;
            double stepSize = 0.1;
            List<Point32> points = new ArrayList<>();
            List<Float> rList = new ArrayList<>();
            List<Float> gList = new ArrayList<>();
            List<Float> bList = new ArrayList<>();

            // 垂直FOVと解像度からステップ数を計算
            int vertSteps = (int)(verticalFovDeg / verticalResolutionDeg) + 1;
            int horizSteps = (int)(360.0 / horizontalResolutionDeg) + 1;

            double minPitch = Math.toRadians(-verticalFovDeg/2);
            double maxPitch = Math.toRadians(verticalFovDeg/2);

            // エンティティ色情報事前取得
            List<Entity> entities = level.getEntities(
                player,
                new AABB(px - maxDistance, py - maxDistance, pz - maxDistance,
                         px + maxDistance, py + maxDistance, pz + maxDistance),
                e -> !e.is(player)
            );
            Map<Entity, float[]> entityColors = new HashMap<>();
            for (Entity e : entities) {
                entityColors.put(e, getEntityColor(e));
            }

            for (int iv = 0; iv < vertSteps; iv++) {
                double pitch = minPitch + (maxPitch-minPitch) * iv/(vertSteps-1);
                for (int ih = 0; ih < horizSteps; ih++) {
                    double yaw = Math.toRadians(-180 + ih*horizontalResolutionDeg);
                    double dx = Math.cos(pitch)*Math.sin(yaw);
                    double dy = Math.sin(pitch);
                    double dz = Math.cos(pitch)*Math.cos(yaw);

                    for (double dist = 0; dist <= maxDistance; dist += stepSize) {
                        double tx = px + dx*dist;
                        double ty = py + dy*dist;
                        double tz = pz + dz*dist;

                        BlockPos bp = new BlockPos((int)Math.floor(tx),(int)Math.floor(ty),(int)Math.floor(tz));
                        BlockState bs = level.getBlockState(bp);
                        if (bs.getBlock().toString().toLowerCase().contains("glass")) continue;
                        VoxelShape vs = bs.getCollisionShape(level,bp);
                        boolean hitBlock=false;
                        if(!vs.isEmpty()){
                            for(AABB a:vs.toAabbs()){ if(a.move(bp).contains(tx,ty,tz)){ hitBlock=true; break; }}
                        }
                        Entity hitEntity=null;
                        if(!hitBlock){ for(Entity e:entities){ if(e.getBoundingBox().contains(tx,ty,tz)){ hitEntity=e; break; } }}

                        if(hitBlock||hitEntity!=null){
                            double rx=tx-px, ry=ty-py, rz=tz-pz;
                            double cosP=Math.cos(pitchRadPlayer), sinP=Math.sin(pitchRadPlayer);
                            double ryP=ry*cosP-rz*sinP, rzP=ry*sinP+rz*cosP;
                            double cosY=Math.cos(yawRadPlayer), sinY=Math.sin(yawRadPlayer);
                            double rxY=rx*cosY+rzP*sinY, rzY=-rx*sinY+rzP*cosY;
                            Point32 pt=new Point32(); pt.setX((float)rzY);
                            pt.setY((float)rxY); pt.setZ((float)ryP); points.add(pt);
                            if(hitBlock){int c=bs.getMapColor(level,bp).col;
                                rList.add(((c>>16)&0xFF)/255f);
                                gList.add(((c>>8)&0xFF)/255f);
                                bList.add((c&0xFF)/255f);
                            } else {
                                float[] col=entityColors.get(hitEntity);
                                if(col==null){rList.add(1f);gList.add(1f);bList.add(1f);} else {rList.add(col[0]);gList.add(col[1]);bList.add(col[2]);}
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
                LOGGER.warn("Skipping texture for entity: {}", texLocation);
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
