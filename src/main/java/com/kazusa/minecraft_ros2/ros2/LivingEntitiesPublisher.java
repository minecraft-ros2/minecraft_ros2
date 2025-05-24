package com.kazusa.minecraft_ros2.ros2;

import net.minecraft.client.Minecraft;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.level.Level;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.Vec3;
import org.ros2.rcljava.Time;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.timer.WallTimer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import geometry_msgs.msg.Pose;
import geometry_msgs.msg.Point;
import geometry_msgs.msg.Quaternion;
import geometry_msgs.msg.Vector3;
import std_msgs.msg.Header;
import minecraft_msgs.msg.LivingEntityArray;
import minecraft_msgs.msg.MobCategory;

import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;
import java.util.HashMap;
import java.util.ArrayList;


public class LivingEntitiesPublisher extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(LivingEntitiesPublisher.class);

    private final Publisher<minecraft_msgs.msg.LivingEntityArray> livingEntitiesPublisher;
    private final Minecraft minecraft;

    private final int maxMobCount = 100;
    private final double searchHorizontalRadius = 64.0;
    private final double searchVerticalRadius = 16.0;
    private final int publishRateHz = 10;

    private final WallTimer timer;

    private final Map<Integer, String> mobIdMap = new HashMap<>();
    private int mobCounter = 0;

    public LivingEntitiesPublisher() {
        super("minecraft_living_entities_publisher");

        livingEntitiesPublisher = node.createPublisher(minecraft_msgs.msg.LivingEntityArray.class, "/player/nearby_living_entities");
        minecraft = Minecraft.getInstance();

        long periodMs = 1000 / publishRateHz;
        timer = node.createWallTimer(periodMs, TimeUnit.MILLISECONDS, this::publishNearbyLivingEntities);
    }

    private void publishNearbyLivingEntities() {
        if (minecraft.player == null || minecraft.level == null) return;
        
        double px = minecraft.player.getX();
        double py = minecraft.player.getY();
        double pz = minecraft.player.getZ();

        Level level = minecraft.level;
        Entity player = minecraft.player;

        List<net.minecraft.world.entity.LivingEntity> nearbyLivingEntities =
            findNearbyLivingEntities(level, player, px, py, pz);

        if (nearbyLivingEntities.isEmpty()) {
            return;
        }
        
        minecraft_msgs.msg.LivingEntityArray entityArray = new LivingEntityArray();
        Header header = new Header();
        header.setStamp(Time.now());
        header.setFrameId("world");
        entityArray.setHeader(header);
        
        List<minecraft_msgs.msg.LivingEntity> entities = new ArrayList<>();
        for (net.minecraft.world.entity.LivingEntity mob : nearbyLivingEntities) {
            minecraft_msgs.msg.LivingEntity entityMsg = createLivingEntityMessage(mob);
            entities.add(entityMsg);
        }

        entityArray.setEntities(entities);
        livingEntitiesPublisher.publish(entityArray);
    }
    
    protected AABB getHitbox(net.minecraft.world.entity.LivingEntity mob) {
        return mob.getBoundingBox();
    }

    private List<net.minecraft.world.entity.LivingEntity> findNearbyLivingEntities(Level level, Entity player, double px, double py, double pz) {
        AABB area = new AABB(px - searchHorizontalRadius, py - searchVerticalRadius, pz - searchHorizontalRadius,
                             px + searchHorizontalRadius, py + searchVerticalRadius, pz + searchHorizontalRadius);
        
        Vec3 playerPos = new Vec3(px, py, pz);
        List<net.minecraft.world.entity.LivingEntity> entities = level.getEntities(player, area, e -> e instanceof net.minecraft.world.entity.LivingEntity && e != player)
                   .stream()
                   .map(e -> (net.minecraft.world.entity.LivingEntity)e)
                   .sorted((a, b) -> {
                       double distA = new Vec3(a.getX(), a.getY(), a.getZ()).distanceTo(playerPos);
                       double distB = new Vec3(b.getX(), b.getY(), b.getZ()).distanceTo(playerPos);
                       return Double.compare(distA, distB);
                   })
                   .collect(Collectors.toList());
        
        // Remove entities with duplicate IDs (keeping the first occurrence)
        Map<Integer, net.minecraft.world.entity.LivingEntity> uniqueEntities = new HashMap<>();
        for (net.minecraft.world.entity.LivingEntity entity : entities) {
            if (!uniqueEntities.containsKey(entity.getId())) {
                uniqueEntities.put(entity.getId(), entity);
            }
        }
        
        return new ArrayList<>(uniqueEntities.values()).stream()
               .limit(maxMobCount)
               .collect(Collectors.toList());
    }

    private minecraft_msgs.msg.LivingEntity createLivingEntityMessage(net.minecraft.world.entity.LivingEntity mob) {
        minecraft_msgs.msg.LivingEntity entityMsg = new minecraft_msgs.msg.LivingEntity();
        entityMsg.setDescriptionId(mob.getType().getDescriptionId());
        entityMsg.setName(mob.getName().getString());
        entityMsg.setId(mob.getId());
        entityMsg.setHealth(mob.getHealth());
        entityMsg.setMaxHealth(mob.getMaxHealth());

        Pose pose = createMobPose(mob);
        entityMsg.setPose(pose);

        AABB hitbox = getHitbox(mob);
        Vector3 hitBoxSize = new Vector3();

        hitBoxSize.setX(hitbox.maxZ - hitbox.minZ);
        hitBoxSize.setY(hitbox.maxX - hitbox.minX);
        hitBoxSize.setZ(hitbox.maxY - hitbox.minY);
        entityMsg.setHitBox(hitBoxSize);

        MobCategory category = new MobCategory();
        switch (mob.getType().getCategory()) {
            case MONSTER:
                category.setMobCategory(MobCategory.MONSTER);
                break;
            case CREATURE:
                category.setMobCategory(MobCategory.CREATURE);
                break;
            case AMBIENT:
                category.setMobCategory(MobCategory.AMBIENT);
                break;
            case AXOLOTLS:
                category.setMobCategory(MobCategory.AXOLOTLS);
                break;
            case UNDERGROUND_WATER_CREATURE:
                category.setMobCategory(MobCategory.UNDERGROUND_WATER_CREATURE);
                break;
            case WATER_CREATURE:
                category.setMobCategory(MobCategory.WATER_CREATURE);
                break;
            case WATER_AMBIENT:
                category.setMobCategory(MobCategory.WATER_AMBIENT);
                break;
            default:
                category.setMobCategory(MobCategory.MISC);
                break;
        }
        entityMsg.setCategory(category);
        
        return entityMsg;
    }

    private Pose createMobPose(net.minecraft.world.entity.LivingEntity mob) {
        Pose pose = new Pose();
        Point position = new Point();

        position.setX(mob.getZ());
        position.setY(mob.getX());
        position.setZ(mob.getY() - 63.0);
        pose.setPosition(position);

        Quaternion orientation = new Quaternion();
        double yawRad = Math.toRadians(mob.getYRot());

        double cy = Math.cos(-yawRad * 0.5);
        double sy = Math.sin(-yawRad * 0.5);
        
        orientation.setW(cy);
        orientation.setX(0.0);
        orientation.setY(0.0);
        orientation.setZ(sy);
        
        pose.setOrientation(orientation);
        return pose;
    }
}
