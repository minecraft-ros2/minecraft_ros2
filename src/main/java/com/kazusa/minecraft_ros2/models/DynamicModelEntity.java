package com.kazusa.minecraft_ros2.models;

import com.kazusa.minecraft_ros2.ros2.RobotTwistSubscriber;
import net.minecraft.world.entity.LivingEntity;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.entity.Mob;
import net.minecraft.world.entity.EntityType;
import net.minecraft.world.level.Level;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.core.BlockPos;
import net.minecraft.network.syncher.SynchedEntityData.Builder;
import net.minecraft.network.syncher.SynchedEntityData;
import net.minecraft.network.protocol.Packet;
import net.minecraft.network.protocol.game.ClientGamePacketListener;
import net.minecraft.network.protocol.game.ClientboundAddEntityPacket;
import net.minecraft.server.level.ServerEntity;
import net.minecraft.core.particles.ParticleTypes;
import net.minecraft.network.syncher.*;
import net.minecraft.nbt.CompoundTag;
import net.minecraft.world.phys.shapes.*;
import net.minecraft.world.entity.Pose;
import net.minecraft.world.entity.EntityDimensions;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.world.phys.Vec3;
import net.minecraft.world.entity.ai.attributes.AttributeSupplier;
import net.minecraft.world.entity.ai.attributes.Attributes;
import net.minecraft.world.entity.ai.control.BodyRotationControl;

import software.bernie.geckolib.animatable.GeoEntity;
import software.bernie.geckolib.animatable.instance.AnimatableInstanceCache;
import software.bernie.geckolib.util.GeckoLibUtil;
import software.bernie.geckolib.animation.AnimatableManager;

public class DynamicModelEntity extends Mob implements GeoEntity {

    private static final EntityDataAccessor<CompoundTag> DATA_SHAPE =
        SynchedEntityData.defineId(DynamicModelEntity.class, EntityDataSerializers.COMPOUND_TAG);

    private RobotTwistSubscriber twistSubscriber;

    private int modelId;

    public DynamicModelEntity(EntityType<? extends DynamicModelEntity> type, Level world) {
        super(type, world);
        modelId = 0;
        this.setNoGravity(false);
    }

    public void initRobotTwistSubscriber() {
        this.twistSubscriber = new RobotTwistSubscriber(this, this.getCustomName() != null ? this.getCustomName().getString() : "");
    }

    public RobotTwistSubscriber getRobotTwistSubscriber() {
        return this.twistSubscriber;
    }

    public void setModelId(int id) {
        this.modelId = id;
    }

    public int getModelId() {
        return this.modelId;
    }

    // ── Entity 必須オーバーライド ──
    @Override
    protected void defineSynchedData(Builder builder) {
        super.defineSynchedData(builder);
        builder.define(DATA_SHAPE, new CompoundTag());
    }

    @Override
    public boolean canBeCollidedWith() { return true; }

    @Override
    public boolean isPushable()       { return true; }

    public static AttributeSupplier.Builder createAttributes() {
        return Mob.createMobAttributes()
            .add(Attributes.MAX_HEALTH, 20.0)  // 最大体力
            .add(Attributes.MOVEMENT_SPEED, 0.2)  // 移動速度
            .add(Attributes.STEP_HEIGHT, 1.0); // ステップ高さ
    }

    // ── GeoEntity 実装 ──

    // GeckoLib 用のキャッシュを生成
    private final AnimatableInstanceCache cache = GeckoLibUtil.createInstanceCache(this);

    @Override
    public AnimatableInstanceCache getAnimatableInstanceCache() {
        return this.cache;
    }

    @Override
    public void registerControllers(AnimatableManager.ControllerRegistrar controllers) {
        // コントローラーの登録は必要に応じて行う
        // 例: controllers.add(new AnimationController<>(this, "controllerName", 0, this::predicate));
    }

}