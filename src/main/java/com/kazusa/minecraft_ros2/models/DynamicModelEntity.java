package com.kazusa.minecraft_ros2.models;

import com.kazusa.minecraft_ros2.ros2.RobotTwistSubscriber;
import net.minecraft.world.entity.Mob;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.entity.EntityType;
import net.minecraft.world.entity.EntityDimensions;
import net.minecraft.world.level.Level;
import net.minecraft.network.syncher.SynchedEntityData.Builder;
import net.minecraft.network.syncher.SynchedEntityData;
import net.minecraft.network.syncher.*;
import net.minecraft.nbt.CompoundTag;
import net.minecraft.world.entity.ai.attributes.AttributeSupplier;
import net.minecraft.world.entity.ai.attributes.Attributes;
import net.minecraft.world.entity.ai.control.BodyRotationControl;

import software.bernie.geckolib.animatable.GeoEntity;
import software.bernie.geckolib.animatable.instance.AnimatableInstanceCache;
import software.bernie.geckolib.util.GeckoLibUtil;
import software.bernie.geckolib.animation.AnimatableManager;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import net.minecraft.client.Minecraft;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.server.packs.resources.Resource;
import net.minecraft.util.GsonHelper;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.Vec3;

import java.lang.reflect.Field;
import java.util.Optional;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Objects;

public class DynamicModelEntity extends Mob implements GeoEntity {

    private static final EntityDataAccessor<CompoundTag> DATA_SHAPE =
        SynchedEntityData.defineId(DynamicModelEntity.class, EntityDataSerializers.COMPOUND_TAG);

    private RobotTwistSubscriber twistSubscriber;

    private int modelId;

    private float computedWidthInM = 1.0f;
    private float computedHeightInM = 1.0f;
    private float computedDepthInM = 1.0f;

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

    /**
     * GeoJSON を読み込んで、EntityDimensions（幅・高さ）を計算するユーティリティメソッド
     */
    private void computeDimensionsFromGeoJson(ResourceLocation geoJsonLoc) {
        try {
            // 1) リソースマネージャーから .geo.json の InputStream を取得
            Optional<Resource> resource = Minecraft.getInstance()
                                          .getResourceManager()
                                          .getResource(geoJsonLoc);
            try (InputStream is = resource.get().open();
                 InputStreamReader reader = new InputStreamReader(is)) {

                // 2) JSON を読み込む
                JsonObject root = GsonHelper.parse(reader);
                if (root == null) {
                    throw new IllegalStateException(".geo.json を読み込めませんでした: " + geoJsonLoc);
                }

                // 3) "minecraft:geometry" 配列を取り出す
                //    通常はサイズ 1 の配列で、最初の要素を見れば良い
                JsonArray geometryArr = GsonHelper.getAsJsonArray(root, "minecraft:geometry");
                if (geometryArr.size() == 0) {
                    throw new IllegalStateException("minecraft:geometry 配列が空です: " + geoJsonLoc);
                }

                JsonObject geom0 = geometryArr.get(0).getAsJsonObject();

                // 4) "bones" 配列を取り出す
                JsonArray bonesArr = GsonHelper.getAsJsonArray(geom0, "bones");
                if (bonesArr.size() == 0) {
                    // bones が空 → 当たり判定をデフォルト 1x1 にする
                    computedWidthInM = 1.0f;
                    computedHeightInM = 1.0f;
                    return;
                }

                // モデル全体のブロック単位 (ピクセル単位) での最小コーナー／最大コーナーを保持
                Vec3 minCorner = new Vec3(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
                Vec3 maxCorner = new Vec3(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

                // 5) 全てのボーンを巡回
                for (JsonElement boneElem : bonesArr) {
                    if (!boneElem.isJsonObject()) continue;
                    JsonObject boneObj = boneElem.getAsJsonObject();

                    // 6) もし当該ボーンに "cubes" がなければスキップ
                    if (!boneObj.has("cubes")) continue;
                    JsonArray cubesArr = GsonHelper.getAsJsonArray(boneObj, "cubes");

                    // 7) 各キューブ (JsonObject) を巡回
                    for (JsonElement cubeElem : cubesArr) {
                        if (!cubeElem.isJsonObject()) continue;
                        JsonObject cubeObj = cubeElem.getAsJsonObject();

                        // 8) "origin": [ox, oy, oz] を取得 (ピクセル単位・ブロック16分割）
                        JsonArray originArr = GsonHelper.getAsJsonArray(cubeObj, "origin");
                        double ox = originArr.get(0).getAsDouble();
                        double oy = originArr.get(1).getAsDouble();
                        double oz = originArr.get(2).getAsDouble();

                        // 9) "size": [sx, sy, sz] を取得
                        JsonArray sizeArr = GsonHelper.getAsJsonArray(cubeObj, "size");
                        double sx = sizeArr.get(0).getAsDouble();
                        double sy = sizeArr.get(1).getAsDouble();
                        double sz = sizeArr.get(2).getAsDouble();

                        // 10) このキューブの「最小コーナー (px)」と「最大コーナー (px)」を計算
                        Vec3 thisMin = new Vec3(ox, oy, oz);
                        Vec3 thisMax = new Vec3(ox + sx, oy + sy, oz + sz);

                        // 11) 全体の min/max を更新
                        minCorner = new Vec3(
                            Math.min(minCorner.x, thisMin.x),
                            Math.min(minCorner.y, thisMin.y),
                            Math.min(minCorner.z, thisMin.z)
                        );
                        maxCorner = new Vec3(
                            Math.max(maxCorner.x, thisMax.x),
                            Math.max(maxCorner.y, thisMax.y),
                            Math.max(maxCorner.z, thisMax.z)
                        );
                    }
                }

                // 12) ブロック単位 (ピクセル/16) → メートル単位に変換
                double widthBlocks  = maxCorner.x - minCorner.x;
                double heightBlocks = maxCorner.y - minCorner.y;
                double depthBlocks  = maxCorner.z - minCorner.z;

                // 13) メートル単位に：1 ブロック = 1.0f (= 1m) = 16 ピクセル相当
                //     →  (blocks / 16.0) [m]
                computedWidthInM  = (float)(widthBlocks  / 16.0);
                computedHeightInM = (float)(heightBlocks / 16.0);
                computedDepthInM  = (float)(depthBlocks  / 16.0);

                // 14) 幅や高さが 0 にならないようフォールバック
                if (computedWidthInM <= 0.0f) {
                    computedWidthInM = 1.0f;
                }
                if (computedHeightInM <= 0.0f) {
                    computedHeightInM = 1.0f;
                }
                if (computedDepthInM <= 0.0f) {
                    computedDepthInM = 1.0f;
                }

            } // InputStream/Reader の try-with-resources 終了

        } catch (Exception ex) {
            ex.printStackTrace();
            computedWidthInM  = 1.0f;
            computedHeightInM = 1.0f;
            computedDepthInM  = 1.0f;
        }
    }

    private void adjustDimensionsAndRefresh(float w, float h, float d) {
        try {
            // 1) Entity クラスの private フィールド 'dimensions' を取得
            Field dimField = Entity.class.getDeclaredField("dimensions");
            dimField.setAccessible(true);

            // 2) 新しい EntityDimensions オブジェクトをインスタンス化
            EntityDimensions newDim = EntityDimensions.fixed((w + d) / 2, h);

            // 3) 'dimensions' フィールドを書き換え
            dimField.set(this, newDim);

            // 4) refreshDimensions() を呼んで、AABB を新しい dimensions に合わせる
            this.refreshDimensions();
        }
        catch (Exception ex) {
            ex.printStackTrace();
        }
    }

    public void setModelDimensions() {
        var geoLoc = new ResourceLocation(
            "runtime_geo", "geo/dynamic_model_" + this.getModelId() + ".geo.json");
        computeDimensionsFromGeoJson(geoLoc);
        adjustDimensionsAndRefresh(computedWidthInM, computedHeightInM, computedDepthInM);
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