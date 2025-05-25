package com.kazusa.minecraft_ros2.models;

import net.minecraft.resources.ResourceLocation;
import software.bernie.geckolib.model.GeoModel;

public class DynamicModelEntityModel extends GeoModel<DynamicModelEntity> {
    // geometry.json のパス
    private static final ResourceLocation GEO =
        new ResourceLocation("minecraft_ros2", "geo/custom_entity.geo.json");
    // テクスチャのパス（お好きなものを）
    private static final ResourceLocation TEX =
        new ResourceLocation("minecraft_ros2", "textures/entity/custom_entity.png");
    // アニメーションファイルが無ければ同じパスで空ファイル or 無視して OK
    private static final ResourceLocation ANIM = null;

    @Override
    public ResourceLocation getModelResource(DynamicModelEntity object) {
        return object.getGeometryLocation() != null ? 
            object.getGeometryLocation() : GEO; // 動的に設定されたジオメトリを使用
    }

    @Override
    public ResourceLocation getTextureResource(DynamicModelEntity object) {
        return TEX;
    }

    @Override
    public ResourceLocation getAnimationResource(DynamicModelEntity animatable) {
        return ANIM;
    }
}
