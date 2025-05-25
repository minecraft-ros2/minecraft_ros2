package com.kazusa.minecraft_ros2.models;

import com.kazusa.minecraft_ros2.models.DynamicModelEntity;
import net.minecraft.client.renderer.entity.EntityRendererProvider;

import software.bernie.geckolib.renderer.GeoEntityRenderer;

public class DynamicModelEntityRenderer
    extends GeoEntityRenderer<DynamicModelEntity>
{
    public DynamicModelEntityRenderer(EntityRendererProvider.Context ctx) {
        super(ctx, new DynamicModelEntityModel());
        this.shadowRadius = 0.5f;  // 影の大きさ
        this.scaleWidth = 1.0f; // モデルのスケール
        this.scaleHeight = 1.0f; // モデルの高さ
    }
}
