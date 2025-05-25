package com.kazusa.minecraft_ros2.graphics;

import net.minecraft.client.model.geom.ModelLayerLocation;
import net.minecraft.resources.ResourceLocation;
import net.minecraftforge.client.event.EntityRenderersEvent;
import net.minecraftforge.eventbus.api.SubscribeEvent;

public class ModelHandler {
    public static final ModelLayerLocation LIDAR_LAYER = new ModelLayerLocation(
            new ResourceLocation("minecraft", "player"), "minecraft_ros2:lidar");

    @SubscribeEvent
    public void onRegisterLayerDefinitions(EntityRenderersEvent.RegisterLayerDefinitions event) {
        event.registerLayerDefinition(LIDAR_LAYER, LidarModel::createBodyLayer);
    }
}
