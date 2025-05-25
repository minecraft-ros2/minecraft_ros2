package com.kazusa.minecraft_ros2.models;

import net.minecraft.client.Minecraft;
import net.minecraft.resources.ResourceLocation;
import software.bernie.geckolib.model.GeoModel;
import net.minecraft.server.packs.resources.ResourceManager;
import net.minecraft.server.packs.resources.ReloadableResourceManager;
import net.minecraft.server.packs.resources.Resource;

import java.util.List;
import java.io.IOException;
import java.util.Optional;
import java.io.InputStream;

public class DynamicModelEntityModel extends GeoModel<DynamicModelEntity> {
    // geometry.json のパス
    private static final ResourceLocation DEFAULT_GEO =
        new ResourceLocation("minecraft_ros2", "geo/custom_entity.geo.json");
    private static final List<ResourceLocation> CUSTOM_GEO_LIST = List.of(
        new ResourceLocation("runtime_geo", "geo/dynamic_model_0.geo.json"),
        new ResourceLocation("runtime_geo", "geo/dynamic_model_1.geo.json"),
        new ResourceLocation("runtime_geo", "geo/dynamic_model_2.geo.json"),
        new ResourceLocation("runtime_geo", "geo/dynamic_model_3.geo.json"),
        new ResourceLocation("runtime_geo", "geo/dynamic_model_4.geo.json"),
        new ResourceLocation("runtime_geo", "geo/dynamic_model_5.geo.json"),
        new ResourceLocation("runtime_geo", "geo/dynamic_model_6.geo.json"),
        new ResourceLocation("runtime_geo", "geo/dynamic_model_7.geo.json"),
        new ResourceLocation("runtime_geo", "geo/dynamic_model_8.geo.json"),
        new ResourceLocation("runtime_geo", "geo/dynamic_model_9.geo.json")
    );

    // テクスチャのパス（お好きなものを）
    private static final ResourceLocation TEX =
        new ResourceLocation("minecraft_ros2", "textures/entity/custom_entity.png");
    // アニメーションファイルが無ければ同じパスで空ファイル or 無視して OK
    private static final ResourceLocation ANIM = null;

    @Override
    public ResourceLocation getModelResource(DynamicModelEntity object) {
        return resourceExists(CUSTOM_GEO_LIST.get(object.getModelId())) ?
            CUSTOM_GEO_LIST.get(object.getModelId()) : DEFAULT_GEO; // 動的に設定されたジオメトリを使用
    }

    @Override
    public ResourceLocation getTextureResource(DynamicModelEntity object) {
        return TEX;
    }

    @Override
    public ResourceLocation getAnimationResource(DynamicModelEntity animatable) {
        return ANIM;
    }


    public static boolean resourceExists(ResourceLocation loc) {
        ResourceManager rm = Minecraft.getInstance().getResourceManager();
    
        // Forgeの実装では hasResource(ResourceLocation) があるので、あればそれを使う
        if (rm instanceof ReloadableResourceManager rrm) {
            Optional<Resource> opt = rrm.getResource(loc);
            if (opt.isPresent()) {
                Resource res = opt.get();
                // try-with は InputStream に対して
                try (InputStream in = res.open()) {
                    // リソースが存在する場合は
                    return true;
                } catch (IOException e) {
                    // 例外が発生した場合はリソースが存在しないと判断
                    return false;
                }
            }
        }
        return false; // ReloadableResourceManager でなければ、またはリソースが見つからない場合は false
    }
}
