package com.kazusa.minecraft_ros2.models;

import com.kazusa.minecraft_ros2.minecraft_ros2;
import net.minecraft.world.entity.EntityType;
import net.minecraft.world.entity.MobCategory;
import net.minecraftforge.registries.DeferredRegister;
import net.minecraftforge.registries.ForgeRegistries;
import net.minecraftforge.registries.RegistryObject;
import net.minecraftforge.eventbus.api.IEventBus;

public class ModEntities {
    // DeferredRegister を使って EntityType の登録を管理
    public static final DeferredRegister<EntityType<?>> ENTITIES =
        DeferredRegister.create(ForgeRegistries.ENTITY_TYPES, minecraft_ros2.MOD_ID);

    // RegistryObject によって遅延登録
    public static final RegistryObject<EntityType<DynamicModelEntity>> CUSTOM_ENTITY =
        ENTITIES.register("custom_entity", () ->
            // Builder.of(エンティティ生成関数, 分類)
            EntityType.Builder.<DynamicModelEntity>of(DynamicModelEntity::new, MobCategory.MISC)
                // エンティティの幅と高さ（ブロック単位）
                .sized(1.0F, 1.0F)
                // 更新間隔（ティック数）
                .updateInterval(1)
                .noSave()
                // 結合して EntityType を生成
                .build(minecraft_ros2.MOD_ID + ":custom_entity")
        );

    public static void register(IEventBus bus) {
        ENTITIES.register(bus);
    }
}