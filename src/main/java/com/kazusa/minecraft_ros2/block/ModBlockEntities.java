package com.kazusa.minecraft_ros2.block;

import com.kazusa.minecraft_ros2.minecraft_ros2;
import net.minecraft.world.level.block.entity.BlockEntityType;
import net.minecraftforge.registries.DeferredRegister;
import net.minecraftforge.registries.RegistryObject;
import net.minecraftforge.registries.ForgeRegistries;

public class ModBlockEntities {
    // DeferredRegister for block entities
    public static final DeferredRegister<BlockEntityType<?>> BLOCK_ENTITIES =
        DeferredRegister.create(ForgeRegistries.BLOCK_ENTITY_TYPES, minecraft_ros2.MOD_ID);

    public static final RegistryObject<BlockEntityType<RedStonePubSubBlockEntity>> RED_STONE_PUB_SUB_BLOCK_ENTITY =
        BLOCK_ENTITIES.register("red_stone_pub_sub_block_entity",
            () -> BlockEntityType.Builder
                .of(RedStonePubSubBlockEntity::new, ModBlocks.REDSTONE_PUB_SUB.get())
                .build(null)
        );

    // Register method to be called in main mod class
    public static void register(net.minecraftforge.eventbus.api.IEventBus bus) {
        BLOCK_ENTITIES.register(bus);
    }
}