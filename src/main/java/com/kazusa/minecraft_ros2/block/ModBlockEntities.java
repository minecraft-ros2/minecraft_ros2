package com.kazusa.minecraft_ros2.block;

import com.kazusa.minecraft_ros2.minecraft_ros2;
import net.minecraft.world.level.block.entity.BlockEntity;
import net.minecraft.world.level.block.entity.BlockEntityType;
import net.minecraftforge.registries.DeferredRegister;
import net.minecraftforge.registries.RegistryObject;
import net.minecraftforge.registries.ForgeRegistries;

public class ModBlockEntities {
    // DeferredRegister for block entities
    public static final DeferredRegister<BlockEntityType<?>> BLOCK_ENTITIES =
        DeferredRegister.create(ForgeRegistries.BLOCK_ENTITY_TYPES, minecraft_ros2.MOD_ID);

    /**
     * RegistryObject for our NamedBlockEntity type.
     * The factory is NamedBlockEntity::new and the valid block is RedstoneToggleBlock (or specific class).
     */
    public static final RegistryObject<BlockEntityType<NamedBlockEntity>> NAMED_BLOCK_ENTITY =
        BLOCK_ENTITIES.register("named_block_entity",
            () -> BlockEntityType.Builder
                .of(NamedBlockEntity::new, ModBlocks.REDSTONE_PUB_SUB.get())
                .build(null)
        );

    // Register method to be called in main mod class
    public static void register(net.minecraftforge.eventbus.api.IEventBus bus) {
        BLOCK_ENTITIES.register(bus);
    }
}