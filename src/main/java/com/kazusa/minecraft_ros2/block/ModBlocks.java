package com.kazusa.minecraft_ros2.block;

import com.kazusa.minecraft_ros2.minecraft_ros2;
import net.minecraft.world.level.block.Block;
import net.minecraftforge.registries.DeferredRegister;
import net.minecraftforge.registries.RegistryObject;
import net.minecraftforge.registries.ForgeRegistries;

public class ModBlocks {
    public static final DeferredRegister<Block> BLOCKS =
        DeferredRegister.create(ForgeRegistries.BLOCKS, minecraft_ros2.MOD_ID);

    public static final RegistryObject<Block> REDSTONE_PUB_SUB =
        BLOCKS.register("redstone_pub_sub", RedstonePubSubBlock::new);
}