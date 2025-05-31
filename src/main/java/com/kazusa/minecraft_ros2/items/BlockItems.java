package com.kazusa.minecraft_ros2.items;

import com.kazusa.minecraft_ros2.minecraft_ros2;
import com.kazusa.minecraft_ros2.block.ModBlocks;
import net.minecraft.world.item.BlockItem;
import net.minecraft.world.item.Item;
import net.minecraftforge.registries.DeferredRegister;
import net.minecraftforge.registries.ForgeRegistries;
import net.minecraftforge.registries.RegistryObject;

public class BlockItems {
    public static final DeferredRegister<Item> ITEMS =
        DeferredRegister.create(ForgeRegistries.ITEMS, minecraft_ros2.MOD_ID);

    public static final RegistryObject<Item> REDSTONE_PUB_SUB_ITEM =
        ITEMS.register("redstone_pub_sub", () ->
            new BlockItem(ModBlocks.REDSTONE_PUB_SUB.get(),
                new Item.Properties()
            )
        );
}