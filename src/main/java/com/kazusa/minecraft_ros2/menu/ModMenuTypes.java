package com.kazusa.minecraft_ros2.menu;

import com.kazusa.minecraft_ros2.minecraft_ros2;
import net.minecraft.world.inventory.MenuType;
import net.minecraftforge.registries.DeferredRegister;
import net.minecraftforge.registries.ForgeRegistries;
import net.minecraftforge.registries.RegistryObject;
import net.minecraftforge.common.extensions.IForgeMenuType;

public class ModMenuTypes {
    public static final DeferredRegister<MenuType<?>> MENUS =
        DeferredRegister.create(ForgeRegistries.MENU_TYPES, minecraft_ros2.MOD_ID);

    // RedStonePubSub 用 ScreenHandler を登録
    public static final RegistryObject<MenuType<RedStonePubSubBlockContainer>> REDSTONE_PUB_SUB_BLOCK_MENU =
        MENUS.register("red_stone_pub_sub_block_menu",
            () -> IForgeMenuType.create(RedStonePubSubBlockContainer::new)
        );
}
