package com.kazusa.minecraft_ros2.ros2;

import com.kazusa.minecraft_ros2.items.BlockItems;
import net.minecraft.world.item.CreativeModeTabs;
import net.minecraftforge.event.BuildCreativeModeTabContentsEvent;
import net.minecraftforge.eventbus.api.SubscribeEvent;
import net.minecraftforge.fml.common.Mod;

@Mod.EventBusSubscriber(modid = ModArmorMaterials.MODID, bus = Mod.EventBusSubscriber.Bus.MOD)
public class CreativeTabEvents {

    @SubscribeEvent
    public static void onBuildCreativeModeTabContents(BuildCreativeModeTabContentsEvent event) {
        if (event.getTabKey() == CreativeModeTabs.COMBAT) {
            event.accept(ModItems.HESAI_XT32);
            event.accept(ModItems.HESAI_FT120);
            event.accept(ModItems.VELODYNE_VLP16);
            event.accept(ModItems.RS_LIDAR_M1);
            event.accept(ModItems.UTM_30LN);
        }
        if (event.getTabKey() == CreativeModeTabs.REDSTONE_BLOCKS) {
            event.accept(BlockItems.REDSTONE_PUB_SUB_ITEM.get());
        }
    }
}
