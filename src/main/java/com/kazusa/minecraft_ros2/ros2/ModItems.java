package com.kazusa.minecraft_ros2.ros2;

import com.kazusa.minecraft_ros2.items.LidarItem;
import net.minecraft.core.Holder;
import net.minecraft.core.registries.Registries;
import net.minecraft.world.item.ArmorItem;
import net.minecraft.world.item.Item;
import net.minecraftforge.eventbus.api.IEventBus;
import net.minecraftforge.registries.DeferredRegister;
import net.minecraftforge.registries.RegistryObject;

public class ModItems {
    public static final String MODID = ModArmorMaterials.MODID;

    public static final DeferredRegister<Item> ITEMS =
            DeferredRegister.create(Registries.ITEM, MODID);

    public static final RegistryObject<Item> VELODYNE_VLP16 = ITEMS.register("velodyne_vlp16",
            () -> new LidarItem(
                    Holder.direct(ModArmorMaterials.MYSTIC_MATERIAL),
                    ArmorItem.Type.HELMET,
                    new Item.Properties()
            )
    );
    public static final RegistryObject<Item> HESAI_XT32 = ITEMS.register("hesai_xt32",
            () -> new LidarItem(
                    Holder.direct(ModArmorMaterials.MYSTIC_MATERIAL),
                    ArmorItem.Type.HELMET,
                    new Item.Properties()
            )
    );
    public static final RegistryObject<Item> HESAI_FT120 = ITEMS.register("hesai_ft120",
        () -> new LidarItem(
            Holder.direct(ModArmorMaterials.MYSTIC_MATERIAL),  
            ArmorItem.Type.HELMET,
            new Item.Properties()
        )
    );
    public static final RegistryObject<Item> RS_LIDAR_M1 = ITEMS.register("rs_lidar_m1",
        () -> new LidarItem(
            Holder.direct(ModArmorMaterials.MYSTIC_MATERIAL),  
            ArmorItem.Type.HELMET,
            new Item.Properties()
        )
    );
    public static final RegistryObject<Item> UTM_30LN = ITEMS.register("utm_30ln",
        () -> new LidarItem(
            Holder.direct(ModArmorMaterials.MYSTIC_MATERIAL),  
            ArmorItem.Type.HELMET,
            new Item.Properties()
        )
    );

    public static void register(IEventBus bus) {
        ITEMS.register(bus);
    }
}
