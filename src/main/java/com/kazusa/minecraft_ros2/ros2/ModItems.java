package com.kazusa.minecraft_ros2.ros2;

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
        () -> new ArmorItem(
            Holder.direct(ModArmorMaterials.MYSTIC_MATERIAL),  
            ArmorItem.Type.HELMET,
            new Item.Properties()
        )
    );

    public static final RegistryObject<Item> HESAI_XT32 = ITEMS.register("hesai_xt32",
        () -> new ArmorItem(
            Holder.direct(ModArmorMaterials.MYSTIC_MATERIAL),  
            ArmorItem.Type.HELMET,
            new Item.Properties()
        )
    );
    public static void register(IEventBus bus) {
        ITEMS.register(bus);
    }
}
