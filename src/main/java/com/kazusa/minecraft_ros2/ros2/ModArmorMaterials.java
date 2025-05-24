package com.kazusa.minecraft_ros2.ros2;

import net.minecraft.core.Holder;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.sounds.SoundEvent;
import net.minecraft.sounds.SoundEvents;
import net.minecraft.world.item.ArmorItem;
import net.minecraft.world.item.ArmorMaterial;
import net.minecraft.world.item.Items;
import net.minecraft.world.item.crafting.Ingredient;

import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

public class ModArmorMaterials {
    public static final String MODID = "minecraft_ros2";

    public static final ArmorMaterial MYSTIC_MATERIAL = createMysticMaterial();

    private static ArmorMaterial createMysticMaterial() {
        Map<ArmorItem.Type, Integer> defenseMap = Map.of(
            ArmorItem.Type.HELMET,    3,
            ArmorItem.Type.CHESTPLATE,6,
            ArmorItem.Type.LEGGINGS,  8,
            ArmorItem.Type.BOOTS,     3
        );
        // SoundEvent は Holder.direct でラップ
        Holder<SoundEvent> equipSound = SoundEvents.ARMOR_EQUIP_DIAMOND;
        Supplier<Ingredient> repairMat = () -> Ingredient.of(Items.DIAMOND);
        List<ArmorMaterial.Layer> layers = List.of(
            new ArmorMaterial.Layer(
                new ResourceLocation(MODID, "mystic"),
                "",
                false
            )
        );
        return new ArmorMaterial(
            defenseMap,
            25,
            equipSound,
            repairMat,
            layers,
            2.0F,
            0.1F
        );
    }
}
