package com.kazusa.minecraft_ros2.items;

import com.kazusa.minecraft_ros2.graphics.LidarModel;
import com.kazusa.minecraft_ros2.graphics.ModelHandler;
import net.minecraft.client.Minecraft;
import net.minecraft.client.model.HumanoidModel;
import net.minecraft.core.Holder;
import net.minecraft.world.entity.EquipmentSlot;
import net.minecraft.world.entity.LivingEntity;
import net.minecraft.world.item.ArmorItem;
import net.minecraft.world.item.ArmorMaterial;
import net.minecraft.world.item.ItemStack;
import net.minecraftforge.client.extensions.common.IClientItemExtensions;
import org.jetbrains.annotations.NotNull;

import java.util.function.Consumer;

public class LidarItem extends ArmorItem {
    public LidarItem(Holder<ArmorMaterial> armorMaterial, Type type, Properties properties) {
        super(armorMaterial, type, properties);
    }

    @Override
    public void initializeClient(Consumer<IClientItemExtensions> consumer) {
        // クライアント側の初期化処理をここに記述
        // 例えば、アイテムのレンダリングやツールチップのカスタマイズなど
        consumer.accept(new IClientItemExtensions() {
            LidarModel model;
            // 必要なメソッドを実装


            @Override
            public @NotNull HumanoidModel<?> getHumanoidArmorModel(LivingEntity livingEntity, ItemStack itemStack,
                                                                   EquipmentSlot equipmentSlot, HumanoidModel<?> original) {
                if (model == null) {
                    var layer = Minecraft.getInstance().getEntityModels().bakeLayer(ModelHandler.LIDAR_LAYER);
                    model = new LidarModel(layer);
                }

                return model;
            }
        });
    }
}
