package com.kazusa.minecraft_ros2.menu;

import com.kazusa.minecraft_ros2.block.RedstonePubSubBlock;
import com.kazusa.minecraft_ros2.block.NamedBlockEntity;
import net.minecraft.core.BlockPos;
import net.minecraft.network.FriendlyByteBuf;
import net.minecraft.world.entity.player.Inventory;
import net.minecraft.world.entity.player.Player;
import net.minecraft.world.inventory.AbstractContainerMenu;
import net.minecraft.world.inventory.MenuType;
import net.minecraft.world.item.ItemStack;
import net.minecraft.server.level.ServerPlayer;
import net.minecraft.network.chat.Component;

public class NamedBlockContainer extends AbstractContainerMenu {
    public final BlockPos pos;
    private final NamedBlockEntity blockEntity;

    // サーバー側用コンストラクタ
    public NamedBlockContainer(int id, Inventory inv, BlockPos pos) {
        super(ModMenuTypes.NAMED_BLOCK_MENU.get(), id);
        this.pos = pos;
        this.blockEntity = (NamedBlockEntity) inv.player.level().getBlockEntity(pos);
    }

    // クライアント側用コンストラクタ
    public NamedBlockContainer(int id, Inventory inv, FriendlyByteBuf buf) {
        this(id, inv, buf.readBlockPos());
    }

    @Override
    public boolean stillValid(Player player) {
        return player.distanceToSqr(
            pos.getX() + 0.5,
            pos.getY() + 0.5,
            pos.getZ() + 0.5
        ) < 64.0;
    }

    @Override
    public ItemStack quickMoveStack(Player player, int index) {
        return ItemStack.EMPTY;
    }

    // Called from RenamePacket handler
    public void onRename(String name, ServerPlayer player) {
        blockEntity.setCustomName(Component.literal(name));
        // サーバー側（＝Level#isClientSide()==false）のみで初期化
        if (!player.level().isClientSide()) {
            RedstonePubSubBlock block = blockEntity.getBlock();
            block.initializePublisher(pos, name);
            block.initializeSubscriber(pos, name);
        }
    }
}
