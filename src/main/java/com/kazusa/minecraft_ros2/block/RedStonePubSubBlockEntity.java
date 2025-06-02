package com.kazusa.minecraft_ros2.block;

import com.kazusa.minecraft_ros2.menu.RedStonePubSubBlockContainer;
import net.minecraft.core.BlockPos;
import net.minecraft.network.chat.Component;
import net.minecraft.world.entity.player.Inventory;
import net.minecraft.world.entity.player.Player;
import net.minecraft.world.inventory.AbstractContainerMenu;
import net.minecraft.world.MenuProvider;
import net.minecraft.world.level.block.entity.BlockEntity;
import net.minecraft.world.Nameable;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.nbt.CompoundTag;
import net.minecraft.network.protocol.game.ClientboundBlockEntityDataPacket;
import net.minecraft.core.HolderLookup;

public class RedStonePubSubBlockEntity extends BlockEntity
    implements Nameable, MenuProvider {

    private Component customName;

    public RedStonePubSubBlockEntity(BlockPos pos, BlockState state) {
        super(ModBlockEntities.RED_STONE_PUB_SUB_BLOCK_ENTITY.get(), pos, state);
    }

    // Nameable インターフェイス
    @Override
    public Component getName() {
        return customName != null
            ? customName
            : Component.literal("Redstone Pub Sub Block");
    }

    @Override
    public Component getDisplayName() {
        return getName();
    }

    // GUI 用 Factory（ContainerType〈…〉を later register しておくこと）
    @Override
    public AbstractContainerMenu createMenu(int syncId, Inventory inv, Player player) {
        return new RedStonePubSubBlockContainer(syncId, inv, getBlockPos());
    }

    // 外部から呼ぶ setter
    public void setCustomName(Component name) {
        this.customName = name;
        setChanged();  // マークしてデータ同期
    }

    /** ブロック本体を返す */
    public RedstonePubSubBlock getBlock() {
       return (RedstonePubSubBlock) getBlockState().getBlock();
    }


    // ■ 保存時（チャンクセーブ／ワールドセーブ）
    @Override
    protected void saveAdditional(CompoundTag tag, HolderLookup.Provider lookup) {
        super.saveAdditional(tag, lookup);
        if (customName != null) {
            tag.putString("CustomName", customName.getString());
        }
        // デバッグ用ログ（必須ではありませんが、ちゃんと呼ばれているかの確認になります）
        System.out.println("MyBlockEntity.saveAdditional: CustomName = " + customName);
    }

    // ■ 読み込み時（チャンクロード）
    @Override
    protected void loadAdditional(CompoundTag tag, HolderLookup.Provider lookup) {
        super.loadAdditional(tag, lookup);
        if (tag.contains("CustomName", 8)) {  // 8 = TAG_STRING
            customName = Component.literal(tag.getString("CustomName"));
            RedstonePubSubBlock block = getBlock();
            block.initializePublisher(getBlockPos(), customName.getString());
            block.initializeSubscriber(getBlockPos(), customName.getString());
            block.onLoad(getBlockPos());
        } else {
            customName = null;
        }
        System.out.println("MyBlockEntity.loadAdditional: CustomName = " + customName);
    }

    // ■ クライアント同期用
    @Override
    public CompoundTag getUpdateTag(HolderLookup.Provider lookup) {
        CompoundTag tag = super.getUpdateTag(lookup);
        saveAdditional(tag, lookup);
        return tag;
    }

    @Override
    public ClientboundBlockEntityDataPacket getUpdatePacket() {
        return ClientboundBlockEntityDataPacket.create(this);
    }
}
