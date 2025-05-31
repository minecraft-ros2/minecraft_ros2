package com.kazusa.minecraft_ros2.network;

import com.kazusa.minecraft_ros2.menu.NamedBlockContainer;
import net.minecraft.core.BlockPos;
import net.minecraft.network.FriendlyByteBuf;
import net.minecraftforge.event.network.CustomPayloadEvent;
import net.minecraft.server.level.ServerPlayer;

public class RenamePacket {
    private final BlockPos pos;
    private final String name;

    public RenamePacket(BlockPos pos, String name) {
        this.pos = pos;
        this.name = name;
    }

    public RenamePacket(FriendlyByteBuf buf) {
        this.pos = buf.readBlockPos();
        this.name = buf.readUtf(32767); // 32767 is the max length for a UTF-8 string in Minecraft
    }

    public void encode(FriendlyByteBuf buf) {
        buf.writeBlockPos(this.pos);
        buf.writeUtf(this.name);
    }

    public void handle(CustomPayloadEvent.Context ctx) {
        ctx.enqueueWork(() -> {
            ServerPlayer player = ctx.getSender();
            if (player != null && player.containerMenu instanceof NamedBlockContainer menu) {
                menu.onRename(this.name, player);
            }
        });
        ctx.setPacketHandled(true);
    }
}
