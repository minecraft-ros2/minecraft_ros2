package com.kazusa.minecraft_ros2.network;

import com.kazusa.minecraft_ros2.minecraft_ros2;
import net.minecraftforge.network.ChannelBuilder;
import net.minecraftforge.network.SimpleChannel;
import net.minecraft.resources.ResourceLocation;
import net.minecraftforge.network.NetworkDirection;
import net.minecraftforge.network.PacketDistributor;

import java.lang.Object;

public class NetworkHandler {
    public static final SimpleChannel CHANNEL = ChannelBuilder.named(
        new ResourceLocation(minecraft_ros2.MOD_ID, "main"))
        .serverAcceptedVersions((status, version) -> true)
        .clientAcceptedVersions((status, version) -> true)
        .networkProtocolVersion(1)
        .simpleChannel();

    public static void register() {
        CHANNEL.messageBuilder(
            RenamePacket.class,
            NetworkDirection.PLAY_TO_SERVER
        )
        .encoder(RenamePacket::encode)
        .decoder(RenamePacket::new)
        .consumerMainThread(RenamePacket::handle)
        .add();
    }

    public static void sendToServer(Object msg) {
        CHANNEL.send(msg, PacketDistributor.SERVER.noArg());
    }
}
