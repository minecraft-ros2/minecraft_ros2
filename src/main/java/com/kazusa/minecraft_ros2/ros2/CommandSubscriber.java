package com.kazusa.minecraft_ros2.ros2;

import com.mojang.brigadier.exceptions.CommandSyntaxException;
import net.minecraft.commands.CommandSourceStack;
import net.minecraft.server.MinecraftServer;
import net.minecraftforge.server.ServerLifecycleHooks;
import org.ros2.rcljava.node.BaseComposableNode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import std_msgs.msg.String;

public class CommandSubscriber extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(CommandSubscriber.class);

    public CommandSubscriber() {
        super("minecraft_command_subscriber");
        this.node.<String>createSubscription(
            String.class, "/minecraft/command", this::twistCallback);
    }

    private void twistCallback(final String msg) {
        MinecraftServer server = ServerLifecycleHooks.getCurrentServer();
        if (server == null) {
            LOGGER.error("Failed to retrieve the server instance.");
            return;
        }

        server.execute(() -> {
            CommandSourceStack source = server.createCommandSourceStack()
                                            .withPermission(2);
            try {
                server.getCommands()
                    .getDispatcher()
                    .execute(msg.getData(), source);
            } catch (CommandSyntaxException e) {
                LOGGER.error("A syntax error occurred while executing the command: {}", e.getMessage());
            } catch (Exception e) {
                LOGGER.error("An unexpected exception occurred during command execution: {}", e);
            }
        });
    }
}
