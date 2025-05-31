package com.kazusa.minecraft_ros2.ros2;

import com.mojang.brigadier.exceptions.CommandSyntaxException;
import net.minecraft.commands.CommandSourceStack;
import net.minecraft.server.MinecraftServer;
import net.minecraftforge.server.ServerLifecycleHooks;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.service.RMWRequestId;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import minecraft_msgs.srv.Command;
import minecraft_msgs.srv.Command_Request;
import minecraft_msgs.srv.Command_Response;

public class CommandSubscriber extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(CommandSubscriber.class);

    public CommandSubscriber() {
        super("minecraft_command_subscriber");
        try {
            this.node.<Command>createService(
                    Command.class,
                    "/minecraft/command",
                    (RMWRequestId header, Command_Request request, Command_Response response) -> this
                            .handleService(header, request, response));
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException("Can not create service", e);
        }
    }

    private void handleService(final RMWRequestId header,
            final Command_Request request,
            final Command_Response response) {
        var command = request.getCommand();
        if (command == null || command.isEmpty()){
            LOGGER.error("Invalid request: " + request);
            response.setMessage("Invalid request: " + request);
            response.setSuccess(false);
            return;
        }

        MinecraftServer server = ServerLifecycleHooks.getCurrentServer();
        if (server == null) {
            LOGGER.error("Failed to retrieve the server instance.");
            response.setMessage("Failed to retrieve the server instance.");
            response.setSuccess(false);
            return;
        }

        server.execute(() -> {
            CommandSourceStack source = server.createCommandSourceStack()
                    .withPermission(2);
            try {
                server.getCommands()
                        .getDispatcher()
                        .execute(command, source);
            } catch (CommandSyntaxException e) {
                LOGGER.error("A syntax error occurred while executing the command: {}", e.getMessage());
                response.setMessage("A syntax error occurred while executing the command");
                response.setSuccess(false);
                return;
            } catch (Exception e) {
                LOGGER.error("An unexpected exception occurred during command execution: {}", e);
                response.setMessage("An unexpected exception occurred during command execution");
                response.setSuccess(false);
                return;
            }
        });
        response.setMessage("Success");
        response.setSuccess(true);
    }
}
