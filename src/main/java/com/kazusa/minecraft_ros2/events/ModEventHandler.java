package com.kazusa.minecraft_ros2.events;

import com.kazusa.minecraft_ros2.ros2.ROS2Manager;
import net.minecraftforge.event.TickEvent;
import net.minecraftforge.event.server.ServerStartingEvent;
import net.minecraftforge.event.server.ServerStoppingEvent;
import net.minecraftforge.eventbus.api.SubscribeEvent;
import net.minecraftforge.fml.common.Mod;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Handles Forge events for the minecraft_ros2 mod
 */
@Mod.EventBusSubscriber
public class ModEventHandler {
    private static final Logger LOGGER = LoggerFactory.getLogger(ModEventHandler.class);
    private static boolean initialized = false;

    /**
     * Initialize ROS2 when the server starts
     */
    @SubscribeEvent
    public static void onServerStarting(ServerStartingEvent event) {
        if (!initialized) {
            LOGGER.info("Server starting, initializing ROS2...");
            ROS2Manager.getInstance().initialize();
            initialized = true;
        }
    }

    /**
     * Shutdown ROS2 when the server stops
     */
    @SubscribeEvent
    public static void onServerStopping(ServerStoppingEvent event) {
        if (initialized) {
            LOGGER.info("Server stopping, shutting down ROS2...");
            ROS2Manager.getInstance().shutdown();
            initialized = false;
        }
    }

    /**
     * Process ROS2 messages during client tick
     */
    @SubscribeEvent
    public static void onClientTick(TickEvent.ClientTickEvent event) {
        // Only process on the end phase to avoid processing twice per tick
        if (event.phase == TickEvent.Phase.END && initialized) {
            // Process ROS2 twist messages to update player movement
            ROS2Manager.getInstance().processTwistMessages();
        }
    }
}