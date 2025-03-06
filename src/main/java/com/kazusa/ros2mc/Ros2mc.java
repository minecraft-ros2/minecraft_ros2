package com.kazusa.ros2mc;

import com.kazusa.ros2mc.config.Config;
import com.kazusa.ros2mc.ros2.ROS2Manager;
import net.minecraftforge.common.MinecraftForge;
import net.minecraftforge.fml.ModLoadingContext;
import net.minecraftforge.fml.common.Mod;
import net.minecraftforge.fml.config.ModConfig;
import net.minecraftforge.fml.event.config.ModConfigEvent;
import net.minecraftforge.fml.event.lifecycle.FMLClientSetupEvent;
import net.minecraftforge.fml.event.lifecycle.FMLCommonSetupEvent;
import net.minecraftforge.fml.javafmlmod.FMLJavaModLoadingContext;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@Mod(Ros2mc.MOD_ID)
public class Ros2mc {
    public static final String MOD_ID = "ros2mc";
    private static final Logger LOGGER = LoggerFactory.getLogger(Ros2mc.class);

    public Ros2mc() {
        LOGGER.info("Initializing ROS2MC mod");
        
        // Register the setup methods for mod loading
        FMLJavaModLoadingContext.get().getModEventBus().addListener(this::setup);
        FMLJavaModLoadingContext.get().getModEventBus().addListener(this::clientSetup);
        
        // Register the configuration
        ModLoadingContext.get().registerConfig(ModConfig.Type.COMMON, Config.COMMON_SPEC);
        FMLJavaModLoadingContext.get().getModEventBus().addListener(this::onConfigLoad);
        
        // Register this mod to the MinecraftForge event bus
        MinecraftForge.EVENT_BUS.register(this);
        
        LOGGER.info("ROS2MC mod initialized");
    }

    private void setup(final FMLCommonSetupEvent event) {
        LOGGER.info("ROS2MC common setup");
        // Performed for both client and server setup
    }

    private void clientSetup(final FMLClientSetupEvent event) {
        LOGGER.info("ROS2MC client setup");
        
        // Initialize ROS2 system on a separate thread to prevent blocking the main thread
        event.enqueueWork(() -> {
            try {
                LOGGER.info("Attempting to initialize ROS2 during client setup...");
                ROS2Manager.getInstance().initialize();
            } catch (Exception e) {
                LOGGER.error("Failed to initialize ROS2 during client setup", e);
            }
        });
    }

    private void onConfigLoad(final ModConfigEvent.Loading event) {
        if (event.getConfig().getSpec() == Config.COMMON_SPEC) {
            LOGGER.info("Loading ROS2MC configuration");
            // Configuration values are accessed directly from Config.COMMON
        }
    }
}
