package com.kazusa.minecraft_ros2;

import com.kazusa.minecraft_ros2.config.Config;
import com.kazusa.minecraft_ros2.ros2.ROS2Manager;
import com.kazusa.minecraft_ros2.ros2.ModItems;
import net.minecraftforge.common.MinecraftForge;
import net.minecraftforge.eventbus.api.IEventBus;
import net.minecraftforge.fml.ModLoadingContext;
import net.minecraftforge.fml.common.Mod;
import net.minecraftforge.fml.config.ModConfig;
import net.minecraftforge.fml.event.config.ModConfigEvent;
import net.minecraftforge.fml.event.lifecycle.FMLClientSetupEvent;
import net.minecraftforge.fml.event.lifecycle.FMLCommonSetupEvent;
import net.minecraftforge.fml.javafmlmod.FMLJavaModLoadingContext;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;



@Mod(minecraft_ros2.MOD_ID)
public class minecraft_ros2 {
    public static final String MOD_ID = "minecraft_ros2";
    private static final Logger LOGGER = LoggerFactory.getLogger(minecraft_ros2.class);

    public minecraft_ros2() throws NoSuchFieldException, IllegalAccessException {
        LOGGER.info("Initializing minecraft_ros2 mod");

        // Register the setup methods for mod loading
        FMLJavaModLoadingContext.get().getModEventBus().addListener(this::setup);
        FMLJavaModLoadingContext.get().getModEventBus().addListener(this::clientSetup);

        // Register the configuration
        ModLoadingContext.get().registerConfig(ModConfig.Type.COMMON, Config.COMMON_SPEC);
        FMLJavaModLoadingContext.get().getModEventBus().addListener(this::onConfigLoad);

        // Register this mod to the MinecraftForge event bus
        MinecraftForge.EVENT_BUS.register(this);

        IEventBus bus = FMLJavaModLoadingContext.get().getModEventBus();
        ModItems.register(bus);

        LOGGER.info("minecraft_ros2 mod initialized");
    }

    private void setup(final FMLCommonSetupEvent event) {
        LOGGER.info("minecraft_ros2 common setup");
        // Performed for both client and server setup
    }



    private void clientSetup(final FMLClientSetupEvent event) {
        LOGGER.info("minecraft_ros2 client setup");

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
            LOGGER.info("Loading minecraft_ros2 configuration");
            // Configuration values are accessed directly from Config.COMMON
        }
    }
}
