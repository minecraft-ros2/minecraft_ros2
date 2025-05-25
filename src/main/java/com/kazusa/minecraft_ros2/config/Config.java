package com.kazusa.minecraft_ros2.config;

import net.minecraftforge.common.ForgeConfigSpec;
import org.apache.commons.lang3.tuple.Pair;

public class Config {
    public static final ForgeConfigSpec COMMON_SPEC;
    public static final CommonConfig COMMON;

    static {
        Pair<CommonConfig, ForgeConfigSpec> specPair = new ForgeConfigSpec.Builder().configure(CommonConfig::new);
        COMMON_SPEC = specPair.getRight();
        COMMON = specPair.getLeft();
    }

    public static class CommonConfig {
        public final ForgeConfigSpec.IntValue maxSpeed;
        public final ForgeConfigSpec.BooleanValue enableLogging;
        public final ForgeConfigSpec.ConfigValue<String> topicName;
        public final ForgeConfigSpec.BooleanValue enableDebugDataStreaming;

        public CommonConfig(ForgeConfigSpec.Builder builder) {
            builder.comment("ROS2 Minecraft Mod Configuration").push("general");

            maxSpeed = builder
                    .comment("Maximum speed of movement in Minecraft units per second")
                    .defineInRange("maxSpeed", 10, 1, 100);

            enableLogging = builder
                    .comment("Enable or disable detailed logging")
                    .define("enableLogging", true);
                    
            topicName = builder
                    .comment("ROS2 topic name to subscribe for Twist messages")
                    .define("topicName", "cmd_vel");
                    
            enableDebugDataStreaming = builder
                    .comment("Enable or disable the debug data stream for additional information")
                    .define("enableDebugDataStreaming", false);

            builder.pop();
        }
    }
}