package com.kazusa.ros2mc.events;

import com.kazusa.ros2mc.ros2.ROS2Manager;
import com.kazusa.ros2mc.ros2.ImagePublisher;
import net.minecraftforge.event.TickEvent;
import net.minecraftforge.eventbus.api.SubscribeEvent;
import net.minecraftforge.fml.common.Mod;

@Mod.EventBusSubscriber
public class RenderEventHandler {
    private static int renderTickCount = 0;

    @SubscribeEvent
    public static void onRenderTick(TickEvent.RenderTickEvent event) {
        if (event.phase == TickEvent.Phase.END) {
            renderTickCount++;
            if (renderTickCount==3) {
                renderTickCount = 0; // Reset the counter

                ROS2Manager ros2 = ROS2Manager.getInstance();
                if (ros2.isInitialized()) {
                    ImagePublisher publisher = ros2.getImagePublisher();
                    if (publisher != null) {
                        publisher.captureAndPublish(); // 3回に1回画像取得
                    }
                }
            }
        }
    }
}
