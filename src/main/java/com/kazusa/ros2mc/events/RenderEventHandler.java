package com.kazusa.ros2mc.events;

import com.kazusa.ros2mc.ros2.ROS2Manager;
import com.kazusa.ros2mc.ros2.ImagePublisher;
import net.minecraftforge.event.TickEvent;
import net.minecraftforge.eventbus.api.SubscribeEvent;
import net.minecraftforge.fml.common.Mod;

@Mod.EventBusSubscriber
public class RenderEventHandler {
    @SubscribeEvent
    public static void onRenderTick(TickEvent.RenderTickEvent event) {
        if (event.phase == TickEvent.Phase.END) {
            ROS2Manager ros2 = ROS2Manager.getInstance();
            if (ros2.isInitialized()) {
                ImagePublisher publisher = ros2.getImagePublisher();
                if (publisher != null) {
                    publisher.captureAndPublish(); // 描画直後に画像取得
                }
            }
        }
    }
}
