package com.kazusa.minecraft_ros2.ros2;

import net.minecraft.client.Minecraft;
import org.lwjgl.BufferUtils;
import org.lwjgl.opengl.GL11;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.publisher.Publisher;
import sensor_msgs.msg.Image;
import org.ros2.rcljava.Time;
import java.nio.ByteBuffer;
import java.util.concurrent.CompletableFuture;

public class ImagePublisher extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(ImagePublisher.class);

    private final Publisher<Image> publisher;
    private final Minecraft minecraft;

    public ImagePublisher() {
        super("minecraft_image_publisher");
        publisher = this.node.createPublisher(Image.class, "/player/image_raw");
        minecraft = Minecraft.getInstance();
        LOGGER.info("ImagePublisher initialized and publishing to '/player/image_raw'");
    }

    public void captureAndPublish() {
        try {
            int width = minecraft.getMainRenderTarget().width;
            int height = minecraft.getMainRenderTarget().height;

            // 100x200くらいになるように調整

            int scale = 2;
            int scaledWidth = width / scale;
            int scaledHeight = height / scale;

            // 画面全体のRGBAピクセルを取得
            ByteBuffer buffer = BufferUtils.createByteBuffer(width * height * 4);
            GL11.glReadPixels(0, 0, width, height, GL11.GL_RGBA, GL11.GL_UNSIGNED_BYTE, buffer);

            // RGBAからRGBに変換しつつ縮小
            byte[] rgbData = new byte[scaledWidth * scaledHeight * 3];
            for (int y = 0; y < scaledHeight; y++) {
                for (int x = 0; x < scaledWidth; x++) {
                    int srcX = x * scale;
                    int srcY = y * scale;
                    int srcIndex = ((height - 1 - srcY) * width + srcX) * 4;
                    int dstIndex = (y * scaledWidth + x) * 3;

                    rgbData[dstIndex] = buffer.get(srcIndex);         // R
                    rgbData[dstIndex + 1] = buffer.get(srcIndex + 1); // G
                    rgbData[dstIndex + 2] = buffer.get(srcIndex + 2); // B
                }
            }

            // ROS2 Imageメッセージ作成
            Image rosImage = new Image();
            rosImage.getHeader().setFrameId("player");
            rosImage.getHeader().setStamp(Time.now());
            rosImage.setWidth(scaledWidth);
            rosImage.setHeight(scaledHeight);
            rosImage.setEncoding("rgb8");
            rosImage.setStep(scaledWidth * 3);
            rosImage.setData(rgbData);

            // 非同期で送信（メインスレッドの負荷軽減）
            CompletableFuture.runAsync(() -> publisher.publish(rosImage));

        } catch (Exception e) {
            LOGGER.error("Failed to capture and publish image", e);
        }
    }
}
