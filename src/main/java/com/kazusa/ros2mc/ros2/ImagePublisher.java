package com.kazusa.ros2mc.ros2;

import net.minecraft.client.Minecraft;
import org.lwjgl.BufferUtils;
import org.lwjgl.opengl.GL11;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.timer.WallTimer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import java.util.concurrent.TimeUnit;
import sensor_msgs.msg.Image;

import java.nio.ByteBuffer;

public class ImagePublisher extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(ImagePublisher.class);
    private final Publisher<Image> publisher;
    private final WallTimer timer;
    private final Minecraft minecraft;

    public ImagePublisher() {
        super("minecraft_image_publisher");
        publisher = this.node.createPublisher(Image.class, "/player/image_raw");
        minecraft = Minecraft.getInstance();
        timer = this.node.createWallTimer(1000, TimeUnit.MILLISECONDS, this::publishPlayerViewImage);
        LOGGER.info("ImagePublisher initialized and publishing to 'player/image_raw' topic");
    }

    public void publishPlayerViewImage() {
        try {
            minecraft.execute(() -> {
                try {
                    int width = minecraft.getMainRenderTarget().width;
                    int height = minecraft.getMainRenderTarget().height;

                    ByteBuffer buffer = BufferUtils.createByteBuffer(width * height * 4);
                    GL11.glReadPixels(0, 0, width, height, GL11.GL_RGBA, GL11.GL_UNSIGNED_BYTE, buffer);

                    byte[] pixelData = new byte[buffer.remaining()];
                    buffer.get(pixelData);

                    // Convert RGBA to RGB by removing the alpha channel
                    byte[] rgbData = new byte[width * height * 3];

                    // Flip the image vertically
                    for (int y = 0; y < height; y++) {
                        for (int x = 0; x < width; x++) {
                            int srcIndex = ((height - 1 - y) * width + x) * 4; // Source index in RGBA buffer
                            int dstIndex = (y * width + x) * 3; // Destination index in RGB buffer

                            rgbData[dstIndex] = pixelData[srcIndex];       // Red
                            rgbData[dstIndex + 1] = pixelData[srcIndex + 1]; // Green
                            rgbData[dstIndex + 2] = pixelData[srcIndex + 2]; // Blue
                        }
                    }

                    Image rosImage = new Image();
                    rosImage.setWidth(width);
                    rosImage.setHeight(height);
                    rosImage.setEncoding("rgb8");
                    rosImage.setData(rgbData);
                    rosImage.setStep(width * 3); // Step size in bytes (3 bytes per pixel for RGB)

                    publisher.publish(rosImage);
                    LOGGER.info("Image published");
                } catch (Exception e) {
                    LOGGER.error("Failed to publish image", e);
                }
            });
        } catch (Exception e) {
            LOGGER.error("Failed to publish image", e);
        }
    }
}