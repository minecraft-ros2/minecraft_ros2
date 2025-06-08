package com.kazusa.minecraft_ros2.ros2;

import com.mojang.blaze3d.pipeline.RenderTarget;
import com.mojang.blaze3d.pipeline.TextureTarget;
import com.mojang.blaze3d.platform.NativeImage;
import com.mojang.blaze3d.systems.RenderSystem;
import net.minecraft.client.Minecraft;
import org.ros2.rcljava.Time;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.publisher.Publisher;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import sensor_msgs.msg.Image;

import java.util.concurrent.CompletableFuture;

public class ImagePublisher extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(ImagePublisher.class);

    private static final int WIDTH = 1920;
    private static final int HEIGHT = 1080;

    private final Minecraft minecraft;
    private final Publisher<Image> publisher;
    private final NativeImage rgbTexture, depthTexture;
    private final RenderTarget renderTarget;
    private Image rosImage;

    public ImagePublisher() {
        super("minecraft_image_publisher");
        minecraft = Minecraft.getInstance();
        publisher = this.node.createPublisher(Image.class, "/player/image_raw");
        rgbTexture = new NativeImage(NativeImage.Format.RGBA, WIDTH, HEIGHT, false);
        depthTexture = new NativeImage(NativeImage.Format.LUMINANCE, WIDTH, HEIGHT, false);
        LOGGER.info("ImagePublisher initialized and publishing to '/player/image_raw'" + rgbTexture);
        renderTarget = new TextureTarget(WIDTH, HEIGHT, true, false);
    }

    public void captureAndPublish() {
        if (!RenderSystem.isOnRenderThread()) {
            RenderSystem.recordRenderCall(this::captureAndPublish);
            return;
        }
        if (minecraft.level == null) {
            return;
        }

        try {
            int width = renderTarget.width;
            int height = renderTarget.height;

            minecraft.gameRenderer.setRenderBlockOutline(false);
            minecraft.gameRenderer.setPanoramicMode(true);
            minecraft.levelRenderer.graphicsChanged();

            renderTarget.bindWrite(true);
            minecraft.gameRenderer.renderLevel(1.0F, 0L);
            
            // 100x200くらいになるように調整

            int scale = 2;
            int scaledWidth = width / scale;
            int scaledHeight = height / scale;

            // 画面全体のRGBAピクセルを取得
            RenderSystem.bindTexture(renderTarget.getColorTextureId());
            rgbTexture.downloadTexture(0, true);
            RenderSystem.bindTexture(renderTarget.getDepthTextureId());
            renderTarget.blitToScreen(WIDTH, HEIGHT);
            depthTexture.downloadDepthBuffer(0F);

            int[] textureData = rgbTexture.getPixelsRGBA();

            // RGBAからRGBに変換しつつ縮小
            byte[] rgbData = new byte[scaledWidth * scaledHeight * 3];
            for (int y = 0; y < scaledHeight; y++) {
                for (int x = 0; x < scaledWidth; x++) {
                    int srcX = x * scale;
                    int srcY = y * scale;
                    int srcIndex = ((height - 1 - srcY) * width + srcX);
                    int dstIndex = (y * scaledWidth + x) * 3;

                    rgbData[dstIndex] = (byte) (textureData[srcIndex]);
                    rgbData[dstIndex + 1] = (byte) (textureData[srcIndex] >> 8); // G
                    rgbData[dstIndex + 2] = (byte) (textureData[srcIndex] >> 16); // B
                }
            }

            // ROS2 Imageメッセージ作成
            if (rosImage == null) {
                rosImage = new Image();
            }
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
        } finally {
            minecraft.gameRenderer.setRenderBlockOutline(true);
            minecraft.gameRenderer.setPanoramicMode(false);
            minecraft.levelRenderer.graphicsChanged();
            minecraft.getMainRenderTarget().bindWrite(true);
        }
    }
}
