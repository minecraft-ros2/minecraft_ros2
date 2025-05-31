package com.kazusa.minecraft_ros2.menu;

import com.kazusa.minecraft_ros2.minecraft_ros2;
import com.kazusa.minecraft_ros2.menu.NamedBlockContainer;
import com.kazusa.minecraft_ros2.network.NetworkHandler;
import com.kazusa.minecraft_ros2.network.RenamePacket;
import net.minecraft.client.gui.GuiGraphics;
import net.minecraft.client.gui.components.Button;
import net.minecraft.client.gui.components.EditBox;
import net.minecraft.client.gui.screens.inventory.AbstractContainerScreen;
import net.minecraft.network.chat.Component;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.world.entity.player.Inventory;
import com.mojang.blaze3d.platform.InputConstants;

public class NamedBlockScreen extends AbstractContainerScreen<NamedBlockContainer> {
    private static final ResourceLocation BG = new ResourceLocation(
        minecraft_ros2.MOD_ID, "textures/gui/named_block_bg.png"
    );
    private EditBox nameField;

    public NamedBlockScreen(NamedBlockContainer cont, Inventory inv, Component title) {
        super(cont, inv, title);
        this.imageWidth = 176;
        this.imageHeight = 80;
    }

    @Override
    protected void init() {
        super.init();
        nameField = new EditBox(font, leftPos + 8, topPos + 20, 160, 20, Component.literal(""));
        nameField.setMaxLength(32);
        addRenderableWidget(nameField);
        addRenderableWidget(Button.builder(Component.literal("OK"), btn -> {
            NetworkHandler.sendToServer(
                new RenamePacket(menu.pos, nameField.getValue())
            );
            minecraft.player.closeContainer();
        }).bounds(leftPos + 60, topPos + 50, 56, 20).build());
    }

    @Override
    public void render(GuiGraphics graphics, int mouseX, int mouseY, float partialTicks) {
        // 背景を描画（四引数のシグネチャに注意）
        renderBackground(graphics, mouseX, mouseY, partialTicks);
        // 親クラスの描画（スロットやタイトルなど）
        super.render(graphics, mouseX, mouseY, partialTicks);
        // テキストフィールドの描画
        nameField.render(graphics, mouseX, mouseY, partialTicks);
    }

    @Override
    protected void renderBg(GuiGraphics gg, float delta, int mouseX, int mouseY) {
        gg.blit(BG, leftPos, topPos, 0, 0, imageWidth, imageHeight);
    }

    @Override
    public boolean keyPressed(int keyCode, int scanCode, int modifiers) {
        // 押されたキーのInputConstants.Keyオブジェクトを取得
        InputConstants.Key key = InputConstants.getKey(keyCode, scanCode);

        // インベントリキー（デフォルトE）が押されていたら消費して閉じない
        if (minecraft.options.keyInventory.isActiveAndMatches(key)) {
            return true;
        }
        // それ以外は通常処理
        return super.keyPressed(keyCode, scanCode, modifiers);
    }
}