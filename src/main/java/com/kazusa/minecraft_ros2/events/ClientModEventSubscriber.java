package com.kazusa.minecraft_ros2.events;

import com.kazusa.minecraft_ros2.minecraft_ros2;
import com.kazusa.minecraft_ros2.models.DynamicModelEntity;
import com.kazusa.minecraft_ros2.models.DynamicModelEntityRenderer;
import com.kazusa.minecraft_ros2.models.ModEntities;
import com.kazusa.minecraft_ros2.menu.ModMenuTypes;
import com.kazusa.minecraft_ros2.menu.NamedBlockScreen;
import com.kazusa.minecraft_ros2.network.NetworkHandler;
import net.minecraftforge.api.distmarker.Dist;
import net.minecraftforge.client.event.EntityRenderersEvent;
import net.minecraftforge.event.entity.EntityAttributeCreationEvent;
import net.minecraftforge.eventbus.api.SubscribeEvent;
import net.minecraftforge.fml.common.Mod;
import net.minecraftforge.event.AddPackFindersEvent;
import net.minecraft.server.packs.PackType;
import net.minecraft.server.packs.repository.FolderRepositorySource;
import net.minecraft.server.packs.repository.PackSource;
import net.minecraft.world.level.validation.DirectoryValidator;
import net.minecraft.client.gui.screens.MenuScreens;
import net.minecraftforge.fml.event.lifecycle.FMLClientSetupEvent;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.PathMatcher;

@Mod.EventBusSubscriber(modid = minecraft_ros2.MOD_ID, bus = Mod.EventBusSubscriber.Bus.MOD, value = Dist.CLIENT)
public class ClientModEventSubscriber {

    @SubscribeEvent
    public static void onRegisterRenderers(EntityRenderersEvent.RegisterRenderers event) {
        // ModEntities.CUSTOM_ENTITY は DeferredRegister で作った RegistryObject<EntityType<DynamicModelEntity>>
        event.registerEntityRenderer(
            ModEntities.CUSTOM_ENTITY.get(),
            DynamicModelEntityRenderer::new
        );
    }

    @SubscribeEvent
    public static void onEntityAttributeCreation(EntityAttributeCreationEvent event) {
        event.put(
            ModEntities.CUSTOM_ENTITY.get(),        // RegistryObject<EntityType<DynamicModelEntity>>
            DynamicModelEntity.createAttributes().build()
        );
    }

    @SubscribeEvent
    public static void onAddPackFinders(AddPackFindersEvent event) {
        if (event.getPackType() == PackType.CLIENT_RESOURCES) {
            Path packDir = Paths.get("run/resourcepacks/runtime_geo");
            PathMatcher pathMatcher = Paths
                .get("**/*")
                .getFileSystem()
                .getPathMatcher("glob:**/*.{geo.json,mcmeta}"); // パック内の .geo.json と pack.mcmeta ファイルを対象とする
            DirectoryValidator validator = new DirectoryValidator(pathMatcher);
            FolderRepositorySource folderSource = new FolderRepositorySource(
                packDir,
                PackType.CLIENT_RESOURCES,      // リソースパックとして扱う
                PackSource.WORLD,   // ワールド外部パックとして扱う
                validator      // ディレクトリ検証のデフォルト実装
            ); // コンストラクタ( Path, PackType, PackSource, DirectoryValidator ) :contentReference[oaicite:0]{index=0}
            event.addRepositorySource(folderSource);
        }
    }

    @SubscribeEvent
    public static void onClientSetup(FMLClientSetupEvent event) {
        event.enqueueWork(() -> {
            // ここでリソースパックの初期化やその他のクライアント側のセットアップを行う
            NetworkHandler.register(); // パケットハンドラーの登録
        });
        event.enqueueWork(() -> {
            MenuScreens.register(
                ModMenuTypes.NAMED_BLOCK_MENU.get(),
                NamedBlockScreen::new
            );
        });
    }

}
