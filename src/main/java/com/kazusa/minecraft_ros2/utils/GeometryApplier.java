package com.kazusa.minecraft_ros2.utils;

import net.minecraftforge.api.distmarker.Dist;
import net.minecraftforge.api.distmarker.OnlyIn;
import net.minecraftforge.fml.loading.FMLPaths;

import net.minecraft.client.Minecraft;
import net.minecraftforge.server.ServerLifecycleHooks;
import net.minecraft.server.MinecraftServer;
import net.minecraft.server.packs.repository.FolderRepositorySource;
import net.minecraft.server.packs.PackType;
import net.minecraft.server.packs.repository.PackSource;
import net.minecraft.world.level.validation.DirectoryValidator;
import net.minecraft.server.packs.repository.PackRepository;
import net.minecraft.server.packs.repository.Pack;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.PathMatcher;
import java.nio.file.StandardCopyOption;
import java.nio.charset.StandardCharsets;
import java.util.Collection;
import java.util.ArrayList;
import java.util.stream.Collectors;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

@OnlyIn(Dist.CLIENT)
public class GeometryApplier {
    private static final Logger LOGGER = LogManager.getLogger();

    public static void initResourcePack() {
        try {
            // ゲームディレクトリの resourcepacks/runtime_geo を初期化
            Path gameDir = FMLPaths.GAMEDIR.get();
            Path packDir = gameDir.resolve("resourcepacks").resolve("runtime_geo");
            Path mcmeta = packDir.resolve("pack.mcmeta");
            if (!Files.exists(mcmeta)) {
                Files.createDirectories(packDir);
                String mcmetaJson = """
                {
                  "pack": {
                    "description": "Runtime geometry pack",
                    "pack_format": 32
                  }
                }
                """;
                Files.write(mcmeta, mcmetaJson.getBytes(StandardCharsets.UTF_8));
            }
            String jsonTemplate = """
            {
              "format_version": "1.12.0",
              "minecraft:geometry": [
                {
                  "description": {
                    "identifier": "voxel_model",
                    "texture_width": 16,
                    "texture_height": 16
                  },
                  "bones": [
                    {
                      "name": "root",
                      "pivot": [
                        0,
                        0,
                        0
                      ],
                      "cubes": [
                        {
                          "origin": [
                            0.0,
                            0.0,
                            0.0
                          ],
                          "size": [
                            16.0,
                            16.0,
                            16.0
                          ],
                          "uv": [
                            0,
                            0
                          ],
                          "inflate": 0.0
                        },
                        {
                          "origin": [
                            0.0,
                            16.0,
                            0.0
                          ],
                          "size": [
                            16.0,
                            16.0,
                            16.0
                          ],
                          "uv": [
                            0,
                            0
                          ],
                          "inflate": 0.0
                        }
                      ]
                    }
                  ]
                }
              ]
            }
            """;
            for (int i = 0; i < 10; i++) {
                Path destJson = packDir.resolve("assets")
                    .resolve("runtime_geo")
                    .resolve("geo")
                    .resolve("dynamic_model_" + i + ".geo.json");
                Files.createDirectories(destJson.getParent());
                Files.write(destJson, jsonTemplate.getBytes(StandardCharsets.UTF_8));
            }
            LOGGER.info("Initialized runtime geometry resource pack at: {}", packDir);
        } catch (Exception e) {
            LOGGER.error("Failed to initialize runtime geometry resource pack", e);
        }
    }

    /**
     * @param sourceJsonPath  生成済みの .geo.json ファイルへのパス
     * @param namespace       JSON を置く namespace（ResourceLocation の前半部分）
     * @param geomName        JSON ファイル名（拡張子抜き）
     */
    public static void applyJson(Path sourceJsonPath, String namespace, int geoNumber) {
        try {
            // ゲームディレクトリの resourcepacks/runtime_geo にコピー
            Path gameDir = FMLPaths.GAMEDIR.get();
            Path packDir = gameDir.resolve("resourcepacks").resolve("runtime_geo");
            Path destJson = packDir
                .resolve("assets")
                .resolve(namespace)
                .resolve("geo")
                .resolve("dynamic_model_" + geoNumber + ".geo.json");
            Files.createDirectories(destJson.getParent());
            Files.copy(sourceJsonPath, destJson, StandardCopyOption.REPLACE_EXISTING);

            // リソースパックとして登録・リロード
            addExternalResourcePack("runtime_geo", packDir.toString());

            LOGGER.info("Applied dynamic geometry: {}", destJson);
        } catch (Exception e) {
            LOGGER.error("Failed to apply dynamic geometry JSON", e);
        }
    }

    @OnlyIn(Dist.CLIENT)
    public static void addExternalResourcePack(String packName, String directoryPath) {
        File packDir = new File(directoryPath);
        PathMatcher pathMatcher = Paths
            .get("**/*")
            .getFileSystem()
            .getPathMatcher("glob:**/*.{geo.json,mcmeta}"); // パック内の .geo.json と pack.mcmeta ファイルを対象とする
        DirectoryValidator validator = new DirectoryValidator(pathMatcher);
        FolderRepositorySource folderSource = new FolderRepositorySource(
            packDir.toPath(),
            PackType.CLIENT_RESOURCES,      // リソースパックとして扱う
            PackSource.WORLD,   // ワールド外部パックとして扱う
            validator      // ディレクトリ検証のデフォルト実装
        ); // コンストラクタ( Path, PackType, PackSource, DirectoryValidator ) :contentReference[oaicite:0]{index=0}

        // リポジトリへ追加
        PackRepository repo = Minecraft.getInstance()
                                               .getResourcePackRepository();
        repo.addPackFinder(folderSource);

        // F3+T 相当の再読み込み
        Minecraft.getInstance().reloadResourcePacks();  // 全パックをリロード :contentReference[oaicite:2]{index=2}
        MinecraftServer server = ServerLifecycleHooks.getCurrentServer();
        Collection<String> resourcePacks = new ArrayList<>();
        repo.getSelectedPacks().stream()
            .map(Pack::getId)
            .map(Object::toString)            // getId() が ResourceLocation を返す場合
            .forEach(resourcePacks::add);
        server.reloadResources(resourcePacks);
    }
}
