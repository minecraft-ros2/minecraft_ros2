package com.kazusa.minecraft_ros2.ros2;

import com.kazusa.minecraft_ros2.minecraft_ros2;
import com.kazusa.minecraft_ros2.models.ModEntities;
import com.kazusa.minecraft_ros2.models.DynamicModelEntity;
import com.kazusa.minecraft_ros2.utils.GeometryApplier;
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.node.BaseComposableNode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import simulation_interfaces.srv.SpawnEntity;
import simulation_interfaces.srv.SpawnEntity_Request;
import simulation_interfaces.srv.SpawnEntity_Response;
import simulation_interfaces.msg.Result;
import geometry_msgs.msg.PoseStamped;
import org.ros2.rcljava.service.RMWRequestId;
import org.ros2.rcljava.service.Service;

import net.minecraftforge.server.ServerLifecycleHooks;
import net.minecraft.server.MinecraftServer;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.Level;
import net.minecraft.world.entity.monster.Zombie;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.entity.EntityType;
import net.minecraft.world.entity.MobCategory;
import net.minecraft.world.entity.LivingEntity;
import net.minecraft.world.entity.EntityDimensions;
import net.minecraft.core.BlockPos;
import net.minecraft.core.registries.BuiltInRegistries;
import net.minecraft.core.Registry;
import net.minecraft.resources.ResourceLocation;
import net.minecraftforge.registries.RegistryObject;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.stream.Collectors;
import java.util.List;
import java.util.HashMap;
import java.util.Map;

public class SpawnEntityService  extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(SpawnEntityService.class);

    private final Service<SpawnEntity> service;

    public SpawnEntityService() {
        super("add_two_ints_service");

        try {
            this.service = this.node.<SpawnEntity>createService(
                SpawnEntity.class,            // サービス定義クラス
                "spawn_entity",               // トピック名
                (RMWRequestId header, SpawnEntity_Request request,
                    SpawnEntity_Response response)
                    -> this.handleService(header, request, response));
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException("サービス生成に失敗", e);
        }
        LOGGER.info("SpawnEntityService initialized and listening on '/add_two_ints'");
    }

    private void handleService(final RMWRequestId header,
            final SpawnEntity_Request request,
            final SpawnEntity_Response response) {
        String modelName = request.getName();
        String modelUri = request.getUri();
        String modelResourceString = request.getResourceString();
        if (modelName == null && (modelUri == null || modelResourceString == null)) {
            LOGGER.error("Invalid request: " + request);
            return;
        }
        if (modelResourceString == null || modelResourceString.isEmpty()) {
            try {
                modelResourceString = Files.readString(Path.of(modelUri));
            } catch (IOException e) {
                LOGGER.error("Failed to read model file: " + modelUri, e);
                return;
            }
        }
        try {
            urdfToObj(modelResourceString, "/tmp/" + modelName + ".obj");
            createJsonFromObj("/tmp/" + modelName + ".obj");
        } catch (Exception e) {
            throw new RuntimeException("URDF to glTF conversion failed", e);
        }

        // jsonファイルのパス
        Path jsonPath = Path.of("/tmp/" + modelName + ".geo.json");
        GeometryApplier.applyJson(
            jsonPath, "runtime_geo", modelName
        );
        //DynamicModelEntity.setGeometryLocation(
        //    new ResourceLocation("runtime_geo", "geo/" + modelName + ".geo.json")
        //);

        double x = request.getInitialPose().getPose().getPosition().getX();
        double y = request.getInitialPose().getPose().getPosition().getY();
        double z = request.getInitialPose().getPose().getPosition().getZ();

        MinecraftServer server = ServerLifecycleHooks.getCurrentServer();
        ServerLevel world = server.getLevel(Level.OVERWORLD);

        // 3) Entity を生成して座標をセット
        RegistryObject<EntityType<DynamicModelEntity>> ro = ModEntities.CUSTOM_ENTITY;
        EntityType<DynamicModelEntity> type = ro.get();
        DynamicModelEntity robot = type.create(world);
        System.out.println("SpawnEntityService: " + robot);
        System.out.println("SpawnEntityService: " + robot);
        System.out.println("SpawnEntityService: " + robot);
        System.out.println("SpawnEntityService: " + robot);
        System.out.println("SpawnEntityService: " + robot);

        robot.setGeometryLocation(new ResourceLocation("runtime_geo", "geo/" + modelName + ".geo.json"));

        // 4) 位置と回転を設定
        robot.moveTo(x, y, z, /* yaw= */ 0.0F, /* pitch= */ 0.0F);

        // 5) ワールドにスポーン
        world.addFreshEntity(robot);

        for (int i = 0; i < 100; i++) {
            if (robot != null) {
                // ここで現在位置を取得・ログに出力
                double px = robot.getX();
                double py = robot.getY();
                double pz = robot.getZ();
                BlockPos bpos = robot.blockPosition();  // 小数点以下切り捨てしてブロック座標に
            
                LOGGER.info("Spawned DynamicModelEntity at raw: ({}, {}, {}), block: {}", px, py, pz, bpos);
            }
            try {
                Thread.sleep(100);  // 100ms 待機
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();  // 割り込みフラグをセット
                LOGGER.error("Thread interrupted while waiting for entity spawn", e);
                return;
            }
        }

        Result result = new Result();
        byte code = (byte) 0;//(robot != null ? 1 : 0);
        result.setResult(Byte.valueOf(code)); // 成功
        response.setResult(result);
        System.out.println("request: " + x + ", " + y + ", " + z);
    }

    /**
     * URDF テキストを読み込み、Collada (.dae) 経由で obj に変換します。
     *
     * @param urdfText     URDF の内容
     * @param outputObj    出力する OBJ ファイルのパス
     * @throws IOException          ファイル I/O またはプロセス実行時のエラー
     * @throws InterruptedException プロセスの割り込み
     * @throws RuntimeException  プロセスの終了コードが 0 でない場合
     */
    private static void urdfToObj(String urdfText, String outputObj)
    throws IOException, InterruptedException {
        // 1) 一時 URDF ファイル
        Path tmpUrdf = Files.createTempFile("tmp_model", ".urdf");
        Files.writeString(tmpUrdf, urdfText);

        // 2) Collada (.dae) 用のパス
        Path tmpDae = tmpUrdf.resolveSibling(tmpUrdf.getFileName().toString().replace(".urdf", ".dae"));

        try {
            // 3) URDF → Collada
            ProcessBuilder pb1 = new ProcessBuilder(
                    "/usr/lib/collada_urdf/urdf_to_collada",
                    tmpUrdf.toString(), tmpDae.toString()
            );
            pb1.inheritIO();  // 標準入出力を親プロセスと共有（ログを見たい場合）
            Process p1 = pb1.start();
            int exit1 = p1.waitFor();
            if (exit1 != 0) {
                throw new RuntimeException("urdf_to_collada failed with exit code " + exit1);
            }

            // 4) Collada → glTF
            String expr = String.format(
                "import bpy; " +
                "bpy.ops.wm.read_factory_settings(use_empty=True); " +
                "bpy.ops.wm.collada_import(filepath=r'%s'); " +
                "bpy.ops.export_scene.obj(filepath=r'%s')",
                tmpDae.toString(), outputObj.toString()
            );
            ProcessBuilder pb2 = new ProcessBuilder(
                "blender", "--background", "--python-expr", expr
            );
            pb2.inheritIO();
            Process p2 = pb2.start();
            int exit2 = p2.waitFor();
            if (exit2 != 0) {
                throw new RuntimeException("blender export failed with exit code " + exit2);
            }

        } finally {
            // 5) 後始末：一時ファイルを削除
            try { Files.deleteIfExists(tmpUrdf); } catch (IOException ignored) {}
            try { Files.deleteIfExists(tmpDae);  } catch (IOException ignored) {}
        }
    }

    private void createJsonFromObj(String objPath) throws IOException, InterruptedException {
        // Python スクリプトを生成して、obj → JSON 変換を行う
        String script = """
import trimesh
import numpy as np
import json
import sys

def voxel_to_cubes(matrix, pitch, atlas_u=0, atlas_v=0):
    cubes = []
    for x, y, z in np.argwhere(matrix):
        origin = [float(x) * pitch * 16, float(y) * pitch * 16, float(z) * pitch * 16]
        size = [pitch * 16, pitch * 16, pitch * 16]
        cubes.append({
            "origin": origin,
            "size": size,
            "uv": [atlas_u, atlas_v],  # テクスチャ UV 左上
            "inflate": 0.0
        })
    return cubes


def main(obj_path, pitch=0.1, atlas_u=0, atlas_v=0, identifier="voxel_model", tex_w=16, tex_h=16):
    # OBJ 読み込み
    mesh = trimesh.load(obj_path)
    if isinstance(mesh, trimesh.scene.Scene):
        mesh = trimesh.util.concatenate([trimesh.Trimesh(m.vertices, m.faces) for m in mesh.geometry.values()])
    # Voxel 化
    vox = mesh.voxelized(pitch)
    matrix = vox.matrix

    # Geckolib 用 .geo.json
    geo = {
        "format_version": "1.12.0",
        "minecraft:geometry": [
            {
                "description": {
                    "identifier": identifier,
                    "texture_width": tex_w,
                    "texture_height": tex_h
                },
                "bones": [
                    {
                        "name": "root",
                        "pivot": [0, 0, 0],
                        "cubes": voxel_to_cubes(matrix, pitch, atlas_u, atlas_v)
                    }
                ]
            }
        ]
    }

    out_path = obj_path.rsplit('.', 1)[0] + '.geo.json'
    with open(out_path, 'w') as f:
        json.dump(geo, f, indent=2)
    print(f"Generated: {out_path}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python obj2mcgeo.py model.obj [pitch] [atlas_u] [atlas_v] [identifier] [tex_w] [tex_h]")
        sys.exit(1)
    obj_file = sys.argv[1]
    p = float(sys.argv[2]) if len(sys.argv) >= 3 else 0.01
    u = int(sys.argv[3]) if len(sys.argv) >= 4 else 0
    v = int(sys.argv[4]) if len(sys.argv) >= 5 else 0
    ident = sys.argv[5] if len(sys.argv) >= 6 else "voxel_model"
    tw = int(sys.argv[6]) if len(sys.argv) >= 7 else 16
    th = int(sys.argv[7]) if len(sys.argv) >= 8 else 16
    main(obj_file, p, u, v, ident, tw, th)
            """;
        
        Path tmpPython = Files.createTempFile("convert_obj_to_json", ".py");
        Files.writeString(tmpPython, script);

        try {
            ProcessBuilder pb2 = new ProcessBuilder(
                "python3", tmpPython.toString(), objPath.toString(), "0.05"
            );
            pb2.inheritIO();
            Process p2 = pb2.start();
            int exit2 = p2.waitFor();
            if (exit2 != 0) {
                throw new RuntimeException("blender export failed with exit code " + exit2);
            }

        } finally {
            // 5) 後始末：一時ファイルを削除
            try { Files.deleteIfExists(tmpPython); } catch (IOException ignored) {}
        }
    }

}