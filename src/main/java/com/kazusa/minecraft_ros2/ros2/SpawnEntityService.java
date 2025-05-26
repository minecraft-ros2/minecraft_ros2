package com.kazusa.minecraft_ros2.ros2;

import com.kazusa.minecraft_ros2.minecraft_ros2;
import com.kazusa.minecraft_ros2.models.ModEntities;
import com.kazusa.minecraft_ros2.models.DynamicModelEntity;
import com.kazusa.minecraft_ros2.models.DynamicModelEntityModel;
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
import net.minecraft.world.entity.Entity;
import net.minecraft.world.entity.EntityType;
import net.minecraft.resources.ResourceLocation;
import net.minecraftforge.registries.RegistryObject;
import net.minecraft.network.chat.Component;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.ArrayList;

public class SpawnEntityService  extends BaseComposableNode {
    public final List<DynamicModelEntity> spawnedEntities = new ArrayList<>();

    private static final Logger LOGGER = LoggerFactory.getLogger(SpawnEntityService.class);

    private final Service<SpawnEntity> service;

    // モデルの名前の空配列
    private final List<String> jsonFileNames;

    private int current_model_number;

    public SpawnEntityService() {
        super("spawn_entity_service");
        current_model_number = 0;
        // 空のリストを作成！
        jsonFileNames = new ArrayList<>();

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
        //String modelResourceString = request.getResourceString();
        if (modelName == null || modelName.isEmpty() || modelUri == null || modelUri.isEmpty()) {
            LOGGER.error("Invalid request: " + request);
            Result errorResult = new Result();
            errorResult.setResult(Byte.valueOf((byte) 0)); // Failure code
            errorResult.setErrorMessage("Invalid request: modelName or modelUri is empty");
            response.setResult(errorResult);
            return;
        }

        boolean isUnique = true;
        for (DynamicModelEntity entity : spawnedEntities) {
            if (entity.getCustomName() != null && entity.getCustomName().getString().equals(modelName)) {
                isUnique = false;
                break;
            }
        }
        if (!isUnique) {
            LOGGER.error("Entity with name '" + modelName + "' already exists.");
            Result errorResult = new Result();
            errorResult.setResult(Byte.valueOf((byte) 0)); // Failure code
            errorResult.setErrorMessage("Entity with name '" + modelName + "' already exists.");
            response.setResult(errorResult);
            return;
        }

        if (modelUri.startsWith("file://")) {
            modelUri = modelUri.replace("file://", ""); // file:// プレフィックスを削除
        }

        if (modelUri.startsWith("http://") || modelUri.startsWith("https://")) {
            LOGGER.error("Remote URIs are not supported: " + modelUri);
            Result errorResult = new Result();
            errorResult.setResult(Byte.valueOf((byte) 0)); // Failure code
            errorResult.setErrorMessage("Remote URIs are not supported: " + modelUri);
            response.setResult(errorResult);
            return;
        }

        if (!modelUri.endsWith(".geo.json")) {
            LOGGER.error("Invalid model URI, must end with .geo.json: " + modelUri);
            Result errorResult = new Result();
            errorResult.setResult(Byte.valueOf((byte) 0)); // Failure code
            errorResult.setErrorMessage("Invalid model URI, must end with .geo.json: " + modelUri);
            response.setResult(errorResult);
            return;
        }

        // jsonファイルの中身が有効かチェック
        Path jsonPath = Paths.get(modelUri);
        if (!Files.exists(jsonPath)) {
            LOGGER.error("Model URI does not exist: " + modelUri);
            Result errorResult = new Result();
            errorResult.setResult(Byte.valueOf((byte) 0)); // Failure code
            errorResult.setErrorMessage("Model URI does not exist: " + modelUri);
            response.setResult(errorResult);
            return;
        }
        try {
            String content = Files.readString(jsonPath);
            if (!content.contains("\"format_version\"") || !content.contains("\"minecraft:geometry\"")) {
                LOGGER.error("Invalid geometry JSON format: " + modelUri);
                Result errorResult = new Result();
                errorResult.setResult(Byte.valueOf((byte) 0)); // Failure code
                errorResult.setErrorMessage("Invalid geometry JSON format: " + modelUri);
                response.setResult(errorResult);
                return;
            }
        } catch (IOException e) {
            LOGGER.error("Failed to read model URI: " + modelUri, e);
            Result errorResult = new Result();
            errorResult.setResult(Byte.valueOf((byte) 0)); // Failure code
            errorResult.setErrorMessage("Failed to read model URI: " + modelUri);
            response.setResult(errorResult);
            return;
        }

        // jsonファイル名がリストに含まれていない場合は追加
        String jsonFileName = modelUri.substring(modelUri.lastIndexOf('/') + 1, modelUri.lastIndexOf('.'));
        if (!jsonFileNames.contains(jsonFileName)) {
            if (current_model_number >= DynamicModelEntityModel.MAX_MODEL_COUNT) {
                LOGGER.error("Maximum model number reached, overwriting existing models.");
                Result errorResult = new Result();
                errorResult.setResult(Byte.valueOf((byte) 0)); // Failure code
                errorResult.setErrorMessage("Maximum model number reached, overwriting existing models.");
                response.setResult(errorResult);
                return;
            }
            jsonFileNames.add(jsonFileName);
            GeometryApplier.applyJson(
                Paths.get(modelUri), "runtime_geo", current_model_number
            );
            current_model_number++;
        }

        double x = request.getInitialPose().getPose().getPosition().getX();
        double y = request.getInitialPose().getPose().getPosition().getY();
        double z = request.getInitialPose().getPose().getPosition().getZ();

        MinecraftServer server = ServerLifecycleHooks.getCurrentServer();
        ServerLevel world = server.getLevel(Level.OVERWORLD);

        // 3) Entity を生成して座標をセット
        RegistryObject<EntityType<DynamicModelEntity>> ro = ModEntities.CUSTOM_ENTITY;
        EntityType<DynamicModelEntity> type = ro.get();
        DynamicModelEntity robot = type.create(world);
        if (robot == null) {
            LOGGER.error("Failed to create entity of type: " + type);
            Result errorResult = new Result();
            errorResult.setResult(Byte.valueOf((byte) 0)); // Failure code
            errorResult.setErrorMessage("Failed to create entity of type: " + type);
            response.setResult(errorResult);
            return;
        }
        robot.setCustomName(Component.literal(modelName)); // Entityの名前を設定

        spawnedEntities.add(robot);

        // jsonファイル名から番号を取得
        int jsonIndex = jsonFileNames.indexOf(jsonFileName);
        robot.setModelId(jsonIndex);

        // 4) 位置と回転を設定
        robot.moveTo(x, y, z, /* yaw= */ 0.0F, /* pitch= */ 0.0F);

        // 5) ワールドにスポーン
        world.addFreshEntity(robot);

        Result result = new Result();
        byte code = (byte) (robot != null ? 1 : 0);
        result.setResult(Byte.valueOf(code)); // 成功
        response.setResult(result);
        System.out.println("request: " + x + ", " + y + ", " + z);
    }

}