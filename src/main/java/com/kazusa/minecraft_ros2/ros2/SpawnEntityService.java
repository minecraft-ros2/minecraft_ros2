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
import net.minecraft.world.entity.Entity;
import net.minecraft.world.entity.EntityType;
import net.minecraft.resources.ResourceLocation;
import net.minecraftforge.registries.RegistryObject;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.ArrayList;

public class SpawnEntityService  extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(SpawnEntityService.class);

    private final Service<SpawnEntity> service;

    // モデルの名前の空配列
    private final List<String> jsonFileNames;

    private int current_model_number;

    public SpawnEntityService() {
        super("add_two_ints_service");
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
            return;
        }

        // jsonファイル名がリストに含まれていない場合は追加
        String jsonFileName = modelUri.substring(modelUri.lastIndexOf('/') + 1, modelUri.lastIndexOf('.'));
        if (!jsonFileNames.contains(jsonFileName)) {
            if (current_model_number >= 10) {
                LOGGER.error("Maximum model number reached, overwriting existing models.");
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