package com.kazusa.minecraft_ros2.ros2;

import com.kazusa.minecraft_ros2.config.Config;
import net.minecraft.client.Minecraft;
import net.minecraft.world.entity.Entity;
import org.ros2.rcljava.node.BaseComposableNode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import geometry_msgs.msg.Twist;

public class RobotTwistSubscriber extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(TwistSubscriber.class);

    private Minecraft minecraft;
    private Entity entity;

    private double lastLinearX = 0.0;
    private double lastLinearY = 0.0;
    private double lastLinearZ = 0.0;
    private double lastAngularY = 0.0;
    private double lastAngularZ = 0.0;

    public RobotTwistSubscriber(Entity inputEntity, String namespace) {
        super("minecraft_robot_twist_subscriber");

        entity = inputEntity;

        String topicName = "cmd_vel";
        if (namespace != null && !namespace.isEmpty()) {
            topicName = "/" + namespace + "/" + topicName;
        }
        this.node.<Twist>createSubscription(
            Twist.class, topicName, this::twistCallback);
        LOGGER.info("RobotTwistSubscriber initialized and listening on 'cmd_vel' topic");
    }

    private void twistCallback(final Twist msg) {
        // Store the received twist values
        lastLinearX = msg.getLinear().getX();
        lastLinearY = msg.getLinear().getY();
        lastLinearZ = msg.getLinear().getZ();
        lastAngularY = msg.getAngular().getY();
        lastAngularZ = msg.getAngular().getZ();
        
        LOGGER.debug("Received twist: linear_x={}, linear_y={}, linear_z={}, angular_y={}, angular_z={}", 
                lastLinearX, lastLinearY, lastLinearZ, lastAngularY, lastAngularZ);
    }
    
    /**
     * Apply the stored twist values to move the entity
     * Called from the game tick event to ensure movement happens on the client/main thread
     */
    public void applyEntityMovement() {
        minecraft = Minecraft.getInstance();

        if (entity != null && !minecraft.isPaused()) {
            double maxSpeed = Config.COMMON.maxSpeed.get();
            double speedFactor = maxSpeed / 20.0;

            if (Math.abs(lastLinearX) > 0.1 || Math.abs(lastLinearY) > 0.1 || Math.abs(lastLinearZ) > 0.1) {
                float yaw = entity.getYRot();
                double yawRad = Math.toRadians(yaw);

                double forward = lastLinearX * speedFactor;
                double strafe = -lastLinearY * speedFactor;

                double dx = -Math.sin(yawRad) * forward - Math.cos(yawRad) * strafe;
                double dz = Math.cos(yawRad) * forward - Math.sin(yawRad) * strafe;

                double dy = entity.getDeltaMovement().y(); // preserve current vertical motion

                // ジャンプ処理（ジャンプ中でなければジャンプ）
                // ジャンプ処理（ジャンプ中でなければジャンプ）
                if (lastLinearZ > 0.1 && entity.verticalCollision) {
                    dy = 0.42; // Minecraft のジャンプ速度
                }


                // deltaMovementを使って自然な移動にする
                entity.setDeltaMovement(dx, dy, dz);
                entity.hasImpulse = true; // deltaMovementを使う場合にはtrueにする
            }
            else if (Math.abs(lastAngularZ) > 0.1 || Math.abs(lastAngularY) > 0.1) {
                // “ダミー” の水平速度をつける（向き更新のトリガーにするため）
                double dy = entity.getDeltaMovement().y; // 垂直速度はそのまま
                double dummyV = 0.01;
                float yaw = entity.getYRot();
                double yawRad = Math.toRadians(yaw);
                double dx = -Math.sin(yawRad) * dummyV;
                double dz =  Math.cos(yawRad) * dummyV;
                entity.setDeltaMovement(dx, dy, dz);
                entity.hasImpulse = true;
            }

            // 回転処理（なめらかにする）
            if (Math.abs(lastAngularZ) > 0.1 || Math.abs(lastAngularY) > 0.1) {
                float rotZ = (float)(-lastAngularZ * speedFactor * 10);
                float rotY = (float)(lastAngularY * speedFactor * 10);
                entity.setYRot(entity.getYRot() + rotZ);
                entity.setXRot(entity.getXRot() + rotY);
            }
        }
    }
}