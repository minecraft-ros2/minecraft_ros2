package com.kazusa.ros2mc.ros2;

import com.kazusa.ros2mc.config.Config;
import org.ros2.rcljava.RCLJava;
import net.minecraft.client.Minecraft;
import net.minecraft.world.entity.player.Player;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.subscription.Subscription;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import geometry_msgs.msg.Twist;

public class TwistSubscriber extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(TwistSubscriber.class);
    private Subscription<Twist> subscription;
    private double lastLinearX = 0.0;
    private double lastLinearY = 0.0;
    private double lastLinearZ = 0.0;
    private double lastAngularY = 0.0;
    private double lastAngularZ = 0.0;

    public TwistSubscriber() {
        super("minecraft_twist_subscriber");
        subscription = this.node.<Twist>createSubscription(
            Twist.class, "/cmd_vel", this::twistCallback);
        LOGGER.info("TwistSubscriber initialized and listening on 'cmd_vel' topic");
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
     * Apply the stored twist values to move the player
     * Called from the game tick event to ensure movement happens on the client/main thread
     */
    public void applyPlayerMovement() {
        Minecraft minecraft = Minecraft.getInstance();
        Player player = minecraft.player;
        
        if (player != null && !minecraft.isPaused()) {
            // Scale the values based on config
            double maxSpeed = Config.COMMON.maxSpeed.get();
            double speedFactor = maxSpeed / 20.0; // Convert to blocks per tick
            
            // Apply linear movement (forward/backward and strafe)
            if (Math.abs(lastLinearX) > 0.1 || Math.abs(lastLinearY) > 0.1 || Math.abs(lastLinearZ) > 0.1) {
                // Convert ROS coordinate system to Minecraft
                // In ROS, X is forward, Y is left, Z is up
                // Apply the movement directly to the player's motion
                double forwardMovement = lastLinearX * speedFactor;
                double sidewaysMovement = -lastLinearY * speedFactor; // Negative because ROS Y is left
                double verticalMovement = lastLinearZ > 0.1 ? 1.0 : 0.0; // Jump if linear_z > 0.1
                
                // Get player's current look direction for movement relative to where they're facing
                float yaw = player.getYRot();
                double yawRadians = Math.toRadians(yaw);
                
                // Calculate the components of movement based on player's orientation
                double movX = -Math.sin(yawRadians) * forwardMovement - Math.cos(yawRadians) * sidewaysMovement;
                double movZ = Math.cos(yawRadians) * forwardMovement - Math.sin(yawRadians) * sidewaysMovement;
                
                // Set the player's position
                player.setPos(player.getX() + movX, player.getY() + verticalMovement, player.getZ() + movZ);
            }
            
            // Apply rotation (angular movement)
            if (Math.abs(lastAngularZ) > 0.1 || Math.abs(lastAngularY) > 0.1) {
                // Rotate the player (negative because ROS angular Z is counterclockwise)
                float rotationAmountZ = (float)(-lastAngularZ * speedFactor * 10); // Adjust sensitivity
                float rotationAmountY = (float)(lastAngularY * speedFactor * 10); // Adjust sensitivity for vertical look
                player.setYRot(player.getYRot() + rotationAmountZ);
                player.setXRot(player.getXRot() + rotationAmountY);
            }
        }
    }
}