package com.kazusa.minecraft_ros2.ros2;

import net.minecraft.client.Minecraft;
import net.minecraft.world.entity.Entity;
import org.ros2.rcljava.Time;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.publisher.Publisher;
import sensor_msgs.msg.Imu;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;


public class IMUPublisher extends BaseComposableNode  {
    private final Minecraft minecraft;
    private final Publisher<Imu> publisher;
    private Imu message;

    private double lastVx = 0;
    private double lastVy = 0;
    private double lastVz = 0;
    private double lastYaw = 0;

    private long delta_time = 10;
    double dt;

    public IMUPublisher(){
        super("minecraft_imu_publisher");
        minecraft = Minecraft.getInstance();
        publisher = node.createPublisher(Imu.class, "/player/imu");
        message = new Imu();
        dt = 1000.0 / (double)delta_time;
        node.createWallTimer(delta_time, TimeUnit.MILLISECONDS, this::publishImuData);

    }

    public void publishImuData(){
         CompletableFuture.runAsync(() -> {
            Entity player = minecraft.player;
            if (player == null) return;
            if (message == null) return;
            var dv = player.getDeltaMovement();

            double vx = dv.x * 20;
            double vy = dv.y * 20;
            double vz = dv.z * 20;

            double ax = (vx - lastVx) / dt;
            double ay = (vy - lastVy) / dt;
            double az = (vz - lastVz) / dt;
            
            lastVx = vx;
            lastVy = vy;
            lastVz = vz;

            double currentYaw = Math.toRadians(player.getYRot());
            double yawVel = (currentYaw - lastYaw) / dt;  
            lastYaw = currentYaw;

            double accX = ax * Math.sin(currentYaw) + az * Math.cos(currentYaw);
            double accY = ax * Math.cos(currentYaw) - az * Math.sin(currentYaw);
            double accZ = ay;

            message.getHeader().setStamp(Time.now());
            message.getHeader().setFrameId("player");
            message.getLinearAcceleration().setX(accX);
            message.getLinearAcceleration().setY(accY);
            message.getLinearAcceleration().setZ(accZ);
            message.getAngularVelocity().setX(0.0);
            message.getAngularVelocity().setY(0.0);
            message.getAngularVelocity().setZ(yawVel);

            publisher.publish(message);
         });
    }
}
