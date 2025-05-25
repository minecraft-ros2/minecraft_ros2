package com.kazusa.minecraft_ros2.graphics;

import com.google.common.collect.ImmutableList;
import net.minecraft.client.model.HumanoidModel;
import net.minecraft.client.model.geom.ModelPart;
import net.minecraft.client.model.geom.PartPose;
import net.minecraft.client.model.geom.builders.CubeDeformation;
import net.minecraft.client.model.geom.builders.CubeListBuilder;
import net.minecraft.client.model.geom.builders.LayerDefinition;
import net.minecraft.world.entity.LivingEntity;
import org.jetbrains.annotations.NotNull;

public class LidarModel extends HumanoidModel<LivingEntity> {
    private static final String LIDAR = "lidar";

    private final ModelPart lidar;

    public LidarModel(ModelPart part) {
        super(part);
        this.lidar = part.getChild("head").getChild(LIDAR);
    }

    @Override
    protected @NotNull Iterable<ModelPart> headParts() {
        this.lidar.copyFrom(this.head);
        this.lidar.x += 1.0F; // Adjust the X position of the lidar
        return ImmutableList.of();
    }

    @Override
    protected @NotNull Iterable<ModelPart> bodyParts() {
        ImmutableList.Builder<ModelPart> parts = ImmutableList.builder();

        parts.add(
                this.body,
                this.lidar,
                this.leftArm,
                this.rightArm
        );


        return parts.build();
    }

    public static LayerDefinition createBodyLayer() {
        var mesh = HumanoidModel.createMesh(new CubeDeformation(1.5F), 0F);
        var root = mesh.getRoot().getChild("head");

        root.addOrReplaceChild(LIDAR, CubeListBuilder.create()
                        .texOffs(36, 48).addBox(4.0F, -10.0F, -3.0F, 1.0F, 10.0F, 2.0F, new CubeDeformation(0.0F))
                        .texOffs(0, 39).addBox(5.0F, -10.0F, -1.0F, 1.0F, 10.0F, 4.0F, new CubeDeformation(0.0F))
                        .texOffs(22, 37).addBox(4.0F, -10.0F, -1.0F, 1.0F, 10.0F, 6.0F, new CubeDeformation(0.0F))
                        .texOffs(42, 48).addBox(1.0F, -10.0F, -5.0F, 2.0F, 10.0F, 1.0F, new CubeDeformation(0.0F))
                        .texOffs(10, 39).addBox(-3.0F, -10.0F, -6.0F, 4.0F, 10.0F, 2.0F, new CubeDeformation(0.0F))
                        .texOffs(48, 48).addBox(-5.0F, -10.0F, -5.0F, 2.0F, 10.0F, 1.0F, new CubeDeformation(0.0F))
                        .texOffs(36, 0).addBox(-5.0F, -10.0F, 5.0F, 9.0F, 10.0F, 1.0F, new CubeDeformation(0.0F))
                        .texOffs(36, 37).addBox(-5.0F, -10.0F, 6.0F, 8.0F, 10.0F, 1.0F, new CubeDeformation(0.0F))
                        .texOffs(0, 19).addBox(-6.0F, -10.0F, -4.0F, 1.0F, 10.0F, 10.0F, new CubeDeformation(0.0F))
                        .texOffs(40, 25).addBox(-3.0F, -10.0F, 7.0F, 4.0F, 10.0F, 1.0F, new CubeDeformation(0.0F))
                        .texOffs(0, 0).addBox(-5.0F, -10.0F, -4.0F, 9.0F, 10.0F, 9.0F, new CubeDeformation(0.0F))
                        .texOffs(22, 19).addBox(-7.0F, -10.0F, -3.0F, 1.0F, 10.0F, 8.0F, new CubeDeformation(0.0F))
                        .texOffs(40, 11).addBox(-8.0F, -10.0F, -1.0F, 1.0F, 10.0F, 4.0F, new CubeDeformation(0.0F)),
                PartPose.offset(0.0F, 0.0F, 0.0F));

        return LayerDefinition.create(mesh, 64, 64);
    }
}