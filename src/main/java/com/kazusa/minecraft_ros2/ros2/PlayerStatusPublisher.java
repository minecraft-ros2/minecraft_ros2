package com.kazusa.minecraft_ros2.ros2;

import java.util.ArrayList;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;
import minecraft_msgs.msg.Item;
import minecraft_msgs.msg.PlayerStatus;
import net.minecraft.client.Minecraft;
import net.minecraft.world.effect.MobEffect;
import net.minecraft.world.effect.MobEffectInstance;
import net.minecraft.world.entity.player.Player;
import net.minecraft.world.item.ItemStack;
import net.minecraft.world.InteractionHand;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.Time;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public class PlayerStatusPublisher extends BaseComposableNode {
    private static final Logger LOGGER = LoggerFactory.getLogger(PlayerStatusPublisher.class);
    private final Minecraft minecraft;
    private final Publisher<PlayerStatus> publisher;
    private PlayerStatus message;
    private final int publishRateHz = 10;

    public PlayerStatusPublisher() {
        super("minecraft_player_status_publisher");
        minecraft = Minecraft.getInstance();
        publisher = node.createPublisher(PlayerStatus.class, "/player/status");
        message = new PlayerStatus();
        
        long periodMs = 1000 / publishRateHz;
        node.createWallTimer(periodMs, TimeUnit.MILLISECONDS, this::publishPlayerStatus);
        LOGGER.info("Player status publisher initialized with topic: /player/status at 10Hz");
    }

    public void publishPlayerStatus() {
        CompletableFuture.runAsync(() -> {
            Player player = minecraft.player;
            if (player == null || message == null) {
                return;
            }

            message.setName(player.getName().getString());
            message.setDimension(player.level().dimension().location().toString());
            message.setFoodLevel((byte)player.getFoodData().getFoodLevel());
            message.setScore(player.getScore());
            message.setSleepTimer((byte)player.getSleepTimer());
            message.setXpLevel(player.experienceLevel);
            message.setXpProgress(player.experienceProgress);
            message.setTotalXp(player.totalExperience);
            message.setHealth(player.getHealth());
            message.setMaxHealth(player.getMaxHealth());
            message.setAir((short)player.getAirSupply());
            message.setMaxAir((short)player.getMaxAirSupply());

            ArrayList<String> activeEffects = new ArrayList<>();
            for (MobEffectInstance effect : player.getActiveEffects()) {
                if (effect != null) {
                    MobEffect mobEffect = effect.getEffect().value();
                    if (mobEffect != null) {
                        activeEffects.add(mobEffect.getDescriptionId());
                    } else {
                        activeEffects.add("Unknown Effect");
                    }
                }
            }
            message.setActiveEffects(activeEffects.toArray(new String[0]));
            
            // Inventory items
            ArrayList<Item> inventoryItems = new ArrayList<>();
            for (int i = 0; i < player.getInventory().getContainerSize(); i++) {
                ItemStack stack = player.getInventory().getItem(i);
                if (!stack.isEmpty()) {
                    Item item = new Item();
                    item.setName(stack.getItem().getDescriptionId());
                    item.setCount((byte) stack.getCount());
                    item.setDamage((short) stack.getDamageValue());
                    item.setMaxDamage((short) stack.getMaxDamage());
                    inventoryItems.add(item);
                }
            }
            message.setInventoryItems(inventoryItems);

            ItemStack mainHandStack = player.getItemInHand(InteractionHand.MAIN_HAND);
            if (!mainHandStack.isEmpty()) {
                Item mainHandItem = new Item();
                mainHandItem.setName(mainHandStack.getItem().getDescriptionId());
                mainHandItem.setCount((byte) mainHandStack.getCount());
                mainHandItem.setDamage((short) mainHandStack.getDamageValue());
                mainHandItem.setMaxDamage((short) mainHandStack.getMaxDamage());
                message.setMainHandItem(mainHandItem);
            } else {
                message.setMainHandItem(null);
            }

            ItemStack offHandStack = player.getItemInHand(InteractionHand.OFF_HAND);
            if (!offHandStack.isEmpty()) {
                Item offHandItem = new Item();
                offHandItem.setName(offHandStack.getItem().getDescriptionId());
                offHandItem.setCount((byte) offHandStack.getCount());
                offHandItem.setDamage((short) offHandStack.getDamageValue());
                offHandItem.setMaxDamage((short) offHandStack.getMaxDamage());
                message.setOffHandItem(offHandItem);
            } else {
                message.setOffHandItem(null);
            }

            try {
                publisher.publish(message);
            } catch (Exception e) {
                LOGGER.error("Error publishing player status", e);
            }
        });
    }
}

