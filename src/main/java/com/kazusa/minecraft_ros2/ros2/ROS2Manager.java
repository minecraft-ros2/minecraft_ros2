package com.kazusa.minecraft_ros2.ros2;

import com.kazusa.minecraft_ros2.models.DynamicModelEntity;
import com.kazusa.minecraft_ros2.config.Config;
import com.kazusa.minecraft_ros2.block.RedstonePubSubBlock;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.block.Block;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.client.Minecraft;
import org.ros2.rcljava.RCLJava;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.Set;

/**
 * Manages ROS2 initialization, execution, and shutdown
 */
public final class ROS2Manager {
    private static final Logger LOGGER = LoggerFactory.getLogger(ROS2Manager.class);
    private static ROS2Manager instance;
    
    private final AtomicBoolean initialized = new AtomicBoolean(false);
    private ExecutorService executorService;
    private TwistSubscriber twistSubscriber;
    private CommandSubscriber commandSubscriber;
    private ImagePublisher imagePublisher;
    private PointCloudPublisher pointCloudPublisher;
    private IMUPublisher imuPublisher;
    private GroundTruthPublisher groundTruthPublisher;
    private LivingEntitiesPublisher livingEntitiesPublisher;
    private PlayerStatusPublisher playerStatusPublisher;

    private SpawnEntityService spawnEntityService;
    private DigBlockService digBlockService;
    
    private ROS2Manager() {
        // Private constructor for singleton
    }

    public ImagePublisher getImagePublisher() {
        return imagePublisher;
    }

    
    /**
     * Get the singleton instance of the ROS2Manager
     */
    public static ROS2Manager getInstance() {
        if (instance == null) {
            synchronized (ROS2Manager.class) {
                if (instance == null) {
                    instance = new ROS2Manager();
                }
            }
        }
        return instance;
    }
    
    /**
     * Initialize ROS2 system
     */
    public void initialize() {
        if (!initialized.getAndSet(true)) {
            try {
                LOGGER.info("Initializing ROS2...");

                RCLJava.rclJavaInit();
                
                // Create subscriber
                twistSubscriber = new TwistSubscriber();
                commandSubscriber = new CommandSubscriber();
                imagePublisher = new ImagePublisher();
                pointCloudPublisher = new PointCloudPublisher();
                imuPublisher = new IMUPublisher();
                groundTruthPublisher = new GroundTruthPublisher();

                if (Config.COMMON.enableDebugDataStreaming.get()) {
                    LOGGER.info("Debug data stream enabled");
                    livingEntitiesPublisher = new LivingEntitiesPublisher();
                    playerStatusPublisher = new PlayerStatusPublisher();
                } else {
                    LOGGER.info("Debug data stream disabled");
                }


                spawnEntityService = new SpawnEntityService();
                digBlockService = new DigBlockService();
                
                // Create and start executor thread for ROS2 spin
                executorService = Executors.newSingleThreadExecutor(r -> {
                    Thread t = new Thread(r, "ROS2-Executor");
                    t.setDaemon(true); // Make it a daemon thread so it doesn't prevent game exit
                    return t;
                });
                
                executorService.submit(() -> {
                    LOGGER.info("ROS2 spin thread started");
                    try {
                        while (!Thread.currentThread().isInterrupted() && RCLJava.ok()) {
                            RCLJava.spinSome(twistSubscriber);
                            RCLJava.spinSome(commandSubscriber);
                            RCLJava.spinSome(imagePublisher);
                            RCLJava.spinSome(pointCloudPublisher);
                            RCLJava.spinSome(imuPublisher);
                            RCLJava.spinSome(groundTruthPublisher);

                            RCLJava.spinSome(spawnEntityService);
                            RCLJava.spinSome(digBlockService);

                            Level world = Minecraft.getInstance().level;
                            if (world != null) {
                                Set<BlockPos> all = RedstonePubSubBlock.getAllInstances();
                                for (BlockPos pos : all) {
                                    BlockState state = world.getBlockState(pos);
                                    Block block = state.getBlock();
                                    if (block instanceof RedstonePubSubBlock redstoneBlock) {
                                        BlockIntPublisher publisher = redstoneBlock.getPublisher();
                                        if (publisher != null) {
                                            RCLJava.spinSome(publisher);
                                        }
                                        BlockBoolSubscriber subscriber = redstoneBlock.getSubscriber();
                                        if (subscriber != null) {
                                            RCLJava.spinSome(subscriber);
                                        }
                                    }
                                }
                            }

                            if (spawnEntityService.spawnedEntities != null) {
                                for (DynamicModelEntity entity : spawnEntityService.spawnedEntities) {
                                    if (entity.getRobotTwistSubscriber() != null) {
                                        RCLJava.spinSome(entity.getRobotTwistSubscriber());
                                    }
                                }
                            }
                            //captureAndPublishImage
                            
                            if (livingEntitiesPublisher != null) {
                                RCLJava.spinSome(livingEntitiesPublisher);
                            }
                            if (playerStatusPublisher != null) {
                                RCLJava.spinSome(playerStatusPublisher);
                            }
                            try {
                                Thread.sleep(5); // Don't hog CPU
                            } catch (InterruptedException e) {
                                LOGGER.info("ROS2 spin thread interrupted");
                                Thread.currentThread().interrupt();
                            }
                        }
                    } catch (Exception e) {
                        LOGGER.error("Error in ROS2 spin thread", e);
                    }
                    LOGGER.info("ROS2 spin thread exiting");
                });
                
                LOGGER.info("ROS2 initialized successfully");
            } catch (Exception e) {
                LOGGER.error("Failed to initialize ROS2", e);
                shutdown();
            }
        }
    }
    
    /**
     * Shutdown ROS2 system
     */
    public void shutdown() {
        if (initialized.getAndSet(false)) {
            LOGGER.info("Shutting down ROS2...");
            
            if (executorService != null) {
                executorService.shutdownNow();
                executorService = null;
            }
            
            try {
                RCLJava.shutdown();
                LOGGER.info("ROS2 shutdown complete");
            } catch (Exception e) {
                LOGGER.error("Error during ROS2 shutdown", e);
            }
            
            twistSubscriber = null;
            commandSubscriber = null;
        }
    }
    
    /**
     * Apply player movement based on the latest twist message
     * Called every game tick
     */
    public void processTwistMessages() {
        if (!initialized.get()) {
            return; // ROS2 not initialized, skip processing
        }
        if (twistSubscriber != null) {
            twistSubscriber.applyPlayerMovement();
        }
        if (spawnEntityService.spawnedEntities != null) {
            for (DynamicModelEntity entity : spawnEntityService.spawnedEntities) {
                if (entity.getRobotTwistSubscriber() != null) {
                    entity.getRobotTwistSubscriber().applyEntityMovement();
                }
            }
        }
    }
    
    /**
     * Check if ROS2 is currently initialized
     */
    public boolean isInitialized() {
        return initialized.get();
    }

}