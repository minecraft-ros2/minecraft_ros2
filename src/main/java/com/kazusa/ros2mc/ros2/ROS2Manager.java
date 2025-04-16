package com.kazusa.ros2mc.ros2;

import org.ros2.rcljava.RCLJava;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.lang.reflect.Field;
import java.util.Arrays;

/**
 * Manages ROS2 initialization, execution, and shutdown
 */
public class ROS2Manager {
    private static final Logger LOGGER = LoggerFactory.getLogger(ROS2Manager.class);
    private static ROS2Manager instance;
    
    private final AtomicBoolean initialized = new AtomicBoolean(false);
    private ExecutorService executorService;
    private TwistSubscriber twistSubscriber;
    
    private ROS2Manager() {
        // Private constructor for singleton
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
                            Thread.sleep(10); // Don't hog CPU
                        }
                    } catch (InterruptedException e) {
                        LOGGER.info("ROS2 spin thread interrupted");
                        Thread.currentThread().interrupt();
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
        }
    }
    
    /**
     * Apply player movement based on the latest twist message
     * Called every game tick
     */
    public void processTwistMessages() {
        if (initialized.get() && twistSubscriber != null) {
            twistSubscriber.applyPlayerMovement();
        }
    }
    
    /**
     * Check if ROS2 is currently initialized
     */
    public boolean isInitialized() {
        return initialized.get();
    }

}