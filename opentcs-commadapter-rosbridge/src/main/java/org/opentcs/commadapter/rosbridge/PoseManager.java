package org.opentcs.commadapter.rosbridge;

import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;
import org.opentcs.data.model.Triple;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Manages and monitors vehicle pose data.
 */
public class PoseManager {
  private static final Logger LOG = LoggerFactory.getLogger(PoseManager.class);

  private final Map<String, StampedPose> poseHistory;
  private final List<Consumer<StampedPose>> poseListeners;
  private final ScheduledExecutorService scheduler;

  // Current robot pose (thread-safe)
  private StampedPose currentPose;
  private final Object poseLock = new Object();

  private ScheduledFuture<?> monitorTask;
  private ScheduledFuture<?> cleanupTask;

  /**
   * Creates a new instance.
   *
   * @param scheduler The executor service to schedule monitoring tasks on.
   */
  public PoseManager(ScheduledExecutorService scheduler) {
    this.scheduler = scheduler;
    this.poseHistory = new ConcurrentHashMap<>();
    this.poseListeners = new CopyOnWriteArrayList<>();
  }

  /**
   * Initializes periodic monitoring/cleanup tasks.
   */
  public void initialize() {
    if (monitorTask != null && !monitorTask.isCancelled()) {
      return;
    }

    // Start pose health monitoring task
    monitorTask = scheduler.scheduleAtFixedRate(
        this::monitorPoseHealth,
        5, 5, TimeUnit.SECONDS
    );

    // Start history cleanup task
    cleanupTask = scheduler.scheduleAtFixedRate(
        this::cleanupOldPoses,
        30, 30, TimeUnit.MINUTES
    );

    LOG.info("PoseManager initialized");
  }

  /**
   * Updates the manager with a new pose sample.
   *
   * @param position The current position.
   * @param orientation The current orientation in degrees.
   */
  public void onPoseUpdate(Triple position, double orientation) {
    StampedPose pose = new StampedPose(position, orientation, System.currentTimeMillis());
    updateCurrentPose(pose);
    notifyPoseListeners(pose);
    storePoseHistory(pose);
  }

  private void updateCurrentPose(StampedPose pose) {
    synchronized (poseLock) {
      this.currentPose = pose;
    }
  }

  /**
   * Returns the current pose sample if available.
   *
   * @return The current pose sample, or {@code null} if none has been received.
   */
  public StampedPose getCurrentPose() {
    synchronized (poseLock) {
      return currentPose;
    }
  }

  private void storePoseHistory(StampedPose pose) {
    String key = "pose_" + pose.timestamp;
    poseHistory.put(key, pose);
  }

  private void cleanupOldPoses() {
    long cutoffTime = System.currentTimeMillis() - (30 * 60 * 1000); // 30 minutes ago
    poseHistory.entrySet().removeIf(entry -> entry.getValue().timestamp < cutoffTime);
    LOG.info("Cleaned up old poses, remaining: {}", poseHistory.size());
  }

  private void monitorPoseHealth() {
    synchronized (poseLock) {
      if (currentPose == null) {
        LOG.warn("⚠️ Warning: No pose data received yet");
        return;
      }

      long timeSinceLastUpdate = System.currentTimeMillis() - currentPose.timestamp;
      if (timeSinceLastUpdate > 5000) { // No data for 5 seconds
        LOG.warn(
            "⚠️ Warning: Pose data might have stopped updating (last update: {} ms ago)",
            timeSinceLastUpdate
        );
      }

      // Check if pose is valid
      if (Double.isNaN(currentPose.position.getX())
          || Double.isNaN(currentPose.position.getY())) {
        LOG.warn("⚠️ Warning: Pose data contains NaN values");
      }
    }
  }

  /**
   * Registers a listener for pose updates.
   *
   * @param listener The listener to register.
   */
  public void addPoseListener(Consumer<StampedPose> listener) {
    poseListeners.add(listener);
  }

  private void notifyPoseListeners(StampedPose pose) {
    for (Consumer<StampedPose> listener : poseListeners) {
      listener.accept(pose);
    }
  }

  /**
   * Cancels all scheduled tasks.
   */
  public void terminate() {
    if (monitorTask != null) {
      monitorTask.cancel(false);
      monitorTask = null;
    }
    if (cleanupTask != null) {
      cleanupTask.cancel(false);
      cleanupTask = null;
    }
    LOG.info("PoseManager terminated");
  }

  /**
   * A pose sample with an associated timestamp.
   */
  public static class StampedPose {
    private final Triple position;
    private final double orientation;
    private final long timestamp;

    /**
     * Creates a new instance.
     *
     * @param position The position.
     * @param orientation The orientation in degrees.
     * @param timestamp The timestamp in milliseconds.
     */
    public StampedPose(Triple position, double orientation, long timestamp) {
      this.position = position;
      this.orientation = orientation;
      this.timestamp = timestamp;
    }

    /**
     * Returns the position.
     *
     * @return The position.
     */
    public Triple getPosition() {
      return position;
    }

    /**
     * Returns the orientation in degrees.
     *
     * @return The orientation in degrees.
     */
    public double getOrientation() {
      return orientation;
    }

    /**
     * Returns the timestamp in milliseconds.
     *
     * @return The timestamp in milliseconds.
     */
    public long getTimestamp() {
      return timestamp;
    }

    @Override
    public String toString() {
      return String.format(
          "StampedPose[pos=%s, orient=%.1f, ts=%d]",
          position,
          orientation,
          timestamp
      );
    }
  }
}
