package org.opentcs.commadapter.rosbridge;

import static java.util.Objects.requireNonNull;

import com.google.inject.assistedinject.Assisted;
import jakarta.annotation.Nonnull;
import jakarta.inject.Inject;
import java.util.concurrent.ScheduledExecutorService;
import org.opentcs.data.model.Triple;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.BasicVehicleCommAdapter;
import org.opentcs.drivers.vehicle.MovementCommand;
import org.opentcs.drivers.vehicle.SimVehicleCommAdapter;
import org.opentcs.drivers.vehicle.VehicleCommAdapterMessage;
import org.opentcs.util.MapValueExtractor;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * A {@link org.opentcs.drivers.vehicle.VehicleCommAdapter} that communicates with ROS2 via
 * rosbridge.
 */
public class ROS2BridgeCommAdapter
    extends
      BasicVehicleCommAdapter
    implements
      SimVehicleCommAdapter {

  /**
   * This class's Logger.
   */
  private static final Logger LOG = LoggerFactory.getLogger(ROS2BridgeCommAdapter.class);
  /**
   * This instance's configuration.
   */
  private final ROS2BridgeConfiguration configuration;
  /**
   * Extracts values from maps.
   */
  private final MapValueExtractor mapValueExtractor;
  /**
   * The vehicle to this comm adapter instance.
   */
  private final Vehicle vehicle;
  /**
   * Indicates whether the adapter is initialized or not.
   */
  private volatile boolean initialized;
  /**
   * The ROS2 bridge client.
   */
  private ROS2BridgeClient ros2BridgeClient;

  /**
   * Creates a new instance.
   *
   * @param configuration This class's configuration.
   * @param mapValueExtractor Extracts values from maps.
   * @param vehicle The vehicle this adapter is associated with.
   * @param kernelExecutor The kernel's executor.
   */
  @Inject
  public ROS2BridgeCommAdapter(
      ROS2BridgeConfiguration configuration,
      MapValueExtractor mapValueExtractor,
      @Assisted
      Vehicle vehicle,
      @org.opentcs.customizations.kernel.KernelExecutor
      ScheduledExecutorService kernelExecutor
  ) {
    super(
        new ROS2BridgeVehicleModel(vehicle),
        configuration.commandQueueCapacity(),
        configuration.rechargeOperation(),
        kernelExecutor
    );
    this.vehicle = requireNonNull(vehicle, "vehicle");
    this.configuration = requireNonNull(configuration, "configuration");
    this.mapValueExtractor = requireNonNull(mapValueExtractor, "mapValueExtractor");
  }

  @Override
  public void initialize() {
    if (isInitialized()) {
      return;
    }
    super.initialize();

    // Initialize ROS2 bridge client
    ros2BridgeClient = new ROS2BridgeClient(configuration.rosbridgeUrl());
    ros2BridgeClient.connect();

    // Publish map data to ROS2
    publishMapToROS2();

    // Initialize vehicle position if specified
    String initialPos = vehicle.getProperties().get(ROS2BridgeConstants.PROPKEY_INITIAL_POSITION);
    if (initialPos != null) {
      initVehiclePosition(initialPos);
    }

    getProcessModel().setState(Vehicle.State.IDLE);
    initialized = true;
    LOG.info("ROS2 bridge adapter initialized for vehicle {}", vehicle.getName());
  }

  /**
   * Publishes the entire map to ROS2.
   */
  private void publishMapToROS2() {
    try {
      LOG.info("Publishing map data to ROS2...");

      // In a real implementation, we would get the plant model from the object service
      // For now, we'll create a simple test map with the vehicle's current position

      // Publish the vehicle's current position with coordinates
      if (vehicle.getPose() != null && vehicle.getPose().getPosition() != null) {
        Triple position = vehicle.getPose().getPosition();
        String positionName = vehicle.getCurrentPosition() != null ? vehicle.getCurrentPosition().getName() : "unknown";
        ros2BridgeClient.publishVehiclePositionWithCoords(
            positionName,
            position.getX(),
            position.getY(),
            position.getZ()
        );
      }

      LOG.info("Map data publishing completed");
    }
    catch (Exception e) {
      LOG.error("Failed to publish map data: {}", e.getMessage(), e);
    }
  }

  @Override
  public boolean isInitialized() {
    return initialized;
  }

  @Override
  public void terminate() {
    if (!isInitialized()) {
      return;
    }

    // Disconnect ROS2 bridge client
    if (ros2BridgeClient != null) {
      ros2BridgeClient.disconnect();
      ros2BridgeClient = null;
    }

    super.terminate();
    initialized = false;
    LOG.info("ROS2 bridge adapter terminated for vehicle {}", vehicle.getName());
  }

  @Override
  public void sendCommand(MovementCommand cmd) {
    requireNonNull(cmd, "cmd");

    LOG.debug("Sending command to ROS2: {}", cmd);

    // Send command to ROS2 via rosbridge
    if (ros2BridgeClient != null && ros2BridgeClient.isConnected()) {
      ros2BridgeClient.sendMovementCommand(cmd);

      // Send additional status message
      ros2BridgeClient.sendStringMessage("/vehicle/status", "executing");
    }

    // Update vehicle state
    getProcessModel().setState(Vehicle.State.IDLE);
  }

  @Override
  public void initVehiclePosition(String newPos) {
    LOG.debug("Initializing vehicle position to: {}", newPos);
    getProcessModel().setPosition(newPos);

    // Publish initial position to ROS2 with coordinates
    if (ros2BridgeClient != null && ros2BridgeClient.isConnected()) {
      // Get the point's coordinates from the vehicle's pose if available
      long x = 0;
      long y = 0;
      long z = 0;

      if (vehicle.getPose() != null && vehicle.getPose().getPosition() != null) {
        Triple position = vehicle.getPose().getPosition();
        x = position.getX();
        y = position.getY();
        z = position.getZ();
      }

      // Publish vehicle position with coordinates
      ros2BridgeClient.publishVehiclePositionWithCoords(newPos, x, y, z);

      LOG.info("Published vehicle position: {} at ({}, {}, {}) mm", newPos, x, y, z);
    }
  }

  @Override
  public void processMessage(
      @Nonnull
      VehicleCommAdapterMessage message
  ) {
    LOG.debug("Processing message: {}", message);

    // Handle INIT_POSITION message from kernelcontrolcenter
    if ("tcs:virtualVehicle:initPosition".equals(message.getType())) {
      String newPos = message.getParameters().get("position");
      if (newPos != null) {
        initVehiclePosition(newPos);
      }
    }
  }

  @Override
  public void onVehiclePaused(boolean paused) {
    LOG.debug("Vehicle paused state changed to: {}", paused);
    // VehicleProcessModel doesn't have setPaused method, use property instead
    getProcessModel().setProperty("paused", Boolean.toString(paused));
  }

  @Override
  public org.opentcs.util.ExplainedBoolean canProcess(org.opentcs.data.order.TransportOrder order) {
    LOG.debug("Checking if can process order: {}", order.getName());
    // For now, always return true
    return new org.opentcs.util.ExplainedBoolean(true, "");
  }

  @Override
  protected void connectVehicle() {
    // Connect to ROS2 bridge
    if (ros2BridgeClient != null) {
      ros2BridgeClient.connect();
    }
  }

  @Override
  protected void disconnectVehicle() {
    // Disconnect from ROS2 bridge
    if (ros2BridgeClient != null) {
      ros2BridgeClient.disconnect();
    }
  }

  @Override
  protected boolean isVehicleConnected() {
    // Check if connected to ROS2 bridge
    return ros2BridgeClient != null && ros2BridgeClient.isConnected();
  }

  @Override
  public ROS2BridgeVehicleModel getProcessModel() {
    return (ROS2BridgeVehicleModel) super.getProcessModel();
  }
}
