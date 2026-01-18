package org.opentcs.commadapter.rosbridge;

import static java.util.Objects.requireNonNull;

import com.google.inject.assistedinject.Assisted;
import jakarta.annotation.Nonnull;
import jakarta.inject.Inject;
import java.util.concurrent.ScheduledExecutorService;
import org.opentcs.components.kernel.services.TCSObjectService;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Pose;
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
   * The pose manager for monitoring vehicle pose.
   */
  private final PoseManager poseManager;

  /**
   * The object service to fetch model objects.
   */
  private final TCSObjectService objectService;

  /**
   * Creates a new instance.
   *
   * @param configuration This class's configuration.
   * @param mapValueExtractor Extracts values from maps.
   * @param objectService The object service.
   * @param vehicle The vehicle this adapter is associated with.
   * @param kernelExecutor The kernel's executor.
   */
  @Inject
  public ROS2BridgeCommAdapter(
      ROS2BridgeConfiguration configuration,
      MapValueExtractor mapValueExtractor,
      TCSObjectService objectService,
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
    this.objectService = requireNonNull(objectService, "objectService");
    this.poseManager = new PoseManager(kernelExecutor);
  }

  @Override
  public void initialize() {
    if (isInitialized()) {
      return;
    }
    super.initialize();
    // poseManager.initialize();

    // Initialize ROS2 bridge client
    ros2BridgeClient = new ROS2BridgeClient(configuration.rosbridgeUrl());
    ros2BridgeClient.setPositionListener(pos -> {
      getExecutor().execute(() -> {
        LOG.info("Received position update: {}", pos);
        // Only update position directly if we are not executing a command,
        // otherwise let the arrival check handle it to avoid "unexpected position" errors from kernel
        if (getProcessModel().getState() == Vehicle.State.IDLE) {
          getProcessModel().setPosition(pos);
        }
        checkForArrivalByPositionName(pos);
      });
    });

    // Subscribe to pose updates for real-time visualization and order completion
    ros2BridgeClient.setPoseListener((position, orientation) -> {
      getExecutor().execute(() -> {
        getProcessModel().setPose(new Pose(position, orientation));
        poseManager.onPoseUpdate(position, orientation);

        LOG.info(
            "Pose update from rosbridge: x={} y={} z={} orientation={}°",
            position.getX(),
            position.getY(),
            position.getZ(),
            orientation
        );

        MovementCommand curCmd = getSentCommands().peek();
        checkForArrival(position,curCmd);
      });
    });

    ros2BridgeClient.connect();

    // Subscribe to topics
    ros2BridgeClient.subscribe(configuration.poseTopic());
    ros2BridgeClient.subscribe("/vehicle/position", "std_msgs/String");

    // Publish map data to ROS2
    // publishMapToROS2();

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
   * Checks if the vehicle has arrived at the destination of the current command.
   *
   * @param currentPosition The current precise position of the vehicle.
   */
  private void checkForArrival(Triple currentPosition,MovementCommand cmd) {
    LOG.info("检查是否到达目标点, 当前命令: {}", cmd);
    if (cmd == null) {
      return;
    }

    Point destPoint = cmd.getFinalDestination();
    if (destPoint == null) {
      destPoint = cmd.getStep().getDestinationPoint();
    }

    Triple destPosition = destPoint.getPose().getPosition();
    if (destPosition == null) {
      return;
    }

    // Calculate distance (Euclidean)
    double dx = currentPosition.getX() - destPosition.getX();
    double dy = currentPosition.getY() - destPosition.getY();
    // We ignore Z for 2D navigation usually, or include it if necessary
    // double dz = currentPosition.getZ() - destPosition.getZ();

    double distance = Math.sqrt(dx * dx + dy * dy);


    // Threshold for arrival (e.g. 100 mm)
    // This could be configurable
    double threshold = 2000.0;
    LOG.info(
        "Distance to destination {} {}: {}mm (threshold={}mm)",
        destPoint.getName(),
        destPosition,
        distance,
        threshold
    );

    // 当前车辆位置与目标点距离小于阈值，视为到达目标点
    if (distance < threshold) {
      LOG.info("Vehicle arrived at destination (dist={}mm), command executed", distance);
      // getProcessModel().setState(Vehicle.State.IDLE);
      // Remove command from queue
      finishMovementCommand(cmd);
    }
  }

  private void finishMovementCommand(MovementCommand command) {
    // Set the vehicle state to idle if queues are empty
    if (getSentCommands().size() <= 1 && getUnsentCommands().isEmpty()) {
      getProcessModel().setState(Vehicle.State.IDLE);
    }

    if (java.util.Objects.equals(getSentCommands().peek(), command)) {
      LOG.info("Vehicle arrived at destination, command executed: {}", command);
      // Let the comm adapter know we have finished this command.
      getProcessModel().commandExecuted(getSentCommands().poll());

      // Snap to logical position
      Point destPoint = command.getStep().getDestinationPoint();
      if (destPoint == null) {
        destPoint = command.getFinalDestination();
      }
      // The commandExecuted call above should already update the vehicle's position in the kernel.
      // Explicitly setting it again here might cause race conditions or "unexpected position" errors
      // if the state transition isn't fully complete.
      // Only set position if we are in IDLE state (which we might have just set above)
      // if (getProcessModel().getState() == Vehicle.State.IDLE) {
      //    String destName = destPoint.getName();
      //    getProcessModel().setPosition(destName);
      // }
      // Update 2024-05: The above check is not enough. If commandExecuted() is async or takes time,
      // the state might still be EXECUTING here.
      // However, since we just called commandExecuted(), we SHOULD NOT set the position manually at all
      // for the finished step, because the Kernel does that automatically when processing the command completion.
      // Setting it manually here (even if IDLE) risks race conditions where the Kernel sees an "unexpected position" update.
      // So we comment out the setPosition entirely for the command completion case.
      LOG.debug("Command finished, letting Kernel handle position update via commandExecuted()");
    }
    else {
      LOG.warn(
          "{}: Simulated command not oldest in sent queue: {} != {}",
          getName(),
          command,
          getSentCommands().peek()
      );
    }
  }

  /**
   * Checks if the vehicle has arrived at the destination by checking the reported position name.
   *
   * @param positionName The name of the current position reported by the vehicle.
   */
  private void checkForArrivalByPositionName(String positionName) {
    MovementCommand cmd = getSentCommands().peek();
    if (cmd == null) {
      return;
    }

    Point destPoint = cmd.getStep().getDestinationPoint();
    if (destPoint == null) {
      destPoint = cmd.getFinalDestination();
    }
    String destName = destPoint.getName();
    if (destName.equals(positionName)) {
      LOG.info("Vehicle arrived at destination (name={}), command executed", positionName);
      finishMovementCommand(cmd);
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

    poseManager.terminate();

    super.terminate();
    initialized = false;
    LOG.info("ROS2 bridge adapter terminated for vehicle {}", vehicle.getName());
  }

  @Override
  public void sendCommand(MovementCommand cmd) {
    requireNonNull(cmd, "cmd");

    LOG.info("Sending command to ROS2: {}", cmd);

    // Send command to ROS2 via rosbridge
    if (ros2BridgeClient != null && ros2BridgeClient.isConnected()) {
      ros2BridgeClient.sendMovementCommand(cmd, configuration.goalTopic());
      // ros2BridgeClient.sendStringMessage("/vehicle/status", "navigating");
    }

    // Update vehicle state to EXECUTING as we have sent a command
    getProcessModel().setState(Vehicle.State.EXECUTING);

    // Check if we are already at the destination.
    // We do this by checking the current position against the command destination
    if (cmd.getStep().getDestinationPoint() != null) {
      String currentPos = getProcessModel().getPosition();
      String destPos = cmd.getStep().getDestinationPoint().getName();

      // If the step name matches the destination point name, it means we are there.
      // NOTE: This immediate finish logic can cause issues if the vehicle is not actually at the physical location yet.
      // It is safer to rely on position updates from ROS2 to confirm arrival.
      // if (cmd.getStep().getDestinationPoint().getName().equals(cmd.getFinalDestination().getName())
      //    && currentPos != null && currentPos.equals(destPos)) {
      //  finishMovementCommand(cmd);
      //  return;
      // }
      
      // Let the arrival check logic handle the completion based on actual distance/position updates
      if (getProcessModel().getPose().getPosition() != null) {
        checkForArrival(getProcessModel().getPose().getPosition(),cmd);
      }
    }
  }

  @Override
  public void initVehiclePosition(String newPos) {
    LOG.info("Initializing vehicle position to: {}", newPos);
    getProcessModel().setPosition(newPos);

    // Publish initial position to ROS2 with coordinates
    if (ros2BridgeClient != null && ros2BridgeClient.isConnected()) {
      // Get the point's coordinates from the model
      long x = 0;
      long y = 0;
      double orientation = 0.0;

      Point point = objectService.fetch(Point.class, newPos).orElse(null);
      if (point != null && point.getPose().getPosition() != null) {
        Triple position = point.getPose().getPosition();
        x = position.getX();
        y = position.getY();
        orientation = point.getPose().getOrientationAngle();
      }
      else {
        LOG.warn("Could not find point '{}' or it has no position defined. Using default (0,0).", newPos);
        // Fallback: try to get from current vehicle pose if it matches
        if (vehicle.getPose() != null 
            && vehicle.getPose().getPosition() != null 
            && newPos.equals(getProcessModel().getPosition())) {
           Triple position = vehicle.getPose().getPosition();
           x = position.getX();
           y = position.getY();
           orientation = vehicle.getPose().getOrientationAngle();
        }
      }

      // Send initial pose to ROS2 (amcl/initialpose)
      ros2BridgeClient.sendInitialPose(x, y, orientation);

      // Publish vehicle position with coordinates (for visualization)
      // ros2BridgeClient.publishVehiclePositionWithCoords(newPos, x, y, 0);

      LOG.info("Published vehicle position: {} at ({}, {}, {}) mm", newPos, x, y, 0);
    }
  }

  @Override
  public void processMessage(
      @Nonnull
      VehicleCommAdapterMessage message
  ) {
    LOG.info("Processing message: {}", message);

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
    LOG.info("Vehicle paused state changed to: {}", paused);
    // VehicleProcessModel doesn't have setPaused method, use property instead
    getProcessModel().setProperty("paused", Boolean.toString(paused));
  }

  @Override
  public org.opentcs.util.ExplainedBoolean canProcess(org.opentcs.data.order.TransportOrder order) {
    LOG.info("Checking if can process order: {}", order.getName());
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
