package org.opentcs.commadapter.rosbridge;

import java.net.URI;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;
import org.opentcs.data.model.PlantModel;
import org.opentcs.data.model.Triple;
import org.opentcs.data.order.Route.Step;
import org.opentcs.drivers.vehicle.MovementCommand;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * A client for communicating with ROS2 via rosbridge.
 */
public class ROS2BridgeClient {

  /**
   * This class's Logger.
   */
  private static final Logger LOG = LoggerFactory.getLogger(ROS2BridgeClient.class);
  /**
   * The URI of the rosbridge server.
   */
  private final URI rosbridgeUri;
  /**
   * The WebSocket client instance.
   */
  private volatile WebSocketClient wsClient;
  /**
   * Indicates whether the client is connected to rosbridge.
   */
  private volatile boolean connected;

  /**
   * Creates a new instance.
   *
   * @param rosbridgeUrl The URL of the rosbridge server.
   */
  public ROS2BridgeClient(String rosbridgeUrl) {
    this.rosbridgeUri = URI.create(rosbridgeUrl);
  }

  /**
   * Connects to the rosbridge server.
   */
  public void connect() {
    try {
      LOG.info("Connecting to rosbridge at {}", rosbridgeUri);

      // Close existing connection if any
      if (wsClient != null && connected) {
        disconnect();
      }

      final CountDownLatch latch = new CountDownLatch(1);

      wsClient = new WebSocketClient(rosbridgeUri) {
        @Override
        public void onOpen(ServerHandshake handshakedata) {
          LOG.info("Connected to rosbridge");
          connected = true;
          latch.countDown();
        }

        @Override
        public void onMessage(String message) {
          LOG.debug("Received message from rosbridge: {}", message);
          // Handle received messages
          onVehicleStateUpdate(message);
        }

        @Override
        public void onClose(int code, String reason, boolean remote) {
          LOG.info("Disconnected from rosbridge: {}", reason);
          connected = false;
        }

        @Override
        public void onError(Exception ex) {
          LOG.error("Error in rosbridge connection: {}", ex.getMessage(), ex);
          connected = false;
          latch.countDown();
        }
      };

      wsClient.connect();

      // Wait for connection to be established (max 5 seconds)
      latch.await(5, TimeUnit.SECONDS);
    }
    catch (Exception e) {
      LOG.error("Failed to connect to rosbridge: {}", e.getMessage(), e);
      connected = false;
    }
  }

  /**
   * Disconnects from the rosbridge server.
   */
  public void disconnect() {
    try {
      LOG.info("Disconnecting from rosbridge at {}", rosbridgeUri);
      if (wsClient != null) {
        wsClient.close();
        wsClient = null;
      }
      connected = false;
      LOG.info("Disconnected from rosbridge");
    }
    catch (Exception e) {
      LOG.error("Failed to disconnect from rosbridge: {}", e.getMessage(), e);
    }
  }

  /**
   * Checks if the client is connected to rosbridge.
   *
   * @return {@code true} if connected, {@code false} otherwise.
   */
  public boolean isConnected() {
    return connected && wsClient != null && wsClient.isOpen();
  }

  /**
   * Sends a movement command to the vehicle via rosbridge.
   *
   * @param command The movement command to send.
   */
  public void sendMovementCommand(MovementCommand command) {
    if (!isConnected()) {
      LOG.warn("Cannot send movement command: not connected to rosbridge");
      return;
    }

    try {
      Step step = command.getStep();
      LOG.debug(
          "Sending movement command: path={}, destination={}, operation={}",
          step.getPath() != null ? step.getPath().getName() : "null",
          step.getDestinationPoint().getName(),
          command.getOperation()
      );

      // Create and send ROS2 message for movement command
      String cmdVel = String.format(
          "{\"op\":\"publish\",\"topic\":\"/cmd_vel\",\"msg\":{\"linear\":{\"x\":0.5,\"y\":0.0,\"z\":0.0},\"angular\":{\"x\":0.0,\"y\":0.0,\"z\":0.0}}}"
      );
      sendMessage(cmdVel);

      // Simulate command execution delay and then stop
      CompletableFuture.runAsync(() -> {
        try {
          Thread.sleep(1000);
          
          // Send stop command
          String stopCmd = String.format(
              "{\"op\":\"publish\",\"topic\":\"/cmd_vel\",\"msg\":{\"linear\":{\"x\":0.0,\"y\":0.0,\"z\":0.0},\"angular\":{\"x\":0.0,\"y\":0.0,\"z\":0.0}}}"
          );
          sendMessage(stopCmd);
          LOG.debug("Movement command executed and stopped");
        }
        catch (InterruptedException e) {
          Thread.currentThread().interrupt();
        }
      });
    }
    catch (Exception e) {
      LOG.error("Failed to send movement command: {}", e.getMessage(), e);
    }
  }

  /**
   * Publishes the vehicle's position to ROS2.
   *
   * @param position The position to publish.
   */
  public void publishVehiclePosition(String position) {
    if (!isConnected()) {
      LOG.warn("Cannot publish vehicle position: not connected to rosbridge");
      return;
    }

    try {
      LOG.debug("Publishing vehicle position: {}", position);
      // Create and send ROS2 message for position
      String posMsg = String.format(
          "{\"op\":\"publish\",\"topic\":\"/vehicle/position\",\"msg\":{\"data\":\"%s\"}}",
          position
      );
      sendMessage(posMsg);
    }
    catch (Exception e) {
      LOG.error("Failed to publish vehicle position: {}", e.getMessage(), e);
    }
  }

  /**
   * Publishes the plant model (map) to ROS2.
   *
   * @param plantModel The plant model to publish.
   */
  public void publishMap(PlantModel plantModel) {
    if (!isConnected()) {
      LOG.warn("Cannot publish map: not connected to rosbridge");
      return;
    }

    try {
      LOG.info("Publishing map to ROS2: {}", plantModel.getName());
      // Create and send ROS2 message for map
      String mapMsg = String.format(
          "{\"op\":\"publish\",\"topic\":\"/map\",\"msg\":{\"name\":\"%s\",\"points\":%d,\"paths\":%d,\"locations\":%d}}",
          plantModel.getName(),
          plantModel.getPoints().size(),
          plantModel.getPaths().size(),
          plantModel.getLocations().size()
      );
      sendMessage(mapMsg);

      // Publish detailed points with coordinates
      for (org.opentcs.data.model.Point point : plantModel.getPoints()) {
        publishPoint(point);
      }

      LOG.debug(
          "Map published: {} points, {} paths, {} locations",
          plantModel.getPoints().size(),
          plantModel.getPaths().size(),
          plantModel.getLocations().size()
      );
    }
    catch (Exception e) {
      LOG.error("Failed to publish map: {}", e.getMessage(), e);
    }
  }

  /**
   * Publishes a single point's coordinates to ROS2.
   *
   * @param point The point to publish.
   */
  public void publishPoint(org.opentcs.data.model.Point point) {
    if (!isConnected()) {
      LOG.warn("Cannot publish point: not connected to rosbridge");
      return;
    }

    try {
      // Get the point's coordinates from Pose -> Triple
      Triple position = point.getPose().getPosition();
      if (position == null) {
        LOG.warn("Point {} has no position, skipping", point.getName());
        return;
      }

      // Convert mm to meters for ROS2 coordinates
      double x = position.getX() / 5000.0;
      double y = position.getY() / 5000.0;
      double z = position.getZ() / 5000.0;
      double orientation = point.getPose().getOrientationAngle();

      LOG.debug(
          "Publishing point: {} (x: {}mm -> {}m, y: {}mm -> {}m, z: {}mm -> {}m, orientation: {}°)",
          point.getName(),
          position.getX(), x,
          position.getY(), y,
          position.getZ(), z,
          orientation
      );

      // Create and send ROS2 message for point
      String pointMsg = String.format(
          "{\"op\":\"publish\",\"topic\":\"/map/points\",\"msg\":{\"name\":\"%s\",\"type\":\"%s\",\"position\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},\"orientation\":%.1f}}",
          point.getName(),
          point.getType().name(),
          x, y, z,
          orientation
      );
      sendMessage(pointMsg);

      // Also publish as a marker for visualization in Rviz
      String markerMsg = String.format(
          "{\"op\":\"publish\",\"topic\":\"/visualization_marker\",\"msg\":{\"header\":{\"frame_id\":\"map\"},\"ns\":\"map_points\",\"id\":%d,\"type\":2,\"action\":0,\"pose\":{\"position\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},\"orientation\":{\"x\":0.0,\"y\":0.0,\"z\":0.0,\"w\":1.0}},\"scale\":{\"x\":0.2,\"y\":0.2,\"z\":0.2},\"color\":{\"r\":1.0,\"g\":0.0,\"b\":0.0,\"a\":1.0},\"text\":\"%s\"}}",
          point.getName().hashCode(),
          x, y, z,
          point.getName()
      );
      sendMessage(markerMsg);
    }
    catch (Exception e) {
      LOG.error("Failed to publish point {}: {}", point.getName(), e.getMessage(), e);
    }
  }

  /**
   * Publishes vehicle position with coordinates to ROS2.
   *
   * @param positionName The name of the position.
   * @param x The x coordinate in mm.
   * @param y The y coordinate in mm.
   * @param z The z coordinate in mm.
   */
  public void publishVehiclePositionWithCoords(String positionName, long x, long y, long z) {
    if (!isConnected()) {
      LOG.warn("Cannot publish vehicle position: not connected to rosbridge");
      return;
    }

    try {
      // Convert mm to meters for ROS2 coordinates
      double rosX = x / 5000.0;
      double rosY = y / 5000.0;
      double rosZ = z / 5000.0;

      LOG.debug(
          "Publishing vehicle position: {} (x: {}mm -> {}m, y: {}mm -> {}m, z: {}mm -> {}m)",
          positionName,
          x, rosX,
          y, rosY,
          z, rosZ
      );

      // Create and send ROS2 message for vehicle position
      String positionMsg = String.format(
          "{\"op\":\"publish\",\"topic\":\"/vehicle/pose\",\"msg\":{\"position\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},\"orientation\":{\"x\":0.0,\"y\":0.0,\"z\":0.0,\"w\":1.0}}}",
          rosX, rosY, rosZ
      );
      sendMessage(positionMsg);

      // Also publish a marker for visualization in Rviz
      String vehicleMarker = String.format(
          "{\"op\":\"publish\",\"topic\":\"/visualization_marker\",\"msg\":{\"header\":{\"frame_id\":\"map\"},\"ns\":\"vehicle\",\"id\":0,\"type\":10,\"action\":0,\"pose\":{\"position\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},\"orientation\":{\"x\":0.0,\"y\":0.0,\"z\":0.0,\"w\":1.0}},\"scale\":{\"x\":1.0,\"y\":0.5,\"z\":0.5},\"color\":{\"r\":0.0,\"g\":1.0,\"b\":0.0,\"a\":1.0}}}",
          rosX, rosY, rosZ
      );
      sendMessage(vehicleMarker);

      // Publish position name for reference
      String positionNameMsg = String.format(
          "{\"op\":\"publish\",\"topic\":\"/vehicle/position\",\"msg\":{\"data\":\"%s\"}}",
          positionName
      );
      sendMessage(positionNameMsg);
    }
    catch (Exception e) {
      LOG.error("Failed to publish vehicle position: {}", e.getMessage(), e);
    }
  }

  /**
   * Receives vehicle state updates from ROS2.
   *
   * @param stateUpdate The state update received.
   */
  public void onVehicleStateUpdate(String stateUpdate) {
    LOG.debug("Received vehicle state update: {}", stateUpdate);
    // Implement vehicle state update handling here
  }

  /**
   * Sends a string message to the specified ROS2 topic.
   *
   * @param topic The topic to publish to.
   * @param message The message to send.
   */
  public void sendStringMessage(String topic, String message) {
    try {
      String rosMessage = String.format(
          "{\"op\":\"publish\",\"topic\":\"%s\",\"msg\":{\"data\":\"%s\"}}",
          topic, message
      );
      sendMessage(rosMessage);
    }
    catch (Exception e) {
      LOG.error("Failed to send string message: {}", e.getMessage(), e);
    }
  }

  /**
   * Sends a vehicle command to the specified topic.
   *
   * @param topic The topic to publish to (typically "/cmd_vel").
   * @param linearVel The linear velocity.
   * @param angularVel The angular velocity.
   */
  public void sendVehicleCommand(String topic, double linearVel, double angularVel) {
    try {
      String rosMessage = String.format(
          "{\"op\":\"publish\",\"topic\":\"%s\",\"msg\":{\"linear\":{\"x\":%.2f,\"y\":0.0,\"z\":0.0},\"angular\":{\"x\":0.0,\"y\":0.0,\"z\":%.2f}}}",
          topic, linearVel, angularVel
      );
      sendMessage(rosMessage);
    }
    catch (Exception e) {
      LOG.error("Failed to send vehicle command: {}", e.getMessage(), e);
    }
  }

  /**
   * Sends position data to the specified topic.
   *
   * @param topic The topic to publish to.
   * @param x The x-coordinate.
   * @param y The y-coordinate.
   * @param z The z-coordinate.
   */
  public void sendPositionData(String topic, double x, double y, double z) {
    try {
      String rosMessage = String.format(
          "{\"op\":\"publish\",\"topic\":\"%s\",\"msg\":{\"position\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}}}",
          topic, x, y, z
      );
      sendMessage(rosMessage);
    }
    catch (Exception e) {
      LOG.error("Failed to send position data: {}", e.getMessage(), e);
    }
  }

  /**
   * Sends a raw JSON message to rosbridge.
   *
   * @param message The JSON message to send.
   */
  private synchronized void sendMessage(String message) {
    if (!isConnected() || wsClient == null) {
      LOG.warn("Cannot send message: not connected to rosbridge");
      return;
    }

    try {
      wsClient.send(message);
      LOG.debug("Sent message to rosbridge: {}", message);
    }
    catch (Exception e) {
      LOG.error("Failed to send message: {}", e.getMessage(), e);
    }
  }
}
