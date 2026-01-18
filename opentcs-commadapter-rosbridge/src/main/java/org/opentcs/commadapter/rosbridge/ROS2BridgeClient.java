package org.opentcs.commadapter.rosbridge;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.net.URI;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;
import org.opentcs.data.model.PlantModel;
import org.opentcs.data.model.Triple;
import org.opentcs.data.model.Vehicle;
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
  private static final ObjectMapper OBJECT_MAPPER = new ObjectMapper();
  private static final String INITIAL_POSE_COVARIANCE
      = "[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, "
          + "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
          + "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
          + "0.06853892326654787]";
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
   * Listener for position updates.
   */
  private Consumer<String> positionListener;
  /**
   * Listener for pose updates (position + orientation).
   */
  private BiConsumer<Triple, Double> poseListener;

  private final Map<String, String> requestedSubscriptions = new ConcurrentHashMap<>();

  /**
   * Creates a new instance.
   *
   * @param rosbridgeUrl The URL of the rosbridge server.
   */
  public ROS2BridgeClient(String rosbridgeUrl) {
    this.rosbridgeUri = URI.create(rosbridgeUrl);
  }

  /**
   * Sets the listener for pose updates.
   *
   * @param listener The listener to set.
   */
  public void setPoseListener(BiConsumer<Triple, Double> listener) {
    this.poseListener = listener;
  }

  /**
   * Sets the listener for position updates.
   *
   * @param listener The listener to set.
   */
  public void setPositionListener(Consumer<String> listener) {
    this.positionListener = listener;
  }

  /**
   * Connects to the rosbridge server.
   */
  public void connect() {
    LOG.info("Connecting to rosbridge at {}", rosbridgeUri);

    // Close existing connection if any
    if (wsClient != null && connected) {
      disconnect();
    }

    final CountDownLatch latch = new CountDownLatch(1);
    wsClient = new RosbridgeWebSocketClient(rosbridgeUri, latch);

    try {
      wsClient.connect();

      // Wait for connection to be established (max 5 seconds)
      latch.await(5, TimeUnit.SECONDS);
    }
    catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      connected = false;
    }
  }

  /**
   * Disconnects from the rosbridge server.
   */
  public void disconnect() {
    LOG.info("Disconnecting from rosbridge at {}", rosbridgeUri);
    if (wsClient != null) {
      wsClient.close();
      wsClient = null;
    }
    connected = false;
    LOG.info("Disconnected from rosbridge");
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
   * Sends a navigation goal to the vehicle.
   *
   * @param topic The topic to publish the goal to.
   * @param position The target position.
   * @param orientationAngle The target orientation in degrees.
   */
  public void sendNavigationGoal(String topic, Triple position, double orientationAngle) {
    if (!isConnected()) {
      LOG.warn("Cannot send navigation goal: not connected to rosbridge");
      return;
    }

    // Convert OpenTCS coordinates to ROS coordinates (meters)
    double rosX = position.getX() / 5000.0;
    double rosY = position.getY() / 5000.0;
    double rosZ = position.getZ() / 5000.0;

    // Convert orientation (degrees) to Quaternion
    // Nav2 uses ROS coordinate system, so we might need to adjust if frames differ.
    // Assuming standard mapping for now.
    double orientationDeg = orientationAngle;
    if (Double.isNaN(orientationDeg)) {
      LOG.warn("Orientation is NaN, defaulting to 0.0");
      orientationDeg = 0.0;
    }

    double yaw = Math.toRadians(orientationDeg);
    double qx = 0.0;
    double qy = 0.0;
    double qz = Math.sin(yaw / 2.0);
    double qw = Math.cos(yaw / 2.0);

    LOG.info(
        "Sending navigation goal: topic={}, x={}, y={}, yaw={}",
        topic,
        rosX,
        rosY,
        orientationDeg
    );

    // Create JSON message for geometry_msgs/PoseStamped
    long now = System.currentTimeMillis();
    long secs = now / 1000;
    long nsecs = (now % 1000) * 1000000;

    String goalMsg = String.format(
        "{\"op\":\"publish\",\"topic\":\"%s\",\"type\":\"geometry_msgs/PoseStamped\",\"msg\":{"
            + "\"header\":{\"frame_id\":\"map\",\"stamp\":{\"secs\":%d,\"nsecs\":%d}},"
            + "\"pose\":{\"position\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
            + "\"orientation\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"w\":%.3f}}}}",
        topic,
        secs,
        nsecs,
        rosX,
        rosY,
        rosZ,
        qx,
        qy,
        qz,
        qw
    );

    sendMessage(goalMsg);
    LOG.info(
        "Navigation goal published to {}: ({}, {}, {})",
        topic,
        rosX,
        rosY,
        orientationDeg
    );
  }

  /**
   * Sends a movement command to the vehicle via rosbridge.
   *
   * @param command The movement command to send.
   * @param goalTopic The topic to publish the goal to.
   */
  public void sendMovementCommand(MovementCommand command, String goalTopic) {
    if (!isConnected()) {
      LOG.warn("Cannot send movement command: not connected to rosbridge");
      return;
    }

    Step step = command.getStep();
    LOG.info(
        "Sending movement command: path={}, destination={}, operation={}",
        step.getPath() != null ? step.getPath().getName() : "null",
        step.getDestinationPoint().getName(),
        command.getOperation()
    );

    if (step.getDestinationPoint() != null) {
      Triple position = step.getDestinationPoint().getPose().getPosition();
      double orientation = step.getDestinationPoint().getPose().getOrientationAngle();
      sendNavigationGoal(goalTopic, position, orientation);
    }
    else {
      LOG.warn("Cannot send movement command: no destination point");
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

    LOG.info("Publishing vehicle position: {}", position);
    String posMsg = String.format(
        "{\"op\":\"publish\",\"topic\":\"/vehicle/position\",\"msg\":{\"data\":\"%s\"}}",
        position
    );
    sendMessage(posMsg);
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

    LOG.info("Publishing map to ROS2: {}", plantModel.getName());
    String mapMsg = String.format(
        "{\"op\":\"publish\",\"topic\":\"/map\",\"msg\":{"
            + "\"name\":\"%s\",\"points\":%d,\"paths\":%d,\"locations\":%d}}",
        plantModel.getName(),
        plantModel.getPoints().size(),
        plantModel.getPaths().size(),
        plantModel.getLocations().size()
    );
    sendMessage(mapMsg);

    for (org.opentcs.data.model.Point point : plantModel.getPoints()) {
      publishPoint(point);
    }

    LOG.info(
        "Map published: {} points, {} paths, {} locations",
        plantModel.getPoints().size(),
        plantModel.getPaths().size(),
        plantModel.getLocations().size()
    );
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

    LOG.info(
        "Publishing point: {} (x: {}mm -> {}m, y: {}mm -> {}m, z: {}mm -> {}m, orientation: {}°)",
        point.getName(),
        position.getX(),
        x,
        position.getY(),
        y,
        position.getZ(),
        z,
        orientation
    );

    // Create and send ROS2 message for point
    String pointMsg = String.format(
        "{\"op\":\"publish\",\"topic\":\"/map/points\",\"msg\":{"
            + "\"name\":\"%s\",\"type\":\"%s\",\"position\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
            + "\"orientation\":%.1f}}",
        point.getName(),
        point.getType().name(),
        x,
        y,
        z,
        orientation
    );
    sendMessage(pointMsg);

    // Also publish as a marker for visualization in Rviz
    String markerMsg = String.format(
        "{\"op\":\"publish\",\"topic\":\"/visualization_marker\",\"msg\":{"
            + "\"header\":{\"frame_id\":\"map\"},\"ns\":\"map_points\",\"id\":%d,\"type\":2,"
            + "\"action\":0,\"pose\":{\"position\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
            + "\"orientation\":{\"x\":0.0,\"y\":0.0,\"z\":0.0,\"w\":1.0}},"
            + "\"scale\":{\"x\":0.2,\"y\":0.2,\"z\":0.2},"
            + "\"color\":{\"r\":1.0,\"g\":0.0,\"b\":0.0,\"a\":1.0},\"text\":\"%s\"}}",
        point.getName().hashCode(),
        x,
        y,
        z,
        point.getName()
    );
    sendMessage(markerMsg);
  }

  /**
   * Sends an initial pose to the vehicle to initialize its position in the map.
   *
   * @param x The x coordinate in mm.
   * @param y The y coordinate in mm.
   * @param orientationAngle The orientation in degrees.
   */
  public void sendInitialPose(long x, long y, double orientationAngle) {
    if (!isConnected()) {
      LOG.warn("Cannot send initial pose: not connected to rosbridge");
      return;
    }

    // Convert mm to meters
    double rosX = x / 5000.0;
    double rosY = y / 5000.0;

    // Default orientation if not provided or NaN
    double orientationDeg = orientationAngle;
    if (Double.isNaN(orientationDeg)) {
      orientationDeg = 0.0;
    }

    // Convert orientation to quaternion
    double yaw = Math.toRadians(orientationDeg);
    double qz = Math.sin(yaw / 2.0);
    double qw = Math.cos(yaw / 2.0);

    LOG.info(
        "Sending initial pose: x={} ({}m), y={} ({}m), yaw={}",
        x,
        rosX,
        y,
        rosY,
        orientationDeg
    );

    // Use current time minus 500ms to account for clock skew/latency
    // This ensures the timestamp is slightly in the past relative to ROS2's clock
    long now = System.currentTimeMillis() - 500;
    long secs = now / 1000;
    long nsecs = (now % 1000) * 1000000;

    String covariance = INITIAL_POSE_COVARIANCE;

    String initialPoseMsg = String.format(
        "{\"op\":\"publish\",\"topic\":\"/initialpose\",\"type\":"
            + "\"geometry_msgs/PoseWithCovarianceStamped\",\"msg\":{"
            + "\"header\":{\"frame_id\":\"map\",\"stamp\":{\"secs\":%d,\"nsecs\":%d}},"
            + "\"pose\":{\"pose\":{\"position\":{\"x\":%.3f,\"y\":%.3f,\"z\":0.0},"
            + "\"orientation\":{\"x\":0.0,\"y\":0.0,\"z\":%.3f,\"w\":%.3f}},"
            + "\"covariance\":%s}}}",
        secs,
        nsecs,
        rosX,
        rosY,
        qz,
        qw,
        covariance
    );

    sendMessage(initialPoseMsg);
    LOG.info("Initial pose published: x={}, y={}", rosX, rosY);
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

    // Convert mm to meters for ROS2 coordinates
    double rosX = x / 5000.0;
    double rosY = y / 5000.0;
    double rosZ = z / 5000.0;

    LOG.info(
        "Publishing vehicle position: {} (x: {}mm -> {}m, y: {}mm -> {}m, z: {}mm -> {}m)",
        positionName,
        x,
        rosX,
        y,
        rosY,
        z,
        rosZ
    );

    // Create and send ROS2 message for vehicle position
    String positionMsg = String.format(
        "{\"op\":\"publish\",\"topic\":\"/vehicle/pose\",\"msg\":{"
            + "\"position\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
            + "\"orientation\":{\"x\":0.0,\"y\":0.0,\"z\":0.0,\"w\":1.0}}}",
        rosX,
        rosY,
        rosZ
    );
    sendMessage(positionMsg);

    // Also publish a marker for visualization in Rviz
    String vehicleMarker = String.format(
        "{\"op\":\"publish\",\"topic\":\"/visualization_marker\",\"msg\":{"
            + "\"header\":{\"frame_id\":\"map\"},\"ns\":\"vehicle\",\"id\":0,\"type\":10,"
            + "\"action\":0,\"pose\":{\"position\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
            + "\"orientation\":{\"x\":0.0,\"y\":0.0,\"z\":0.0,\"w\":1.0}},"
            + "\"scale\":{\"x\":1.0,\"y\":0.5,\"z\":0.5},"
            + "\"color\":{\"r\":0.0,\"g\":1.0,\"b\":0.0,\"a\":1.0}}}",
        rosX,
        rosY,
        rosZ
    );
    sendMessage(vehicleMarker);

    // Publish position name for reference
    String positionNameMsg = String.format(
        "{\"op\":\"publish\",\"topic\":\"/vehicle/position\",\"msg\":{\"data\":\"%s\"}}",
        positionName
    );
    sendMessage(positionNameMsg);
  }

  /**
   * Subscribes to a topic.
   *
   * @param topic The topic to subscribe to.
   */
  public void subscribe(String topic) {
    subscribe(topic, "geometry_msgs/PoseWithCovarianceStamped");
  }

  /**
   * Subscribes to a topic with a specific type.
   *
   * @param topic The topic to subscribe to.
   * @param type The message type of the topic.
   */
  public void subscribe(String topic, String type) {
    String subscribeMsg = String.format(
        "{\"op\":\"subscribe\",\"topic\":\"%s\",\"type\":\"%s\"}",
        topic,
        type
    );
    requestedSubscriptions.put(topic, subscribeMsg);
    trySendSubscribe(topic);
  }

  private void trySendSubscribe(String topic) {
    if (wsClient == null || !wsClient.isOpen()) {
      return;
    }
    String subscribeMsg = requestedSubscriptions.get(topic);
    if (subscribeMsg == null) {
      return;
    }
    wsClient.send(subscribeMsg);
    LOG.info("Subscribed to topic: {}", topic);
  }

  /**
   * Receives vehicle state updates from ROS2.
   *
   * @param stateUpdate The state update received.
   */
  public void onVehicleStateUpdate(String stateUpdate) {
    LOG.info("Received vehicle state update: {}", stateUpdate);
    JsonNode root;
    try {
      root = OBJECT_MAPPER.readTree(stateUpdate);
    }
    catch (IOException e) {
      LOG.warn("Failed to parse vehicle state update: {}", e.getMessage(), e);
      return;
    }

    if (!"publish".equals(root.path("op").asText())) {
      return;
    }

    String topic = root.path("topic").asText();
    if ("/vehicle/position".equals(topic)) {
      handlePositionUpdate(root);
      return;
    }

    handlePoseUpdate(root);
  }

  private void handlePositionUpdate(JsonNode root) {
    String position = root.path("msg").path("data").asText();
    if (positionListener != null && !position.isEmpty()) {
      positionListener.accept(position);
    }
  }

  private void handlePoseUpdate(JsonNode root) {
    LOG.info("接收位置信息: {}", root);
    JsonNode poseNode = root.path("msg").path("pose");
    JsonNode poseDataNode = poseNode.path("pose");
    if (poseDataNode.isMissingNode()) {
      poseDataNode = poseNode;
    }
    if (poseDataNode.isMissingNode()) {
      return;
    }

    JsonNode positionNode = poseDataNode.path("position");
    JsonNode orientationNode = poseDataNode.path("orientation");
    if (positionNode.isMissingNode() || orientationNode.isMissingNode()) {
      return;
    }

    double x = positionNode.path("x").asDouble();
    double y = positionNode.path("y").asDouble();
    double z = positionNode.path("z").asDouble();
    LOG.debug("Received pose update: x={}, y={}, z={}", x, y, z);

    long xMm = (long) (x * 5000.0);
    long yMm = (long) (y * 5000.0);
    long zMm = (long) (z * 5000.0);

    double yawDeg = yawDegreesFromQuaternion(
        orientationNode.path("x").asDouble(),
        orientationNode.path("y").asDouble(),
        orientationNode.path("z").asDouble(),
        orientationNode.path("w").asDouble()
    );

    if (poseListener != null) {
      poseListener.accept(new Triple(xMm, yMm, zMm), yawDeg);
    }
    // 获取目的地坐标
    JsonNode goalNode = root.path("msg").path("goal");
    if (goalNode.isMissingNode()) {
      return;
    }
    JsonNode goalPositionNode = goalNode.path("pose").path("position");
    if (goalPositionNode.isMissingNode()) {
      return;
    }
    double goalX = goalPositionNode.path("x").asDouble();
    double goalY = goalPositionNode.path("y").asDouble();
    double goalZ = goalPositionNode.path("z").asDouble();
    LOG.debug("Received goal position: x={}, y={}, z={}", goalX, goalY, goalZ);
    // 判断是否抵达
    if (Math.abs(x - goalX) < 0.1 && Math.abs(y - goalY) < 0.1 && Math.abs(z - goalZ) < 0.1) {
      LOG.info("Vehicle has arrived at the goal position");


    }
  }

  private static double yawDegreesFromQuaternion(double qx, double qy, double qz, double qw) {
    double sinYCosp = 2.0 * (qw * qz + qx * qy);
    double cosYCosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    double yawRad = Math.atan2(sinYCosp, cosYCosp);
    double yawDeg = Math.toDegrees(yawRad);
    if (yawDeg < 0) {
      yawDeg += 360.0;
    }
    return yawDeg;
  }

  /**
   * Sends a string message to the specified ROS2 topic.
   *
   * @param topic The topic to publish to.
   * @param message The message to send.
   */
  public void sendStringMessage(String topic, String message) {
    String advertiseMsg = String.format(
        "{\"op\":\"advertise\",\"topic\":\"%s\",\"type\":\"std_msgs/String\"}",
        topic
    );
    sendMessage(advertiseMsg);

    String rosMessage = String.format(
        "{\"op\":\"publish\",\"topic\":\"%s\",\"msg\":{\"data\":\"%s\"}}",
        topic,
        message
    );
    sendMessage(rosMessage);
    LOG.info("Sent string message to topic {}: {}", topic, message);
  }

  /**
   * Sends a vehicle command to the specified topic.
   *
   * @param topic The topic to publish to (typically "/cmd_vel").
   * @param linearVel The linear velocity.
   * @param angularVel The angular velocity.
   */
  public void sendVehicleCommand(String topic, double linearVel, double angularVel) {
    String rosMessage = String.format(
        "{\"op\":\"publish\",\"topic\":\"%s\",\"msg\":{\"linear\":{\"x\":%.2f,\"y\":0.0,\"z\":0.0},"
            + "\"angular\":{\"x\":0.0,\"y\":0.0,\"z\":%.2f}}}",
        topic,
        linearVel,
        angularVel
    );
    sendMessage(rosMessage);
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
    String rosMessage = String.format(
        "{\"op\":\"publish\",\"topic\":\"%s\",\"msg\":{\"position\":{"
            + "\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}}}",
        topic,
        x,
        y,
        z
    );
    sendMessage(rosMessage);
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

    wsClient.send(message);
    LOG.info("Sent message to rosbridge: {}", message);
  }

  private final class RosbridgeWebSocketClient
      extends
        WebSocketClient {
    private final CountDownLatch latch;

    private RosbridgeWebSocketClient(URI serverUri, CountDownLatch latch) {
      super(serverUri);
      this.latch = latch;
    }

    @Override
    public void onOpen(ServerHandshake handshakedata) {
      LOG.info("Connected to rosbridge");
      connected = true;
      latch.countDown();
      for (String topic : requestedSubscriptions.keySet()) {
        trySendSubscribe(topic);
      }
    }

    @Override
    public void onMessage(String message) {
      LOG.debug("Received message from rosbridge: {}", message);
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
  }
}
