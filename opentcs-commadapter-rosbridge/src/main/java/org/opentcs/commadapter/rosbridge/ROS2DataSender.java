package org.opentcs.commadapter.rosbridge;

import java.net.URI;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * A simple static class for sending ROS2 data via rosbridge.
 */
public class ROS2DataSender {

  /**
   * This class's Logger.
   */
  private static final Logger LOG = LoggerFactory.getLogger(ROS2DataSender.class);

  /**
   * The WebSocket client instance.
   */
  private static volatile WebSocketClient webSocketClient;

  /**
   * Indicates whether the client is connected.
   */
  private static final AtomicBoolean CONNECTED = new AtomicBoolean(false);

  /**
   * The default rosbridge URL.
   */
  private static final String DEFAULT_ROSBRIDGE_URL = "ws://192.168.31.177:9090";

  /**
   * Private constructor to prevent instantiation.
   */
  private ROS2DataSender() {
    // Prevent instantiation
  }

  /**
   * Connects to the rosbridge server.
   *
   * @param url The URL of the rosbridge server.
   * @return {@code true} if connected successfully, {@code false} otherwise.
   */
  public static boolean connect(String url) {
    LOG.info("Connecting to rosbridge at {}", url);

    // Close existing connection if any
    if (webSocketClient != null && CONNECTED.get()) {
      disconnect();
    }

    final CountDownLatch latch = new CountDownLatch(1);

    try {
      webSocketClient = new RosbridgeWebSocketClient(URI.create(url), latch);
      webSocketClient.connect();

      // Wait for connection to be established (max 5 seconds)
      return latch.await(5, TimeUnit.SECONDS) && CONNECTED.get();
    }
    catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      CONNECTED.set(false);
      return false;
    }
    catch (IllegalArgumentException e) {
      LOG.error("Failed to connect to rosbridge: {}", e.getMessage(), e);
      CONNECTED.set(false);
      return false;
    }
  }

  /**
   * Connects to the rosbridge server using the default URL.
   *
   * @return {@code true} if connected successfully, {@code false} otherwise.
   */
  public static boolean connect() {
    return connect(DEFAULT_ROSBRIDGE_URL);
  }

  /**
   * Disconnects from the rosbridge server.
   */
  public static void disconnect() {
    if (webSocketClient != null) {
      webSocketClient.close();
      webSocketClient = null;
    }
    CONNECTED.set(false);
    LOG.info("Disconnected from rosbridge");
  }

  /**
   * Checks if connected to rosbridge.
   *
   * @return {@code true} if connected, {@code false} otherwise.
   */
  public static boolean isConnected() {
    return CONNECTED.get();
  }

  /**
   * Sends a string message to the specified ROS2 topic.
   *
   * @param topic The topic to publish to.
   * @param message The message to send.
   * @return {@code true} if message was sent successfully, {@code false} otherwise.
   */
  public static boolean sendStringMessage(String topic, String message) {
    String rosMessage = String.format(
        "{\"op\":\"publish\",\"topic\":\"%s\",\"msg\":{\"data\":\"%s\"}}",
        topic,
        message
    );
    return sendMessage(rosMessage);
  }

  /**
   * Sends a vehicle command to the specified topic.
   *
   * @param topic The topic to publish to (typically "/cmd_vel").
   * @param linearVel The linear velocity.
   * @param angularVel The angular velocity.
   * @return {@code true} if message was sent successfully, {@code false} otherwise.
   */
  public static boolean sendVehicleCommand(String topic, double linearVel, double angularVel) {
    String rosMessage = String.format(
        "{\"op\":\"publish\",\"topic\":\"%s\",\"msg\":{\"linear\":{\"x\":%.2f,\"y\":0.0,\"z\":0.0},"
            + "\"angular\":{\"x\":0.0,\"y\":0.0,\"z\":%.2f}}}",
        topic,
        linearVel,
        angularVel
    );
    return sendMessage(rosMessage);
  }

  /**
   * Sends position data to the specified topic.
   *
   * @param topic The topic to publish to.
   * @param x The x-coordinate.
   * @param y The y-coordinate.
   * @param z The z-coordinate.
   * @return {@code true} if message was sent successfully, {@code false} otherwise.
   */
  public static boolean sendPositionData(String topic, double x, double y, double z) {
    String rosMessage = String.format(
        "{\"op\":\"publish\",\"topic\":\"%s\",\"msg\":{\"position\":{"
            + "\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}}}",
        topic,
        x,
        y,
        z
    );
    return sendMessage(rosMessage);
  }

  /**
   * Sends map data to the specified topic.
   *
   * @param topic The topic to publish to.
   * @param mapJson The map data in JSON format.
   * @return {@code true} if message was sent successfully, {@code false} otherwise.
   */
  public static boolean sendMapData(String topic, String mapJson) {
    String rosMessage = String.format(
        "{\"op\":\"publish\",\"topic\":\"%s\",\"msg\":%s}",
        topic,
        mapJson
    );
    return sendMessage(rosMessage);
  }

  /**
   * Sends a raw JSON message to rosbridge.
   *
   * @param message The JSON message to send.
   * @return {@code true} if message was sent successfully, {@code false} otherwise.
   */
  private static synchronized boolean sendMessage(String message) {
    if (!isConnected() || webSocketClient == null) {
      LOG.warn("Cannot send message: not connected to rosbridge");
      return false;
    }

    webSocketClient.send(message);
    LOG.info("Sent message to rosbridge: {}", message);
    return true;
  }

  /**
   * Example usage of the ROS2DataSender class.
   *
   * @param args Command line arguments.
   */
  public static void main(String[] args) {
    // Example: Send a vehicle command
    boolean connected = ROS2DataSender.connect();
    if (connected) {
      // Send a vehicle command to move forward at 0.5 m/s with no rotation
      ROS2DataSender.sendVehicleCommand("/cmd_vel", 0.5, 0.0);

      // Send a position update
      ROS2DataSender.sendPositionData("/vehicle/position", 10.5, 20.3, 0.0);

      // Send a status message
      ROS2DataSender.sendStringMessage("/vehicle/status", "ready");

      // Disconnect after 2 seconds
      try {
        Thread.sleep(2000);
      }
      catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }
      ROS2DataSender.disconnect();
    }
  }

  private static final class RosbridgeWebSocketClient
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
      CONNECTED.set(true);
      latch.countDown();
    }

    @Override
    public void onMessage(String message) {
      LOG.info("Received message from rosbridge: {}", message);
    }

    @Override
    public void onClose(int code, String reason, boolean remote) {
      LOG.info("Disconnected from rosbridge: {}", reason);
      CONNECTED.set(false);
    }

    @Override
    public void onError(Exception ex) {
      LOG.error("Error in rosbridge connection: {}", ex.getMessage(), ex);
      CONNECTED.set(false);
      latch.countDown();
    }
  }
}
