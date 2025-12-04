// SPDX-FileCopyrightText: The openTCS Authors
// SPDX-License-Identifier: MIT

package org.opentcs.commadapter.mqtt;

import java.util.Objects;
import java.util.concurrent.ScheduledExecutorService;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;
import org.opentcs.components.kernel.services.TCSObjectService;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.drivers.vehicle.BasicVehicleCommAdapter;
import org.opentcs.drivers.vehicle.MovementCommand;
import org.opentcs.drivers.vehicle.VehicleProcessModel;
import org.opentcs.drivers.vehicle.management.VehicleProcessModelTO;
import org.opentcs.util.ExplainedBoolean;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class MqttCommunicationAdapter
    extends
      BasicVehicleCommAdapter
    implements
      MqttCallback {

  private static final Logger LOG = LoggerFactory.getLogger(MqttCommunicationAdapter.class);

  private static final String DEFAULT_BROKER_URL = "tcp://localhost:1883";
  private static final String DEFAULT_CLIENT_ID = "openTCS-MQTT-Adapter";
  private static final String DEFAULT_TOPIC_PREFIX = "opentcs/vehicle/";
  private static final String RECHARGE_OPERATION = "RECHARGE";
  private static final int COMMANDS_CAPACITY = 10;

  private final TCSObjectService objectService;
  private final String vehicleName;

  private MqttClient mqttClient;
  private String brokerUrl;
  private String clientId;
  private String topicPrefix;

  public MqttCommunicationAdapter(
      TCSObjectService objectService,
      VehicleProcessModel vehicleProcessModel,
      ScheduledExecutorService executor
  ) {
    super(
        vehicleProcessModel,
        COMMANDS_CAPACITY,
        RECHARGE_OPERATION,
        executor
    );
    this.objectService = Objects.requireNonNull(objectService, "objectService");
    // Get vehicle name directly from the process model
    this.vehicleName = vehicleProcessModel.getName();

    // Initialize default values
    this.brokerUrl = DEFAULT_BROKER_URL;
    this.clientId = DEFAULT_CLIENT_ID;
    this.topicPrefix = DEFAULT_TOPIC_PREFIX + vehicleName + "/";
  }

  @Override
  protected void connectVehicle() {
    try {
      LOG.info("Connecting to MQTT broker: {}", brokerUrl);
      mqttClient = new MqttClient(brokerUrl, clientId, new MemoryPersistence());

      // Set callback for handling messages
      mqttClient.setCallback(this);

      MqttConnectOptions connOpts = new MqttConnectOptions();
      connOpts.setCleanSession(true);

      mqttClient.connect(connOpts);
      LOG.info("Connected to MQTT broker");

      // Subscribe to response topics
      mqttClient.subscribe(topicPrefix + "response");
      mqttClient.subscribe(topicPrefix + "state");
    }
    catch (MqttException e) {
      LOG.error("Failed to connect to MQTT broker", e);
      throw new RuntimeException("Failed to connect to MQTT broker", e);
    }
  }

  @Override
  protected void disconnectVehicle() {
    try {
      if (mqttClient != null && mqttClient.isConnected()) {
        mqttClient.disconnect();
        LOG.info("Disconnected from MQTT broker");
      }
    }
    catch (MqttException e) {
      LOG.error("Failed to disconnect from MQTT broker", e);
    }
    finally {
      mqttClient = null;
    }
  }

  @Override
  protected boolean isVehicleConnected() {
    return mqttClient != null && mqttClient.isConnected();
  }

  @Override
  public void sendCommand(MovementCommand command) {
    try {
      // Convert MovementCommand to MQTT message
      String commandJson = convertToJson(command);

      MqttMessage message = new MqttMessage(commandJson.getBytes());
      message.setQos(2);

      String topic = topicPrefix + "command";
      mqttClient.publish(topic, message);
      LOG.info("Sent command to topic {}: {}", topic, commandJson);
    }
    catch (MqttException e) {
      LOG.error("Failed to send command via MQTT", e);
      // Let the BasicVehicleCommAdapter handle the failure
      throw new RuntimeException("Failed to send command via MQTT", e);
    }
  }

  @Override
  protected VehicleProcessModelTO createCustomTransferableProcessModel() {
    // Create a new VehicleProcessModelTO without constructor parameters
    return new VehicleProcessModelTO();
  }

  private String convertToJson(MovementCommand command) {
    // Simplified JSON conversion - in a real implementation, use a proper JSON library
    StringBuilder json = new StringBuilder();
    json.append("{");
    json.append("\"command\":\"move\",");
    json.append("\"destination\":\"");
    json.append(command.getStep().getDestinationPoint().getName());
    json.append("\",");
    json.append("\"routeIndex\":");
    json.append(command.getStep().getRouteIndex());
    json.append("}");

    return json.toString();
  }

  @Override
  public void connectionLost(Throwable cause) {
    LOG.error("Connection to MQTT broker lost", cause);
    // The BasicVehicleCommAdapter will handle the connection state
  }

  @Override
  public void messageArrived(String topic, MqttMessage message)
      throws Exception {
    String payload = new String(message.getPayload(), "UTF-8");
    LOG.info("Received message on topic {}: {}", topic, payload);

    // Process the message based on topic
    if (topic.endsWith("response")) {
      processCommandResponse(payload);
    }
    else if (topic.endsWith("state")) {
      processVehicleStateUpdate(payload);
    }
  }

  @Override
  public void deliveryComplete(IMqttDeliveryToken token) {
    // Message delivery completed, no action needed
    LOG.debug("Message delivery completed for token: {}", token.getMessageId());
  }

  /**
   * Processes a command response from the vehicle.
   *
   * @param response The response message payload.
   */
  private void processCommandResponse(String response) {
    // Simplified response processing - in a real implementation, use a proper JSON library
    if (response.contains("\"status\":\"success\"")) {
      // Command executed successfully
      if (!getSentCommands().isEmpty()) {
        MovementCommand completedCommand = getSentCommands().remove();
        // Let the BasicVehicleCommAdapter know the command was executed
        getProcessModel().commandExecuted(completedCommand);
      }
    }
    else if (response.contains("\"status\":\"error\"")) {
      // Command failed
      if (!getSentCommands().isEmpty()) {
        MovementCommand failedCommand = getSentCommands().remove();
        // Let the BasicVehicleCommAdapter know the command failed
        getProcessModel().commandFailed(failedCommand);
      }
    }
  }

  /**
   * Processes a vehicle state update.
   *
   * @param stateUpdate The state update message payload.
   */
  private void processVehicleStateUpdate(String stateUpdate) {
    // Simplified state processing - in a real implementation, use a proper JSON library
    VehicleProcessModel model = getProcessModel();

    // Update vehicle position
    if (stateUpdate.contains("\"position\":")) {
      int startIndex = stateUpdate.indexOf("\"position\":\"") + 13;
      int endIndex = stateUpdate.indexOf("\"", startIndex);
      if (startIndex > 12 && endIndex > startIndex) {
        String position = stateUpdate.substring(startIndex, endIndex);
        model.setPosition(position);
      }
    }

    // Update energy level
    if (stateUpdate.contains("\"energyLevel\":")) {
      int startIndex = stateUpdate.indexOf("\"energyLevel\":") + 14;
      int endIndex = stateUpdate.indexOf(",", startIndex);
      if (endIndex == -1) {
        endIndex = stateUpdate.indexOf("}", startIndex);
      }
      if (startIndex > 13 && endIndex > startIndex) {
        try {
          String energyLevelStr = stateUpdate.substring(startIndex, endIndex);
          int energyLevel = Integer.parseInt(energyLevelStr);
          model.setEnergyLevel(energyLevel);
        }
        catch (NumberFormatException e) {
          LOG.warn("Failed to parse energy level from state update: {}", stateUpdate, e);
        }
      }
    }
  }

  @Override
  public void onVehiclePaused(boolean paused) {
    // Not implemented - no action needed for MQTT adapter
  }

  @Override
  public ExplainedBoolean canProcess(TransportOrder transportOrder) {
    // Default implementation - can process any transport order
    return new ExplainedBoolean(true, "MQTT adapter can process any transport order");
  }
}
