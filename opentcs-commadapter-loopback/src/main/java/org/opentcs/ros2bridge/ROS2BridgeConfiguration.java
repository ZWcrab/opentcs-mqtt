package org.opentcs.ros2bridge;

/**
 * Provides configuration for the ROS2 bridge adapter.
 */
public interface ROS2BridgeConfiguration {

  /**
   * The prefix for configuration keys of this class.
   */
  String PREFIX = "opentcs.ros2bridge";

  /**
   * Returns the URL of the rosbridge server.
   *
   * @return The URL of the rosbridge server.
   */
  default String rosbridgeUrl() {
    return ROS2BridgeConstants.DEFAULT_ROSBRIDGE_URL;
  }

  /**
   * Returns the capacity of the command queue.
   *
   * @return The capacity of the command queue.
   */
  default int commandQueueCapacity() {
    return ROS2BridgeConstants.DEFAULT_COMMAND_QUEUE_CAPACITY;
  }

  /**
   * Returns the operation used for recharging.
   *
   * @return The operation used for recharging.
   */
  default String rechargeOperation() {
    return ROS2BridgeConstants.DEFAULT_RECHARGE_OPERATION;
  }

  /**
   * Returns the ROS2 namespace for the vehicle.
   *
   * @return The ROS2 namespace for the vehicle.
   */
  default String vehicleNamespace() {
    return ROS2BridgeConstants.DEFAULT_VEHICLE_NAMESPACE;
  }

  /**
   * Returns the prefix for ROS2 topics.
   *
   * @return The prefix for ROS2 topics.
   */
  default String topicPrefix() {
    return ROS2BridgeConstants.DEFAULT_TOPIC_PREFIX;
  }

  /**
   * Returns whether the adapter is enabled.
   *
   * @return Whether the adapter is enabled.
   */
  default boolean enable() {
    return true;
  }
}
