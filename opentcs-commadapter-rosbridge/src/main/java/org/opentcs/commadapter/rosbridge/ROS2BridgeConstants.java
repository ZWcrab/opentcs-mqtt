package org.opentcs.commadapter.rosbridge;

/**
 * Provides constants for the ROS2 bridge adapter.
 */
public interface ROS2BridgeConstants {

  /**
   * The default URL for the rosbridge server.
   */
  String DEFAULT_ROSBRIDGE_URL = "ws://192.168.31.177:9090";

  /**
   * The default capacity of the command queue.
   */
  int DEFAULT_COMMAND_QUEUE_CAPACITY = 10;

  /**
   * The default operation used for recharging.
   */
  String DEFAULT_RECHARGE_OPERATION = "recharge";

  /**
   * The default ROS2 namespace for vehicles.
   */
  String DEFAULT_VEHICLE_NAMESPACE = "/vehicle";

  /**
   * The default prefix for ROS2 topics.
   */
  String DEFAULT_TOPIC_PREFIX = "opentcs";

  /**
   * The default topic for navigation goals.
   */
  String DEFAULT_GOAL_TOPIC = "/goal_pose";

  /**
   * The default topic for pose updates.
   */
  String DEFAULT_POSE_TOPIC = "/amcl_pose";

  /**
   * The property key for the initial position of a vehicle.
   */
  String PROPKEY_INITIAL_POSITION = "initialPosition";

  /**
   * The property key for the rosbridge URL.
   */
  String PROPKEY_ROSBRIDGE_URL = "rosbridgeUrl";

  /**
   * The property key for the vehicle's ROS2 namespace.
   */
  String PROPKEY_VEHICLE_NAMESPACE = "vehicleNamespace";

  /**
   * The property key for the ROS2 topic prefix.
   */
  String PROPKEY_TOPIC_PREFIX = "topicPrefix";

  /**
   * The name of the adapter factory.
   */
  String ADAPTER_FACTORY_NAME = "ROS2_BRIDGE";
}
