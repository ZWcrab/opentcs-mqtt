package org.opentcs.commadapter.rosbridge;

import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.VehicleProcessModel;

/**
 * The vehicle process model for the ROS2 bridge adapter.
 */
public class ROS2BridgeVehicleModel
    extends
      VehicleProcessModel {

  /**
   * Creates a new instance.
   *
   * @param vehicle The vehicle associated with this process model.
   */
  public ROS2BridgeVehicleModel(Vehicle vehicle) {
    super(vehicle);
  }

  /**
   * Additional attributes specific to ROS2 bridge vehicles.
   */
  public enum Attribute {
    /**
     * Indicates whether the vehicle is connected to ROS2.
     */
    ROS2_CONNECTED,
    /**
     * The current ROS2 namespace.
     */
    ROS2_NAMESPACE,
    /**
     * The current ROS2 topic prefix.
     */
    ROS2_TOPIC_PREFIX
  }
}
