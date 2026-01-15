package org.opentcs.commadapter.rosbridge;

import org.opentcs.data.model.Vehicle;

/**
 * A factory for creating components related to the ROS2 bridge adapter.
 */
public interface ROS2BridgeAdapterComponentsFactory {

  /**
   * Creates a new ROS2BridgeCommAdapter for the given vehicle.
   *
   * @param vehicle The vehicle.
   * @return A new ROS2BridgeCommAdapter for the given vehicle.
   */
  ROS2BridgeCommAdapter createROS2BridgeCommAdapter(Vehicle vehicle);
}
