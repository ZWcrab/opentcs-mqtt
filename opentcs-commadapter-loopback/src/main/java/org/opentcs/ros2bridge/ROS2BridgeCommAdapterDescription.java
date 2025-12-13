package org.opentcs.ros2bridge;

import org.opentcs.drivers.vehicle.VehicleCommAdapterDescription;

/**
 * The ROS2 bridge adapter's {@link VehicleCommAdapterDescription}.
 */
public class ROS2BridgeCommAdapterDescription
    extends
      VehicleCommAdapterDescription {

  /**
   * Creates a new instance.
   */
  public ROS2BridgeCommAdapterDescription() {
  }

  @Override
  public String getDescription() {
    return "ROS2 Bridge Adapter";
  }

  @Override
  public boolean isSimVehicleCommAdapter() {
    return true;
  }
}
