package org.opentcs.ros2bridge;

import jakarta.annotation.Nonnull;
import jakarta.inject.Inject;
import java.util.ArrayList;
import java.util.List;
import org.opentcs.access.KernelServicePortal;
import org.opentcs.data.TCSObjectReference;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.VehicleCommAdapterDescription;
import org.opentcs.drivers.vehicle.management.VehicleCommAdapterPanel;
import org.opentcs.drivers.vehicle.management.VehicleCommAdapterPanelFactory;
import org.opentcs.drivers.vehicle.management.VehicleProcessModelTO;

/**
 * A factory for creating {@link VehicleCommAdapterPanel} instances for the ROS2 bridge adapter.
 */
public class ROS2BridgeCommAdapterPanelFactory
    implements
      VehicleCommAdapterPanelFactory {

  /**
   * The service portal.
   */
  private final KernelServicePortal servicePortal;
  /**
   * Whether this factory is initialized or not.
   */
  private boolean initialized;

  /**
   * Creates a new instance.
   *
   * @param servicePortal The service portal.
   */
  @Inject
  public ROS2BridgeCommAdapterPanelFactory(KernelServicePortal servicePortal) {
    this.servicePortal = servicePortal;
  }

  @Override
  public void initialize() {
    if (isInitialized()) {
      return;
    }
    initialized = true;
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
    initialized = false;
  }

  @Override
  public List<VehicleCommAdapterPanel> getPanelsFor(
      @Nonnull
      VehicleCommAdapterDescription description,
      @Nonnull
      TCSObjectReference<Vehicle> vehicle,
      @Nonnull
      VehicleProcessModelTO processModel
  ) {
    List<VehicleCommAdapterPanel> panels = new ArrayList<>();
    // For now, return an empty list - no custom panel needed
    return panels;
  }
}
