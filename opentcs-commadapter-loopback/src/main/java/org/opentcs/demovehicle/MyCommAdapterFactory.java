package org.opentcs.demovehicle;

import java.util.Objects;
import java.util.concurrent.ScheduledExecutorService;
import org.opentcs.components.kernel.services.TCSObjectService;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.VehicleCommAdapter;
import org.opentcs.drivers.vehicle.VehicleCommAdapterDescription;
import org.opentcs.drivers.vehicle.VehicleCommAdapterFactory;
import org.opentcs.drivers.vehicle.VehicleProcessModel;


public class MyCommAdapterFactory
    implements
      VehicleCommAdapterFactory {

  private final TCSObjectService objectService;
  private final ScheduledExecutorService executor;
  private boolean initialized;
  private final VehicleCommAdapterDescription description;

  @com.google.inject.Inject
  public MyCommAdapterFactory(
      TCSObjectService objectService,
      @org.opentcs.customizations.kernel.KernelExecutor
      ScheduledExecutorService executor
  ) {
    this.objectService = Objects.requireNonNull(objectService, "objectService");
    this.executor = Objects.requireNonNull(executor, "executor");
    this.initialized = false;
    this.description = new MyCommAdapterFactory.MqttCommAdapterDescription();
  }

  @Override
  public VehicleCommAdapterDescription getDescription() {
    return description;
  }

  @Override
  public boolean providesAdapterFor(org.opentcs.data.model.Vehicle vehicle) {
    Objects.requireNonNull(vehicle, "vehicle");
    // Return true for all vehicles
    return true;
  }

  @Override
  public VehicleCommAdapter getAdapterFor(org.opentcs.data.model.Vehicle vehicle) {
    Objects.requireNonNull(vehicle, "vehicle");

    // Create a new process model for each vehicle
    VehicleProcessModel vehicleModel = new VehicleProcessModel(vehicle);

    return new MyVehicleCommAdapter(
        objectService,
        vehicleModel,
        executor
    );
  }

  @Override
  public void initialize() {
    this.initialized = true;
  }

  @Override
  public boolean isInitialized() {
    return initialized;
  }

  @Override
  public void terminate() {
    this.initialized = false;
  }

  /**
   * A description for the MQTT communication adapter.
   */
  private static class MqttCommAdapterDescription
      extends
        VehicleCommAdapterDescription {

    @Override
    public String getDescription() {
      return "MQTT Communication Adapter";
    }

    @Override
    public boolean isSimVehicleCommAdapter() {
      return true;
    }
  }
}
