// SPDX-FileCopyrightText: The openTCS Authors
// SPDX-License-Identifier: MIT

package org.opentcs.commadapter.mqtt;

import java.util.Objects;
import java.util.concurrent.ScheduledExecutorService;
import org.opentcs.components.kernel.services.TCSObjectService;
import org.opentcs.drivers.vehicle.VehicleCommAdapter;
import org.opentcs.drivers.vehicle.VehicleCommAdapterDescription;
import org.opentcs.drivers.vehicle.VehicleCommAdapterFactory;
import org.opentcs.drivers.vehicle.VehicleProcessModel;

public class MqttCommunicationAdapterFactory
    implements
      VehicleCommAdapterFactory {

  private final TCSObjectService objectService;
  private final ScheduledExecutorService executor;
  private boolean initialized;
  private final VehicleCommAdapterDescription description;

  public MqttCommunicationAdapterFactory(
      TCSObjectService objectService,
      ScheduledExecutorService executor
  ) {
    this.objectService = Objects.requireNonNull(objectService, "objectService");
    this.executor = Objects.requireNonNull(executor, "executor");
    this.initialized = false;
    this.description = new MqttCommAdapterDescription();
  }

  @Override
  public VehicleCommAdapterDescription getDescription() {
    return description;
  }

  @Override
  public boolean providesAdapterFor(org.opentcs.data.model.Vehicle vehicle) {
    Objects.requireNonNull(vehicle, "vehicle");
    // Check if this vehicle is configured to use MQTT adapter
    String adapterType = vehicle.getProperty("tcs:vehicleCommAdapter");
    return adapterType != null && adapterType.equals(description.getDescription());
  }

  @Override
  public VehicleCommAdapter getAdapterFor(org.opentcs.data.model.Vehicle vehicle) {
    Objects.requireNonNull(vehicle, "vehicle");

    // Create a new process model for each vehicle
    VehicleProcessModel vehicleModel = new VehicleProcessModel(vehicle);

    return new MqttCommunicationAdapter(
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
      return false;
    }
  }
}
