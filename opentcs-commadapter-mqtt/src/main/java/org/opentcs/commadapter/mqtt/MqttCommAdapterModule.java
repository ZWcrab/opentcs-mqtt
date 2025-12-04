// SPDX-FileCopyrightText: The openTCS Authors
// SPDX-License-Identifier: MIT

package org.opentcs.commadapter.mqtt;

import org.opentcs.customizations.kernel.KernelInjectionModule;

public class MqttCommAdapterModule
    extends
      KernelInjectionModule {

  @Override
  protected void configure() {
    // Register the MQTT communication adapter factory
    vehicleCommAdaptersBinder().addBinding().to(MqttCommunicationAdapterFactory.class);
  }
}
