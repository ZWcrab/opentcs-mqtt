package org.opentcs.commadapter.rosbridge;

import com.google.inject.assistedinject.FactoryModuleBuilder;
import org.opentcs.customizations.kernel.KernelInjectionModule;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Configures/binds the ROS2 bridge communication adapters of the openTCS kernel.
 */
public class ROS2BridgeCommAdapterModule
    extends
      KernelInjectionModule {

  /**
   * This class's logger.
   */
  private static final Logger LOG = LoggerFactory.getLogger(ROS2BridgeCommAdapterModule.class);

  /**
   * Creates a new instance.
   */
  public ROS2BridgeCommAdapterModule() {
  }

  @Override
  protected void configure() {
    // Create a default configuration instance
    ROS2BridgeConfiguration configuration = new ROS2BridgeConfiguration() {
      // Use default values from the interface
    };

    bind(ROS2BridgeConfiguration.class)
        .toInstance(configuration);

    install(new FactoryModuleBuilder().build(ROS2BridgeAdapterComponentsFactory.class));

    // Register the ROS2 bridge adapter factory
    vehicleCommAdaptersBinder().addBinding().to(ROS2BridgeCommAdapterFactory.class);

    LOG.info("ROS2 bridge adapter module configured");
  }
}
