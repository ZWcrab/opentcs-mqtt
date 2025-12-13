package org.opentcs.ros2bridge;

import org.opentcs.customizations.controlcenter.ControlCenterInjectionModule;

/**
 * Registers the ROS2 bridge adapter's panels.
 */
public class ROS2BridgeCommAdapterPanelsModule
    extends
      ControlCenterInjectionModule {

  /**
   * Creates a new instance.
   */
  public ROS2BridgeCommAdapterPanelsModule() {
  }

  @Override
  protected void configure() {
    commAdapterPanelFactoryBinder().addBinding().to(ROS2BridgeCommAdapterPanelFactory.class);
  }
}
