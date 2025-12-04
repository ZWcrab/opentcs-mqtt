package org.opentcs.demovehicle;

import java.util.List;
import org.opentcs.access.KernelServicePortal;
import org.opentcs.access.rmi.KernelServicePortalBuilder;
import org.opentcs.components.kernel.services.PlantModelService;

public class demo {

  public static void main(String[] args) {
//    KernelServicePortal servicePortal = new KernelServicePortalBuilder().build();
//
//// Connect and log in with a kernel somewhere.
//    servicePortal.login("localhost", 1099);
//
//// Get a reference to the plant model service...
//    PlantModelService plantModelService = servicePortal.getPlantModelService();
//// ...and find out the name of the currently loaded model.
//    String modelName = plantModelService.getModelName();
//
//// Poll events, waiting up to a second if none are currently there.
//// This should be done periodically, and probably in a separate thread.
//    List<Object> events = servicePortal.fetchEvents(1000);
  }
}
