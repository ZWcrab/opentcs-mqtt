I will implement the requested features to subscribe to the pose topic, visualize the position in real-time, and handle order completion upon arrival.

### 1. Configuration
*   **File**: `ROS2BridgeConstants.java`
    *   Add `DEFAULT_POSE_TOPIC = "/amcl_pose"`.
*   **File**: `ROS2BridgeConfiguration.java`
    *   Add `poseTopic()` method to allow configuration of the topic name.

### 2. ROS2BridgeClient Enhancements
*   **File**: `ROS2BridgeClient.java`
    *   Add `subscribe(String topic)` method to send the subscription JSON to rosbridge.
    *   Add a `Consumer<Triple, Double> poseListener` to handle incoming position updates.
    *   Update `onVehicleStateUpdate` to parse `geometry_msgs/PoseWithCovarianceStamped`.
        *   Extract position (x, y, z).
        *   Extract orientation (quaternion) and convert it to Euler angle (degrees) for OpenTCS.
        *   Notify the listener with the new position and orientation.

### 3. ROS2BridgeCommAdapter Logic
*   **File**: `ROS2BridgeCommAdapter.java`
    *   In `initialize()`, subscribe to the configured pose topic.
    *   Implement the pose listener callback:
        *   **Coordinate Conversion**: Convert ROS meters to OpenTCS millimeters (`val * 5000`).
        *   **Update Model**: Call `getProcessModel().setPose(...)` to update the precise position for visualization in the Operations Desk.
        *   **Arrival Detection**:
            *   Check the queue of sent commands (`getSentCommands()`).
            *   Calculate the distance between the current precise position and the destination point of the active command.
            *   If the distance is within a threshold (e.g., 100mm):
                *   Mark the command as executed (`commandExecuted()`).
                *   Snap the vehicle position to the logical point (`setPosition()`).

### 4. Verification
*   Recompile the project.
*   The system will automatically subscribe to the pose topic on startup.
*   Real-time movements should be visible in the Operations Desk.
*   Vehicles should automatically complete their transport orders when they get close to the target point.