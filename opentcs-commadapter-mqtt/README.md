# openTCS MQTT Communication Adapter

This is a communication adapter for the openTCS kernel that enables communication with vehicles using the MQTT protocol.

## Features

- Supports MQTT v3.1.1 protocol
- Bidirectional communication between openTCS kernel and vehicles
- Automatic command execution and status updates
- Configurable MQTT broker connection
- Vehicle-specific topic structure
- Support for command response and state updates

## Installation

1. Build the adapter using Gradle:
   ```bash
   ./gradlew :opentcs-commadapter-mqtt:build
   ```

2. The built JAR file will be available in `build/libs/` directory.

3. Copy the JAR file to the `lib/openTCS-extensions/` directory of your openTCS kernel installation.

4. Start the openTCS kernel. The adapter will be automatically detected.

## Configuration

### Vehicle Configuration

To use the MQTT communication adapter for a vehicle, set the following property in the vehicle's properties:

```
tcs:vehicleCommAdapter=MQTT Communication Adapter
```

### Adapter Configuration

The adapter can be configured using the following vehicle properties:

| Property Name | Default Value | Description |
|---------------|---------------|-------------|
| `mqtt:brokerUrl` | `tcp://localhost:1883` | The MQTT broker URL |
| `mqtt:clientId` | `openTCS-MQTT-Adapter` | The MQTT client ID |
| `mqtt:topicPrefix` | `opentcs/vehicle/{vehicle-name}/` | The topic prefix for vehicle communication |

## Topic Structure

The adapter uses the following topic structure for communication:

- `{topicPrefix}command` - Commands sent from openTCS to the vehicle
- `{topicPrefix}response` - Responses from the vehicle to commands
- `{topicPrefix}state` - Vehicle state updates

## Message Formats

### Command Messages

Commands sent from openTCS to the vehicle are JSON objects with the following format:

```json
{
  "command": "move",
  "destination": "Point-01",
  "operation": "LOAD",
  "routeIndex": 0
}
```

### Response Messages

Responses from the vehicle to commands are JSON objects with the following format:

```json
{
  "status": "success",
  "message": "Command executed successfully"
}
```

or for errors:

```json
{
  "status": "error",
  "message": "Failed to execute command"
}
```

### State Update Messages

Vehicle state updates are JSON objects with the following format:

```json
{
  "position": "Point-01",
  "state": "IDLE",
  "energyLevel": 85.5
}
```

## Development

To develop the adapter, follow these steps:

1. Clone the openTCS repository
2. Navigate to the repository root
3. Build the adapter using Gradle
4. Run the kernel with the adapter

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
