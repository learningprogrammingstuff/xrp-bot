# xrp-bot

![knife](https://static.1a64.com/xrpbot.jpg "knife")

## How to Run

### Prerequisites
- Java 17 or later
- Internet connection (for downloading dependencies and loading Three.js from CDN)

### Running the Simulation

1. Run the simulation with:
   ```bash
   ./gradlew simulateJava
   ```

2. The simulation will automatically:
   - Open the WPILib simulation GUI
   - Start an HTTP server on port 5800
   - Open your default browser to `http://localhost:5800` showing a 3D visualization

### 3D Visualization Features

The browser-based visualization displays:
- **Robot**: A green box with a red arrow showing the current position and heading
- **Map Points**: Orange spheres representing obstacles detected by the ultrasonic sensor
- **Ultrasonic Beam**: A yellow line showing the current sensor beam
- **Simulated World**: Translucent blue walls outlining the room boundaries
- **Obstacles**: Red semi-transparent boxes showing static obstacles in the environment
- **HUD**: Real-time pose information (X, Y, heading) in the top-left corner

Camera controls:
- **Left mouse drag**: Rotate view
- **Right mouse drag**: Pan view
- **Scroll wheel**: Zoom in/out

### EKF Tuning

The Extended Kalman Filter uses sensor fusion to estimate the robot pose. Tuning constants are located in `src/main/java/frc/robot/sim/EKFLocalizer.java`:

- `PROCESS_NOISE_POSITION` (default: 0.1 in²) - Controls trust in odometry position estimates
- `PROCESS_NOISE_HEADING` (default: 0.01 rad²) - Controls trust in odometry heading estimates
- `MEASUREMENT_NOISE_GYRO` (default: 0.05 rad²) - Controls trust in gyro measurements

Lower values = more trust in that sensor. Adjust based on observed drift and sensor accuracy.

### Map Persistence

The ultrasonic-based map is automatically saved to `xrp-map.json` in the project root directory every 5 seconds when new points are detected.

To clear the map:
- Delete the `xrp-map.json` file, or
- Call the `clearMap()` method on the OccupancyMapper instance

### Simulated World

The simulation uses a virtual environment defined in `src/main/java/frc/robot/sim/SimWorld.java`:
- Room dimensions: 120 × 96 inches (10 × 8 feet)
- Contains three rectangular obstacles at fixed positions
- Ultrasonic sensor readings are simulated via ray casting with Gaussian noise (σ = 0.5 inches)

To modify the world:
- Edit the `ROOM_WIDTH` and `ROOM_HEIGHT` constants
- Add or modify obstacles in the `SimWorld` constructor

### Architecture

The simulation system consists of:
- **EKFLocalizer**: Sensor fusion combining wheel odometry and IMU gyro
- **OccupancyMapper**: Converts ultrasonic readings to world-frame obstacle points
- **SimWorld**: Simulates the physical environment and sensor readings
- **VisualizationServer**: Embedded HTTP server with Three.js-based 3D visualization
- **Matrix3x3**: Minimal linear algebra utilities for EKF calculations

All simulation code is in the `frc.robot.sim` package.