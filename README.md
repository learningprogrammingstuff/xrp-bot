# xrp-bot

An XRP robot codebase for WPILib in Java.

## Controls

The robot supports both physical joystick and keyboard controls.

### Physical Joystick (Port 0)
- **Axis 1 (Y-axis)**: Forward/Backward movement
- **Axis 2 (Z-axis or twist)**: Rotation (turn left/right)
- **Button 1 (A)**: Set arm to 45 degrees (release to return to 0)
- **Button 2 (B)**: Set arm to 90 degrees (release to return to 0)

### Keyboard Controls (Simulation Only)
When running in simulation, you can use keyboard controls via the `simgui-ds.json` configuration:

- **W**: Drive Forward
- **S**: Drive Backward
- **A**: Turn Left
- **D**: Turn Right
- **Z, X, C, V**: Buttons 1-4

The keyboard controls are mapped to Joystick 0 in simulation, so they work seamlessly with the existing code.

## Building and Running

```bash
# Build the project
./gradlew build

# Run in simulation
./gradlew simulateJava

# Deploy to robot
./gradlew deploy
```

## Project Structure

- `src/main/java/frc/robot/` - Main robot code
  - `Robot.java` - Robot lifecycle management
  - `RobotContainer.java` - Subsystems, commands, and button bindings
  - `Constants.java` - Robot-wide constants
  - `commands/` - Command classes (ArcadeDrive, autonomous routines, etc.)
  - `subsystems/` - Subsystem classes (Drivetrain, Arm)
- `simgui-ds.json` - Simulation keyboard/joystick configuration
