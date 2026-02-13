// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SimWorldTest {
  private SimWorld simWorld;

  @BeforeEach
  void setUp() {
    // Use procedural generation with a fixed seed for reproducible tests
    simWorld = new SimWorld(42);
  }

  @Test
  void ultrasonicReadingNeverExceedsMaxRange() {
    // Facing away from all walls/obstacles at max distance
    // should still return at most MAX_RANGE (1968)
    double range = simWorld.simulateUltrasonicReading(60.0, 48.0, 0.0, 0.0);
    assertTrue(range <= 1968.0, "Range should not exceed MAX_RANGE (1968), got: " + range);
  }

  @Test
  void ultrasonicReadingNeverBelowMinRange() {
    // Even when very close to a wall, reading should be clamped to MIN_RANGE (1.0)
    // Position the sensor right at the left wall (x=0), facing left
    double range = simWorld.simulateUltrasonicReading(0.5, 48.0, Math.PI, 0.0);
    assertTrue(range >= 1.0, "Range should not be below MIN_RANGE (1.0), got: " + range);
  }

  @Test
  void ultrasonicReadingCloseToWallDoesNotReturnMaxRange() {
    // When the sensor is very close to a wall, it should NOT return ~1968 inches.
    // This verifies the fix: previously, measurement < MIN_RANGE returned MAX_RANGE
    // causing wild oscillations between ~1 inch and ~1968 inches.
    // Position very close to left wall (x=0), facing left (theta=PI)
    for (int i = 0; i < 20; i++) {
      double range = simWorld.simulateUltrasonicReading(0.3, 48.0, Math.PI, 0.0);
      assertTrue(range < 100.0,
          "Range near a wall should be small, not MAX_RANGE. Got: " + range);
    }
  }

  @Test
  void ultrasonicReadingDetectsWalls() {
    // From center of room (60, 48), facing right (theta=0),
    // should detect right wall at x=120 → distance ~60 inches
    double range = simWorld.simulateUltrasonicReading(60.0, 48.0, 0.0, 0.0);
    // Allow for noise and obstacles, but should be reasonable
    assertTrue(range > 0 && range < 200.0,
        "Should detect a wall or obstacle within room bounds, got: " + range);
  }

  @Test
  void ultrasonicReadingWithSensorOffset() {
    // Sensor offset moves the origin point forward
    double rangeNoOffset = simWorld.simulateUltrasonicReading(60.0, 48.0, 0.0, 0.0);
    double rangeWithOffset = simWorld.simulateUltrasonicReading(60.0, 48.0, 0.0, 2.0);
    // With offset, sensor is 2 inches closer to the wall → reading should be ~2 less
    // Allow for noise
    assertTrue(Math.abs((rangeNoOffset - rangeWithOffset) - 2.0) < 3.0,
        "Sensor offset should shift reading by ~2 inches. No offset: "
            + rangeNoOffset + ", with offset: " + rangeWithOffset);
  }
}
