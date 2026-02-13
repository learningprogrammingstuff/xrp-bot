// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class RangeFilterTest {
  private RangeFilter filter;

  @BeforeEach
  void setUp() {
    filter = new RangeFilter();
  }

  @Test
  void medianFilterRemovesSingleSpike() {
    // Feed 4 normal values, then a spike, then verify output is the median (no spike)
    filter.filter(1.0);
    filter.filter(1.0);
    filter.filter(1.0);
    filter.filter(1.0);
    // Now feed a spike
    double result = filter.filter(50.0);
    // With 5-sample median of [1.0, 1.0, 1.0, 1.0, 50.0], median = 1.0
    // The outlier gate would also reject the jump (50 - 1 > 1.0 m)
    assertEquals(1.0, result, 0.01, "Spike should be rejected by median + outlier gate");
  }

  @Test
  void medianFilterPassesSteadyValues() {
    double result = 0;
    for (int i = 0; i < 10; i++) {
      result = filter.filter(2.5);
    }
    assertEquals(2.5, result, 0.001, "Steady values should pass through unchanged");
  }

  @Test
  void outlierGateAcceptsAfterConfirmFrames() {
    // Establish a baseline
    for (int i = 0; i < 5; i++) {
      filter.filter(1.0);
    }
    // Now send a large legitimate change persistently
    double result = 0;
    for (int i = 0; i < RangeFilter.SPIKE_CONFIRM_FRAMES + RangeFilter.MEDIAN_WINDOW_SIZE; i++) {
      result = filter.filter(3.0);
    }
    assertEquals(3.0, result, 0.01,
        "Sustained jump should be accepted after confirm frames + median window fills");
  }

  @Test
  void rejectsNaN() {
    filter.filter(1.5);
    filter.filter(1.5);
    double result = filter.filter(Double.NaN);
    assertEquals(1.5, result, 0.001, "NaN should return last good value");
  }

  @Test
  void rejectsNegative() {
    filter.filter(2.0);
    filter.filter(2.0);
    double result = filter.filter(-1.0);
    assertEquals(2.0, result, 0.001, "Negative value should return last good value");
  }

  @Test
  void hasValidMeasurementInitiallyFalse() {
    assertFalse(filter.hasValidMeasurement(), "No valid measurement before any input");
  }

  @Test
  void hasValidMeasurementAfterInput() {
    filter.filter(1.0);
    assertTrue(filter.hasValidMeasurement(), "Should have valid measurement after input");
  }

  @Test
  void resetClearsState() {
    filter.filter(1.0);
    filter.filter(1.0);
    filter.reset();
    assertFalse(filter.hasValidMeasurement(), "Should have no valid measurement after reset");
    assertEquals(0.0, filter.getLastValue(), 0.001);
  }

  @Test
  void computeMedianOddCount() {
    double[] buf = {3.0, 1.0, 2.0, 0.0, 0.0};
    double median = RangeFilter.computeMedian(buf, 3);
    assertEquals(2.0, median, 0.001, "Median of [3,1,2] = 2.0");
  }

  @Test
  void computeMedianEvenCount() {
    double[] buf = {3.0, 1.0, 2.0, 4.0, 0.0};
    double median = RangeFilter.computeMedian(buf, 4);
    assertEquals(2.5, median, 0.001, "Median of [3,1,2,4] = 2.5");
  }

  @Test
  void gradualChangePasses() {
    // Gradually increasing values should all pass through
    double result = 0;
    for (int i = 0; i < 20; i++) {
      result = filter.filter(1.0 + i * 0.05); // 0.05 m per step, well under 1.0 m gate
    }
    assertTrue(result > 1.5, "Gradual increase should be tracked. Got: " + result);
  }
}
