// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import java.util.Arrays;

/**
 * Filters ultrasonic range measurements to reject spikes and outliers.
 *
 * <p>Applies a 5-sample median filter followed by an outlier rejection gate.
 * The filter is deterministic and suitable for real-time use at 50 Hz (20 ms cycle).
 *
 * <h3>Filter constants</h3>
 * <ul>
 *   <li>{@code MEDIAN_WINDOW_SIZE = 5} – median filter window (odd for a true median)</li>
 *   <li>{@code MAX_DELTA_PER_CYCLE_M = 1.0} – max allowed jump per 20 ms cycle (meters)</li>
 *   <li>{@code SPIKE_CONFIRM_FRAMES = 3} – consecutive frames a jump must persist before it
 *       is accepted as a real change</li>
 * </ul>
 */
public class RangeFilter {
  /** Number of samples for the median filter window */
  public static final int MEDIAN_WINDOW_SIZE = 5;

  /** Maximum allowed change per cycle in meters.
   *  At 50 Hz, 1.0 m/cycle = 50 m/s which is far beyond any real robot motion. */
  public static final double MAX_DELTA_PER_CYCLE_M = 1.0;

  /** Number of consecutive frames a large jump must persist before being accepted */
  public static final int SPIKE_CONFIRM_FRAMES = 3;

  private final double[] medianBuffer;
  private int bufferIndex;
  private int samplesReceived;
  private double lastGoodValue;
  private boolean hasGoodValue;
  private int spikeCounter;
  private double pendingSpikeValue;

  /** Creates a new RangeFilter with default settings. */
  public RangeFilter() {
    medianBuffer = new double[MEDIAN_WINDOW_SIZE];
    bufferIndex = 0;
    samplesReceived = 0;
    lastGoodValue = 0.0;
    hasGoodValue = false;
    spikeCounter = 0;
    pendingSpikeValue = 0.0;
  }

  /**
   * Feeds a new range measurement through the filter pipeline.
   *
   * <p>Pipeline:
   * <ol>
   *   <li>Reject NaN / negative values (returns last-known-good)</li>
   *   <li>Apply 5-sample median filter</li>
   *   <li>Apply outlier rejection gate (large single-frame jumps rejected
   *       unless they persist for {@code SPIKE_CONFIRM_FRAMES})</li>
   * </ol>
   *
   * @param rawMeters the raw range measurement in meters
   * @return the filtered range in meters
   */
  public double filter(double rawMeters) {
    // Reject invalid values
    if (Double.isNaN(rawMeters) || Double.isInfinite(rawMeters) || rawMeters < 0.0) {
      return lastGoodValue;
    }

    // Add to median buffer
    medianBuffer[bufferIndex] = rawMeters;
    bufferIndex = (bufferIndex + 1) % MEDIAN_WINDOW_SIZE;
    samplesReceived++;

    // Compute median of available samples
    int count = Math.min(samplesReceived, MEDIAN_WINDOW_SIZE);
    double medianValue = computeMedian(medianBuffer, count);

    // Outlier rejection gate
    if (hasGoodValue) {
      double delta = Math.abs(medianValue - lastGoodValue);
      if (delta > MAX_DELTA_PER_CYCLE_M) {
        // Large jump detected – count as potential spike
        if (Math.abs(medianValue - pendingSpikeValue) < MAX_DELTA_PER_CYCLE_M * 0.5) {
          spikeCounter++;
        } else {
          spikeCounter = 1;
          pendingSpikeValue = medianValue;
        }

        if (spikeCounter >= SPIKE_CONFIRM_FRAMES) {
          // Jump is real – accept it
          lastGoodValue = medianValue;
          spikeCounter = 0;
        }
        // Otherwise return last-known-good
        return lastGoodValue;
      }
    }

    // Normal update
    lastGoodValue = medianValue;
    hasGoodValue = true;
    spikeCounter = 0;
    return lastGoodValue;
  }

  /**
   * Returns the last filtered value without feeding a new sample.
   *
   * @return the last filtered range in meters, or 0.0 if no valid sample has been received
   */
  public double getLastValue() {
    return lastGoodValue;
  }

  /**
   * Returns whether at least one valid measurement has been accepted.
   *
   * @return true if a valid measurement exists
   */
  public boolean hasValidMeasurement() {
    return hasGoodValue;
  }

  /**
   * Resets the filter to its initial state.
   */
  public void reset() {
    Arrays.fill(medianBuffer, 0.0);
    bufferIndex = 0;
    samplesReceived = 0;
    lastGoodValue = 0.0;
    hasGoodValue = false;
    spikeCounter = 0;
    pendingSpikeValue = 0.0;
  }

  /**
   * Computes the median of the first {@code count} elements in the buffer.
   *
   * @param buffer the sample buffer
   * @param count  number of valid samples in the buffer
   * @return the median value
   */
  static double computeMedian(double[] buffer, int count) {
    double[] sorted = new double[count];
    System.arraycopy(buffer, 0, sorted, 0, count);
    Arrays.sort(sorted);
    if (count % 2 == 1) {
      return sorted[count / 2];
    }
    return (sorted[count / 2 - 1] + sorted[count / 2]) / 2.0;
  }
}
