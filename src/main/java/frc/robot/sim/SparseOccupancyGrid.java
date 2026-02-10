// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import java.util.HashMap;
import java.util.Map;

/**
 * Sparse log-odds occupancy grid for real-time mapping from ultrasonic sensor readings.
 * Uses a spatial hash ({@link HashMap}) so the map can grow without predefined bounds.
 *
 * <p>The HC-SR04 ultrasonic sensor is modeled as a cone (not a perfect laser ray)
 * to reflect its real beam pattern.
 */
public class SparseOccupancyGrid {
  /** Grid cell resolution in inches per cell */
  private final double resolution;

  /** Log-odds values stored sparsely – only non-zero cells are present */
  private final Map<Long, Double> logOdds;

  /** Prior log-odds (unknown) */
  private static final double L_PRIOR = 0.0;

  /** Log-odds increment for occupied */
  private static final double L_OCC = 0.85;

  /** Log-odds decrement for free */
  private static final double L_FREE = -0.4;

  /** Clamp bounds for log-odds to prevent saturation */
  private static final double L_MAX = 5.0;
  private static final double L_MIN = -5.0;

  /** HC-SR04 half-cone angle (radians). ~15 degrees typical */
  private static final double HALF_CONE_ANGLE = Math.toRadians(15.0);

  /** Number of rays to cast within the cone for each measurement */
  private static final int CONE_RAYS = 5;

  /** Maximum range for free-space ray traversal (inches) – bounded to avoid infinite updates */
  private static final double MAX_RAY_RANGE = 1968.0; // ~50 m

  /** Minimum valid ultrasonic range (inches) */
  private static final double MIN_RANGE = 1.0;

  /** Sensor offset from robot center (inches) */
  private static final double SENSOR_OFFSET = 2.0;

  /** Log-odds threshold for considering a cell occupied in range prediction */
  private static final double PREDICT_OCCUPIED_THRESHOLD = 0.5;

  /** Thickness of occupied region at beam endpoint (in cells) */
  private static final int OCCUPIED_THICKNESS = 2;

  /**
   * Creates a new sparse occupancy grid.
   *
   * @param resolution Cell size in inches (e.g. 2.0 = 2 inch cells)
   */
  public SparseOccupancyGrid(double resolution) {
    this.resolution = resolution;
    this.logOdds = new HashMap<>();
  }

  // --- Spatial-hash key helpers ---

  private long cellKey(int gx, int gy) {
    // Pack two 32-bit ints into one 64-bit long
    return ((long) gx << 32) | (gy & 0xFFFFFFFFL);
  }

  private int keyToGx(long key) {
    return (int) (key >> 32);
  }

  private int keyToGy(long key) {
    return (int) key;
  }

  // --- Coordinate conversion helpers ---

  private int worldToGridX(double wx) {
    return (int) Math.floor(wx / resolution);
  }

  private int worldToGridY(double wy) {
    return (int) Math.floor(wy / resolution);
  }

  private double gridToWorldX(int gx) {
    return (gx + 0.5) * resolution;
  }

  private double gridToWorldY(int gy) {
    return (gy + 0.5) * resolution;
  }

  private double getLogOddsAt(int gx, int gy) {
    Double val = logOdds.get(cellKey(gx, gy));
    return val != null ? val : L_PRIOR;
  }

  private void setLogOddsAt(int gx, int gy, double value) {
    double clamped = clamp(value);
    if (Math.abs(clamped) < 1e-6) {
      logOdds.remove(cellKey(gx, gy)); // Keep map sparse
    } else {
      logOdds.put(cellKey(gx, gy), clamped);
    }
  }

  private double clamp(double value) {
    return Math.max(L_MIN, Math.min(L_MAX, value));
  }

  /**
   * Updates the grid with a single ultrasonic range measurement.
   * Uses an inverse sensor model: marks cells along each ray in the sensor cone
   * as free, and cells near the endpoint as occupied.
   *
   * @param range      Measured range in inches (no upper-limit filtering here)
   * @param robotX     Robot X position (inches)
   * @param robotY     Robot Y position (inches)
   * @param robotTheta Robot heading (radians)
   * @param isHit      true if the range represents an actual hit (endpoint should be marked occupied)
   */
  public void update(double range, double robotX, double robotY, double robotTheta, boolean isHit) {
    if (range < MIN_RANGE) {
      return;
    }

    // Clamp the ray traversal distance to avoid extremely long traversals
    double traversalRange = Math.min(range, MAX_RAY_RANGE);

    double sensorX = robotX + SENSOR_OFFSET * Math.cos(robotTheta);
    double sensorY = robotY + SENSOR_OFFSET * Math.sin(robotTheta);

    // Cast multiple rays within the cone
    for (int r = 0; r < CONE_RAYS; r++) {
      double angleOffset = (CONE_RAYS > 1)
          ? -HALF_CONE_ANGLE + (2.0 * HALF_CONE_ANGLE * r) / (CONE_RAYS - 1)
          : 0.0;
      double rayAngle = robotTheta + angleOffset;
      updateRay(sensorX, sensorY, rayAngle, traversalRange, !isHit);
    }
  }

  /**
   * Updates a single ray through the grid using Bresenham's line algorithm.
   * Cells along the ray up to (range - thickness) are marked free.
   * Cells near the endpoint are marked occupied (unless noHit = true).
   */
  private void updateRay(double sensorX, double sensorY, double rayAngle,
                          double range, boolean noHit) {
    double endX = sensorX + range * Math.cos(rayAngle);
    double endY = sensorY + range * Math.sin(rayAngle);

    int x0 = worldToGridX(sensorX);
    int y0 = worldToGridY(sensorY);
    int x1 = worldToGridX(endX);
    int y1 = worldToGridY(endY);

    // Bresenham's line algorithm
    int dx = Math.abs(x1 - x0);
    int dy = Math.abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;

    int cx = x0;
    int cy = y0;
    int totalSteps = dx + dy;
    int step = 0;
    int freeStopStep = totalSteps - OCCUPIED_THICKNESS;

    while (true) {
      if (step < freeStopStep || noHit) {
        // Mark as free
        setLogOddsAt(cx, cy, getLogOddsAt(cx, cy) + L_FREE);
      }

      if (cx == x1 && cy == y1) {
        break;
      }

      int e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        cx += sx;
      }
      if (e2 < dx) {
        err += dx;
        cy += sy;
      }
      step++;
    }

    // Mark endpoint region as occupied (only if there was a hit)
    if (!noHit) {
      for (int ox = -OCCUPIED_THICKNESS; ox <= OCCUPIED_THICKNESS; ox++) {
        for (int oy = -OCCUPIED_THICKNESS; oy <= OCCUPIED_THICKNESS; oy++) {
          int gx = x1 + ox;
          int gy = y1 + oy;
          setLogOddsAt(gx, gy, getLogOddsAt(gx, gy) + L_OCC);
        }
      }
    }
  }

  /**
   * Ray-marches through the occupancy grid to predict the expected range
   * for a given pose and beam direction. Used for scan-to-map correction.
   *
   * @param robotX     Robot X position (inches)
   * @param robotY     Robot Y position (inches)
   * @param robotTheta Robot heading (radians)
   * @return Predicted range in inches, or -1 if no occupied cell found
   */
  public double predictRange(double robotX, double robotY, double robotTheta) {
    double sensorX = robotX + SENSOR_OFFSET * Math.cos(robotTheta);
    double sensorY = robotY + SENSOR_OFFSET * Math.sin(robotTheta);

    double dirX = Math.cos(robotTheta);
    double dirY = Math.sin(robotTheta);

    // Step through the grid in small increments
    double stepSize = resolution * 0.5;
    double maxDist = MAX_RAY_RANGE;

    for (double d = 0; d < maxDist; d += stepSize) {
      double wx = sensorX + d * dirX;
      double wy = sensorY + d * dirY;

      int gx = worldToGridX(wx);
      int gy = worldToGridY(wy);

      if (getLogOddsAt(gx, gy) > PREDICT_OCCUPIED_THRESHOLD) {
        return d;
      }
    }

    return -1.0; // No occupied cell found within max range
  }

  /**
   * Gets a list of occupied cell centers for visualization.
   * Returns cells with log-odds above the occupied threshold.
   *
   * @param threshold Log-odds threshold for occupied (e.g. 0.3)
   * @return Array of [x, y] world coordinates for occupied cells
   */
  public double[][] getOccupiedCells(double threshold) {
    int count = 0;
    for (double val : logOdds.values()) {
      if (val > threshold) {
        count++;
      }
    }

    double[][] result = new double[count][2];
    int idx = 0;
    for (Map.Entry<Long, Double> entry : logOdds.entrySet()) {
      if (entry.getValue() > threshold) {
        int gx = keyToGx(entry.getKey());
        int gy = keyToGy(entry.getKey());
        result[idx][0] = gridToWorldX(gx);
        result[idx][1] = gridToWorldY(gy);
        idx++;
      }
    }
    return result;
  }

  /**
   * Resets the entire grid to unknown (clears all log-odds).
   */
  public void reset() {
    logOdds.clear();
  }

  /**
   * Directly marks a world-coordinate position as occupied.
   * Used when restoring a previously saved map.
   *
   * @param wx World X coordinate (inches)
   * @param wy World Y coordinate (inches)
   */
  public void markOccupied(double wx, double wy) {
    int gx = worldToGridX(wx);
    int gy = worldToGridY(wy);
    setLogOddsAt(gx, gy, getLogOddsAt(gx, gy) + L_OCC);
  }

  /**
   * Marks a world-coordinate position as occupied with a specific log-odds value.
   * Used when restoring a previously saved map with confidence data.
   *
   * @param wx    World X coordinate (inches)
   * @param wy    World Y coordinate (inches)
   * @param value Log-odds value to set
   */
  public void markOccupiedWithValue(double wx, double wy, double value) {
    int gx = worldToGridX(wx);
    int gy = worldToGridY(wy);
    setLogOddsAt(gx, gy, value);
  }

  /**
   * Translates all occupied cells by the given offset.
   * Used when re-centering the origin.
   *
   * @param dx Offset in X (inches)
   * @param dy Offset in Y (inches)
   */
  public void translateAll(double dx, double dy) {
    // Collect all entries, clear, re-insert at new positions
    Map<Long, Double> oldEntries = new HashMap<>(logOdds);
    logOdds.clear();
    for (Map.Entry<Long, Double> entry : oldEntries.entrySet()) {
      int gx = keyToGx(entry.getKey());
      int gy = keyToGy(entry.getKey());
      double wx = gridToWorldX(gx) + dx;
      double wy = gridToWorldY(gy) + dy;
      int newGx = worldToGridX(wx);
      int newGy = worldToGridY(wy);
      long newKey = cellKey(newGx, newGy);
      Double existing = logOdds.get(newKey);
      if (existing == null || Math.abs(entry.getValue()) > Math.abs(existing)) {
        logOdds.put(newKey, clamp(entry.getValue()));
      }
    }
  }

  /** Gets the grid resolution. */
  public double getResolution() {
    return resolution;
  }

  /**
   * Gets the raw log-odds value at a grid cell.
   * @param gx Grid X index
   * @param gy Grid Y index
   * @return Log-odds value
   */
  public double getLogOdds(int gx, int gy) {
    return getLogOddsAt(gx, gy);
  }

  /** Gets the number of cells that have been updated (non-zero log-odds). */
  public int getCellCount() {
    return logOdds.size();
  }
}
