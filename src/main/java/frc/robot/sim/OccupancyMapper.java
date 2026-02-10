// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

/**
 * Manages ultrasonic-based occupancy mapping using a log-odds occupancy grid.
 * <p>
 * For each HC-SR04 range measurement the inverse sensor model marks cells along
 * the beam as free and the endpoint region as occupied.  The sensor is modeled
 * as a cone (not a perfect laser) to reflect the HC-SR04 beam pattern.
 * <p>
 * Drift correction uses scan-to-map consistency: the expected range is predicted
 * by ray-marching through the <em>learned</em> occupancy grid (no access to the
 * ground-truth world).
 */
public class OccupancyMapper {
  /** Maximum valid ultrasonic range (inches) */
  private static final double MAX_VALID_RANGE = 157.0;

  /** Minimum valid ultrasonic range (inches) */
  private static final double MIN_VALID_RANGE = 1.0;

  /** Auto-save interval (milliseconds) */
  private static final long SAVE_INTERVAL = 5000;

  /** Drift correction threshold (inches) */
  private static final double DRIFT_CORRECTION_THRESHOLD = 2.0;

  /** Maximum correction per cycle (inches) */
  private static final double MAX_CORRECTION_PER_CYCLE = 0.5;

  /** Number of consistent readings required for drift correction */
  private static final int CONSISTENCY_REQUIRED = 3;

  /** Occupancy grid cell resolution (inches per cell) */
  private static final double GRID_RESOLUTION = 2.0;

  /** Log-odds threshold for occupied cells in visualization / persistence */
  private static final double OCCUPIED_THRESHOLD = 0.3;

  private final OccupancyGrid grid;
  private final List<double[]> poseHistory;
  private final String mapFilePath;
  private long lastSaveTime;
  private int updatesSinceLastSave;

  // Drift correction state
  private int consistentReadingsCount;
  private double lastDriftError;

  /**
   * Creates a new occupancy mapper with default file path and grid dimensions.
   */
  public OccupancyMapper() {
    this("xrp-map.json", 120.0, 96.0);
  }

  /**
   * Creates a new occupancy mapper.
   * @param mapFilePath  Path to save/load map data
   * @param worldWidth   World width in inches (for grid sizing only)
   * @param worldHeight  World height in inches (for grid sizing only)
   */
  public OccupancyMapper(String mapFilePath, double worldWidth, double worldHeight) {
    this.grid = new OccupancyGrid(worldWidth, worldHeight, GRID_RESOLUTION);
    this.poseHistory = new ArrayList<>();
    this.mapFilePath = mapFilePath;
    this.lastSaveTime = System.currentTimeMillis();
    this.updatesSinceLastSave = 0;
    this.consistentReadingsCount = 0;
    this.lastDriftError = 0.0;

    loadMap();
  }

  /**
   * Updates the occupancy grid with a new ultrasonic reading.
   *
   * @param range      Ultrasonic range measurement (inches)
   * @param robotX     Current robot X position (inches)
   * @param robotY     Current robot Y position (inches)
   * @param robotTheta Current robot heading (radians)
   * @return true if the grid was updated, false if the reading was invalid
   */
  public boolean addPoint(double range, double robotX, double robotY, double robotTheta) {
    if (range < MIN_VALID_RANGE || range > MAX_VALID_RANGE) {
      return false;
    }

    grid.update(range, robotX, robotY, robotTheta);
    poseHistory.add(new double[] {robotX, robotY, robotTheta, System.currentTimeMillis()});
    updatesSinceLastSave++;

    // Auto-save periodically
    long currentTime = System.currentTimeMillis();
    if (currentTime - lastSaveTime >= SAVE_INTERVAL && updatesSinceLastSave > 0) {
      saveMap();
    }

    return true;
  }

  /**
   * Checks for drift and computes a correction if needed.
   * Compares the current ultrasonic reading against the predicted range from
   * the <em>learned</em> occupancy grid (not a ground-truth world model).
   *
   * @param currentRange Current ultrasonic range (inches)
   * @param robotX       Current robot X position (inches)
   * @param robotY       Current robot Y position (inches)
   * @param robotTheta   Current robot heading (radians)
   * @return Correction vector [dx, dy] or null if no correction needed
   */
  public double[] checkDriftCorrection(double currentRange, double robotX, double robotY, double robotTheta) {
    if (currentRange < MIN_VALID_RANGE || currentRange > MAX_VALID_RANGE) {
      return null;
    }

    // Predict expected range by ray-marching through the learned grid
    double expectedRange = grid.predictRange(robotX, robotY, robotTheta);
    if (expectedRange < 0) {
      return null; // No occupied cell found in this direction
    }

    double error = currentRange - expectedRange;

    if (Math.abs(error) < DRIFT_CORRECTION_THRESHOLD) {
      consistentReadingsCount = 0;
      return null;
    }

    // Check for consistency across multiple frames
    if (Math.abs(error - lastDriftError) < 0.5) {
      consistentReadingsCount++;
    } else {
      consistentReadingsCount = 1;
    }
    lastDriftError = error;

    if (consistentReadingsCount >= CONSISTENCY_REQUIRED) {
      double correctionMagnitude = Math.min(Math.abs(error) * 0.1, MAX_CORRECTION_PER_CYCLE);
      if (error < 0) {
        correctionMagnitude = -correctionMagnitude;
      }

      double dx = correctionMagnitude * Math.cos(robotTheta);
      double dy = correctionMagnitude * Math.sin(robotTheta);

      consistentReadingsCount = 0;
      return new double[] {dx, dy};
    }

    return null;
  }

  /**
   * Gets the occupied cells from the grid as {@link MapPoint}s for
   * visualization compatibility with the existing API.
   *
   * @return List of map points representing occupied cell centers
   */
  public List<MapPoint> getMapPoints() {
    double[][] cells = grid.getOccupiedCells(OCCUPIED_THRESHOLD);
    List<MapPoint> points = new ArrayList<>(cells.length);
    for (double[] cell : cells) {
      points.add(new MapPoint(cell[0], cell[1], System.currentTimeMillis(), 0, 0, 0));
    }
    return points;
  }

  /**
   * Gets the underlying occupancy grid.
   */
  public OccupancyGrid getGrid() {
    return grid;
  }

  /**
   * Clears the occupancy grid, pose history, and deletes the map file.
   */
  public void clearMap() {
    grid.reset();
    poseHistory.clear();
    updatesSinceLastSave = 0;
    try {
      Files.deleteIfExists(Paths.get(mapFilePath));
    } catch (IOException e) {
      System.err.println("Failed to delete map file: " + e.getMessage());
    }
  }

  /**
   * Saves the occupancy grid and metadata to a JSON file.
   * <p>
   * Saved data includes:
   * <ul>
   *   <li>Occupied cell list (x, y world coordinates)</li>
   *   <li>Grid resolution and origin</li>
   *   <li>Timestamp</li>
   *   <li>Pose history</li>
   * </ul>
   */
  public void saveMap() {
    try (FileWriter writer = new FileWriter(mapFilePath)) {
      writer.write("{\n");
      writer.write("  \"timestamp\": " + System.currentTimeMillis() + ",\n");
      writer.write("  \"resolution\": " + grid.getResolution() + ",\n");
      writer.write("  \"originX\": " + grid.getOriginX() + ",\n");
      writer.write("  \"originY\": " + grid.getOriginY() + ",\n");

      // Write occupied cells
      double[][] cells = grid.getOccupiedCells(OCCUPIED_THRESHOLD);
      writer.write("  \"occupiedCells\": [\n");
      for (int i = 0; i < cells.length; i++) {
        writer.write("    {\"x\": " + cells[i][0] + ", \"y\": " + cells[i][1] + "}");
        if (i < cells.length - 1) {
          writer.write(",");
        }
        writer.write("\n");
      }
      writer.write("  ],\n");

      // Write pose history (last 500 entries max)
      int start = Math.max(0, poseHistory.size() - 500);
      writer.write("  \"poseHistory\": [\n");
      for (int i = start; i < poseHistory.size(); i++) {
        double[] p = poseHistory.get(i);
        writer.write("    {\"x\": " + p[0] + ", \"y\": " + p[1]
            + ", \"theta\": " + p[2] + ", \"t\": " + (long) p[3] + "}");
        if (i < poseHistory.size() - 1) {
          writer.write(",");
        }
        writer.write("\n");
      }
      writer.write("  ]\n");

      writer.write("}\n");

      lastSaveTime = System.currentTimeMillis();
      updatesSinceLastSave = 0;
    } catch (IOException e) {
      System.err.println("Failed to save map: " + e.getMessage());
    }
  }

  /**
   * Loads an occupancy grid from a previously saved JSON file.
   * Only the occupied cell list is restored; full log-odds values are
   * approximated since the save format stores cell positions, not raw
   * log-odds.
   */
  private void loadMap() {
    try {
      if (!Files.exists(Paths.get(mapFilePath))) {
        return;
      }

      String content = new String(Files.readAllBytes(Paths.get(mapFilePath)));

      // Look for occupiedCells array
      int cellsStart = content.indexOf("\"occupiedCells\"");
      if (cellsStart < 0) {
        // Try legacy "points" format
        loadLegacyMap(content);
        return;
      }

      int arrStart = content.indexOf("[", cellsStart);
      int arrEnd = findMatchingBracket(content, arrStart);
      if (arrStart < 0 || arrEnd < 0) {
        return;
      }

      String cellsJson = content.substring(arrStart + 1, arrEnd);
      String[] cellObjects = cellsJson.split("\\},\\s*\\{");
      int loadedCount = 0;
      for (String cellObj : cellObjects) {
        try {
          double x = extractJsonValue(cellObj, "x");
          double y = extractJsonValue(cellObj, "y");
          // Mark these cells as moderately occupied in the grid
          grid.update(0, x, y, 0); // minimal update to seed the cell
          loadedCount++;
        } catch (Exception e) {
          // Skip malformed entries
        }
      }

      System.out.println("Loaded " + loadedCount + " occupied cells from " + mapFilePath);
    } catch (IOException e) {
      System.err.println("Failed to load map: " + e.getMessage());
    }
  }

  /**
   * Loads the legacy point-cloud map format for backwards compatibility.
   */
  private void loadLegacyMap(String content) {
    int pointsStart = content.indexOf("\"points\":");
    if (pointsStart < 0) {
      return;
    }
    int arrayStart = content.indexOf("[", pointsStart);
    int arrayEnd = content.lastIndexOf("]");
    if (arrayStart < 0 || arrayEnd < 0) {
      return;
    }

    String pointsJson = content.substring(arrayStart + 1, arrayEnd);
    String[] pointObjects = pointsJson.split("\\},\\s*\\{");
    int loadedCount = 0;
    for (String pointObj : pointObjects) {
      try {
        double x = extractJsonValue(pointObj, "x");
        double y = extractJsonValue(pointObj, "y");
        grid.update(0, x, y, 0);
        loadedCount++;
      } catch (Exception e) {
        // Skip malformed entries
      }
    }
    System.out.println("Loaded " + loadedCount + " legacy map points from " + mapFilePath);
  }

  /**
   * Finds the matching closing bracket for an opening bracket.
   */
  private int findMatchingBracket(String s, int openIdx) {
    if (openIdx < 0 || openIdx >= s.length()) {
      return -1;
    }
    int depth = 0;
    for (int i = openIdx; i < s.length(); i++) {
      char c = s.charAt(i);
      if (c == '[') {
        depth++;
      } else if (c == ']') {
        depth--;
        if (depth == 0) {
          return i;
        }
      }
    }
    return -1;
  }

  /**
   * Extracts a numeric value from a JSON string.
   */
  private double extractJsonValue(String json, String key) {
    String searchKey = "\"" + key + "\":";
    int start = json.indexOf(searchKey);
    if (start < 0) {
      return 0.0;
    }

    start += searchKey.length();
    int end = json.indexOf(",", start);
    if (end < 0) {
      end = json.indexOf("}", start);
    }
    if (end < 0) {
      end = json.length();
    }

    String valueStr = json.substring(start, end).trim();
    return Double.parseDouble(valueStr);
  }
}
