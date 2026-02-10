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
 * Manages ultrasonic-based occupancy mapping.
 * Converts sensor readings to world-frame obstacle points and maintains a persistent map.
 */
public class OccupancyMapper {
  /** Sensor offset from robot center (inches) */
  private static final double SENSOR_OFFSET = 2.0;
  
  /** Maximum valid ultrasonic range (inches) */
  private static final double MAX_VALID_RANGE = 157.0;
  
  /** Minimum valid ultrasonic range (inches) */
  private static final double MIN_VALID_RANGE = 1.0;
  
  /** Distance threshold for merging nearby points (inches) */
  private static final double MERGE_THRESHOLD = 0.5;
  
  /** Auto-save interval (milliseconds) */
  private static final long SAVE_INTERVAL = 5000;
  
  /** Drift correction threshold (inches) */
  private static final double DRIFT_CORRECTION_THRESHOLD = 2.0;
  
  /** Minimum map points required for drift correction */
  private static final int MIN_POINTS_FOR_CORRECTION = 10;
  
  /** Maximum correction per cycle (inches) */
  private static final double MAX_CORRECTION_PER_CYCLE = 0.5;
  
  /** Number of consistent readings required for drift correction */
  private static final int CONSISTENCY_REQUIRED = 3;

  private final List<MapPoint> mapPoints;
  private final String mapFilePath;
  private long lastSaveTime;
  private int pointsSinceLastSave;
  
  // Drift correction state
  private int consistentReadingsCount;
  private double lastDriftError;

  /**
   * Creates a new occupancy mapper with default file path.
   */
  public OccupancyMapper() {
    this("xrp-map.json");
  }

  /**
   * Creates a new occupancy mapper with specified file path.
   * @param mapFilePath Path to save/load map data
   */
  public OccupancyMapper(String mapFilePath) {
    this.mapPoints = new ArrayList<>();
    this.mapFilePath = mapFilePath;
    this.lastSaveTime = System.currentTimeMillis();
    this.pointsSinceLastSave = 0;
    this.consistentReadingsCount = 0;
    this.lastDriftError = 0.0;
    
    loadMap();
  }

  /**
   * Adds a new map point from an ultrasonic reading.
   * Converts the reading from robot frame to world frame.
   * 
   * @param range Ultrasonic range measurement (inches)
   * @param robotX Current robot X position (inches)
   * @param robotY Current robot Y position (inches)
   * @param robotTheta Current robot heading (radians)
   * @return true if a valid point was added, false otherwise
   */
  public boolean addPoint(double range, double robotX, double robotY, double robotTheta) {
    // Filter invalid readings
    if (range < MIN_VALID_RANGE || range > MAX_VALID_RANGE) {
      return false;
    }

    // Convert to world frame
    // Obstacle is at: sensor_position + range * heading_direction
    double sensorX = robotX + SENSOR_OFFSET * Math.cos(robotTheta);
    double sensorY = robotY + SENSOR_OFFSET * Math.sin(robotTheta);
    
    double obstacleX = sensorX + range * Math.cos(robotTheta);
    double obstacleY = sensorY + range * Math.sin(robotTheta);
    
    // Check if a similar point already exists (merge threshold)
    for (MapPoint existing : mapPoints) {
      if (existing.distanceTo(obstacleX, obstacleY) < MERGE_THRESHOLD) {
        return false; // Point already exists
      }
    }

    // Create and add new point
    MapPoint newPoint = new MapPoint(
      obstacleX, obstacleY,
      System.currentTimeMillis(),
      robotX, robotY, robotTheta
    );
    mapPoints.add(newPoint);
    pointsSinceLastSave++;

    // Auto-save if enough time has passed
    long currentTime = System.currentTimeMillis();
    if (currentTime - lastSaveTime >= SAVE_INTERVAL && pointsSinceLastSave > 0) {
      saveMap();
    }

    return true;
  }

  /**
   * Checks for drift and computes a correction if needed.
   * Compares current ultrasonic reading against expected range from the map.
   * 
   * @param currentRange Current ultrasonic range (inches)
   * @param robotX Current robot X position (inches)
   * @param robotY Current robot Y position (inches)
   * @param robotTheta Current robot heading (radians)
   * @return Correction vector [dx, dy] or null if no correction needed
   */
  public double[] checkDriftCorrection(double currentRange, double robotX, double robotY, double robotTheta) {
    // Need enough map points
    if (mapPoints.size() < MIN_POINTS_FOR_CORRECTION) {
      return null;
    }

    // Invalid reading
    if (currentRange < MIN_VALID_RANGE || currentRange > MAX_VALID_RANGE) {
      return null;
    }

    // Find expected range from map
    double expectedRange = getExpectedRange(robotX, robotY, robotTheta);
    if (expectedRange < 0) {
      return null; // No map data in this direction
    }

    // Compute error
    double error = currentRange - expectedRange;
    
    // Check if error exceeds threshold
    if (Math.abs(error) < DRIFT_CORRECTION_THRESHOLD) {
      consistentReadingsCount = 0;
      return null;
    }

    // Check for consistency
    if (Math.abs(error - lastDriftError) < 0.5) {
      consistentReadingsCount++;
    } else {
      consistentReadingsCount = 1;
    }
    lastDriftError = error;

    // Apply correction if consistent
    if (consistentReadingsCount >= CONSISTENCY_REQUIRED) {
      // Compute correction in robot heading direction
      double correctionMagnitude = Math.min(Math.abs(error) * 0.1, MAX_CORRECTION_PER_CYCLE);
      if (error < 0) correctionMagnitude = -correctionMagnitude;
      
      double dx = correctionMagnitude * Math.cos(robotTheta);
      double dy = correctionMagnitude * Math.sin(robotTheta);
      
      consistentReadingsCount = 0; // Reset after applying correction
      return new double[] {dx, dy};
    }

    return null;
  }

  /**
   * Gets the expected ultrasonic range based on the current map.
   * Looks for map points roughly in the direction the sensor is pointing.
   * 
   * @param robotX Current robot X position
   * @param robotY Current robot Y position
   * @param robotTheta Current robot heading
   * @return Expected range, or -1 if no map data available
   */
  private double getExpectedRange(double robotX, double robotY, double robotTheta) {
    double sensorX = robotX + SENSOR_OFFSET * Math.cos(robotTheta);
    double sensorY = robotY + SENSOR_OFFSET * Math.sin(robotTheta);
    
    double minDistance = Double.MAX_VALUE;
    boolean found = false;
    
    // Look for points within a cone in the sensor direction
    for (MapPoint point : mapPoints) {
      double dx = point.x - sensorX;
      double dy = point.y - sensorY;
      double distance = Math.sqrt(dx * dx + dy * dy);
      
      // Check if point is in the general direction we're looking
      double angleToPoint = Math.atan2(dy, dx);
      double angleDiff = Math.abs(normalizeAngle(angleToPoint - robotTheta));
      
      // Within 30 degree cone
      if (angleDiff < Math.toRadians(30) && distance < minDistance) {
        minDistance = distance;
        found = true;
      }
    }
    
    return found ? minDistance : -1.0;
  }

  /**
   * Normalizes an angle to [-π, π].
   */
  private double normalizeAngle(double angle) {
    while (angle > Math.PI) angle -= 2.0 * Math.PI;
    while (angle < -Math.PI) angle += 2.0 * Math.PI;
    return angle;
  }

  /**
   * Gets all map points.
   * @return List of map points
   */
  public List<MapPoint> getMapPoints() {
    return new ArrayList<>(mapPoints);
  }

  /**
   * Clears all map points and deletes the map file.
   */
  public void clearMap() {
    mapPoints.clear();
    pointsSinceLastSave = 0;
    try {
      Files.deleteIfExists(Paths.get(mapFilePath));
    } catch (IOException e) {
      System.err.println("Failed to delete map file: " + e.getMessage());
    }
  }

  /**
   * Saves the map to a JSON file.
   */
  public void saveMap() {
    try (FileWriter writer = new FileWriter(mapFilePath)) {
      writer.write("{\n");
      writer.write("  \"timestamp\": \"" + System.currentTimeMillis() + "\",\n");
      writer.write("  \"points\": [\n");
      
      for (int i = 0; i < mapPoints.size(); i++) {
        MapPoint p = mapPoints.get(i);
        writer.write("    {\n");
        writer.write("      \"x\": " + p.x + ",\n");
        writer.write("      \"y\": " + p.y + ",\n");
        writer.write("      \"t\": " + p.timestamp + ",\n");
        writer.write("      \"poseX\": " + p.poseX + ",\n");
        writer.write("      \"poseY\": " + p.poseY + ",\n");
        writer.write("      \"poseTheta\": " + p.poseTheta + "\n");
        writer.write("    }");
        if (i < mapPoints.size() - 1) {
          writer.write(",");
        }
        writer.write("\n");
      }
      
      writer.write("  ]\n");
      writer.write("}\n");
      
      lastSaveTime = System.currentTimeMillis();
      pointsSinceLastSave = 0;
    } catch (IOException e) {
      System.err.println("Failed to save map: " + e.getMessage());
    }
  }

  /**
   * Loads the map from a JSON file.
   */
  private void loadMap() {
    try {
      if (!Files.exists(Paths.get(mapFilePath))) {
        return;
      }

      String content = new String(Files.readAllBytes(Paths.get(mapFilePath)));
      
      // Simple JSON parsing (no external library)
      // Extract points array
      int pointsStart = content.indexOf("\"points\":");
      if (pointsStart < 0) return;
      
      int arrayStart = content.indexOf("[", pointsStart);
      int arrayEnd = content.lastIndexOf("]");
      if (arrayStart < 0 || arrayEnd < 0) return;
      
      String pointsJson = content.substring(arrayStart + 1, arrayEnd);
      
      // Parse each point object
      String[] pointObjects = pointsJson.split("\\},\\s*\\{");
      for (String pointObj : pointObjects) {
        try {
          double x = extractJsonValue(pointObj, "x");
          double y = extractJsonValue(pointObj, "y");
          long t = (long) extractJsonValue(pointObj, "t");
          double poseX = extractJsonValue(pointObj, "poseX");
          double poseY = extractJsonValue(pointObj, "poseY");
          double poseTheta = extractJsonValue(pointObj, "poseTheta");
          
          mapPoints.add(new MapPoint(x, y, t, poseX, poseY, poseTheta));
        } catch (Exception e) {
          // Skip malformed points
        }
      }
      
      System.out.println("Loaded " + mapPoints.size() + " map points from " + mapFilePath);
    } catch (IOException e) {
      System.err.println("Failed to load map: " + e.getMessage());
    }
  }

  /**
   * Extracts a numeric value from a JSON string.
   */
  private double extractJsonValue(String json, String key) {
    String searchKey = "\"" + key + "\":";
    int start = json.indexOf(searchKey);
    if (start < 0) return 0.0;
    
    start += searchKey.length();
    int end = json.indexOf(",", start);
    if (end < 0) end = json.indexOf("}", start);
    if (end < 0) end = json.length();
    
    String valueStr = json.substring(start, end).trim();
    return Double.parseDouble(valueStr);
  }
}
