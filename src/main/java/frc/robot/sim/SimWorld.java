// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Simulated world environment for the XRP robot.
 * <p>
 * This is the <b>ground-truth</b> world used only to generate simulated sensor
 * readings.  The robot mapping / localization code must <b>never</b> access this
 * class or the world definition file it reads.
 * <p>
 * The world can be loaded from an external JSON file (preferred), generated
 * procedurally from a random seed, or created with default geometry.
 */
public class SimWorld {
  /** Standard deviation of ultrasonic measurement noise (inches) */
  private static final double MEASUREMENT_NOISE_STDDEV = 0.5;

  /** Maximum valid ultrasonic range (inches) */
  private static final double MAX_RANGE = 157.0;

  /** Minimum valid ultrasonic range (inches) */
  private static final double MIN_RANGE = 1.0;

  private final Random random;
  private final List<Obstacle> obstacles;
  private double roomWidth;
  private double roomHeight;

  /**
   * Represents a rectangular obstacle in the world.
   */
  private static class Obstacle {
    double minX, minY, maxX, maxY;

    Obstacle(double minX, double minY, double maxX, double maxY) {
      this.minX = minX;
      this.minY = minY;
      this.maxX = maxX;
      this.maxY = maxY;
    }
  }

  // ---- Constructors -------------------------------------------------------

  /**
   * Creates a simulated world by loading geometry from a JSON file.
   * Falls back to procedural generation if the file cannot be read.
   *
   * @param worldFilePath Path to the world-definition JSON file
   */
  public SimWorld(String worldFilePath) {
    random = new Random();
    obstacles = new ArrayList<>();
    if (!loadFromFile(worldFilePath)) {
      System.err.println("SimWorld: failed to load '" + worldFilePath
          + "', falling back to procedural generation (seed 42)");
      generateProcedural(42);
    }
  }

  /**
   * Creates a simulated world using procedural generation.
   *
   * @param seed Random seed for reproducible generation
   */
  public SimWorld(long seed) {
    random = new Random();
    obstacles = new ArrayList<>();
    generateProcedural(seed);
  }

  /**
   * Default constructor &mdash; loads from {@code world.json} in the working
   * directory, or falls back to procedural generation with seed&nbsp;42.
   */
  public SimWorld() {
    this("world.json");
  }

  // ---- World loading ------------------------------------------------------

  /**
   * Loads room dimensions and obstacles from a JSON file.
   *
   * <p>Expected format:
   * <pre>{@code
   * {
   *   "room": { "width": 120.0, "height": 96.0 },
   *   "obstacles": [
   *     { "minX": 30, "minY": 35, "maxX": 40, "maxY": 45 },
   *     ...
   *   ]
   * }
   * }</pre>
   *
   * @return {@code true} if loading succeeded
   */
  private boolean loadFromFile(String path) {
    try {
      if (!Files.exists(Paths.get(path))) {
        return false;
      }
      String content = new String(Files.readAllBytes(Paths.get(path)), StandardCharsets.UTF_8);

      // Parse room dimensions
      roomWidth = extractNestedJsonValue(content, "room", "width", 120.0);
      roomHeight = extractNestedJsonValue(content, "room", "height", 96.0);

      // Parse obstacles array
      int obsStart = content.indexOf("\"obstacles\"");
      if (obsStart < 0) {
        return true; // Room with no obstacles is valid
      }
      int arrStart = content.indexOf("[", obsStart);
      int arrEnd = content.indexOf("]", arrStart);
      if (arrStart < 0 || arrEnd < 0) {
        return true;
      }
      String obsJson = content.substring(arrStart + 1, arrEnd);
      String[] objs = obsJson.split("\\},\\s*\\{");
      for (String obj : objs) {
        try {
          double minX = extractJsonValue(obj, "minX");
          double minY = extractJsonValue(obj, "minY");
          double maxX = extractJsonValue(obj, "maxX");
          double maxY = extractJsonValue(obj, "maxY");
          obstacles.add(new Obstacle(minX, minY, maxX, maxY));
        } catch (NumberFormatException e) {
          // skip malformed obstacle
        }
      }

      System.out.println("SimWorld: loaded world from '" + path + "' – "
          + roomWidth + "×" + roomHeight + " in, " + obstacles.size() + " obstacles");
      return true;
    } catch (IOException e) {
      return false;
    }
  }

  /**
   * Procedurally generates a room with random obstacles using the given seed.
   */
  private void generateProcedural(long seed) {
    Random gen = new Random(seed);
    roomWidth = 120.0;
    roomHeight = 96.0;

    int numObstacles = 2 + gen.nextInt(4); // 2–5 obstacles
    for (int i = 0; i < numObstacles; i++) {
      double w = 5 + gen.nextDouble() * 15;  // 5–20 wide
      double h = 5 + gen.nextDouble() * 15;  // 5–20 tall
      double ox = 10 + gen.nextDouble() * (roomWidth - w - 20);
      double oy = 10 + gen.nextDouble() * (roomHeight - h - 20);
      obstacles.add(new Obstacle(ox, oy, ox + w, oy + h));
    }
    System.out.println("SimWorld: procedurally generated " + obstacles.size()
        + " obstacles (seed=" + seed + ")");
  }

  // ---- Simple JSON helpers (no external library) --------------------------

  private double extractJsonValue(String json, String key) {
    String searchKey = "\"" + key + "\"";
    int idx = json.indexOf(searchKey);
    if (idx < 0) {
      return 0.0;
    }
    int colon = json.indexOf(":", idx + searchKey.length());
    if (colon < 0) {
      return 0.0;
    }
    int start = colon + 1;
    int end = start;
    while (end < json.length()) {
      char c = json.charAt(end);
      if (c == ',' || c == '}' || c == ']') {
        break;
      }
      end++;
    }
    return Double.parseDouble(json.substring(start, end).trim());
  }

  private double extractNestedJsonValue(String json, String outerKey, String innerKey, double defaultVal) {
    String outerSearch = "\"" + outerKey + "\"";
    int outerIdx = json.indexOf(outerSearch);
    if (outerIdx < 0) {
      return defaultVal;
    }
    int braceStart = json.indexOf("{", outerIdx);
    if (braceStart < 0) {
      return defaultVal;
    }
    int braceEnd = json.indexOf("}", braceStart);
    if (braceEnd < 0) {
      return defaultVal;
    }
    String inner = json.substring(braceStart, braceEnd + 1);
    try {
      return extractJsonValue(inner, innerKey);
    } catch (NumberFormatException e) {
      return defaultVal;
    }
  }

  // ---- Accessors ----------------------------------------------------------

  /**
   * Gets the room width.
   * @return Room width in inches
   */
  public double getRoomWidth() {
    return roomWidth;
  }

  /**
   * Gets the room height.
   * @return Room height in inches
   */
  public double getRoomHeight() {
    return roomHeight;
  }

  /**
   * Gets the list of obstacles for visualization (ground-truth display only).
   * <p><b>Robot mapping code must NOT call this.</b>
   * @return List of obstacle boundaries [minX, minY, maxX, maxY]
   */
  public List<double[]> getObstacles() {
    List<double[]> result = new ArrayList<>();
    for (Obstacle obs : obstacles) {
      result.add(new double[] {obs.minX, obs.minY, obs.maxX, obs.maxY});
    }
    return result;
  }

  // ---- Sensor simulation --------------------------------------------------

  /**
   * Simulates an ultrasonic range measurement via ray casting.
   *
   * @param robotX Robot X position (inches)
   * @param robotY Robot Y position (inches)
   * @param robotTheta Robot heading (radians)
   * @param sensorOffset Sensor offset from robot center along heading direction (inches)
   * @return Simulated range in inches, or MAX_RANGE if no intersection
   */
  public double simulateUltrasonicReading(double robotX, double robotY, double robotTheta, double sensorOffset) {
    // Compute sensor position
    double sensorX = robotX + sensorOffset * Math.cos(robotTheta);
    double sensorY = robotY + sensorOffset * Math.sin(robotTheta);

    // Ray direction
    double dirX = Math.cos(robotTheta);
    double dirY = Math.sin(robotTheta);

    // Find closest intersection
    double minDistance = MAX_RANGE;

    // Check room walls
    minDistance = Math.min(minDistance, raycastToWalls(sensorX, sensorY, dirX, dirY));

    // Check obstacles
    for (Obstacle obs : obstacles) {
      double dist = raycastToObstacle(sensorX, sensorY, dirX, dirY, obs);
      minDistance = Math.min(minDistance, dist);
    }

    // Add measurement noise
    double noise = random.nextGaussian() * MEASUREMENT_NOISE_STDDEV;
    double measurement = minDistance + noise;

    // Clamp to valid range
    if (measurement < MIN_RANGE || measurement > MAX_RANGE) {
      return MAX_RANGE;
    }

    return measurement;
  }

  /**
   * Ray casts to the room walls.
   * @return Distance to closest wall intersection
   */
  private double raycastToWalls(double x, double y, double dirX, double dirY) {
    double minDist = MAX_RANGE;

    // Left wall (x = 0)
    if (dirX < 0) {
      double t = -x / dirX;
      if (t > 0) {
        double intersectY = y + t * dirY;
        if (intersectY >= 0 && intersectY <= roomHeight) {
          minDist = Math.min(minDist, t);
        }
      }
    }

    // Right wall (x = roomWidth)
    if (dirX > 0) {
      double t = (roomWidth - x) / dirX;
      if (t > 0) {
        double intersectY = y + t * dirY;
        if (intersectY >= 0 && intersectY <= roomHeight) {
          minDist = Math.min(minDist, t);
        }
      }
    }

    // Bottom wall (y = 0)
    if (dirY < 0) {
      double t = -y / dirY;
      if (t > 0) {
        double intersectX = x + t * dirX;
        if (intersectX >= 0 && intersectX <= roomWidth) {
          minDist = Math.min(minDist, t);
        }
      }
    }

    // Top wall (y = roomHeight)
    if (dirY > 0) {
      double t = (roomHeight - y) / dirY;
      if (t > 0) {
        double intersectX = x + t * dirX;
        if (intersectX >= 0 && intersectX <= roomWidth) {
          minDist = Math.min(minDist, t);
        }
      }
    }

    return minDist;
  }

  /**
   * Ray casts to a rectangular obstacle.
   * @return Distance to closest intersection with the obstacle
   */
  private double raycastToObstacle(double x, double y, double dirX, double dirY, Obstacle obs) {
    double minDist = MAX_RANGE;

    // Left edge (x = minX)
    if (Math.abs(dirX) > 1e-10) {
      double t = (obs.minX - x) / dirX;
      if (t > 0) {
        double intersectY = y + t * dirY;
        if (intersectY >= obs.minY && intersectY <= obs.maxY) {
          minDist = Math.min(minDist, t);
        }
      }
    }

    // Right edge (x = maxX)
    if (Math.abs(dirX) > 1e-10) {
      double t = (obs.maxX - x) / dirX;
      if (t > 0) {
        double intersectY = y + t * dirY;
        if (intersectY >= obs.minY && intersectY <= obs.maxY) {
          minDist = Math.min(minDist, t);
        }
      }
    }

    // Bottom edge (y = minY)
    if (Math.abs(dirY) > 1e-10) {
      double t = (obs.minY - y) / dirY;
      if (t > 0) {
        double intersectX = x + t * dirX;
        if (intersectX >= obs.minX && intersectX <= obs.maxX) {
          minDist = Math.min(minDist, t);
        }
      }
    }

    // Top edge (y = maxY)
    if (Math.abs(dirY) > 1e-10) {
      double t = (obs.maxY - y) / dirY;
      if (t > 0) {
        double intersectX = x + t * dirX;
        if (intersectX >= obs.minX && intersectX <= obs.maxX) {
          minDist = Math.min(minDist, t);
        }
      }
    }

    return minDist;
  }
}
