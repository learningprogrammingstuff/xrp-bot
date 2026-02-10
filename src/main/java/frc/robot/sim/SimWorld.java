// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Simulated world environment for the XRP robot.
 * Defines a rectangular room with obstacles and provides ray casting for simulated ultrasonic readings.
 */
public class SimWorld {
  /** Room width in inches (default 120 inches = 10 feet) */
  private static final double ROOM_WIDTH = 120.0;
  
  /** Room height in inches (default 96 inches = 8 feet) */
  private static final double ROOM_HEIGHT = 96.0;
  
  /** Standard deviation of ultrasonic measurement noise (inches) */
  private static final double MEASUREMENT_NOISE_STDDEV = 0.5;
  
  /** Maximum valid ultrasonic range (inches) */
  private static final double MAX_RANGE = 157.0;
  
  /** Minimum valid ultrasonic range (inches) */
  private static final double MIN_RANGE = 1.0;

  private final Random random;
  private final List<Obstacle> obstacles;

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

  /**
   * Creates a new simulated world with default obstacles.
   */
  public SimWorld() {
    random = new Random();
    obstacles = new ArrayList<>();
    
    // Add a few rectangular obstacles in the room
    // Obstacle 1: Small box in the center-left
    obstacles.add(new Obstacle(30, 35, 40, 45));
    
    // Obstacle 2: Vertical wall on the right side
    obstacles.add(new Obstacle(90, 20, 95, 70));
    
    // Obstacle 3: Small box in the lower right
    obstacles.add(new Obstacle(70, 70, 80, 80));
  }

  /**
   * Gets the room width.
   * @return Room width in inches
   */
  public double getRoomWidth() {
    return ROOM_WIDTH;
  }

  /**
   * Gets the room height.
   * @return Room height in inches
   */
  public double getRoomHeight() {
    return ROOM_HEIGHT;
  }

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
        if (intersectY >= 0 && intersectY <= ROOM_HEIGHT) {
          minDist = Math.min(minDist, t);
        }
      }
    }
    
    // Right wall (x = ROOM_WIDTH)
    if (dirX > 0) {
      double t = (ROOM_WIDTH - x) / dirX;
      if (t > 0) {
        double intersectY = y + t * dirY;
        if (intersectY >= 0 && intersectY <= ROOM_HEIGHT) {
          minDist = Math.min(minDist, t);
        }
      }
    }
    
    // Bottom wall (y = 0)
    if (dirY < 0) {
      double t = -y / dirY;
      if (t > 0) {
        double intersectX = x + t * dirX;
        if (intersectX >= 0 && intersectX <= ROOM_WIDTH) {
          minDist = Math.min(minDist, t);
        }
      }
    }
    
    // Top wall (y = ROOM_HEIGHT)
    if (dirY > 0) {
      double t = (ROOM_HEIGHT - y) / dirY;
      if (t > 0) {
        double intersectX = x + t * dirX;
        if (intersectX >= 0 && intersectX <= ROOM_WIDTH) {
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
    
    // Left edge (x = minX) - check all rays moving in positive X direction
    if (Math.abs(dirX) > 1e-10) {
      double t = (obs.minX - x) / dirX;
      if (t > 0) {
        double intersectY = y + t * dirY;
        if (intersectY >= obs.minY && intersectY <= obs.maxY) {
          minDist = Math.min(minDist, t);
        }
      }
    }
    
    // Right edge (x = maxX) - check all rays
    if (Math.abs(dirX) > 1e-10) {
      double t = (obs.maxX - x) / dirX;
      if (t > 0) {
        double intersectY = y + t * dirY;
        if (intersectY >= obs.minY && intersectY <= obs.maxY) {
          minDist = Math.min(minDist, t);
        }
      }
    }
    
    // Bottom edge (y = minY) - check all rays moving in positive Y direction
    if (Math.abs(dirY) > 1e-10) {
      double t = (obs.minY - y) / dirY;
      if (t > 0) {
        double intersectX = x + t * dirX;
        if (intersectX >= obs.minX && intersectX <= obs.maxX) {
          minDist = Math.min(minDist, t);
        }
      }
    }
    
    // Top edge (y = maxY) - check all rays
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

  /**
   * Gets the list of obstacles for visualization.
   * @return List of obstacle boundaries [minX, minY, maxX, maxY]
   */
  public List<double[]> getObstacles() {
    List<double[]> result = new ArrayList<>();
    for (Obstacle obs : obstacles) {
      result.add(new double[] {obs.minX, obs.minY, obs.maxX, obs.maxY});
    }
    return result;
  }
}
