// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

/**
 * Represents a single obstacle point detected by the ultrasonic sensor.
 * Stores the world-frame coordinates, timestamp, and the robot pose at capture time.
 */
public class MapPoint {
  /** X coordinate in world frame (inches) */
  public final double x;
  
  /** Y coordinate in world frame (inches) */
  public final double y;
  
  /** Timestamp when this point was captured (milliseconds) */
  public final long timestamp;
  
  /** Robot X position when this point was captured (inches) */
  public final double poseX;
  
  /** Robot Y position when this point was captured (inches) */
  public final double poseY;
  
  /** Robot heading when this point was captured (radians) */
  public final double poseTheta;

  /**
   * Creates a new map point.
   * @param x World-frame X coordinate (inches)
   * @param y World-frame Y coordinate (inches)
   * @param timestamp Time of capture (milliseconds)
   * @param poseX Robot X position at capture (inches)
   * @param poseY Robot Y position at capture (inches)
   * @param poseTheta Robot heading at capture (radians)
   */
  public MapPoint(double x, double y, long timestamp, double poseX, double poseY, double poseTheta) {
    this.x = x;
    this.y = y;
    this.timestamp = timestamp;
    this.poseX = poseX;
    this.poseY = poseY;
    this.poseTheta = poseTheta;
  }

  /**
   * Computes the distance between this point and another point.
   * @param other The other point
   * @return The Euclidean distance in inches
   */
  public double distanceTo(MapPoint other) {
    double dx = this.x - other.x;
    double dy = this.y - other.y;
    return Math.sqrt(dx * dx + dy * dy);
  }

  /**
   * Computes the distance between this point and a coordinate.
   * @param x The X coordinate
   * @param y The Y coordinate
   * @return The Euclidean distance in inches
   */
  public double distanceTo(double x, double y) {
    double dx = this.x - x;
    double dy = this.y - y;
    return Math.sqrt(dx * dx + dy * dy);
  }

  @Override
  public String toString() {
    return String.format("MapPoint(x=%.2f, y=%.2f, t=%d)", x, y, timestamp);
  }
}
