// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

/**
 * Minimal 3x3 matrix math utilities for EKF implementation.
 * Provides basic operations needed for Extended Kalman Filter calculations.
 */
public class Matrix3x3 {
  private double[][] data;

  /**
   * Creates a new 3x3 matrix with all zeros.
   */
  public Matrix3x3() {
    data = new double[3][3];
  }

  /**
   * Creates a new 3x3 matrix from a 2D array.
   * @param values The values to initialize the matrix with
   */
  public Matrix3x3(double[][] values) {
    if (values.length != 3 || values[0].length != 3) {
      throw new IllegalArgumentException("Matrix must be 3x3");
    }
    data = new double[3][3];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        data[i][j] = values[i][j];
      }
    }
  }

  /**
   * Creates an identity matrix (1s on diagonal, 0s elsewhere).
   * @return A 3x3 identity matrix
   */
  public static Matrix3x3 identity() {
    Matrix3x3 result = new Matrix3x3();
    result.data[0][0] = 1.0;
    result.data[1][1] = 1.0;
    result.data[2][2] = 1.0;
    return result;
  }

  /**
   * Gets the value at the specified row and column.
   * @param row The row index (0-2)
   * @param col The column index (0-2)
   * @return The value at that position
   */
  public double get(int row, int col) {
    return data[row][col];
  }

  /**
   * Sets the value at the specified row and column.
   * @param row The row index (0-2)
   * @param col The column index (0-2)
   * @param value The value to set
   */
  public void set(int row, int col, double value) {
    data[row][col] = value;
  }

  /**
   * Multiplies this matrix by another matrix.
   * @param other The matrix to multiply with
   * @return The result of the multiplication
   */
  public Matrix3x3 multiply(Matrix3x3 other) {
    Matrix3x3 result = new Matrix3x3();
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        double sum = 0.0;
        for (int k = 0; k < 3; k++) {
          sum += this.data[i][k] * other.data[k][j];
        }
        result.data[i][j] = sum;
      }
    }
    return result;
  }

  /**
   * Multiplies this matrix by a scalar value.
   * @param scalar The scalar to multiply by
   * @return The result of the multiplication
   */
  public Matrix3x3 multiplyScalar(double scalar) {
    Matrix3x3 result = new Matrix3x3();
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        result.data[i][j] = this.data[i][j] * scalar;
      }
    }
    return result;
  }

  /**
   * Adds another matrix to this matrix.
   * @param other The matrix to add
   * @return The sum of the two matrices
   */
  public Matrix3x3 add(Matrix3x3 other) {
    Matrix3x3 result = new Matrix3x3();
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        result.data[i][j] = this.data[i][j] + other.data[i][j];
      }
    }
    return result;
  }

  /**
   * Subtracts another matrix from this matrix.
   * @param other The matrix to subtract
   * @return The difference of the two matrices
   */
  public Matrix3x3 subtract(Matrix3x3 other) {
    Matrix3x3 result = new Matrix3x3();
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        result.data[i][j] = this.data[i][j] - other.data[i][j];
      }
    }
    return result;
  }

  /**
   * Computes the transpose of this matrix.
   * @return The transposed matrix
   */
  public Matrix3x3 transpose() {
    Matrix3x3 result = new Matrix3x3();
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        result.data[i][j] = this.data[j][i];
      }
    }
    return result;
  }

  /**
   * Computes the inverse of this matrix using cofactor expansion.
   * @return The inverse matrix
   */
  public Matrix3x3 inverse() {
    double det = determinant();
    if (Math.abs(det) < 1e-10) {
      throw new IllegalStateException("Matrix is singular and cannot be inverted");
    }

    Matrix3x3 result = new Matrix3x3();
    
    // Compute cofactor matrix
    result.data[0][0] = (data[1][1] * data[2][2] - data[1][2] * data[2][1]) / det;
    result.data[0][1] = (data[0][2] * data[2][1] - data[0][1] * data[2][2]) / det;
    result.data[0][2] = (data[0][1] * data[1][2] - data[0][2] * data[1][1]) / det;
    
    result.data[1][0] = (data[1][2] * data[2][0] - data[1][0] * data[2][2]) / det;
    result.data[1][1] = (data[0][0] * data[2][2] - data[0][2] * data[2][0]) / det;
    result.data[1][2] = (data[0][2] * data[1][0] - data[0][0] * data[1][2]) / det;
    
    result.data[2][0] = (data[1][0] * data[2][1] - data[1][1] * data[2][0]) / det;
    result.data[2][1] = (data[0][1] * data[2][0] - data[0][0] * data[2][1]) / det;
    result.data[2][2] = (data[0][0] * data[1][1] - data[0][1] * data[1][0]) / det;
    
    return result;
  }

  /**
   * Computes the determinant of this matrix.
   * @return The determinant value
   */
  public double determinant() {
    return data[0][0] * (data[1][1] * data[2][2] - data[1][2] * data[2][1])
         - data[0][1] * (data[1][0] * data[2][2] - data[1][2] * data[2][0])
         + data[0][2] * (data[1][0] * data[2][1] - data[1][1] * data[2][0]);
  }

  /**
   * Creates a copy of this matrix.
   * @return A new matrix with the same values
   */
  public Matrix3x3 copy() {
    Matrix3x3 result = new Matrix3x3();
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        result.data[i][j] = this.data[i][j];
      }
    }
    return result;
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append("[\n");
    for (int i = 0; i < 3; i++) {
      sb.append("  [");
      for (int j = 0; j < 3; j++) {
        sb.append(String.format("%.4f", data[i][j]));
        if (j < 2) sb.append(", ");
      }
      sb.append("]\n");
    }
    sb.append("]");
    return sb.toString();
  }
}
