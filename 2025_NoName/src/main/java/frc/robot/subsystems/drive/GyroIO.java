package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
  /** Updates the set of loggable inputs. */
  public default void updateInputs() {}

  /** Returns whether the gyro is connected */
  public default boolean getConnected() {
    return false;
  }

  /** Returns the yaw position of the gyro */
  public default Rotation2d getYawPosition() {
    return new Rotation2d();
  }

  /** Returns the yaw velocity in radians per second */
  public default double getYawVelocityRadPerSec() {
    return 0.0;
  }

  /** Returns the timestamps for odometry yaw measurements */
  public default double[] getOdometryYawTimestamps() {
    return new double[] {};
  }

  /** Returns the yaw positions for odometry */
  public default Rotation2d[] getOdometryYawPositions() {
    return new Rotation2d[] {};
  }
}