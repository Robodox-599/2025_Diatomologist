package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {
  /** Updates the set of loggable inputs. */
  public default void updateInputs(int index) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  public default void setTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  public default void setTurnPosition(Rotation2d rotation) {}

  /** Get whether a specific component is connected */
  public default boolean getConnected(String type) {
    return false;
  }

  // Drive motor getters
  /** Get whether the drive motor is connected */
  public default boolean getDriveConnected() {
    return false;
  }

  /** Get the drive motor position in radians */
  public default double getDrivePositionRad() {
    return 0.0;
  }

  /** Get the drive motor velocity in radians per second */
  public default double getDriveVelocityRadPerSec() {
    return 0.0;
  }

  // Turn motor getters
  /** Get whether the turn motor is connected */
  public default boolean getTurnConnected() {
    return false;
  }

  /** Get the turn motor position in radians */
  public default double getTurnPositionRad() {
    return 0.0;
  }

  /** Get the turn motor position as a Rotation2d */
  public default Rotation2d getTurnPosition() {
    return new Rotation2d();
  }

  /** Get the turn motor velocity in radians per second */
  public default double getTurnVelocityRadPerSec() {
    return 0.0;
  }

  // Encoder getters
  /** Get whether the encoder is connected */
  public default boolean getEncoderConnected() {
    return false;
  }

  /** Get the absolute position from the encoder */
  public default Rotation2d getAbsolutePosition() {
    return new Rotation2d();
  }

  // Odometry getters
  /** Get the timestamps for odometry measurements */
  public default double[] getOdometryTimestamps() {
    return new double[] {};
  }

  /** Get the drive positions in radians for odometry */
  public default double[] getOdometryDrivePositionsRad() {
    return new double[] {};
  }

  /** Get the turn positions for odometry */
  public default Rotation2d[] getOdometryTurnPositions() {
    return new Rotation2d[] {};
  }
}