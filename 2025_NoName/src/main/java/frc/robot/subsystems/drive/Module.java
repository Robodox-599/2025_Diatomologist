package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.constants.RealConstants.MAX_LINEAR_SPEED;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.drive.constants.RealConstants;

// import static frc.robot.subsystems.drive.constants.RealConstants;
// Relative + Offset = Absolute

public class Module {
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
  private ModuleIO io;
  private String name;
  private Alert driveDisconnectedAlert;
  private Alert turnDisconnectedAlert;
  private Alert turnEncoderDisconnectedAlert;

  public record ModuleConstants(
      String prefix,
      int driveID,
      int steerID,
      int cancoderID,
      String CANBusName,
      Rotation2d cancoderOffset,
      Slot0Configs steerConfig,
      Slot0Configs driveConfig,
      double WheelRadius,
      boolean invertMotor,
      boolean invertCANcoder,
      boolean invertDrive) {}

  public Module(ModuleIO io) {
    this.io = io;
    this.name = io.getModuleName();
    driveDisconnectedAlert =
        new Alert("Disconnected drive motor on module " + name + ".", AlertType.kError);
    turnDisconnectedAlert =
        new Alert("Disconnected turn motor on module " + name + ".", AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert("Disconnected turn encoder on module " + name + ".", AlertType.kError);
  }

  public void updateInputs() {
    io.updateInputs();
  }

  public void periodic() {
    io.updateInputs();

    int sampleCount = io.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = io.odometryDrivePositionsMeters[i];
      Rotation2d angle = io.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }
    // Update alerts
    driveDisconnectedAlert.set(!io.driveConnected);
    turnDisconnectedAlert.set(!io.turnConnected);
    turnEncoderDisconnectedAlert.set(!io.encoderConnected);
  }

  /** Runs the module closed loop with the specified setpoint state. Returns the optimized state. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    state.optimize(getAngle());
    // Apply Optimized State angle to Turn Setpoint
    io.setTurnSetpoint(state.angle);
    // Apply cosine scaled state velocity to Drive Setpoint with FOC
    io.setDriveSetpoint(
        state.speedMetersPerSecond * Math.cos(state.angle.minus(io.turnPosition).getRadians()));
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveVoltage(output);
    io.setTurnSetpoint(new Rotation2d());
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return io.drivePositionMeters / RealConstants.WHEEL_RADIUS;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(io.driveVelocityMetersPerSec / RealConstants.WHEEL_RADIUS);
  }

  /**
   * Runs the module open loop with the specified setpoint state, velocity in volts. Returns the
   * optimized state.
   */
  public void runVoltageSetpoint(SwerveModuleState state) {
    runVoltageSetpoint(state, true);
  }

  public void runVoltageSetpoint(SwerveModuleState state, boolean focEnabled) {
    // Optimize state based on current angle
    state.optimize(getAngle());
    // Apply Optimized State angle to Turn Setpoint
    io.setTurnSetpoint(state.angle);
    // Apply cosine scaled state velocity to Drive Voltage Setpoint with FOC
    io.setDriveVoltage(
        (state.speedMetersPerSecond
                * Math.cos(state.angle.minus(io.turnPosition).getRadians())
                / MAX_LINEAR_SPEED)
            * 12,
        true);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);
  }

  /** Returns the current turn angle of the module at normal sampling frequency. */
  public Rotation2d getAngle() {
    return io.turnPosition;
  }

  /** Returns the current drive position of the module in meters at normal sampling frequency. */
  public double getPositionMeters() {
    return io.drivePositionMeters;
  }

  /**
   * Returns the current drive velocity of the module in meters per second withat normal sampling
   * frequency.
   */
  public double getVelocityMetersPerSec() {
    return io.driveVelocityMetersPerSec;
  }

  /** Returns the module position (turn angle and drive position) at normal sampling frequency. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity) at normal sampling frequency. */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the timestamps of the samples received this cycle from PhoenixOdometryThread. */
  public double[] getOdometryTimestamps() {
    return io.odometryTimestamps;
  }

  /** Returns the module positions received this cycle from PhoenixOdometryThread. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }
}
