package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.constants.RealConstants;

// import static frc.robot.subsystems.drive.constants.RealConstants;

public class Module {
  public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
  static final double ODOMETRY_FREQUENCY = 250.0;
  private final ModuleConstants constants;
  private final ModuleIO io;
  // private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Rotation2d turnRelativeOffset = null; // Relative + Offset = Absolute
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
  private double lastSpeedSetpoint = 0.0;

  private SwerveModuleState lastSetpoint = new SwerveModuleState();
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
      double WheelRadius) {}

  public Module(ModuleIO io, ModuleConstants constants) {
    this.io = io;
    this.constants = constants;
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + constants.prefix() + ".", AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + constants.prefix() + ".", AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Disconnected turn encoder on module " + constants.prefix() + ".", AlertType.kError);
  }

  public void periodic() {
    // Logger.processInputs(String.format("Drive/%s Module", io.getModuleName()), inputs);

    // Logger.recordOutput(
    //     String.format("Drive/%s Module/Voltage Available", io.getModuleName()),
    //     Math.abs(inputs.driveAppliedVolts - RoboRioDataJNI.getVInVoltage()));
    // // Calculate positions for odometry
    io.updateInputs();

    int sampleCount = io.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = io.odometryDrivePositionsMeters[i];
      Rotation2d angle =
          io.odometryTurnPositions[i].plus(
              turnRelativeOffset != null ? turnRelativeOffset : new Rotation2d());
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }
        // Update alerts
        driveDisconnectedAlert.set(!io.driveConnected);
        turnDisconnectedAlert.set(!io.turnConnected);
        turnEncoderDisconnectedAlert.set(!io.encoderConnected);
  }

  /** Runs the module closed loop with the specified setpoint state. Returns the optimized state. */
  public void runSetpoint(SwerveModuleState state, boolean focEnabled) {
    // Optimize state based on current angle
    state.optimize(getAngle());

    io.setTurnSetpoint(state.angle);
    io.setDriveSetpoint(
      state.speedMetersPerSecond
            * Math.cos(state.angle.minus(io.turnPosition).getRadians()),
        (state.speedMetersPerSecond - lastSetpoint.speedMetersPerSecond) / 0.020);

    lastSetpoint = state;
    // return states;
  }


  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveVoltage(output);
    io.setTurnSetpoint(new Rotation2d());
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return io.drivePositionMeters;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(io.driveVelocityRadPerSec);
  }

  
  /**
   * Runs the module open loop with the specified setpoint state, velocity in volts. Returns the
   * optimized state.
   */
  public SwerveModuleState runVoltageSetpoint(SwerveModuleState state) {
    return runVoltageSetpoint(state, true);
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runDriveCharacterization(double volts) {
    // Closed loop turn control
    io.setTurnSetpoint(Rotation2d.fromRotations(0.0));

    // Open loop drive control
    io.setDriveVoltage(volts);
  }

  /** Runs the module angle with the specified voltage while not moving the drive motor */
  public void runSteerCharacterization(double volts) {
    io.setTurnVoltage(volts);
    io.setDriveVoltage(0.0);
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

  public Command setDaBrake() {
    return Commands.run(
        () -> {
          io.setBrake();
        });
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

  /** Returns the drive velocity in meters/sec. */
  public double getCharacterizationVelocity() {
    return io.driveVelocityMetersPerSec;
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
