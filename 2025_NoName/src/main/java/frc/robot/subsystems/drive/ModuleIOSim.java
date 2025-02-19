package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.drive.constants.RealConstants;
import frc.robot.subsystems.drive.constants.SimConstants;
import frc.robot.util.SimLog;

public class ModuleIOSim extends ModuleIO {
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;
  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController =
      new PIDController(SimConstants.drive_kp, 0, SimConstants.drive_kd);
  private PIDController turnController =
      new PIDController(SimConstants.turn_kp, 0, SimConstants.turn_kd);
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;
  private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
  private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);
  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private String name;
  private double driveFFVolts = 0.0;

  public ModuleIOSim(final String name) {
    this.name = name;
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                SimConstants.drive_gearbox,
                kDriveInertia.magnitude()
                    * kDriveInertia.copySign(kDriveInertia, KilogramSquareMeters),
                (RealConstants.DRIVE_GEAR_RATIO)),
            SimConstants.drive_gearbox);
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                SimConstants.turn_gearbox,
                kSteerInertia.magnitude()
                    * kSteerInertia.copySign(kSteerInertia, KilogramSquareMeters),
                RealConstants.TURN_GEAR_RATIO),
            SimConstants.turn_gearbox);

    turnController.enableContinuousInput(-0.5, 0.5);
    // turnController.setTolerance(0.05);
    // driveController.setTolerance(0.05);
    // DriverStation.silenceJoystickConnectionWarning(true);
  }

  @Override
  public void updateInputs() {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    super.driveConnected = true;
    super.turnConnected = true;
    super.encoderConnected = true;

    super.drivePositionMeters = driveSim.getAngularPositionRad() * RealConstants.WHEEL_RADIUS;
    super.driveVelocityMetersPerSec =
        driveSim.getAngularVelocityRadPerSec() * RealConstants.WHEEL_RADIUS;
    super.driveAppliedVolts = driveAppliedVolts;
    super.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    super.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    super.turnAbsolutePosition =
        new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    super.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    super.turnAppliedVolts = turnAppliedVolts;
    super.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
    super.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    super.odometryDrivePositionsMeters = new double[] {super.drivePositionMeters};
    super.odometryTurnPositions = new Rotation2d[] {super.turnPosition};
    DogLog.log("Drive/Module " + name + "/Drive/Connected", super.driveConnected);
    DogLog.log("Drive/Module " + name + "/Turn/Connected", super.turnConnected);
    DogLog.log("Drive/Module " + name + "/Encoder/Connected", super.encoderConnected);

    DogLog.log("Drive/Module " + name + "/Drive/PositionMeters", super.drivePositionMeters);
    DogLog.log(
        "Drive/Module " + name + "/Drive/VelocityMetersPerSec", super.driveVelocityMetersPerSec);
    DogLog.log("Drive/Module " + name + "/Drive/AppliedVolts", super.driveAppliedVolts);
    DogLog.log("Drive/Module " + name + "/Drive/CurrentAmps", super.driveCurrentAmps);

    DogLog.log("Drive/Module " + name + "/Turn/Position", super.turnPosition);
    DogLog.log("Drive/Module " + name + "/Turn/AbsolutePosition", super.turnAbsolutePosition);
    DogLog.log("Drive/Module " + name + "/Turn/VelocityRadPerSec", super.turnVelocityRadPerSec);
    DogLog.log("Drive/Module " + name + "/Turn/AppliedVolts", super.turnAppliedVolts);
    DogLog.log("Drive/Module " + name + "/Turn/CurrentAmps", super.turnCurrentAmps);

    DogLog.log("Drive/Module " + name + "/Odometry/Timestamps", super.odometryTimestamps);
    DogLog.log(
        "Drive/Module " + name + "/Odometry/DrivePositionsMeters",
        super.odometryDrivePositionsMeters);
    DogLog.log("Drive/Module " + name + "/Odometry/TurnPositions", super.odometryTurnPositions);
    DogLog.log("Drive/Module " + name + "/Odometry/Timestamps", super.odometryTimestamps);
    DogLog.log(
        "Drive/Module " + name + "/Odometry/DrivePositionsRad", super.odometryDrivePositionsMeters);
    SimLog.log("Drive/Module " + name + "/DriveMotor", driveSim);
    DogLog.log("Drive/Module " + name + "/DriveMotor/Connected", super.driveConnected);

    SimLog.log("Drive/Module " + name + "/TurnMotor", turnSim);
    DogLog.log("Drive/Module " + name + "/TurnMotor/Connected", super.turnConnected);

    DogLog.log("Drive/Module " + name + "/Encoder/Connected", super.encoderConnected);
    DogLog.log("Drive/Module " + name + "/Odometry/Timestamps", super.odometryTimestamps);
    DogLog.log(
        "Drive/Module " + name + "/Odometry/DrivePositionsRad", super.odometryDrivePositionsMeters);
    DogLog.log("Drive/Module " + name + "/Odometry/TurnPositions", super.odometryTurnPositions);
  }

  @Override
  public void setDriveVoltage(final double volts, final boolean focEnabled) {
    driveClosedLoop = false;
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnVoltage(final double volts) {
    turnClosedLoop = false;
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void setDriveSetpoint(final double metersPerSecond) {
    driveClosedLoop = true;
    driveFFVolts =
        SimConstants.drive_ks * Math.signum(metersPerSecond)
            + SimConstants.drive_kv * metersPerSecond;
    driveController.setSetpoint(metersPerSecond);
  }

  @Override
  public void setTurnSetpoint(final Rotation2d rotation) {
    turnClosedLoop = true;
    setTurnVoltage(
        turnController.calculate(turnSim.getAngularPositionRotations(), rotation.getRotations()));
  }

  public String getModuleName() {
    return name;
  }
}
