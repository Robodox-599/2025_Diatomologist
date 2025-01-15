package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.drive.Module.ModuleConstants;
import frc.robot.subsystems.drive.constants.SimConstants; // Ensure this import is added
import frc.robot.util.SimLog;

public class ModuleIOSim extends ModuleIO {
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;
  private final ModuleConstants constants;
  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController =
      new PIDController(SimConstants.drive_kp, 0, SimConstants.drive_kd);
  private PIDController turnController =
      new PIDController(SimConstants.turn_kp, 0, SimConstants.turn_kd);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;
  private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
  private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);

  public ModuleIOSim(ModuleConstants constants) {
    this.constants = constants;
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                SimConstants.drive_gearbox,
                kDriveInertia.magnitude()
                    * kDriveInertia.copySign(kDriveInertia, KilogramSquareMeters),
                5.357142857142857),
            SimConstants.drive_gearbox);
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                SimConstants.turn_gearbox,
                kSteerInertia.magnitude()
                    * kSteerInertia.copySign(kSteerInertia, KilogramSquareMeters),
                21.428571428571427),
            SimConstants.turn_gearbox);

    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs() {
    if (driveClosedLoop) {
      driveAppliedVolts = driveFFVolts + driveController.calculate(super.driveVelocityRadPerSec);
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts =
          turnController.calculate(Units.rotationsToRadians(super.turnPosition.getRotations()));
    } else {
      turnController.reset();
    }

    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    // Update simulation state
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    // Update drive inputs
    super.driveConnected = true;
    super.drivePositionRad = driveSim.getAngularPositionRad();
    super.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    super.driveAppliedVolts = driveAppliedVolts;
    super.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    // Update turn inputs
    super.turnConnected = true;
    super.encoderConnected = true;
    super.absolutePosition = new Rotation2d(turnSim.getAngularPositionRad());
    super.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    super.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    super.turnAppliedVolts = turnAppliedVolts;
    super.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
    super.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    super.odometryDrivePositionsRad = new double[] {super.drivePositionRad};
    super.odometryTurnPositions = new Rotation2d[] {super.turnPosition};

    SimLog.log("Drive/Module " + constants.prefix() + "/DriveMotor", driveSim);
    DogLog.log(
        "Drive/Module " + constants.prefix() + "/DriveMotor/Connected", super.driveConnected);

    SimLog.log("Drive/Module " + constants.prefix() + "/TurnMotor", turnSim);
    DogLog.log("Drive/Module " + constants.prefix() + "/TurnMotor/Connected", super.turnConnected);

    DogLog.log("Drive/Module " + constants.prefix() + "/Encoder/Connected", super.encoderConnected);
    DogLog.log(
        "Drive/Module " + constants.prefix() + "/Encoder/AbsolutePosition", super.absolutePosition);

    DogLog.log(
        "Drive/Module " + constants.prefix() + "/Odometry/Timestamps", super.odometryTimestamps);
    DogLog.log(
        "Drive/Module " + constants.prefix() + "/Odometry/DrivePositionsRad",
        super.odometryDrivePositionsRad);
    DogLog.log(
        "Drive/Module " + constants.prefix() + "/Odometry/TurnPositions",
        super.odometryTurnPositions);
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts =
        SimConstants.drive_ks * Math.signum(velocityRadPerSec)
            + SimConstants.drive_kv * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
