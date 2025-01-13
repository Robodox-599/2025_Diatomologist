package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.drive.constants.SimConstants; // Ensure this import is added
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
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(SwerveModuleConstants constants) {
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                SimConstants.drive_gearbox, constants.DriveInertia, constants.DriveMotorGearRatio),
            SimConstants.drive_gearbox);
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                SimConstants.turn_gearbox, constants.SteerInertia, constants.SteerMotorGearRatio),
            SimConstants.turn_gearbox);

    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(int index) {
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

    SimLog.log("Drive/Module " + index + "/DriveMotor", driveSim);
    DogLog.log("Drive/Module " + index + "/DriveMotor/Connected", super.driveConnected);

    SimLog.log("Drive/Module " + index + "/TurnMotor", turnSim);
    DogLog.log("Drive/Module " + index + "/TurnMotor/Connected", super.turnConnected);

    DogLog.log("Drive/Module " + index + "/Encoder/Connected", super.encoderConnected);
    DogLog.log("Drive/Module " + index + "/Encoder/AbsolutePosition", super.absolutePosition);

    DogLog.log("Drive/Module " + index + "/Odometry/Timestamps", super.odometryTimestamps);
    DogLog.log(
        "Drive/Module " + index + "/Odometry/DrivePositionsRad", super.odometryDrivePositionsRad);
    DogLog.log("Drive/Module " + index + "/Odometry/TurnPositions", super.odometryTurnPositions);
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
