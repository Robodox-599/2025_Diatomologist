package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.MotorLog;
import frc.robot.util.SimLog;
import frc.robot.subsystems.drive.constants.SimConstants; // Ensure this import is added

public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController = new PIDController(SimConstants.drive_kp, 0, SimConstants.drive_kd);
  private PIDController turnController = new PIDController(SimConstants.turn_kp, 0, SimConstants.turn_kd);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(SwerveModuleConstants constants) {
    driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            SimConstants.drive_gearbox, constants.DriveInertia, constants.DriveMotorGearRatio),
        SimConstants.drive_gearbox);
    turnSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            SimConstants.turn_gearbox, constants.SteerInertia, constants.SteerMotorGearRatio),
        SimConstants.turn_gearbox);

    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public boolean getDriveConnected() {
    return true;
  }

  @Override
  public double getDrivePositionRad() {
    return driveSim.getAngularPositionRad();
  }

  @Override
  public double getDriveVelocityRadPerSec() {
    return driveSim.getAngularVelocityRadPerSec();
  }

  @Override
  public boolean getTurnConnected() {
    return true;
  }

  @Override
  public double getTurnPositionRad() {
    return turnSim.getAngularPositionRad();
  }

  @Override
  public Rotation2d getTurnPosition() {
    return new Rotation2d(turnSim.getAngularPositionRad());
  }

  @Override
  public double getTurnVelocityRadPerSec() {
    return turnSim.getAngularVelocityRadPerSec();
  }

  @Override
  public boolean getEncoderConnected() {
    return true;
  }

  @Override
  public Rotation2d getAbsolutePosition() {
    return new Rotation2d(turnSim.getAngularPositionRad());
  }

  @Override
  public double[] getOdometryTimestamps() {
    return new double[] {Timer.getFPGATimestamp()};
  }

  @Override
  public double[] getOdometryDrivePositionsRad() {
    return new double[] {driveSim.getAngularPositionRad()};
  }

  @Override
  public Rotation2d[] getOdometryTurnPositions() {
    return new Rotation2d[] {new Rotation2d(turnSim.getAngularPositionRad())};
  }

  @Override
  public void updateInputs(int index) {
    if (driveClosedLoop) {
      driveAppliedVolts = driveFFVolts + driveController.calculate(getDriveVelocityRadPerSec());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(getTurnPositionRad());
    } else {
      turnController.reset();
    }

    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    SimLog.log("Drive/Module " + index + "/DriveMotor", driveSim);
    DogLog.log("Drive/Module " + index + "/DriveMotor/Connected", getDriveConnected());

    SimLog.log("Drive/Module " + index + "/TurnMotor", turnSim);
    DogLog.log("Drive/Module " + index + "/TurnMotor/Connected", getTurnConnected());

    DogLog.log("Drive/Module " + index + "/Encoder/Connected", getEncoderConnected());
    DogLog.log("Drive/Module " + index + "/Encoder/AbsolutePosition", getAbsolutePosition());

    DogLog.log("Drive/Module " + index + "/Odometry/Timestamps", getOdometryTimestamps());
    DogLog.log("Drive/Module " + index + "/Odometry/DrivePositionsRad", getOdometryDrivePositionsRad());
    DogLog.log("Drive/Module " + index + "/Odometry/TurnPositions", getOdometryTurnPositions());
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
    driveFFVolts = SimConstants.drive_ks * Math.signum(velocityRadPerSec) + SimConstants.drive_kv * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
