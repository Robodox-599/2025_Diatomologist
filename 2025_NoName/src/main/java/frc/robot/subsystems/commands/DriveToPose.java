// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.commands;

import dev.doglog.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.constants.RealConstants;
import frc.robot.util.GeomUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveToPose extends Command {
  private static final String SMARTDASHBOARD_PREFIX = "DriveToPose/";

  private static double getSmartDashboardNumber(String key, double defaultValue) {
    return SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + key, defaultValue);
  }

  private double drivekP;
  private double drivekD;
  private double thetakP;
  private double thetakD;
  private double driveMaxVelocity;
  private double driveMaxVelocitySlow;
  private double driveMaxAcceleration;
  private double thetaMaxVelocity;
  private double thetaMaxAcceleration;
  private double driveTolerance;
  private double thetaTolerance;
  private double ffMinRadius;
  private double ffMaxRadius;

  static {
    switch (Constants.getMode()) {
      case REAL:
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "DriveKp", 0.75);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "DriveKd", 0.0);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "ThetaKp", 4.0);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "ThetaKd", 0.0);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "DriveMaxVelocity", 3.8);
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "DriveMaxVelocitySlow", 0.0); // Placeholder for slow velocity
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "DriveMaxAcceleration", 3.0);
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "ThetaMaxVelocity", Units.degreesToRadians(360.0));
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "ThetaMaxAcceleration", 8.0);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "DriveTolerance", 0.1);
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "ThetaTolerance", Units.degreesToRadians(1.0));
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "FFMinRadius", 0.1);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "FFMaxRadius", 0.15);
      case SIM:
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "DriveKp", 0.5);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "DriveKd", 0.18);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "ThetaKp", 10);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "ThetaKd", 0);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "DriveMaxVelocity", 3.8);
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "DriveMaxVelocitySlow", 1.5); // Placeholder for slow velocity
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "DriveMaxAcceleration", 3.0);
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "ThetaMaxVelocity", Units.degreesToRadians(360.0));
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "ThetaMaxAcceleration", 8.0);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "DriveTolerance", 0.05);
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "ThetaTolerance", Units.degreesToRadians(1.0));
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "FFMinRadius", 0.1);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "FFMaxRadius", 0.15);
      default:
        break;
    }
  }

  private final Drive drive;
  private final Supplier<Pose2d> target;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);

  private Translation2d lastSetpointTranslation = new Translation2d();
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  private boolean running = false;
  private Supplier<Pose2d> robot;

  private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
  private DoubleSupplier omegaFF = () -> 0.0;

  private double previousHash = 0;

  public DriveToPose(Drive drive, Supplier<Pose2d> target) {
    this.drive = drive;
    this.target = target;
    this.robot = () -> drive.getPose();

    // Enable continuous input for theta controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  public DriveToPose(Drive drive, Supplier<Pose2d> target, Supplier<Pose2d> robot) {
    this(drive, target);
    this.robot = robot;
  }

  public DriveToPose(
      Drive drive,
      Supplier<Pose2d> target,
      Supplier<Pose2d> robot,
      Supplier<Translation2d> linearFF,
      DoubleSupplier omegaFF) {
    this(drive, target, robot);
    this.linearFF = linearFF;
    this.omegaFF = omegaFF;
  }

  @Override
  public void initialize() {
    Pose2d currentPose = robot.get();
    ChassisSpeeds fieldVelocity = drive.getFieldVelocity();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    driveController.reset(
        currentPose.getTranslation().getDistance(target.get().getTranslation()),
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(
                    target
                        .get()
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();
  }

  @Override
  public void execute() {
    running = true;
    double currentHash =
        SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "DriveKp", 0.75)
            + SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "DriveKd", 0.0)
            + SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "ThetaKp", 4.0)
            + SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "ThetaKd", 0.0)
            + SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "DriveMaxVelocity", 3.8)
            + SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "DriveMaxVelocitySlow", 0.0)
            + SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "DriveMaxAcceleration", 3.0)
            + SmartDashboard.getNumber(
                SMARTDASHBOARD_PREFIX + "ThetaMaxVelocity", Units.degreesToRadians(360.0))
            + SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "ThetaMaxAcceleration", 8.0)
            + SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "DriveTolerance", 0.01)
            + SmartDashboard.getNumber(
                SMARTDASHBOARD_PREFIX + "ThetaTolerance", Units.degreesToRadians(1.0))
            + SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "FFMinRadius", 0.1)
            + SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "FFMaxRadius", 0.15);

    if (currentHash != previousHash) {
      drivekP = SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "DriveKp", 0.75);
      drivekD = SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "DriveKd", 0.0);
      thetakP = SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "ThetaKp", 4.0);
      thetakD = SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "ThetaKd", 0.0);
      driveMaxVelocity = SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "DriveMaxVelocity", 3.8);
      driveMaxVelocitySlow =
          SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "DriveMaxVelocitySlow", 0.0);
      driveMaxAcceleration =
          SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "DriveMaxAcceleration", 3.0);
      thetaMaxVelocity =
          SmartDashboard.getNumber(
              SMARTDASHBOARD_PREFIX + "ThetaMaxVelocity", Units.degreesToRadians(360.0));
      thetaMaxAcceleration =
          SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "ThetaMaxAcceleration", 8.0);
      driveTolerance = SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "DriveTolerance", 0.01);
      thetaTolerance =
          SmartDashboard.getNumber(
              SMARTDASHBOARD_PREFIX + "ThetaTolerance", Units.degreesToRadians(1.0));
      ffMinRadius = SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "FFMinRadius", 0.1);
      ffMaxRadius = SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + "FFMaxRadius", 0.15);

      driveController.setP(drivekP);
      driveController.setD(drivekD);
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
      driveController.setTolerance(driveTolerance);

      thetaController.setP(thetakP);
      thetaController.setD(thetakD);
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration));
      thetaController.setTolerance(thetaTolerance);

      previousHash = currentHash;
    }

    // Get current pose and target pose
    Pose2d currentPose = robot.get();
    Pose2d targetPose = target.get();

    // Calculate drive speed
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScaler =
        MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation();

    // Scale feedback velocities by input ff
    final double linearS = linearFF.get().getNorm() * 3.0;
    final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
    driveVelocity =
        driveVelocity.interpolate(linearFF.get().times(RealConstants.MAX_LINEAR_SPEED), linearS);
    thetaVelocity =
        MathUtil.interpolate(
            thetaVelocity, omegaFF.getAsDouble() * RealConstants.MAX_ANGULAR_SPEED, thetaS);

    // Command speeds
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

    // Log data
    DogLog.log("DriveToPose/DistanceMeasured", currentDistance);
    DogLog.log("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    DogLog.log("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    DogLog.log("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    DogLog.log(
        "DriveToPose/Setpoint",
        new Pose2d(
            lastSetpointTranslation,
            Rotation2d.fromRadians(thetaController.getSetpoint().position)));
    DogLog.log("DriveToPose/CurrentPose", currentPose);
    DogLog.log("DriveToPose/Target", targetPose);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    running = false;
    // Clear logs
    DogLog.log("DriveToPose/Setpoint", new Pose2d[] {});
    DogLog.log("DriveToPose/Goal", new Pose2d[] {});
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }
}
