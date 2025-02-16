package frc.robot.subsystems.commands;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.BooleanSupplier;

public class MoveToPointCommand {
  private static final ProfiledPIDController driveController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
  private static final ProfiledPIDController thetaController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
  private static double driveError;
  private static double thetaError;
  private static Translation2d lastSetpointTranslation;

  private static final String SMARTDASHBOARD_PREFIX = "DriveToPose/";

  private static double getSmartDashboardNumber(String key, double defaultValue) {
    return SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + key, defaultValue);
  }

  static {
    switch (Constants.getMode()) {
      case REAL:
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "DriveKp", 2.0);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "DriveKd", 0.0);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "ThetaKp", 5.0);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "ThetaKd", 0.0);
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "DriveMaxVelocity", Units.inchesToMeters(150.0));
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "DriveMaxVelocitySlow", Units.inchesToMeters(50.0));
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "DriveMaxAcceleration", Units.inchesToMeters(95.0));
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "ThetaMaxVelocity", Units.degreesToRadians(360.0));
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "ThetaMaxVelocitySlow", Units.degreesToRadians(90.0));
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "ThetaMaxAcceleration", Units.degreesToRadians(720.0));
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "DriveTolerance", 0.01);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "DriveToleranceSlow", 0.06);
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "ThetaTolerance", Units.degreesToRadians(1.0));
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "ThetaToleranceSlow", Units.degreesToRadians(3.0));
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "MinDistance", 0.2);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "MaxDistance", 0.8);
      case SIM:
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "DriveKp", 2.0);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "DriveKd", 0.0);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "ThetaKp", 5.0);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "ThetaKd", 0.0);
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "DriveMaxVelocity", Units.inchesToMeters(150.0));
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "DriveMaxVelocitySlow", Units.inchesToMeters(50.0));
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "DriveMaxAcceleration", Units.inchesToMeters(95.0));
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "ThetaMaxVelocity", Units.degreesToRadians(360.0));
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "ThetaMaxVelocitySlow", Units.degreesToRadians(90.0));
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "ThetaMaxAcceleration", Units.degreesToRadians(720.0));
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "DriveTolerance", 0.01);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "DriveToleranceSlow", 0.06);
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "ThetaTolerance", Units.degreesToRadians(1.0));
        SmartDashboard.putNumber(
            SMARTDASHBOARD_PREFIX + "ThetaToleranceSlow", Units.degreesToRadians(3.0));
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "MinDistance", 0.2);
        SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "MaxDistance", 0.8);
      default:
        break;
    }
  }

  public static Command moveToPointCommand(Drive drive, Pose2d point, BooleanSupplier slow) {
    // Update from SmartDashboard tunable numbers
    double driveKp = getSmartDashboardNumber("DriveKp", 2.0);
    double driveKd = getSmartDashboardNumber("DriveKd", 0.0);
    double thetaKp = getSmartDashboardNumber("ThetaKp", 5.0);
    double thetaKd = getSmartDashboardNumber("ThetaKd", 0.0);
    double driveMaxVelocity =
        getSmartDashboardNumber("DriveMaxVelocity", Units.inchesToMeters(150.0));
    double driveMaxVelocitySlow =
        getSmartDashboardNumber("DriveMaxVelocitySlow", Units.inchesToMeters(50.0));
    double driveMaxAcceleration =
        getSmartDashboardNumber("DriveMaxAcceleration", Units.inchesToMeters(95.0));
    double thetaMaxVelocity =
        getSmartDashboardNumber("ThetaMaxVelocity", Units.degreesToRadians(360.0));
    double thetaMaxVelocitySlow =
        getSmartDashboardNumber("ThetaMaxVelocitySlow", Units.degreesToRadians(90.0));
    double thetaMaxAcceleration =
        getSmartDashboardNumber("ThetaMaxAcceleration", Units.degreesToRadians(720.0));
    double driveTolerance = getSmartDashboardNumber("DriveTolerance", 0.01);
    double driveToleranceSlow = getSmartDashboardNumber("DriveToleranceSlow", 0.06);
    double thetaTolerance = getSmartDashboardNumber("ThetaTolerance", Units.degreesToRadians(1.0));
    double thetaToleranceSlow =
        getSmartDashboardNumber("ThetaToleranceSlow", Units.degreesToRadians(3.0));
    double minDistance = getSmartDashboardNumber("MinDistance", 0.2);
    double maxDistance = getSmartDashboardNumber("MaxDistance", 0.8);
    // Update drive controller parameters
    driveController.setP(driveKp);
    driveController.setD(driveKd);
    driveController.setConstraints(
        new TrapezoidProfile.Constraints(
            slow.getAsBoolean() ? driveMaxVelocitySlow : driveMaxVelocity, driveMaxAcceleration));
    driveController.setTolerance(slow.getAsBoolean() ? driveToleranceSlow : driveTolerance);

    // Update theta controller parameters
    thetaController.setP(thetaKp);
    thetaController.setD(thetaKd);
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(
            slow.getAsBoolean() ? thetaMaxVelocitySlow : thetaMaxVelocity, thetaMaxAcceleration));
    thetaController.setTolerance(slow.getAsBoolean() ? thetaToleranceSlow : thetaTolerance);

    return Commands.run(
        () -> {
          Pose2d currentPose = drive.getPose();
          // Reset all controllers
          driveController.reset(
              currentPose.getTranslation().getDistance(point.getTranslation()),
              Math.min(
                  0.0,
                  -new Translation2d(drive.getFieldVelocity().dx, drive.getFieldVelocity().dy)
                      .rotateBy(
                          point
                              .getTranslation()
                              .minus(drive.getPose().getTranslation())
                              .getAngle()
                              .unaryMinus())
                      .getX()));

          thetaController.reset(currentPose.getRotation().getRadians(), drive.getYawVelocity());

          lastSetpointTranslation = drive.getPose().getTranslation();

          // Calculate drive speed
          double currentDistance = currentPose.getTranslation().getDistance(point.getTranslation());
          /**
           * If the robot is closer than ffMinRadius, the scaler is 0 (no feedforward contribution).
           * If the robot is farther than ffMaxRadius, the scaler is 1 (full feedforward
           * contribution). Between these distances, the scaler interpolates linearly.
           */
          double feedForwardScaling =
              MathUtil.clamp(
                  (currentDistance - maxDistance) / (maxDistance - minDistance), 0.0, 1.0);

          // current error is the distance from bot to point
          driveError = currentDistance;
          // reset controller but keeping error and velocity of the controller, prevents the
          // integral windup (idk i read it on cd)
          driveController.reset(
              lastSetpointTranslation.getDistance(point.getTranslation()),
              driveController.getSetpoint().velocity);

          /**
           * gets the velocity of the controller setpoint, then scales it with the
           * feedForwardScaling. once scaled, it adds in the positional PID calculation. this is a
           * combination of feedforward and feedback control.
           */
          double scaledDriveVelocity =
              driveController.getSetpoint().velocity * feedForwardScaling
                  + driveController.calculate(driveError, 0.0);

          if (currentDistance < driveController.getPositionTolerance()) {
            // if the current distance is within the error tolerance of the controller, then just
            // update the drive velocity to be 0 so it wont drive cus its in range of setpoint.
            scaledDriveVelocity = 0.0;
          }
          // stores the setpoint to be reused for the distance between the point and the setpoint
          // (chatgpt gave me this cus i didnt know how to do it)
          lastSetpointTranslation =
              new Pose2d(
                      point.getTranslation(),
                      currentPose.getTranslation().minus(point.getTranslation()).getAngle())
                  .transformBy(
                      new Transform2d(
                          new Translation2d(driveController.getSetpoint().position, 0.0),
                          new Rotation2d()))
                  .getTranslation();

          // same thing as above just for the angle control to make sure we face the direction as
          // our point.
          double scaledThetaVelocity =
              thetaController.getSetpoint().velocity * feedForwardScaling
                  + thetaController.calculate(
                      currentPose.getRotation().getRadians(), point.getRotation().getRadians());

          if (thetaError < thetaController.getPositionTolerance()) {
            scaledThetaVelocity = 0.0;
          }

          var driveVelocity =
              new Pose2d(
                      new Translation2d(),
                      currentPose.getTranslation().minus(point.getTranslation()).getAngle())
                  .transformBy(
                      new Transform2d(
                          new Translation2d(scaledDriveVelocity, 0.0), new Rotation2d()))
                  .getTranslation();
          DogLog.log("Commands/DriveToPose/DistanceMeasured", currentDistance);
          DogLog.log(
              "Commands/DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
          DogLog.log("Commands/DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
          DogLog.log("Commands/DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
          DogLog.log(
              "Odometry/DriveToPoseSetpoint",
              new Pose2d(
                  lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
          DogLog.log("Odometry/DriveToPoseGoal", point);

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  driveVelocity.getX(),
                  driveVelocity.getY(),
                  scaledThetaVelocity,
                  currentPose.getRotation()));
        },
        drive);
  }

  public static Command moveToPointFast(Drive drive, Pose2d point) {
    return moveToPointCommand(drive, point, () -> false);
  }

  public static Command moveToPointSlow(Drive drive, Pose2d point) {
    return moveToPointCommand(drive, point, () -> true);
  }
}
