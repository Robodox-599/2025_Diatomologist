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
    double value = SmartDashboard.getNumber(SMARTDASHBOARD_PREFIX + key, defaultValue);
    DogLog.log("DriveToPose/SmartDashboard/" + key, value);
    return value;
  }

  // Parameters from SmartDashboard
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
        break;
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
        break;
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
    DogLog.log("DriveToPose/Initialize/CurrentPose", currentPose.toString());

    ChassisSpeeds fieldVelocity = drive.getFieldVelocity();
    DogLog.log("DriveToPose/Initialize/FieldVelocity", fieldVelocity.toString());

    double initialDistance =
        currentPose.getTranslation().getDistance(target.get().getTranslation());
    DogLog.log("DriveToPose/Initialize/InitialDistance", initialDistance);

    // Reset the drive controller with current distance and field velocity X-component
    driveController.reset(
        initialDistance,
        Math.min(
            0.0,
            -new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond)
                .rotateBy(
                    target
                        .get()
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    DogLog.log(
        "DriveToPose/Initialize/DriveControllerReset", "Reset with distance " + initialDistance);

    // Reset the theta controller with current heading and angular velocity
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    DogLog.log(
        "DriveToPose/Initialize/ThetaControllerReset",
        "Reset with heading " + currentPose.getRotation().getRadians());

    lastSetpointTranslation = currentPose.getTranslation();
    DogLog.log(
        "DriveToPose/Initialize/LastSetpointTranslation", lastSetpointTranslation.toString());
  }

  @Override
  public void execute() {
    running = true;

    // --- Update PID parameters if SmartDashboard values have changed ---
    double currentHash =
        getSmartDashboardNumber("DriveKp", 0.75)
            + getSmartDashboardNumber("DriveKd", 0.0)
            + getSmartDashboardNumber("ThetaKp", 4.0)
            + getSmartDashboardNumber("ThetaKd", 0.0)
            + getSmartDashboardNumber("DriveMaxVelocity", 3.8)
            + getSmartDashboardNumber("DriveMaxVelocitySlow", 0.0)
            + getSmartDashboardNumber("DriveMaxAcceleration", 3.0)
            + getSmartDashboardNumber("ThetaMaxVelocity", Units.degreesToRadians(360.0))
            + getSmartDashboardNumber("ThetaMaxAcceleration", 8.0)
            + getSmartDashboardNumber("DriveTolerance", 0.01)
            + getSmartDashboardNumber("ThetaTolerance", Units.degreesToRadians(1.0))
            + getSmartDashboardNumber("FFMinRadius", 0.1)
            + getSmartDashboardNumber("FFMaxRadius", 0.15);
    DogLog.log("DriveToPose/Execute/CurrentHash", currentHash);

    if (currentHash != previousHash) {
      drivekP = getSmartDashboardNumber("DriveKp", 0.75);
      drivekD = getSmartDashboardNumber("DriveKd", 0.0);
      thetakP = getSmartDashboardNumber("ThetaKp", 4.0);
      thetakD = getSmartDashboardNumber("ThetaKd", 0.0);
      driveMaxVelocity = getSmartDashboardNumber("DriveMaxVelocity", 3.8);
      driveMaxVelocitySlow = getSmartDashboardNumber("DriveMaxVelocitySlow", 0.0);
      driveMaxAcceleration = getSmartDashboardNumber("DriveMaxAcceleration", 3.0);
      thetaMaxVelocity = getSmartDashboardNumber("ThetaMaxVelocity", Units.degreesToRadians(360.0));
      thetaMaxAcceleration = getSmartDashboardNumber("ThetaMaxAcceleration", 8.0);
      driveTolerance = getSmartDashboardNumber("DriveTolerance", 0.01);
      thetaTolerance = getSmartDashboardNumber("ThetaTolerance", Units.degreesToRadians(1.0));
      ffMinRadius = getSmartDashboardNumber("FFMinRadius", 0.1);
      ffMaxRadius = getSmartDashboardNumber("FFMaxRadius", 0.15);

      DogLog.log(
          "DriveToPose/Execute/SmartDashboardValues",
          "drivekP="
              + drivekP
              + ", drivekD="
              + drivekD
              + ", thetakP="
              + thetakP
              + ", thetakD="
              + thetakD);
      DogLog.log(
          "DriveToPose/Execute/SmartDashboardValues",
          "driveMaxVelocity="
              + driveMaxVelocity
              + ", driveMaxAcceleration="
              + driveMaxAcceleration);
      DogLog.log(
          "DriveToPose/Execute/SmartDashboardValues",
          "thetaMaxVelocity="
              + thetaMaxVelocity
              + ", thetaMaxAcceleration="
              + thetaMaxAcceleration);
      DogLog.log(
          "DriveToPose/Execute/SmartDashboardValues",
          "driveTolerance=" + driveTolerance + ", thetaTolerance=" + thetaTolerance);
      DogLog.log(
          "DriveToPose/Execute/SmartDashboardValues",
          "ffMinRadius=" + ffMinRadius + ", ffMaxRadius=" + ffMaxRadius);

      // Update drive controller parameters
      driveController.setP(drivekP);
      driveController.setD(drivekD);
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
      driveController.setTolerance(driveTolerance);
      DogLog.log(
          "DriveToPose/Execute/DriveControllerUpdated",
          "P="
              + drivekP
              + ", D="
              + drivekD
              + ", MaxVel="
              + driveMaxVelocity
              + ", MaxAcc="
              + driveMaxAcceleration);

      // Update theta controller parameters
      thetaController.setP(thetakP);
      thetaController.setD(thetakD);
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration));
      thetaController.setTolerance(thetaTolerance);
      DogLog.log(
          "DriveToPose/Execute/ThetaControllerUpdated",
          "P="
              + thetakP
              + ", D="
              + thetakD
              + ", MaxVel="
              + thetaMaxVelocity
              + ", MaxAcc="
              + thetaMaxAcceleration);

      previousHash = currentHash;
      DogLog.log("DriveToPose/Execute/PreviousHashUpdated", previousHash);
    }

    // --- Get current and target poses ---
    Pose2d currentPose = robot.get();
    DogLog.log("DriveToPose/Execute/CurrentPose", currentPose.toString());
    Pose2d targetPose = target.get();
    DogLog.log("DriveToPose/Execute/TargetPose", targetPose.toString());

    // --- Compute drive distance and feed-forward scaler ---
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    DogLog.log("DriveToPose/Execute/CurrentDistance", currentDistance);
    double ffScaler =
        MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
    DogLog.log("DriveToPose/Execute/FFScaler", ffScaler);

    driveErrorAbs = currentDistance;

    // --- Reset drive controller with the last setpoint translation ---
    double lastDistance = lastSetpointTranslation.getDistance(targetPose.getTranslation());
    double currentControllerVelocity = driveController.getSetpoint().velocity;
    DogLog.log("DriveToPose/Execute/LastSetpointDistance", lastDistance);
    DogLog.log("DriveToPose/Execute/CurrentControllerVelocity", currentControllerVelocity);
    driveController.reset(lastDistance, currentControllerVelocity);
    DogLog.log(
        "DriveToPose/Execute/DriveControllerReset", "Reset with lastDistance " + lastDistance);

    // --- Compute drive velocity scalar ---
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    DogLog.log("DriveToPose/Execute/DriveVelocityScalar", driveVelocityScalar);

    if (currentDistance < driveController.getPositionTolerance()) {
      driveVelocityScalar = 0.0;
      DogLog.log("DriveToPose/Execute/DriveVelocityScalar", "Within tolerance: set to 0.");
    }

    // --- Update last setpoint translation using transformation ---
    Pose2d tempPose =
        new Pose2d(
            targetPose.getTranslation(),
            currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle());
    Pose2d transformedPose =
        tempPose.transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0));
    lastSetpointTranslation = transformedPose.getTranslation();
    DogLog.log("DriveToPose/Execute/LastSetpointTranslation", lastSetpointTranslation.toString());

    // --- Compute theta velocity ---
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    DogLog.log("DriveToPose/Execute/ThetaVelocity", thetaVelocity);

    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    DogLog.log("DriveToPose/Execute/ThetaErrorAbs", thetaErrorAbs);
    if (thetaErrorAbs < thetaController.getPositionTolerance()) {
      thetaVelocity = 0.0;
      DogLog.log("DriveToPose/Execute/ThetaVelocity", "Within theta tolerance: set to 0.");
    }

    // --- Calculate drive velocity vector ---
    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation();
    DogLog.log("DriveToPose/Execute/DriveVelocityVector", driveVelocity.toString());

    // --- Scale feedback velocities by input feedforward ---
    final double linearS = linearFF.get().getNorm() * 3.0;
    final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
    DogLog.log(
        "DriveToPose/Execute/FeedForwardScalars", "linearS: " + linearS + ", thetaS: " + thetaS);

    driveVelocity =
        driveVelocity.interpolate(linearFF.get().times(RealConstants.MAX_LINEAR_SPEED), linearS);
    DogLog.log("DriveToPose/Execute/InterpolatedDriveVelocity", driveVelocity.toString());

    thetaVelocity =
        MathUtil.interpolate(
            thetaVelocity, omegaFF.getAsDouble() * RealConstants.MAX_ANGULAR_SPEED, thetaS);
    DogLog.log("DriveToPose/Execute/InterpolatedThetaVelocity", thetaVelocity);

    // --- Command chassis speeds ---
    ChassisSpeeds chassisSpeeds =
        new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity);
    DogLog.log("DriveToPose/Execute/ChassisSpeeds", chassisSpeeds.toString());

    drive.runVelocity(chassisSpeeds);

    // --- Log additional state info ---
    Pose2d setpointPose =
        new Pose2d(
            lastSetpointTranslation,
            Rotation2d.fromRadians(thetaController.getSetpoint().position));
    DogLog.log("DriveToPose/Execute/SetpointPose", setpointPose.toString());
    DogLog.log("DriveToPose/Execute/CurrentPose", currentPose.toString());
    DogLog.log("DriveToPose/Execute/TargetPose", targetPose.toString());
  }

  @Override
  public boolean isFinished() {
    boolean finished = running && driveController.atGoal() && thetaController.atGoal();
    DogLog.log("DriveToPose/isFinished", finished);
    return finished;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    running = false;
    DogLog.log("DriveToPose/End", "Command ended. Interrupted: " + interrupted);
    // Optionally clear setpoint/goal logs
    DogLog.log("DriveToPose/Setpoint", "Cleared");
    DogLog.log("DriveToPose/Goal", "Cleared");
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    boolean atGoal = running && driveController.atGoal() && thetaController.atGoal();
    DogLog.log("DriveToPose/atGoal", atGoal);
    return atGoal;
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance() {
    boolean withinTol =
        running
            && Math.abs(driveErrorAbs) < driveTolerance
            && Math.abs(thetaErrorAbs) < thetaTolerance;
    DogLog.log("DriveToPose/withinTolerance", withinTol);
    return withinTol;
  }
}
