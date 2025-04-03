// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

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
  private static double drivekP;
  private static double drivekD;
  private static double drivekI;
  private static double thetakP;
  private static double thetakI;
  private static double thetakD;
  private static double driveMaxVelocity;
  private static double driveMaxVelocitySlow;
  private static double driveMaxAcceleration;
  private static double thetaMaxVelocity;
  private static double thetaMaxAcceleration;
  private static double driveTolerance;
  private static double thetaTolerance;
  private static double ffMinRadius;
  private static double ffMaxRadius;

  static {
    switch (Constants.getMode()) {
      case REAL:
        drivekP = 0.8;
        drivekD = 0.0;
        drivekI = 0.0;
        thetakP = 0.7;
        thetakI = 0.0;
        thetakD = 0.0;
        driveMaxVelocity = RealConstants.MAX_LINEAR_SPEED * 0.75;
        driveMaxVelocitySlow = 0.0;
        driveMaxAcceleration = RealConstants.MAX_LINEAR_ACCELERATION * 0.15;
        thetaMaxVelocity = RealConstants.MAX_ANGULAR_SPEED;
        thetaMaxAcceleration = RealConstants.MAX_ANGULAR_ACCELERATION;
        driveTolerance = 0.03;
        thetaTolerance = Units.degreesToRadians(5);
        ffMinRadius = 0.15;
        ffMaxRadius = 0.40;
        break;
      case SIM:
        drivekP = 0.6;
        drivekD = 0.0;
        thetakP = 0.6;
        thetakI = 0.001;
        thetakD = 0.0;
        driveMaxVelocity = RealConstants.MAX_LINEAR_SPEED * .75;
        driveMaxVelocitySlow = 0.0;
        driveMaxAcceleration = RealConstants.MAX_LINEAR_ACCELERATION * .75;
        thetaMaxVelocity = RealConstants.MAX_ANGULAR_SPEED * .75;
        thetaMaxAcceleration = RealConstants.MAX_ANGULAR_ACCELERATION * .75;
        driveTolerance = 0.03;
        thetaTolerance = Units.degreesToRadians(1);
        ffMinRadius = 0.1;
        ffMaxRadius = 0.15;
      default:
        break;
    }
  }

  private final Drive drive;
  private final Supplier<Pose2d> target;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          drivekP,
          drivekI,
          drivekD,
          new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration),
          0.02);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetakP,
          thetakI,
          thetakD,
          new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration),
          0.02);

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
    DogLog.log("DriveToPose/Initialize/CurrentPose", currentPose);

    ChassisSpeeds fieldVelocity = drive.getFieldVelocity();
    DogLog.log("DriveToPose/Initialize/FieldVelocity", fieldVelocity);

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
    DogLog.log("DriveToPose/Initialize/LastSetpointTranslation", lastSetpointTranslation);
  }

  @Override
  public void execute() {
    running = true;
    // driveController.setP(drivekP);
    // driveController.setD(drivekD);
    // driveController.setConstraints(
    //     new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
    driveController.setTolerance(driveTolerance);
    // thetaController.setP(thetakP);
    // thetaController.setI(thetakI);
    // thetaController.setD(thetakD);
    // thetaController.setConstraints(
    //     new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration));
    thetaController.setTolerance(thetaTolerance);

    // --- Get current and target poses ---
    Pose2d currentPose = robot.get();
    DogLog.log("DriveToPose/Execute/CurrentPose", currentPose);
    Pose2d targetPose = target.get();
    DogLog.log("DriveToPose/Execute/TargetPose", targetPose);

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
    DogLog.log("DriveToPose/Execute/LastSetpointTranslation", lastSetpointTranslation);

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
    DogLog.log("DriveToPose/Execute/DriveVelocityVector", driveVelocity);

    // --- Scale feedback velocities by input feedforward ---
    final double linearS = linearFF.get().getNorm() * 3.0;
    final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
    DogLog.log(
        "DriveToPose/Execute/FeedForwardScalars", "linearS: " + linearS + ", thetaS: " + thetaS);

    driveVelocity =
        driveVelocity.interpolate(linearFF.get().times(RealConstants.MAX_LINEAR_SPEED), linearS);
    DogLog.log("DriveToPose/Execute/InterpolatedDriveVelocity", driveVelocity);

    thetaVelocity =
        MathUtil.interpolate(
            thetaVelocity, omegaFF.getAsDouble() * RealConstants.MAX_ANGULAR_SPEED, thetaS);
    DogLog.log("DriveToPose/Execute/InterpolatedThetaVelocity", thetaVelocity);

    // --- Command chassis speeds ---
    // if (thetaErrorAbs > Units.degreesToRadians(20)) {
    //   ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, thetaVelocity);
    //   DogLog.log("DriveToPose/Execute/ChassisSpeeds", chassisSpeeds);
    //   drive.runVelocity(chassisSpeeds);
    // } else {
    ChassisSpeeds chassisSpeeds =
        new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity);
    DogLog.log("DriveToPose/Execute/ChassisSpeeds", chassisSpeeds);
    drive.runVelocity(chassisSpeeds);
    // }

    // --- Log additional state info ---
    Pose2d setpointPose =
        new Pose2d(
            lastSetpointTranslation,
            Rotation2d.fromRadians(thetaController.getSetpoint().position));
    DogLog.log("DriveToPose/Execute/SetpointPose", setpointPose);
    DogLog.log("DriveToPose/Execute/CurrentPose", currentPose);
    DogLog.log("DriveToPose/Execute/TargetPose", targetPose);
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
