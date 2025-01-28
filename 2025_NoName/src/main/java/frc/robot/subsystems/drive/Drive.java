// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.constants.RealConstants.TRACK_WIDTH_X;
import static frc.robot.subsystems.drive.constants.RealConstants.TRACK_WIDTH_Y;

import choreo.trajectory.SwerveSample;
import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.constants.RealConstants;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

public class Drive extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final Module[] modules; // FL, FR, BL, BR
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Pose2d pose = new Pose2d();
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, pose);

  private final PIDController choreoPathXController = new PIDController(10, 0, 0);
  private final PIDController choreoPathYController = new PIDController(10, 0, 0);
  private final PIDController choreoPathAngleController = new PIDController(7, 0, 0);
  private Twist2d fieldVelocity = new Twist2d();

  public Drive(GyroIO gyroIO, ModuleIO[] moduleIOs) {
    this.gyroIO = gyroIO;

    modules = new Module[moduleIOs.length];

    for (int i = 0; i < moduleIOs.length; i++) {
      modules[i] = new Module(moduleIOs[i]);
    }
    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    choreoPathAngleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Constructs an array of swerve module ios corresponding to the real robot.
   *
   * @return The array of swerve module ios.
   */
  public static ModuleIO[] createTalonFXModules() {
    return new ModuleIO[] {
      new ModuleIOReal(RealConstants.frontLeft),
      new ModuleIOReal(RealConstants.frontRight),
      new ModuleIOReal(RealConstants.backLeft),
      new ModuleIOReal(RealConstants.backRight)
    };
  }

  /**
   * Constructs an array of swerve module ios corresponding to a simulated robot.
   *
   * @return The array of swerve module ios.
   */
  public static ModuleIO[] createSimModules() {
    return new ModuleIO[] {
      new ModuleIOSim("Front Left"),
      new ModuleIOSim("Front Right"),
      new ModuleIOSim("Back Left"),
      new ModuleIOSim("Back Right")
    };
  }

  // Regularly called method to update subsystem state
  @Override
  public void periodic() {
    // locks odom, updates all inputs
    updateInputs();

    // calls disabled actions, to run if bot is disabled
    disabledActions();

    // apply odom update
    updateOdom();

    // apply field velocity update
    updateFieldVelo();
  }

  private void disabledActions() {
    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      DogLog.log("SwerveStates/Setpoints", new SwerveModuleState[] {});
      DogLog.log("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
      // updateVision();
    }
  }

  private void updateInputs() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs();

    for (var module : modules) {
      module.updateInputs();
    }

    odometryLock.unlock();

    for (var module : modules) {
      module.periodic();
    }
  }

  private void updateFieldVelo() {
    // Update field velocity
    SwerveModuleState[] measuredStates = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      measuredStates[i] = modules[i].getState();
    }
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(measuredStates);
    Translation2d linearFieldVelocity =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .rotateBy(getRotation());
    fieldVelocity =
        new Twist2d(
            linearFieldVelocity.getX(),
            linearFieldVelocity.getY(),
            gyroIO.connected ? gyroIO.yawVelocityRadPerSec : chassisSpeeds.omegaRadiansPerSecond);

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroIO.connected && Constants.currentMode != Mode.SIM);

    // DogLog.log("Odometry/Robot", getPose());
    DogLog.log("SwerveChassisSpeeds/Measured", getChassisSpeeds());
    DogLog.log("SwerveStates/Measured", getModuleStates());
  }

  private void updateOdom() {
    // Update odometry // This updates based on sensor data and kinematics
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    for (int i = 0; i < sampleTimestamps.length; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[modules.length];
      for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroIO.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroIO.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
      DogLog.log("Odometry/Pose", getPose());
      DogLog.log("Odometry/Velocity", getVelocity());
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, RealConstants.MAX_LINEAR_SPEED);

    DogLog.log("Swerve/Target Speeds", discreteSpeeds);
    DogLog.log("Swerve/Speed Error", discreteSpeeds.minus(getVelocity()));
    DogLog.log(
        "Swerve/Target Chassis Speeds Field Relative",
        ChassisSpeeds.fromRobotRelativeSpeeds(discreteSpeeds, getRotation()));
    for (int i = 0; i < modules.length; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }
  }

  public void followChoreoPath(SwerveSample sample) {
    Pose2d pose = getPose();
    ChassisSpeeds speeds = getChassisSpeeds();
    DogLog.log("Choreo/RobotPose2d", pose);
    DogLog.log("Choreo/SwerveSample", sample);

    DogLog.log("Choreo/SwerveSample/ChoreoVelocity", sample);

    DogLog.log("Choreo/RobotMeasuredVelocity", speeds);

    var pathTargetSpeeds = sample.getChassisSpeeds();
    pathTargetSpeeds.vxMetersPerSecond += choreoPathXController.calculate(pose.getX(), sample.x);

    pathTargetSpeeds.vyMetersPerSecond += choreoPathYController.calculate(pose.getY(), sample.y);
    pathTargetSpeeds.omegaRadiansPerSecond +=
        choreoPathAngleController.calculate(pose.getRotation().getRadians(), sample.heading);

    runVelocity(pathTargetSpeeds);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < modules.length; i++) {
      modules[i].runCharacterization(output);
    }
  }

  public Command zeroGyroCommand() {
    return new InstantCommand(
        () -> {
          overrideGyroAngle(0);
          gyroIO.setYaw(0);
        });
  }

  void overrideGyroAngle(double angleDegrees) {
    rawGyroRotation = Rotation2d.fromDegrees(angleDegrees);
    setPose(new Pose2d(getPose().getTranslation(), rawGyroRotation));
  }

  public Command zeroPosition() {
    return new InstantCommand(
        () -> {
          zeroPose();
        });
  }

  void zeroPose() {
    setPose(new Pose2d(new Translation2d(0, 0), getPose().getRotation()));
  }

  public Command runVelocityCmd(Supplier<ChassisSpeeds> speeds) {
    return this.run(() -> runVelocity(speeds.get()));
  }

  public Command runVelocityFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.runVelocityCmd(
        () -> ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), getPose().getRotation()));
  }

  public Command runVelocityTeleopFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.runVelocityCmd(
        () ->
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.get(),
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                    ? getPose().getRotation()
                    : getPose().getRotation().minus(Rotation2d.fromDegrees(180))));
  }

  public Command runVoltageTeleopFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.run(
        () -> {
          var allianceSpeeds =
              ChassisSpeeds.fromRobotRelativeSpeeds(
                  speeds.get(),
                  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                      ? getPose().getRotation()
                      : getPose().getRotation().minus(Rotation2d.fromDegrees(180)));
          // Calculate module setpoints
          ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(allianceSpeeds, 0.02);
          SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
          SwerveDriveKinematics.desaturateWheelSpeeds(
              setpointStates, RealConstants.MAX_LINEAR_SPEED);

          DogLog.log("Swerve/Target Speeds", discreteSpeeds);
          DogLog.log("Swerve/Field Speed Error", discreteSpeeds.minus(getVelocity()));
          DogLog.log(
              "Swerve/Target Chassis Speeds Field Relative",
              ChassisSpeeds.fromRobotRelativeSpeeds(discreteSpeeds, getRotation()));
          // final boolean focEnable =
          //     Math.sqrt(
          //             Math.pow(this.getVelocity().vxMetersPerSecond, 2)
          //                 + Math.pow(this.getVelocity().vyMetersPerSecond, 2))
          //         < RealConstants.MAX_LINEAR_SPEED * 0.9;

          // Send setpoints to modules
          for (int i = 0; i < modules.length; i++) {
            setpointStates[i].optimize(modules[i].getAngle());
            modules[i].runVoltageSetpoint(
                new SwerveModuleState(
                    setpointStates[i].speedMetersPerSecond * 12.0 / RealConstants.MAX_LINEAR_SPEED,
                    setpointStates[i].angle),
                true);
          }
          // Log setpoint states
          DogLog.log("SwerveStates/OptimizedSetpoints", setpointStates);
        });
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public Command stopWithXCmd() {
    return this.run(
        () -> {
          Rotation2d[] headings = new Rotation2d[modules.length];
          for (int i = 0; i < modules.length; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
          }
          kinematics.resetHeadings(headings);
          for (int i = 0; i < modules.length; i++) {
            modules[i].runSetpoint(new SwerveModuleState(0.0, headings[i]));
          }
        });
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < modules.length; i++) {
      output += modules[i].getFFCharacterizationVelocity() / modules.length;
    }
    return output;
  }

  public ChassisSpeeds getVelocity() {
    var speeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            kinematics.toChassisSpeeds(
                Arrays.stream(modules).map((m) -> m.getState()).toArray(SwerveModuleState[]::new)),
            getRotation());
    return new ChassisSpeeds(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  /** Returns the current odometry pose. */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return RealConstants.MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / RealConstants.DRIVE_BASE_RADIUS;
  }

  public Twist2d getFieldVelocity() {
    return fieldVelocity;
  }

  public double getYawVelocity() {
    return gyroIO.yawVelocityRadPerSec;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }
}
