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

import static frc.robot.subsystems.drive.constants.RealConstants.TRACK_WIDTH_X;
import static frc.robot.subsystems.drive.constants.RealConstants.TRACK_WIDTH_Y;

import choreo.trajectory.SwerveSample;
import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.constants.RealConstants;
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
  private final Field2d field = new Field2d();
  private final PIDController choreoPathXController;
  private final PIDController choreoPathYController;
  private final PIDController choreoPathAngleController;
  private ChassisSpeeds maxMeasuredSpeed = new ChassisSpeeds();

  ProfiledPIDController thetaController =
      new ProfiledPIDController(
          1.5,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(
              RealConstants.MAX_ANGULAR_SPEED, RealConstants.MAX_ANGULAR_ACCELERATION));
  private final PIDController translationController = new PIDController(1.7, 0.0, 0.0);

  public Drive(GyroIO gyroIO, ModuleIO[] moduleIOs) {
    SmartDashboard.putData("Field", field);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Units.degreesToRadians(3));
    translationController.setTolerance(0.02);

    switch (Constants.currentMode) {
      case REAL: // in meters
        choreoPathXController = new PIDController(1.5, 0.0, 0.0); // 0.3
        choreoPathYController = new PIDController(1.5, 0.0, 0.0); // 0.3
        choreoPathAngleController = new PIDController(0, 0, 0); // 0.15
        break;
      case SIM:
        choreoPathXController = new PIDController(0, 0, 0);
        choreoPathYController = new PIDController(0, 0, 0);
        choreoPathAngleController = new PIDController(0, 0, 0);
        break;
      default:
        choreoPathXController = new PIDController(1, 0, 0);
        choreoPathYController = new PIDController(1, 0, 0);
        choreoPathAngleController = new PIDController(0, 0, 0);
        break;
    }

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
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  private void disabledActions() {
    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      DogLog.log("Drive/SwerveStates/OptimizedSetpoints", new SwerveModuleState[] {});
    }
  }

  private void updateInputs() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs();

    for (var module : modules) {
      module.periodic();
    }

    odometryLock.unlock();
  }

  private void updateOdom() {
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
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
      field.setRobotPose(getPose());
      gyroDisconnectedAlert.set(!gyroIO.connected && Constants.currentMode != Mode.SIM);
      DogLog.log("Odometry/Pose", getPose());
      DogLog.log("Odometry/FieldVelocity", getFieldVelocity());
      DogLog.log("Odometry/RobotVelocity", getRobotVelocity());
      DogLog.log("Drive/SwerveStates/Measured", getModuleStates());
      if (getRobotVelocity().vxMetersPerSecond > maxMeasuredSpeed.vxMetersPerSecond
          || getRobotVelocity().vyMetersPerSecond > maxMeasuredSpeed.vyMetersPerSecond) {
        maxMeasuredSpeed = getRobotVelocity();
      }
      DogLog.log("Drive/MaxMeasuredSpeed", maxMeasuredSpeed);
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? getPose().getRotation()
                : getPose().getRotation().minus(Rotation2d.fromDegrees(180)));
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, RealConstants.MAX_LINEAR_SPEED);

    DogLog.log("Drive/RobotRelativeTargetSpeeds", discreteSpeeds);
    DogLog.log("Drive/SwerveStates/OptimizedSetpoints", setpointStates);
    for (int i = 0; i < modules.length; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }
  }

  public void followChoreoPath(SwerveSample sample) {
    Pose2d pose = getPose();
    DogLog.log("Drive/Choreo/RobotPose2d", pose);
    DogLog.log("Drive/Choreo/SwerveSample", sample);
    DogLog.log("Drive/Choreo/SwerveSample/ChoreoPosition", sample.getPose());
    DogLog.log("Drive/Choreo/RealRobotPosition", pose);

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx + choreoPathXController.calculate(pose.getX(), sample.x),
            sample.vy + choreoPathYController.calculate(pose.getY(), sample.y),
            sample.omega
                + choreoPathAngleController.calculate(
                    pose.getRotation().getRadians(), sample.heading));
    DogLog.log("Drive/Choreo/RobotSetpointSpeedsAfterPID", speeds);

    // Calculate module setpoints
    ChassisSpeeds allianceSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rawGyroRotation);
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(allianceSpeeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, RealConstants.MAX_LINEAR_SPEED);

    DogLog.log("Drive/RobotRelativeTargetSpeeds", discreteSpeeds);
    DogLog.log("Drive/SwerveStates/OptimizedSetpoints", setpointStates);
    for (int i = 0; i < modules.length; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }
  }

  public Command runVelocityTeleopFieldRelative(Supplier<ChassisSpeeds> joystickSpeeds // ,
      ) {
    return this.run(
        () -> {
          // Calculate module setpoints
          ChassisSpeeds speeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  joystickSpeeds.get(),
                  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                      ? rawGyroRotation
                      : rawGyroRotation.plus(Rotation2d.fromDegrees(180)));
          ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
          SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
          SwerveDriveKinematics.desaturateWheelSpeeds(
              setpointStates, RealConstants.MAX_LINEAR_SPEED);

          DogLog.log("Drive/RobotRelativeTargetSpeeds", discreteSpeeds);
          DogLog.log("Drive/SwerveStates/OptimizedSetpoints", setpointStates);
          for (int i = 0; i < modules.length; i++) {
            modules[i].runSetpoint(setpointStates[i]);
          }
        });
  }

  public Command zeroGyroCommand() {
    return new InstantCommand(
        () -> {
          overrideGyroAngle(0);
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

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
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

  public ChassisSpeeds getFieldVelocity() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    var speeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(kinematics.toChassisSpeeds(states), getRotation());
    return new ChassisSpeeds(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  public ChassisSpeeds getRobotVelocity() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(getFieldVelocity(), getRotation());
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < modules.length; i++) {
      modules[i].runCharacterization(output);
    }
  }

  public void resetPose(Pose2d pose) {
    rawGyroRotation = (pose.getRotation());
    gyroIO.setYaw(rawGyroRotation.getDegrees());
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  void overrideGyroAngle(double angleDegrees) {
    resetPose(
        new Pose2d(
            getPose().getTranslation(), new Rotation2d(Units.degreesToRadians(angleDegrees))));
  }

  /** Returns the current odometry pose. */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return RealConstants.MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / RealConstants.DRIVE_BASE_RADIUS;
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

  public Command moveToPoint(Supplier<Pose2d> targetPose) {
    return this.run(
            () -> {
              Pose2d setpoint = targetPose.get();
              DogLog.log("DriveToPose/Setpoint", setpoint);
              Pose2d currentPose = getPose();
              DogLog.log("DriveToPose/CurrentPose", currentPose);
              double x_velo = translationController.calculate(currentPose.getX(), setpoint.getX());
              double y_velo = translationController.calculate(currentPose.getY(), setpoint.getY());
              double theta_velo =
                  thetaController.calculate(
                      currentPose.getRotation().getRadians(), setpoint.getRotation().getRadians());
              ChassisSpeeds speeds = new ChassisSpeeds(x_velo, y_velo, theta_velo);
              ChassisSpeeds allianceSpeeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rawGyroRotation);
              ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(allianceSpeeds, 0.02);
              SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
              SwerveDriveKinematics.desaturateWheelSpeeds(
                  setpointStates, RealConstants.MAX_LINEAR_SPEED);

              DogLog.log("Drive/RobotRelativeTargetSpeeds", discreteSpeeds);
              DogLog.log("Drive/SwerveStates/OptimizedSetpoints", setpointStates);
              for (int i = 0; i < modules.length; i++) {
                modules[i].runSetpoint(setpointStates[i]);
              }
            })
        .until(() -> (thetaController.atGoal() && translationController.atSetpoint()));
  }
}
