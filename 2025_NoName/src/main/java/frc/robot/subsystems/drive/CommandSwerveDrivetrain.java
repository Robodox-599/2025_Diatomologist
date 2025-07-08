package frc.robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.subsystems.drive.constants.TunerConstants.TunerSwerveDrivetrain;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds =
      new SwerveRequest.ApplyFieldSpeeds();

  private final SwerveRequest.FieldCentric swreq_drive =
      new SwerveRequest.FieldCentric().withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  private final PIDController choreoTranslationPID = new PIDController(10, 0, 0);
  private final ProfiledPIDController choreoThetaPID =
      new ProfiledPIDController(
          10,
          0,
          0,
          new TrapezoidProfile.Constraints(
              TunerConstants.MAX_ANGULAR_SPEED, TunerConstants.MAX_ANGULAR_ACCELERATION));
  ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(
              TunerConstants.MAX_ANGULAR_SPEED, TunerConstants.MAX_ANGULAR_ACCELERATION));
  private final PIDController translationController = new PIDController(0.0, 0.0, 0.0);

  private CurrentState currentState = CurrentState.TELEOP_DRIVE;
  private WantedState wantedState = WantedState.TELEOP_DRIVE;

  private Pose2d desiredPoseForDriveToPoint = new Pose2d();

  public enum WantedState {
    TELEOP_DRIVE,
    DRIVE_TO_POINT,
  }

  public enum CurrentState {
    TELEOP_DRIVE,
    DRIVE_TO_POINT,
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    choreoThetaPID.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Units.degreesToRadians(3));
    translationController.setTolerance(0.02);
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    choreoThetaPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    choreoThetaPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public void updateInputs() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    
    handleStateTransitions();
    }

    DogLog.log("RobotPose", getState().Pose);
  }

  private CurrentState handleStateTransitions() {
    switch (wantedState) {
      case TELEOP_DRIVE:
        currentState = CurrentState.TELEOP_DRIVE;
        break;
      case DRIVE_TO_POINT:
        currentState = CurrentState.DRIVE_TO_POINT;
        break;
      default:
        currentState = CurrentState.TELEOP_DRIVE;
        break;
    }
    return currentState;
  }

  private void applyStates() {
    switch (currentState) {
      case TELEOP_DRIVE:
        break;
      case DRIVE_TO_POINT:
        DogLog.log("Drive/DriveToPose/DesiredPoseForDriveToPoint", desiredPoseForDriveToPoint);

        Pose2d currentPose = getState().Pose;
        DogLog.log("Drive/DriveToPose/CurrentPose", currentPose);

        double xSpeed = translationController.calculate(currentPose.getX(), desiredPoseForDriveToPoint.getX());
        double ySpeed = translationController.calculate(currentPose.getY(), desiredPoseForDriveToPoint.getY());
        double thetaSpeed =
          thetaController.calculate(
              currentPose.getRotation().getRadians(),
              desiredPoseForDriveToPoint.getRotation().getRadians());
        setControl(
          swreq_drive
              .withVelocityX(xSpeed)
              .withVelocityY(ySpeed)
              .withRotationalRate(thetaSpeed));
        break;
      default:
        break;
    }
  }

  public ChassisSpeeds getChassisSpeeds() {
    return getState().Speeds;
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  public Pose2d getPose() {
    return getState().Pose;
  }

  public void followChoreoPath(SwerveSample sample) {
    var pose = getState().Pose;

    var targetSpeeds = sample.getChassisSpeeds();
    DogLog.log("Drive/Choreo/RobotPose2d", pose);
    DogLog.log("Drive/Choreo/SwerveSample", sample);
    DogLog.log("Drive/Choreo/SwerveSample/ChoreoPosition", sample.getPose());
    DogLog.log("Drive/Choreo/RealRobotPosition", pose);

    targetSpeeds.vxMetersPerSecond += choreoTranslationPID.calculate(pose.getX(), sample.x);
    targetSpeeds.vyMetersPerSecond += choreoTranslationPID.calculate(pose.getY(), sample.y);
    targetSpeeds.omegaRadiansPerSecond +=
        choreoThetaPID.calculate(pose.getRotation().getRadians(), sample.heading);

    DogLog.log("Drive/Choreo/RobotSetpointSpeedsAfterPID", targetSpeeds);

    setControl(
        m_pathApplyFieldSpeeds.withSpeeds(targetSpeeds)
        .withWheelForceFeedforwardsX(sample.moduleForcesX())
        .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
  }

  public void zeroGyro() {
    resetRotation(new Rotation2d(0.0));
  }

  public Command zeroGyroCommand() {
    return new InstantCommand(
        () -> {
          resetRotation(new Rotation2d(0.0));
        });
  }

  public Command move3mForward() {
    return this.run(
        () -> {
          Pose2d setpoint =
              (getState().Pose)
                  .plus(new Transform2d(new Translation2d(1.0, 0.0), new Rotation2d()));
        });
  }

  public Command moveToPoint(
      Supplier<Pose2d> targetPose,
      boolean thetaToleranceEnabled,
      boolean translationToleranceEnabled) {
    return this.run(
            () -> {
              Pose2d setpoint = targetPose.get();
              DogLog.log("Drive/DriveToPose/Setpoint", setpoint);

              Pose2d currentPose = getState().Pose;
              DogLog.log("Drive/DriveToPose/CurrentPose", currentPose);

              double xSpeed = translationController.calculate(currentPose.getX(), setpoint.getX());
              double ySpeed = translationController.calculate(currentPose.getY(), setpoint.getY());
              double thetaSpeed =
                  thetaController.calculate(
                      currentPose.getRotation().getRadians(),
                      targetPose.get().getRotation().getRadians());

              setControl(
                  swreq_drive
                      .withVelocityX(xSpeed)
                      .withVelocityY(ySpeed)
                      .withRotationalRate(thetaSpeed));
            })
        .until(
            () ->
                ((!thetaToleranceEnabled || thetaController.atGoal())
                    && (!translationToleranceEnabled || translationController.atSetpoint())));
  }

  public void setDesiredPoseForDriveToPoint(Pose2d desiredPose) {
    this.desiredPoseForDriveToPoint = desiredPose;
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    // super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to {@link
   * #setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement in the form
   *     [x, y, theta]ᵀ, with units in meters and radians.
   */
  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    // super.addVisionMeasurement(
    // visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }
}
