package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import choreo.util.ChoreoAllianceFlipUtil;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.subsystems.drive.constants.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.SubsystemChecker;
import frc.robot.util.Tracer;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
  private final SubsystemChecker subsystemChecker;

  private final Telemetry telemetry =
      new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
  CommandXboxController driver;
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(0)
          .withRotationalDeadband(0) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.Velocity); // Use open-loop control for drive motors

  private final double DRIVE_TO_POINT_MAX_VELOCITY_OUTPUT = 3.0;
  public static final double DRIVE_TO_POINT_TRANSLATION_ERROR_TOLERANCE = 0.02; // 2 cm
  private final double DRIVE_TO_POINT_Y_ERROR_TOLERANCE = 0.05; // 5 cm
  private final double DRIVE_TO_POINT_ANGULAR_ERROR_TOLERANCE = Units.degreesToRadians(8);
  private boolean withinCoralRaiseDistance = false;
  private static boolean withinAlgaeRaiseDistance = false;
  private static boolean withinTroughRaiseDistance = false;
  private static boolean atDriveToPointSetpoints = false;
  private static boolean withinReefZone = false;
  private static double driveToPointXError;
  private static double driveToPointYError;

  private final PIDController choreoXController = new PIDController(7, 0, 0);
  private final PIDController choreoYController = new PIDController(7, 0, 0);
  private final PIDController choreoThetaPID = new PIDController(7, 0, 0);
  private SwerveSample choreoSampleToBeApplied;

  private Pose2d targetPoseForDriveToPoint = new Pose2d();

  private final PIDController driveToPointController =
      new PIDController(2.5, 0.0, 0.0); // P: 3.0/3.6, D: 0.1
  private final SwerveRequest.FieldCentricFacingAngle driveAtAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position);

  private CurrentState currentState = CurrentState.TELEOP_DRIVE;
  private WantedState wantedState = WantedState.TELEOP_DRIVE;

  public enum WantedState {
    TELEOP_DRIVE,
    DRIVE_TO_POINT,
    CHOREO_TRAJECTORY,
  }

  public enum CurrentState {
    TELEOP_DRIVE,
    DRIVE_TO_POINT,
    CHOREO_TRAJECTORY,
  }

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

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
      CommandXboxController driver,
      SubsystemChecker subsystemChecker,
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    this.driver = driver;
    this.subsystemChecker = subsystemChecker;

    driveAtAngle.HeadingController = new PhoenixPIDController(5, 0, 0);
    driveAtAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    choreoThetaPID.enableContinuousInput(-Math.PI, Math.PI);

    this.registerTelemetry(telemetry::telemeterize);

    resetDriveToPoint();
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

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  public void updateInputs() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    // if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
    //   DriverStation.getAlliance()
    //       .ifPresent(
    //           allianceColor -> {
    //             setOperatorPerspectiveForward(
    //                 allianceColor == Alliance.Red
    //                     ? kRedAlliancePerspectiveRotation
    //                     : kBlueAlliancePerspectiveRotation);
    //             m_hasAppliedOperatorPerspective = true;
    //           });
    // }
    Tracer.traceFunc("HandleStateTransitions", this::handleStateTransitions);
    Tracer.traceFunc("ApplyStates", this::applyStates);
    DogLog.log("Drive/CurrentState", currentState);
    DogLog.log("Drive/WantedState", wantedState);
    DogLog.log("RobotPose", getState().Pose);
    updateDistancesAndSetpoints();

    subsystemChecker.setRobotPose(getState().Pose);
  }

  public void setWantedState(WantedState wantedState) {
    if (this.wantedState == WantedState.DRIVE_TO_POINT
        && wantedState
            == WantedState
                .TELEOP_DRIVE) { // when going from drive to point to teleop, reset drive to point
      resetDriveToPoint();
    }
    this.wantedState = wantedState;
  }

  private void handleStateTransitions() {
    switch (wantedState) {
      case TELEOP_DRIVE:
        currentState = CurrentState.TELEOP_DRIVE;
        break;
      case DRIVE_TO_POINT:
        if (isAtDriveToPointSetpoints()) {
          if (DriverStation.isAutonomous()) {
            wantedState = WantedState.CHOREO_TRAJECTORY;
            currentState = CurrentState.CHOREO_TRAJECTORY;
          } else {
            wantedState = WantedState.TELEOP_DRIVE;
            currentState = CurrentState.TELEOP_DRIVE;
          }
          resetDriveToPoint();
        } else {
          currentState = CurrentState.DRIVE_TO_POINT;
        }
        break;
      case CHOREO_TRAJECTORY:
        if (!DriverStation.isAutonomous()) {
          wantedState = WantedState.TELEOP_DRIVE;
          currentState = CurrentState.TELEOP_DRIVE;
        } else {
          currentState = CurrentState.CHOREO_TRAJECTORY;
        }
        break;
      default:
        currentState = CurrentState.TELEOP_DRIVE;
        break;
    }
  }

  private void applyStates() {
    switch (currentState) {
      case TELEOP_DRIVE:
        setControl(
            drive
                .withVelocityX(
                    -joystickDeadbandApply(driver.getLeftY())
                        * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(
                    -joystickDeadbandApply(driver.getLeftX())
                        * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(
                    joystickDeadbandApply(-driver.getRightX())
                        * MaxAngularRate)); // Drive counterclockwise with negative X (left));
        break;
      case DRIVE_TO_POINT:
        Translation2d translationToTarget =
            targetPoseForDriveToPoint.getTranslation().minus(getState().Pose.getTranslation());

        double linearDistance = translationToTarget.getNorm();

        Rotation2d direction = translationToTarget.getAngle();
        double velocityOutput =
            Math.min(
                Math.abs(driveToPointController.calculate(linearDistance, 0)),
                DRIVE_TO_POINT_MAX_VELOCITY_OUTPUT);

        double xVelocity = velocityOutput * direction.getCos();
        double yVelocity = velocityOutput * direction.getSin();

        DogLog.log("Drive/DriveToPose/TargetPoseForDriveToPoint", targetPoseForDriveToPoint);
        DogLog.log("Drive/DriveToPose/VelocityOutput", velocityOutput);
        DogLog.log("Drive/DriveToPose/XVelocity", xVelocity);
        DogLog.log("Drive/DriveToPose/YVelocity", yVelocity);

        setControl(
            driveAtAngle
                .withVelocityX(xVelocity)
                .withVelocityY(yVelocity)
                .withTargetDirection(targetPoseForDriveToPoint.getRotation()));
        break;
      case CHOREO_TRAJECTORY:
        if (choreoSampleToBeApplied != null) {
          SwerveSample sample = choreoSampleToBeApplied;
          choreoSampleToBeApplied = null;

          var pose = getState().Pose;

          var targetSpeeds = sample.getChassisSpeeds();
          DogLog.log("Drive/Choreo/RobotPose2d", pose);
          DogLog.log("Drive/Choreo/SwerveSample", sample);
          DogLog.log("Drive/Choreo/SwerveSample/ChoreoPosition", sample.getPose());
          DogLog.log("Drive/Choreo/RealRobotPosition", pose);

          targetSpeeds.vxMetersPerSecond += choreoXController.calculate(pose.getX(), sample.x);
          targetSpeeds.vyMetersPerSecond += choreoYController.calculate(pose.getY(), sample.y);
          targetSpeeds.omegaRadiansPerSecond +=
              choreoThetaPID.calculate(pose.getRotation().getRadians(), sample.heading);

          DogLog.log("Drive/Choreo/RobotSetpointSpeedsAfterPID", targetSpeeds);

          setControl(
              m_pathApplyFieldSpeeds
                  .withSpeeds(targetSpeeds)
                  .withWheelForceFeedforwardsX(sample.moduleForcesX())
                  .withWheelForceFeedforwardsY(sample.moduleForcesY()));
        } else {
          setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        }
        break;
      default:
        break;
    }
  }

  public void updateDistancesAndSetpoints() {
    double distanceFromCenter =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? (FieldConstants.Reef.center.minus(getState().Pose.getTranslation())).getNorm()
            : ChoreoAllianceFlipUtil.flip(new Pose2d(FieldConstants.Reef.center, new Rotation2d()))
                .getTranslation()
                .minus(getState().Pose.getTranslation())
                .getNorm();
    Translation2d translationToTarget =
        targetPoseForDriveToPoint.getTranslation().minus(getState().Pose.getTranslation());
    double linearDistance = translationToTarget.getNorm();

    if (distanceFromCenter <= 2.5) { // distance from center + half bumper width + reef zone
      withinCoralRaiseDistance = true;
      withinReefZone = true;
    } else {
      withinCoralRaiseDistance = false;
      withinReefZone = false;
    }
    if (distanceFromCenter <= 2.0) {
      withinAlgaeRaiseDistance = true;
    } else {
      withinAlgaeRaiseDistance = false;
    }
    if (distanceFromCenter <= 2.5) {
      withinTroughRaiseDistance = true;
    } else {
      withinTroughRaiseDistance = false;
    }
    boolean atDriveToPointTranslationSetpoint =
        MathUtil.isNear(0.0, linearDistance, DRIVE_TO_POINT_TRANSLATION_ERROR_TOLERANCE);
    boolean atDriveToPointAngularSetpoint =
        driveAtAngle.HeadingController.getPositionError() < DRIVE_TO_POINT_ANGULAR_ERROR_TOLERANCE;
    atDriveToPointSetpoints = atDriveToPointTranslationSetpoint && atDriveToPointAngularSetpoint;
    driveToPointXError = translationToTarget.getX();
    driveToPointYError = translationToTarget.getY();

    DogLog.log("Drive/DriveToPose/LinearDistance", linearDistance);
    DogLog.log("Drive/DriveToPose/xError", driveToPointXError);
    DogLog.log("Drive/DriveToPose/yError", driveToPointYError);
    DogLog.log("Drive/DriveToPose/WithinCoralRaiseDistance", withinCoralRaiseDistance);
    DogLog.log("Drive/DriveToPose/WithinAlgaeRaiseDistance", withinAlgaeRaiseDistance);
    DogLog.log("Drive/DriveToPose/WithinTroughRaiseDistance", withinTroughRaiseDistance);
    DogLog.log("Drive/DriveToPose/WithinReefZone", withinReefZone);
    DogLog.log(
        "Drive/DriveToPose/AtDriveToPointTranslationSetpoint", atDriveToPointTranslationSetpoint);
    DogLog.log("Drive/DriveToPose/AtDriveToPointAngularSetpoint", atDriveToPointAngularSetpoint);
    DogLog.log("Drive/DriveToPose/AtDriveToPointSetpoints", atDriveToPointSetpoints);
  }

  public void setTargetPoseForDriveToPoint(Pose2d targetPose) {
    resetDriveToPoint();
    this.targetPoseForDriveToPoint = targetPose;
    this.wantedState = WantedState.DRIVE_TO_POINT;
  }

  public boolean isDrivingToPointOrAtSetpoints() {
    return this.wantedState == WantedState.DRIVE_TO_POINT || atDriveToPointSetpoints;
  }

  public boolean isWithinCoralRaiseDistance() {
    return withinCoralRaiseDistance;
  }

  public boolean isReadyToRaiseAutoScoreCoral() {
    return isWithinCoralRaiseDistance() && isDrivingToPointOrAtSetpoints();
  }

  public boolean isWithinAlgaeRaiseDistance() {
    return withinAlgaeRaiseDistance;
  }

  public boolean isReadyToRaiseAutoIntakeAlgae() {
    return isWithinAlgaeRaiseDistance() && isDrivingToPointOrAtSetpoints();
  }

  public boolean isWithinTroughRaiseDistance() {
    return withinTroughRaiseDistance;
  }

  public boolean isReadyToRaiseAutoTroughCoral() {
    return isWithinTroughRaiseDistance() && isDrivingToPointOrAtSetpoints();
  }

  public boolean isWithinReefZone() {
    return withinReefZone;
  }

  public boolean isAtDriveToPointSetpoints() {
    return atDriveToPointSetpoints;
  }

  public boolean isYErrorWithinTolerance() {
    return Math.abs(driveToPointYError) < DRIVE_TO_POINT_Y_ERROR_TOLERANCE;
  }

  public void resetDriveToPoint() {
    driveToPointController.reset();
    driveAtAngle.HeadingController.reset();
  }

  public void setDesiredChoreoTrajectory(SwerveSample sample) {
    this.choreoSampleToBeApplied = sample;
    this.wantedState = WantedState.CHOREO_TRAJECTORY;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return getState().Speeds;
  }

  private static double joystickDeadbandApply(double x) {
    return MathUtil.applyDeadband(
        (Math.signum(x) * (1.01 * Math.pow(x, 2) - 0.0202 * x + 0.0101)), 0.02);
  }

  public Pose2d getPose() {
    return getState().Pose;
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

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
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
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }
}
