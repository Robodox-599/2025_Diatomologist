package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.AutomationLevel;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.constants.CameraConstants;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.endefector.endefectorrollers.Rollers;
import frc.robot.subsystems.endefector.endefectorrollers.RollersIOSim;
import frc.robot.subsystems.endefector.endefectorrollers.RollersIOTalonFX;
import frc.robot.subsystems.endefector.endefectorwrist.Wrist;
import frc.robot.subsystems.endefector.endefectorwrist.WristIOSim;
import frc.robot.subsystems.endefector.endefectorwrist.WristIOTalonFX;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDsIOReal;
import frc.robot.subsystems.leds.LEDsIOSim;
import frc.robot.subsystems.vision.CameraReal;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
  // Controllers
  private final CommandXboxController driver =
      new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);
  private final CommandXboxController operator =
      new CommandXboxController(Constants.ControllerConstants.kOperatorControllerPort);

  // Subsystems
  private CommandSwerveDrivetrain drivetrain;
  private final Elevator elevator;
  private final Wrist wrist;
  private final Rollers rollers;
  private final LEDs leds;
  //   private final Climb climb;
  private final Vision vision;
  private SafetyChecker safetyChecker;
  private final Superstructure superstructureCommands;
  private final Telemetry logger =
      new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
  private final AutoRoutines autoRoutines;
  private final AutoFactory autoFactory;
  public final AutoChooser autoChooser = new AutoChooser();

  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  // Setting up bindings for necessary control of the swerve drive platform
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(0)
          .withRotationalDeadband(0) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.Velocity); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public RobotContainer() {
    safetyChecker = new SafetyChecker();
    switch (Constants.currentMode) {
      case REAL:
        elevator = new Elevator(new ElevatorIOTalonFX(), safetyChecker);
        rollers = new Rollers(new RollersIOTalonFX(), safetyChecker);
        wrist = new Wrist(new WristIOTalonFX(), safetyChecker);
        drivetrain = TunerConstants.createDrivetrain();
        leds = new LEDs(new LEDsIOReal());
        // climb = new Climb(new ClimbIOTalonFX());
        vision =
            new Vision(
                drivetrain::addVisionMeasurement,
                new CameraReal(CameraConstants.frontLeftCameraConstants),
                new CameraReal(CameraConstants.frontRightCameraConstants),
                new CameraReal(CameraConstants.backCameraConstants));
        autoFactory =
            new AutoFactory(
                drivetrain::getPose,
                drivetrain::resetPose,
                drivetrain::followChoreoPath,
                false,
                drivetrain);
        break;
      case SIM:
        DriverStation.silenceJoystickConnectionWarning(true);
        elevator = new Elevator(new ElevatorIOSim(), safetyChecker);
        rollers = new Rollers(new RollersIOSim(), safetyChecker);
        wrist = new Wrist(new WristIOSim(), safetyChecker);
        drivetrain = TunerConstants.createDrivetrain();
        leds = new LEDs(new LEDsIOSim());
        // climb = new Climb(new ClimbIOSim());
        vision =
            new Vision(
                drivetrain::addVisionMeasurement,
                new CameraReal(CameraConstants.frontLeftCameraConstants),
                new CameraReal(CameraConstants.frontRightCameraConstants),
                new CameraReal(CameraConstants.backCameraConstants));
        autoFactory =
            new AutoFactory(
                drivetrain::getPose,
                drivetrain::resetPose,
                drivetrain::followChoreoPath,
                false,
                drivetrain);
        break;
      default:
        DriverStation.silenceJoystickConnectionWarning(true);
        elevator = new Elevator(new ElevatorIOSim(), safetyChecker);
        rollers = new Rollers(new RollersIOSim(), safetyChecker);
        wrist = new Wrist(new WristIOSim(), safetyChecker);
        drivetrain = TunerConstants.createDrivetrain();
        leds = new LEDs(new LEDsIOSim());
        // climb = new Climb(new ClimbIOSim());
        vision =
            new Vision(
                drivetrain::addVisionMeasurement,
                new CameraReal(CameraConstants.frontLeftCameraConstants),
                new CameraReal(CameraConstants.frontRightCameraConstants),
                new CameraReal(CameraConstants.backCameraConstants));
        autoFactory =
            new AutoFactory(
                drivetrain::getPose,
                drivetrain::resetPose,
                drivetrain::followChoreoPath,
                false,
                drivetrain);
        break;
    }

    drivetrain.registerTelemetry(logger::telemeterize);

    superstructureCommands =
        new Superstructure(
            drivetrain,
            elevator,
            wrist,
            rollers,
            leds,
            // climb,
            vision,
            safetyChecker,
            driver,
            operator);

    autoRoutines = new AutoRoutines(autoFactory, superstructureCommands);

    // Auto chooser setup
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    /** AUTO ROUTINES - DO NOT TOUCH */
    // COMPETITION
    autoChooser.addRoutine("Left Auto - 3 Coral", autoRoutines::leftAutoRoutine);
    autoChooser.addRoutine("Right Auto - 3 Coral", autoRoutines::rightAutoRoutine);
    autoChooser.addRoutine(
        "Middle Auto & Algae - 1 Coral + Grab Algae", autoRoutines::middleAutoAndGrabAlgaeRoutine);
    autoChooser.addRoutine("Middle Auto - 1 Coral", autoRoutines::middleAutoRoutine);
    autoChooser.addRoutine("Taxi Auto - Taxi", autoRoutines::taxiAutoRoutine);

    // TESTING ONLY

    SmartDashboard.putData("AutoChooser", autoChooser);

    DogLog.setOptions(
        new DogLogOptions()
            .withCaptureDs(true)
            .withCaptureNt(true)
            .withNtPublish(true)
            .withCaptureConsole(true));

    // superstructureCommands.configureBindings();
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommandScheduler();
  }

  public void configureBindings() {
    //                               DRIVER BINDS
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -joystickDeadbandApply(driver.getLeftY())
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -joystickDeadbandApply(driver.getLeftX())
                            * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        joystickDeadbandApply(-driver.getRightX())
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    // // // reset the field-centric heading on left bumper press
    // // driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // SET DRIVE VELOCITY (FOR PID TUNING)
    driver
        .x()
        .whileTrue(
            drivetrain.applyRequest(
                () -> drive.withVelocityX(1.5).withVelocityY(0).withRotationalRate(0)));
    // BRAKE (FOR PID TUNING)
    driver.b().whileTrue(drivetrain.applyRequest(() -> brake));

    // // Run SysId routines when holding back/start and X/Y.
    // // Note that each routine ssdx hould be run exactly once in a single log.
    // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // ZERO GYRO
    driver.y().onTrue(superstructureCommands.zeroGyroCommand());
    // // UPDATE WANTED STATE WITH QUEUED STATE
    driver.rightBumper().onTrue(superstructureCommands.updateWantedSuperStateCommand());
    // // SET WANTED STATE TO INTAKING ALGAE GROUND
    driver
        .leftBumper()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(
                WantedSuperState.INTAKING_ALGAE_GROUND));
    driver
        .leftBumper()
        .onFalse(
            superstructureCommands.setWantedSuperStateCommand(
                WantedSuperState.POSITION_ALGAE_PROCESSOR));
    // // SET WANTED STATE TO SCORING CORAL
    driver
        .rightTrigger()
        .onTrue(superstructureCommands.setWantedSuperStateCommand(WantedSuperState.SCORING_CORAL));
    // // SET WANTED STATE TO SCORING ALGAE
    driver
        .leftTrigger()
        .onTrue(superstructureCommands.setWantedSuperStateCommand(WantedSuperState.SCORING_ALGAE));
    // // SET WANTED STATE TO AUTO SCORE CORAL ON LEFT BRANCH (OR AUTO ALIGN ONLY IF AUTOMATION LEVEL IS MANUAL OR IF NO
    // CORAL STATE IS SET)
    driver
        .povLeft()
        .whileTrue(
            superstructureCommands.setWantedSuperStateCommand(
                superstructureCommands.returnAutoCoralScoreState(true)));
    // // SET WANTED STATE TO AUTO SCORE CORAL ON RIGHT BRANCH (OR AUTO ALIGN ONLY IF AUTOMATION LEVEL IS MANUAL OR IF
    // NO CORAL STATE IS SET)
    driver
        .povRight()
        .whileTrue(
            superstructureCommands.setWantedSuperStateCommand(
                superstructureCommands.returnAutoCoralScoreState(false)));
    // // SET WANTED STATE TO AUTO INTAKE ALGAE FROM THE REEF (OR AUTO ALIGN ONLY IF AUTOMATION LEVEL IS MANUAL)
    driver
        .a()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(
                superstructureCommands.returnAutoAlgaeIntakeState()));
    // SET TELEOP DRIVE STATE WHEN AUTO ALIGN IS RELEASED
    driver.povLeft().onFalse(superstructureCommands.setTeleopDriveStateCommand());
    driver.povRight().onFalse(superstructureCommands.setTeleopDriveStateCommand());
    driver.a().onFalse(superstructureCommands.setTeleopDriveStateCommand());
    // SET AUTOMATION LEVEL TO AUTO SCORE (AUTO ALIGN, RAISE, AND SCORE)
    driver
        .povUp()
        .onTrue(superstructureCommands.setAutomationLevelCommand(AutomationLevel.AUTO_ACTION));
    // SET AUTOMATION LEVEL TO MANUAL (ONLY AUTO ALIGN)
    driver
        .povDown()
        .onTrue(superstructureCommands.setAutomationLevelCommand(AutomationLevel.MANUAL));

    //                                OPERATOR BINDS
    // // SET QUEUED STATE TO L1
    operator
        .x()
        .onTrue(
            superstructureCommands.setQueuedSuperStateCommand(WantedSuperState.POSITION_CORAL_L1));
    // // SET WANTED STATE TO L1
    operator
        .x()
        .and(operator.leftTrigger())
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.POSITION_CORAL_L1));
    // // SET QUEUED STATE TO L2
    operator
        .a()
        .onTrue(
            superstructureCommands.setQueuedSuperStateCommand(WantedSuperState.POSITION_CORAL_L2));
    // // SET WANTED STATE TO L2
    operator
        .a()
        .and(operator.leftTrigger())
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.POSITION_CORAL_L2));
    // // SET QUEUED STATE TO L3
    operator
        .b()
        .onTrue(
            superstructureCommands.setQueuedSuperStateCommand(WantedSuperState.POSITION_CORAL_L3));
    // // SET WANTED STATE TO L3
    operator
        .b()
        .and(operator.leftTrigger())
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.POSITION_CORAL_L3));
    // // SET QUEUED STATE TO L4
    operator
        .y()
        .onTrue(
            superstructureCommands.setQueuedSuperStateCommand(WantedSuperState.POSITION_CORAL_L4));
    // // SET WANTED STATE TO L4
    operator
        .y()
        .and(operator.leftTrigger())
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.POSITION_CORAL_L4));
    // // SET WANTED STATE TO CORAL STATION INTAKE
    operator
        .rightBumper()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(
                WantedSuperState.INTAKING_CORAL_STATION));
    // // SET WANTED STATE TO PREPARE
    operator
        .leftBumper()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.POSITION_PREPARED));
    // // SET WANTED STATE TO PREPARE CLIMB
    operator
        .rightTrigger()
        .and(operator.leftTrigger())
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(
                WantedSuperState.POSITION_CLIMB_PREPARED));
    // // SET QUEUED STATE TO BARGE
    operator
        .povUp()
        .onTrue(
            superstructureCommands.setQueuedSuperStateCommand(
                WantedSuperState.POSITION_ALGAE_BARGE));
    // // SET WANTED STATE TO BARGE
    operator
        .povUp()
        .and(operator.leftTrigger())
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(
                WantedSuperState.POSITION_ALGAE_BARGE));
    // // SET QUEUED STATE TO PROCESSOR
    operator
        .povDown()
        .onTrue(
            superstructureCommands.setQueuedSuperStateCommand(
                WantedSuperState.POSITION_ALGAE_PROCESSOR));
    // // SET WANTED STATE TO PROCESSOR
    operator
        .povDown()
        .and(operator.leftTrigger())
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(
                WantedSuperState.POSITION_ALGAE_PROCESSOR));
    // // SET QUEUED STATE TO INTAKE ALGAE L2
    operator
        .povLeft()
        .onTrue(
            superstructureCommands.setQueuedSuperStateCommand(WantedSuperState.INTAKING_ALGAE_L2));
    // // SET WANTED STATE TO INTAKE ALGAE L2
    operator
        .povLeft()
        .and(operator.leftTrigger())
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.INTAKING_ALGAE_L2));
    // // SET QUEUED STATE TO INTAKE ALGAE L3
    operator
        .povRight()
        .onTrue(
            superstructureCommands.setQueuedSuperStateCommand(WantedSuperState.INTAKING_ALGAE_L3));
    // // SET WANTED STATE TO INTAKE ALGAE L3
    operator
        .povRight()
        .and(operator.leftTrigger())
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(WantedSuperState.INTAKING_ALGAE_L3));
  }

  private static double joystickDeadbandApply(double x) {
    return MathUtil.applyDeadband(
        (Math.signum(x) * (1.01 * Math.pow(x, 2) - 0.0202 * x + 0.0101)), 0.02);
  }
}
