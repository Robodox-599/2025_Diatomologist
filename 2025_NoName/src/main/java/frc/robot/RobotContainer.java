package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.AlgaeLevel;
import frc.robot.subsystems.Superstructure.AutoAlignSide;
import frc.robot.subsystems.Superstructure.AutomationLevel;
import frc.robot.subsystems.Superstructure.CoralScoreLevel;
import frc.robot.subsystems.Superstructure.GamePieceState;
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

  //   // Setting up bindings for necessary control of the swerve drive platform
  //   private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  //   private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public RobotContainer() {
    safetyChecker = new SafetyChecker();
    switch (Constants.currentMode) {
      case REAL:
        elevator = new Elevator(new ElevatorIOTalonFX(), safetyChecker);
        rollers = new Rollers(new RollersIOTalonFX(), safetyChecker);
        wrist = new Wrist(new WristIOTalonFX(), safetyChecker);
        drivetrain = TunerConstants.createDrivetrain(driver);
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
        drivetrain = TunerConstants.createDrivetrain(driver);
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
        drivetrain = TunerConstants.createDrivetrain(driver);
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
    // ZERO GYRO
    driver.y().onTrue(superstructureCommands.zeroGyroCommand());
    // // SET WANTED STATE TO A LOGIC STATE
    driver
        .rightBumper()
        .onTrue(
            superstructureCommands.setWantedSuperStateCommand(
                superstructureCommands.returnLogicState()));
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
    // SET WANTED STATE TO AUTO SCORE CORAL (OR AUTO ALIGN ONLY IF AUTOMATION LEVEL IS MANUAL)
    driver
        .povRight()
        .whileTrue(
            superstructureCommands.setWantedSuperStateCommand(
                superstructureCommands.returnAutoCoralScoreState()));
    // // SET WANTED STATE TO AUTO INTAKE ALGAE FROM THE REEF (OR AUTO ALIGN ONLY IF AUTOMATION
    // LEVEL IS MANUAL)
    driver
        .povLeft()
        .whileTrue(
            superstructureCommands.setWantedSuperStateCommand(
                superstructureCommands.returnAutoAlgaeIntakeState()));
    // SET TELEOP DRIVE STATE WHEN AUTO ALIGN IS RELEASED
    driver.povLeft().onFalse(superstructureCommands.setTeleopDriveStateCommand());
    driver.povRight().onFalse(superstructureCommands.setTeleopDriveStateCommand());
    // SET AUTOMATION LEVEL TO AUTO SCORE (AUTO ALIGN, RAISE, AND SCORE)
    driver
        .povUp()
        .onTrue(superstructureCommands.setAutomationLevelCommand(AutomationLevel.AUTO_ACTION));
    // SET AUTOMATION LEVEL TO MANUAL (ONLY AUTO ALIGN)
    driver
        .povDown()
        .onTrue(superstructureCommands.setAutomationLevelCommand(AutomationLevel.MANUAL));

    //                                OPERATOR BINDS
    // // QUEUE CORAL L1 OR QUEUE ALGAE L2
    operator
        .x()
        .onTrue(
            Commands.either(
                superstructureCommands.setCoralScoreLevelCommand(CoralScoreLevel.POSITION_CORAL_L1),
                superstructureCommands.setAlgaeLevelCommand(AlgaeLevel.INTAKING_ALGAE_L2),
                superstructureCommands.isGamePieceStateCoral()));
    // // SET WANTED STATE TO L1 OR SET WANTED STATE TO ALGAE L2
    operator
        .x()
        .and(operator.leftTrigger())
        .onTrue(
            Commands.either(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.POSITION_CORAL_L1),
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.INTAKING_ALGAE_L2),
                superstructureCommands.isGamePieceStateCoral()));
    // // QUEUE CORAL L2 OR QUEUE ALGAE PROCESSOR
    operator
        .a()
        .onTrue(
            Commands.either(
                superstructureCommands.setCoralScoreLevelCommand(CoralScoreLevel.POSITION_CORAL_L2),
                superstructureCommands.setAlgaeLevelCommand(AlgaeLevel.POSITION_ALGAE_PROCESSOR),
                superstructureCommands.isGamePieceStateCoral()));
    // // SET WANTED STATE TO L2 OR SET WANTED STATE TO ALGAE PROCESSOR
    operator
        .a()
        .and(operator.leftTrigger())
        .onTrue(
            Commands.either(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.POSITION_CORAL_L2),
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.POSITION_ALGAE_PROCESSOR),
                superstructureCommands.isGamePieceStateCoral()));
    // // QUEUE CORAL L3 OR QUEUE ALGAE L3
    operator
        .b()
        .onTrue(
            Commands.either(
                superstructureCommands.setCoralScoreLevelCommand(CoralScoreLevel.POSITION_CORAL_L3),
                superstructureCommands.setAlgaeLevelCommand(AlgaeLevel.INTAKING_ALGAE_L3),
                superstructureCommands.isGamePieceStateCoral()));
    // // SET WANTED STATE TO L3 OR SET WANTED STATE TO ALGAE L3
    operator
        .b()
        .and(operator.leftTrigger())
        .onTrue(
            Commands.either(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.POSITION_CORAL_L3),
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.INTAKING_ALGAE_L3),
                superstructureCommands.isGamePieceStateCoral()));
    // // QUEUE CORAL L4 OR QUEUE ALGAE BARGE
    operator
        .y()
        .onTrue(
            Commands.either(
                superstructureCommands.setCoralScoreLevelCommand(CoralScoreLevel.POSITION_CORAL_L4),
                superstructureCommands.setAlgaeLevelCommand(AlgaeLevel.POSITION_ALGAE_BARGE),
                superstructureCommands.isGamePieceStateCoral()));
    // // SET WANTED STATE TO L4 OR SET WANTED STATE TO ALGAE BARGE
    operator
        .y()
        .and(operator.leftTrigger())
        .onTrue(
            Commands.either(
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.POSITION_CORAL_L4),
                superstructureCommands.setWantedSuperStateCommand(
                    WantedSuperState.POSITION_ALGAE_BARGE),
                superstructureCommands.isGamePieceStateCoral()));
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
    // // SET AUTO ALIGN TO LEFT
    operator.povLeft().onTrue(superstructureCommands.setAutoAlignSideCommand(AutoAlignSide.LEFT));
    // // SET AUTO ALIGN TO RIGHT
    operator.povRight().onTrue(superstructureCommands.setAutoAlignSideCommand(AutoAlignSide.RIGHT));
    // // SET GAME PIECE STATE TO CORAL
    operator.povUp().onTrue(superstructureCommands.setGamePieceStateCommand(GamePieceState.CORAL));
    // // SET GAME PIECE STATE TO ALGAE
    operator
        .povDown()
        .onTrue(superstructureCommands.setGamePieceStateCommand(GamePieceState.ALGAE));
  }

  /* DRIVE COMMANDS (NOT USED IN COMPETITION)
  * // // // reset the field-centric heading on left bumper press
   // // driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

   // // SET DRIVE VELOCITY (FOR PID TUNING)
   // driver
   //     .x()
   //     .whileTrue(
   //         drivetrain.applyRequest(
   //             () -> drive.withVelocityX(1.5).withVelocityY(0).withRotationalRate(0)));
   // // BRAKE (FOR PID TUNING)
   // driver.b().whileTrue(drivetrain.applyRequest(() -> brake));

   // // Run SysId routines when holding back/start and X/Y.
   // // Note that each routine ssdx hould be run exactly once in a single log.
   // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
   // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
   // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
   // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
  */
}
