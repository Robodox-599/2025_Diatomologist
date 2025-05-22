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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.constants.RealConstants;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;

public class RobotContainer {
  // Controllers
  private final CommandXboxController driver =
      new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);
  private final CommandXboxController operator =
      new CommandXboxController(Constants.ControllerConstants.kOperatorControllerPort);

  // Subsystems
  private CommandSwerveDrivetrain drivetrain;
  private Elevator elevator;
  private Wrist wrist;
  private Rollers rollers;
  private LEDs LEDs;
  private Vision vision;
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
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
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
        LEDs = new LEDs(new LEDsIOReal());
        vision =
            new Vision(
                drivetrain::addVisionMeasurement,
                drivetrain::getChassisSpeeds,
                new VisionIOReal(RealConstants.cam2Constants),
                new VisionIOReal(RealConstants.cam1Constants),
                new VisionIOReal(RealConstants.cam3Constants));
        autoFactory =
            new AutoFactory(
                drivetrain::getPose,
                drivetrain::resetPose,
                drivetrain::followChoreoPath,
                true,
                drivetrain);
        break;
      case SIM:
        DriverStation.silenceJoystickConnectionWarning(true);
        elevator = new Elevator(new ElevatorIOSim(), safetyChecker);
        rollers = new Rollers(new RollersIOSim(), safetyChecker);
        wrist = new Wrist(new WristIOSim(), safetyChecker);
        drivetrain = TunerConstants.createDrivetrain();
        LEDs = new LEDs(new LEDsIOSim());
        vision =
            new Vision(
                drivetrain::addVisionMeasurement,
                drivetrain::getChassisSpeeds,
                new VisionIOSim(RealConstants.cam2Constants, drivetrain::getPose),
                new VisionIOSim(RealConstants.cam1Constants, drivetrain::getPose),
                new VisionIOSim(RealConstants.cam3Constants, drivetrain::getPose));
        autoFactory =
            new AutoFactory(
                drivetrain::getPose,
                drivetrain::resetPose,
                drivetrain::followChoreoPath,
                true,
                drivetrain);
        break;
      default:
        DriverStation.silenceJoystickConnectionWarning(true);
        elevator = new Elevator(new ElevatorIOSim(), safetyChecker);
        rollers = new Rollers(new RollersIOSim(), safetyChecker);
        wrist = new Wrist(new WristIOSim(), safetyChecker);
        drivetrain = TunerConstants.createDrivetrain();
        LEDs = new LEDs(new LEDsIOSim());
        vision =
            new Vision(
                drivetrain::addVisionMeasurement,
                drivetrain::getChassisSpeeds,
                new VisionIOSim(RealConstants.cam2Constants, drivetrain::getPose),
                new VisionIOSim(RealConstants.cam1Constants, drivetrain::getPose),
                new VisionIOSim(RealConstants.cam3Constants, drivetrain::getPose));
        autoFactory =
            new AutoFactory(
                drivetrain::getPose,
                drivetrain::resetPose,
                drivetrain::followChoreoPath,
                true,
                drivetrain);
        break;
    }

    drivetrain.registerTelemetry(logger::telemeterize);

    superstructureCommands =
        new Superstructure(drivetrain, elevator, wrist, rollers, LEDs, safetyChecker);

    autoRoutines = new AutoRoutines(autoFactory, superstructureCommands);

    // Auto chooser setup
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    // Add auto routines
    // COMPETITION
    // autoChooser.addRoutine("rightAutoRoutine", autoRoutines::rightAutoRoutine);
    // autoChooser.addRoutine("taxiAutoRoutine", autoRoutines::taxiAutoRoutine);
    // autoChooser.addRoutine("leftAutoRoutine", autoRoutines::leftAutoRoutine);
    // autoChooser.addRoutine("middleAutoRoutine", autoRoutines::middleAutoRoutine);
    // autoChooser.addRoutine(
    // "DO NOT USE - middleAutoRoutineWithAlgae", autoRoutines::middleAutoRoutineWithAlgae);

    // TESTING ONLY
    // autoChooser.addRoutine("DO NOT USE - testingAutoRoutine", autoRoutines::testingAutoRoutine);
    // autoChooser.addRoutine("startTo15FeetAutoRoutine", autoRoutines::startTo15FeetAutoRoutine);

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

  public Command rumbleControllers() {
    return new StartEndCommand(
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 1),
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0))
        .alongWith(
            new StartEndCommand(
                () -> operator.getHID().setRumble(RumbleType.kBothRumble, 1),
                () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0)))
        .withTimeout(0.5);
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

    // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // driver
    //     .b()
    //     .whileTrue(
    //         drivetrain.applyRequest(
    //             () ->
    //                 point.withModuleDirection(
    //                     new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // reset the field-centric heading on left bumper press
    // driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // ZERO GYRO
    driver.y().onTrue(superstructureCommands.zeroGyroCommand());
    // // UPDATE STATE WITH OPERATOR STATE
    driver.rightTrigger().onTrue(superstructureCommands.updateWantedSuperStateCommand());
    // // INTAKE ALGAE GROUND
    driver
        .leftTrigger()
        .onTrue(
            Commands.sequence(
                superstructureCommands.setNextSuperStateCommand(
                    WantedSuperState.INTAKING_ALGAE_GROUND),
                superstructureCommands.updateWantedSuperStateCommand()));
    // // SCORE ALGAE
    driver
        .leftBumper()
        .onTrue(
            Commands.sequence(
                superstructureCommands.setNextSuperStateCommand(WantedSuperState.SCORING_ALGAE),
                superstructureCommands.updateWantedSuperStateCommand()));

    //                                OPERATOR BINDS
    // // MOVE TO L1
    operator
        .x()
        .onTrue(superstructureCommands.setNextSuperStateCommand(WantedSuperState.SCORING_CORAL_L1));
    // // MOVE TO L2
    operator
        .a()
        .onTrue(superstructureCommands.setNextSuperStateCommand(WantedSuperState.SCORING_CORAL_L2));
    // // MOVE TO L3
    operator
        .b()
        .onTrue(superstructureCommands.setNextSuperStateCommand(WantedSuperState.SCORING_CORAL_L3));
    // // MOVE TO L4
    operator
        .y()
        .onTrue(superstructureCommands.setNextSuperStateCommand(WantedSuperState.SCORING_CORAL_L4));
    // // MOVE TO BARGE
    operator
        .povLeft()
        .onTrue(
            superstructureCommands.setNextSuperStateCommand(
                WantedSuperState.MOVING_TO_ALGAE_BARGE));
    // // MOVE TO PROCESSOR
    operator
        .povRight()
        .onTrue(
            superstructureCommands.setNextSuperStateCommand(
                WantedSuperState.MOVING_TO_ALGAE_PROCESSOR));
    // // CORAL STATION INTAKE
    operator
        .rightBumper()
        .onTrue(
            Commands.sequence(
                superstructureCommands.setNextSuperStateCommand(
                    WantedSuperState.INTAKING_CORAL_STATION),
                superstructureCommands.updateWantedSuperStateCommand()));
    // // PREPARE
    operator
        .leftBumper()
        .onTrue(
            Commands.sequence(
                superstructureCommands.setNextSuperStateCommand(WantedSuperState.PREPARED),
                superstructureCommands.updateWantedSuperStateCommand()));
    // // INTAKE ALGAE L2
    operator
        .povDown()
        .onTrue(
            superstructureCommands.setNextSuperStateCommand(WantedSuperState.INTAKING_ALGAE_L2));
    // // INTAKE ALGAE L3
    operator
        .povUp()
        .onTrue(
            superstructureCommands.setNextSuperStateCommand(WantedSuperState.INTAKING_ALGAE_L3));
  }

  private static double joystickDeadbandApply(double x) {
    return MathUtil.applyDeadband(
        (Math.signum(x) * (1.01 * Math.pow(x, 2) - 0.0202 * x + 0.0101)), 0.02);
  }
}
