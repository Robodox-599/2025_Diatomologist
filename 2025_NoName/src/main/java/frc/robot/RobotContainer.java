package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.*;
import frc.robot.commands.SuperstructureCommands;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
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
import frc.robot.subsystems.leds.LEDsConstants.LEDStates;
import frc.robot.subsystems.leds.LEDsIOReal;
import frc.robot.subsystems.leds.LEDsIOSim;
import frc.robot.subsystems.subsystemvisualizer.SubsystemVisualizer;
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
  private final Drive drive;
  private Elevator elevator;
  private Wrist wrist;
  private Rollers rollers;
  private LEDs LEDs;
  private Vision vision;
  private SafetyChecker safetyChecker;
  private SubsystemVisualizer subsystemVisualizer;
  private final SuperstructureCommands superstructureCommands;
  // Auto components
  private final AutoRoutines autoRoutines;
  private final AutoFactory autoFactory;

  public final AutoChooser autoChooser = new AutoChooser();

  public RobotContainer() {
    safetyChecker = new SafetyChecker();

    switch (Constants.currentMode) {
      case REAL:
        elevator = new Elevator(new ElevatorIOTalonFX(), safetyChecker);
        rollers = new Rollers(new RollersIOTalonFX());
        wrist = new Wrist(new WristIOTalonFX(), safetyChecker);
        drive = new Drive(new GyroIOPigeon2(), Drive.createTalonFXModules());
        LEDs = new LEDs(new LEDsIOReal());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOReal(RealConstants.cam2Constants, drive::getPose),
                new VisionIOReal(RealConstants.cam1Constants, drive::getPose),
                new VisionIOReal(RealConstants.cam3Constants, drive::getPose));
        autoFactory =
            new AutoFactory(drive::getPose, drive::resetPose, drive::followChoreoPath, true, drive);
        break;
      case SIM:
        DriverStation.silenceJoystickConnectionWarning(true);
        elevator = new Elevator(new ElevatorIOSim(), safetyChecker);
        rollers = new Rollers(new RollersIOSim());
        wrist = new Wrist(new WristIOSim(), safetyChecker);
        drive = new Drive(new GyroIO() {}, Drive.createSimModules());
        LEDs = new LEDs(new LEDsIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOSim(RealConstants.cam2Constants, drive::getPose),
                new VisionIOSim(RealConstants.cam1Constants, drive::getPose),
                new VisionIOReal(RealConstants.cam3Constants, drive::getPose));
        autoFactory =
            new AutoFactory(drive::getPose, drive::resetPose, drive::followChoreoPath, true, drive);
        break;
      default:
        DriverStation.silenceJoystickConnectionWarning(true);
        elevator = new Elevator(new ElevatorIOSim(), safetyChecker);
        rollers = new Rollers(new RollersIOSim());
        wrist = new Wrist(new WristIOSim(), safetyChecker);
        drive = new Drive(new GyroIO() {}, Drive.createSimModules());
        LEDs = new LEDs(new LEDsIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOSim(RealConstants.cam2Constants, drive::getPose),
                new VisionIOSim(RealConstants.cam1Constants, drive::getPose));
        autoFactory =
            new AutoFactory(drive::getPose, drive::resetPose, drive::followChoreoPath, true, drive);
        break;
    }

    superstructureCommands =
        new SuperstructureCommands(drive, elevator, wrist, rollers, LEDs, driver, operator);
    autoRoutines = new AutoRoutines(autoFactory, superstructureCommands);

    // Run no state when disabled
    RobotModeTriggers.disabled().onTrue(LEDs.setState(LEDStates.IDLE));

    // Auto chooser setup
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    // Add auto routines
    // COMPETITION
    autoChooser.addRoutine("rightAutoRoutine", autoRoutines::rightAutoRoutine);
    autoChooser.addRoutine("taxiAutoRoutine", autoRoutines::taxiAutoRoutine);
    autoChooser.addRoutine("leftAutoRoutine", autoRoutines::leftAutoRoutine);
    autoChooser.addRoutine("middleAutoRoutine", autoRoutines::middleAutoRoutine);
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

    configureBindings();
  }

  private void configureBindings() {
    superstructureCommands.configureBindings();
  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommandScheduler();
  }

  /*
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Setting up bindings for necessary control of the swerve drive platform 
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
 */
}
