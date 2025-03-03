package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.*;
import frc.robot.subsystems.commands.SuperstructureCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.constants.RealConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.subsystemvisualizer.SubsystemVisualizer;

public class RobotContainer {
  // Controllers
  private final CommandXboxController driver =
      new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);
  private final CommandXboxController operator =
      new CommandXboxController(Constants.ControllerConstants.kOperatorControllerPort);

  // Subsystems
  private final Drive drive;
  // private Elevator elevator;
  // private Wrist wrist;
  // private Rollers rollers;
  // private Climb climb;
  // private LEDs LEDs;
  // private Vision vision;
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
        // elevator = new Elevator(new ElevatorIOTalonFX(), safetyChecker);
        // rollers = new Rollers(new RollersIOTalonFX());
        // wrist = new Wrist(new WristIOTalonFX(), safetyChecker);
        // climb = new Climb(new ClimbIOTalonFX());
        drive = new Drive(new GyroIOPigeon2(), Drive.createTalonFXModules());
        // LEDs = new LEDs(new LEDsIOReal());
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOReal(RealConstants.camConstants, drive::getPose));
        autoFactory =
            new AutoFactory(drive::getPose, drive::resetPose, drive::followChoreoPath, true, drive);
        // autoFactory.bind("intake", stationIntake()).bind("score", scoring(ElevatorStates.L4));
        break;
      case SIM:
        DriverStation.silenceJoystickConnectionWarning(true);
        // elevator = new Elevator(new ElevatorIOSim(), safetyChecker);
        // rollers = new Rollers(new RollersIOSim());
        // wrist = new Wrist(new WristIOSim(), safetyChecker);
        // climb = new Climb(new ClimbIOSim());
        drive = new Drive(new GyroIO() {}, Drive.createSimModules());
        // LEDs = new LEDs(new LEDsIOSim());
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOSim(RealConstants.cam1Constants, drive::getPose),
        //         new VisionIOSim(RealConstants.cam2Constants, drive::getPose));
        autoFactory =
            new AutoFactory(drive::getPose, drive::resetPose, drive::followChoreoPath, true, drive);
        // autoFactory.bind("intake", stationIntake()).bind("score", scoring(ElevatorStates.L4));
        break;
      default:
        DriverStation.silenceJoystickConnectionWarning(true);
        // elevator = new Elevator(new ElevatorIOSim(), safetyChecker);
        // rollers = new Rollers(new RollersIOSim());
        // wrist = new Wrist(new WristIOSim(), safetyChecker);
        // climb = new Climb(new ClimbIOSim());
        drive = new Drive(new GyroIO() {}, Drive.createSimModules());
        // LEDs = new LEDs(new LEDsIOSim());
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOSim(RealConstants.camConstants, drive::getPose));
        autoFactory =
            new AutoFactory(drive::getPose, drive::resetPose, drive::followChoreoPath, true, drive);
        // autoFactory.bind("intake", stationIntake()).bind("score", scoring(ElevatorStates.L4));
        break;
    }

    // subsystemVisualizer = new SubsystemVisualizer(elevator, climb, wrist, rollers);
    superstructureCommands = new SuperstructureCommands(drive, driver, operator);
    autoRoutines = new AutoRoutines(autoFactory, superstructureCommands);

    // Auto chooser setup
    SmartDashboard.putData("AutoChooser", autoChooser);
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    // Add auto routines
    autoChooser.addRoutine("rightAutoRoutine", autoRoutines::rightAutoRoutine);
    autoChooser.addRoutine("taxiAutoRoutine", autoRoutines::taxiAutoRoutine);
    autoChooser.addRoutine("leftAutoRoutine", autoRoutines::leftAutoRoutine);

    // Logging setup
    // DataLogManager.start();
    // DriverStation.startDataLog(DataLogManager.getLog());
    DogLog.setOptions(
        new DogLogOptions()
            .withCaptureDs(true)
            .withCaptureNt(true)
            .withNtPublish(true)
            .withCaptureConsole(true));

    configureBindings();
  }
  private void configureBindings(){
    superstructureCommands.configureBindings();
  }
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommandScheduler();
  }
}
