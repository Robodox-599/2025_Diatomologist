package frc.robot;

import choreo.auto.AutoChooser;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.*;
import frc.robot.commands.SuperstructureCommands;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
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
import frc.robot.subsystems.leds.LEDsConstants.LEDStates;
import frc.robot.subsystems.leds.LEDsIOReal;
import frc.robot.subsystems.leds.LEDsIOSim;
import frc.robot.subsystems.subsystemvisualizer.SubsystemVisualizer;
import frc.robot.subsystems.vision.Vision;

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
  private SubsystemVisualizer subsystemVisualizer;
  private final SuperstructureCommands superstructureCommands;
  // Auto components
  //   private final AutoRoutines autoRoutines;
  //   private final AutoFactory autoFactory;

  public final AutoChooser autoChooser = new AutoChooser();

  public RobotContainer() {
    safetyChecker = new SafetyChecker();

    switch (Constants.currentMode) {
      case REAL:
        elevator = new Elevator(new ElevatorIOTalonFX(), safetyChecker);
        rollers = new Rollers(new RollersIOTalonFX());
        wrist = new Wrist(new WristIOTalonFX(), safetyChecker);
        drivetrain = TunerConstants.createDrivetrain();
        LEDs = new LEDs(new LEDsIOReal());
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOReal(RealConstants.cam2Constants, drive::getPose),
        //         new VisionIOReal(RealConstants.cam1Constants, drive::getPose),
        //         new VisionIOReal(RealConstants.cam3Constants, drive::getPose));
        // autoFactory =
        //     new AutoFactory(drive::getPose, drive::resetPose, drive::followChoreoPath, true,
        // drive);
        break;
      case SIM:
        DriverStation.silenceJoystickConnectionWarning(true);
        elevator = new Elevator(new ElevatorIOSim(), safetyChecker);
        rollers = new Rollers(new RollersIOSim());
        wrist = new Wrist(new WristIOSim(), safetyChecker);
        drivetrain = TunerConstants.createDrivetrain();
        LEDs = new LEDs(new LEDsIOSim());
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOSim(RealConstants.cam2Constants, drive::getPose),
        //         new VisionIOSim(RealConstants.cam1Constants, drive::getPose),
        //         new VisionIOReal(RealConstants.cam3Constants, drive::getPose));
        // autoFactory =
        //     new AutoFactory(drive::getPose, drive::resetPose, drive::followChoreoPath, true,
        // drive);
        break;
      default:
        DriverStation.silenceJoystickConnectionWarning(true);
        elevator = new Elevator(new ElevatorIOSim(), safetyChecker);
        rollers = new Rollers(new RollersIOSim());
        wrist = new Wrist(new WristIOSim(), safetyChecker);
        drivetrain = TunerConstants.createDrivetrain();
        LEDs = new LEDs(new LEDsIOSim());
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOSim(RealConstants.cam2Constants, drive::getPose),
        //         new VisionIOSim(RealConstants.cam1Constants, drive::getPose));
        // autoFactory =
        //     new AutoFactory(drive::getPose, drive::resetPose, drive::followChoreoPath, true,
        // drive);
        break;
    }

    // superstructureCommands =
    //     new SuperstructureCommands(drive, elevator, wrist, rollers, LEDs, driver, operator);
    superstructureCommands =
        new SuperstructureCommands(drivetrain, elevator, wrist, rollers, LEDs, driver, operator);
    // autoRoutines = new AutoRoutines(autoFactory, superstructureCommands);

    // Run no state when disabled
    RobotModeTriggers.disabled().onTrue(LEDs.setState(LEDStates.IDLE));

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

    configureBindings();
  }

  private void configureBindings() {
    superstructureCommands.configureBindings();
  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommandScheduler();
  }
}
