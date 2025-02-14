package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.*;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbConstants.ClimbStates;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.constants.RealConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.endefector.endefectorrollers.Rollers;
import frc.robot.subsystems.endefector.endefectorrollers.RollersConstants;
import frc.robot.subsystems.endefector.endefectorrollers.RollersIOSim;
import frc.robot.subsystems.endefector.endefectorrollers.RollersIOTalonFX;
import frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.EndefectorRollerStates;
import frc.robot.subsystems.endefector.endefectorwrist.Wrist;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants;
import frc.robot.subsystems.endefector.endefectorwrist.WristIOSim;
import frc.robot.subsystems.endefector.endefectorwrist.WristIOTalonFX;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants.WristStates;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.subsystemvisualizer.subsystemvisualizer;
import frc.robot.SafetyChecker;
import java.util.Map;

public class RobotContainer {
  // Controllers
  private final CommandXboxController driver =
      new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);
  private final CommandXboxController operator =
      new CommandXboxController(Constants.ControllerConstants.kOperatorControllerPort);

  // Subsystems
  private LEDs LEDs;
  private subsystemvisualizer subsystemVisualizer;
  private Elevator elevator;
  private Rollers endefectorRollers;
  private Wrist endefectorWrist;
  private Climb climb;
  // private SafteyChecker safetyChecker;
  private final Drive drive;
  private int counter = 0;

  // Auto components
  private final AutoRoutines autoRoutines;
  private final AutoFactory autoFactory;
  public final AutoChooser autoChooser = new AutoChooser();

  public RobotContainer() {
    var safetyChecker = new SafetyChecker();
    switch (Constants.currentMode) {
      case REAL:
        elevator = new Elevator(new ElevatorIOTalonFX(), safetyChecker);
        endefectorRollers = new Rollers(new RollersIOTalonFX());
        endefectorWrist = new Wrist(new WristIOTalonFX(), safetyChecker);
        climb = new Climb(new ClimbIOTalonFX());
        drive = new Drive(new GyroIOPigeon2(), Drive.createTalonFXModules());
        autoFactory =
            new AutoFactory(drive::getPose, drive::zeroPose, drive::followChoreoPath, true, drive);
        autoRoutines = new AutoRoutines(autoFactory);
        break;

      case SIM:
        DriverStation.silenceJoystickConnectionWarning(true);
        elevator = new Elevator(new ElevatorIOSim(), safetyChecker);
        endefectorRollers = new Rollers(new RollersIOSim());
        endefectorWrist = new Wrist(new WristIOSim(), safetyChecker);
        climb = new Climb(new ClimbIOSim());
        drive = new Drive(new GyroIO() {}, Drive.createSimModules());
        autoFactory =
            new AutoFactory(drive::getPose, drive::resetPose, drive::followChoreoPath, true, drive);
        autoRoutines = new AutoRoutines(autoFactory);
        break;

      default:
        drive = new Drive(new GyroIOPigeon2(), Drive.createTalonFXModules());
        autoFactory =
            new AutoFactory(drive::getPose, drive::resetPose, drive::followChoreoPath, true, drive);
        autoRoutines = new AutoRoutines(autoFactory);
        break;
    }

    // Auto chooser setup
    SmartDashboard.putData("AutoChooser", autoChooser);
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    // Add auto routines
    autoChooser.addRoutine("rightAutoRoutine", autoRoutines::rightAutoRoutine);
    autoChooser.addRoutine("taxiAutoRoutine", autoRoutines::taxiAutoRoutine);
    // autoChooser.addRoutine("forwardBackTestAutoRoutine",
    // autoRoutines::forwardBackTestAutoRoutine);
    // autoChooser.addRoutine("diamondTestAutoRoutine", autoRoutines::diamondTestAutoRoutine);
    // autoChooser.addRoutine("turningTestAutoRoutine", autoRoutines::turningTestAutoRoutine);
    // autoChooser.addRoutine("leftAutoRoutine", autoRoutines::leftAutoRoutine);

    // Logging setup
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DogLog.setOptions(
        new DogLogOptions().withCaptureDs(true).withCaptureNt(true).withNtPublish(true));

    // Initialize subsystem visualizer
    subsystemVisualizer =
        new subsystemvisualizer(elevator, climb, endefectorWrist, endefectorRollers);

    logAllReefPositions();
    configureBindings();
  }

  private void configureBindings() {
    RobotController.getSerialNumber();

    // Driver controls
    driver.a().whileTrue(elevator.moveToState(ElevatorConstants.ElevatorStates.L1));
    driver.b().whileTrue(elevator.moveToState(ElevatorConstants.ElevatorStates.L4));
    driver.y().whileTrue(climb.moveToState(ClimbConstants.ClimbStates.CLIMB));
    driver.x().whileTrue(climb.moveToState(ClimbConstants.ClimbStates.CLIMBREADY));
    driver
        .rightTrigger()
        .whileTrue(endefectorRollers.moveToState(RollersConstants.EndefectorRollerStates.SCORE));
    driver
        .rightBumper()
        .whileTrue(endefectorRollers.moveToState(RollersConstants.EndefectorRollerStates.STOP));
    driver
        .leftTrigger()
        .whileTrue(endefectorRollers.moveToState(RollersConstants.EndefectorRollerStates.INTAKE));

    // Drive controls
    drive.setDefaultCommand(
        drive.runVelocityTeleopFieldRelative(
            () ->
                new ChassisSpeeds(
                    -joystickDeadbandApply(operator.getLeftY())
                        * RealConstants.MAX_LINEAR_SPEED
                        * 0.85,
                    -joystickDeadbandApply(operator.getLeftX())
                        * RealConstants.MAX_LINEAR_SPEED
                        * 0.85,
                    joystickDeadbandApply(operator.getRightX())
                        * RealConstants.MAX_ANGULAR_SPEED)));

    operator.y().onTrue(drive.zeroGyroCommand());
    operator.x().onTrue(drive.zeroPosition());
    drive.zeroPosition().runsWhenDisabled();
  }

  public Command stowAll() {
    return Commands.parallel(
        Commands.sequence(
            elevator.moveToState(ElevatorStates.STOW),
            endefectorWrist.moveToState(WristConstants.WristStates.STOW)),
        climb.moveToState(ClimbStates.STOW),
        endefectorRollers.moveToState(RollersConstants.EndefectorRollerStates.STOP));
  }

  public Command climbPrep() {
    return climb.moveToState(ClimbStates.CLIMBREADY);
  }

  public Command climbStow() {
    return climb.moveToState(ClimbStates.STOW);
  }

  public Command climbFull() {
    return climb.moveToState(ClimbStates.CLIMB);
  }

  public Command score() {
    return Commands.sequence(
        endefectorWrist.moveToState(WristConstants.WristStates.SCORING),
        endefectorRollers
            .moveToState(RollersConstants.EndefectorRollerStates.SCORE)
            .andThen(rumbleControllers()));
  }

  // saftey code in subsystems, not in commands.

  public Command stationIntake() {
    return Commands.sequence(
      Commands.either(
        Commands.sequence(
          elevator.moveToState(ElevatorStates.INTAKE), 
          endefectorWrist.moveToState(WristStates.STATIONINTAKE), 
          endefectorRollers.moveToState(EndefectorRollerStates.INTAKE)), 
        Commands.sequence(
          endefectorWrist.moveToState(WristStates.SCORING),
          elevator.moveToState(ElevatorStates.INTAKE),
          endefectorWrist.moveToState(WristStates.STATIONINTAKE),
          endefectorRollers.moveToState(EndefectorRollerStates.INTAKE)),
        ()->safetyChecker.isSafeElevator() && safteyChecker.isSafeWrist()
          ));
  }
  public Command groundIntake() {
    return Commands.sequence(
      Commands.either(
        Commands.sequence(
          elevator.moveToState(ElevatorStates.INTAKE), 
          endefectorWrist.moveToState(WristStates.GROUNDINTAKE)), 
          endefectorRollers.moveToState(EndefectorRollerStates.INTAKE), 
        Commands.sequence(
          endefectorWrist.moveToState(WristStates.SCORING),
          elevator.moveToState(ElevatorStates.INTAKE),
          endefectorWrist.moveToState(WristStates.GROUNDINTAKE),
          endefectorRollers.moveToState(EndefectorRollerStates.INTAKE)),
        ()->safetyChecker.isSafeElevator && safteyChecker.isSafeWrist
          ));}

  public Command reefIntake() {
    return Commands.sequence(
      Commands.either(
        Commands.sequence(
          elevator.moveToState(ElevatorStates.INTAKE), 
          endefectorWrist.moveToState(WristStates.REEFINTAKE)), 
          endefectorRollers.moveToState(EndefectorRollerStates.REEFINTAKE), 
        Commands.sequence(
          endefectorWrist.moveToState(WristStates.SCORING),
          elevator.moveToState(ElevatorStates.INTAKE),
          endefectorWrist.moveToState(WristStates.GROUNDINTAKE),
          endefectorRollers.moveToState(EndefectorRollerStates.REEFINTAKE)),
        ()->safetyChecker.isSafeElevator && safteyChecker.isSafeWrist
          ));
  
  public Command scorring() {
    return Commands.sequence(
      Commands.either(
        Commands.sequence(
          elevator.moveToState(ElevatorStates.INTAKE), 
          endefectorWrist.moveToState(WristStates.SCORING)), 
          endefectorRollers.moveToState(EndefectorRollerStates.SCORE), 
        Commands.sequence(
          endefectorWrist.moveToState(WristStates.SCORING),
          elevator.moveToState(ElevatorStates.INTAKE),
          endefectorWrist.moveToState(WristStates.SCORING),
          endefectorRollers.moveToState(EndefectorRollerStates.SCORE)),
        ()->safetyChecker.isSafeElevator && safteyChecker.isSafeWrist
          ));

  public Command climb() {
    return Commands.sequence(
      Commands.either(
        Commands.sequence(
          elevator.moveToState(ElevatorStates.INTAKE), 
          endefectorWrist.moveToState(WristStates.CLIMB)), 
          endefectorRollers.moveToState(EndefectorRollerStates.STOP), 
        Commands.sequence(
          endefectorWrist.moveToState(WristStates.SCORING),
          elevator.moveToState(ElevatorStates.INTAKE),
          endefectorWrist.moveToState(WristStates.CLIMB),
          endefectorRollers.moveToState(EndefectorRollerStates.STOP)),
        ()->safetyChecker.isSafeElevator && safteyChecker.isSafeWrist
          ));
    // return Commands.sequence(
    //     endefectorWrist
    //         .moveToState(WristConstants.WristStates.STATIONINTAKE)
    //         .unless(() -> elevator.getIO().getState() != ElevatorConstants.ElevatorStates.INTAKE),
    //     // creates conditional command to
    //     Commands.either(
    //         elevator.moveToState(ElevatorStates.INTAKE),
    //         endefectorWrist.moveToState(WristConstants.WristStates.STATIONINTAKE),
    //         () -> elevator.getIO().getState() != ElevatorConstants.ElevatorStates.INTAKE),
    //     endefectorRollers
    //         .moveToState(RollersConstants.EndefectorRollerStates.INTAKE)
    //         .andThen(rumbleControllers()));
  }

  public Command setElevatorScoringLevel(ElevatorConstants.ElevatorStates level) {
    switch (endefectorWrist.getIO().getCurrentState()) {
      case SCORING:
        return Commands.sequence(elevator.moveToState(level));
      default:
        return Commands.sequence(
            endefectorWrist.moveToState(WristConstants.WristStates.SCORING),
            setElevatorScoringLevel(level));
    }
  }

  public Command setAlgaeIntakeReefPosition() {
    return Commands.sequence(updateAlgaeIntakeState());
  }

  public Command rumbleControllers() {
    return new StartEndCommand(
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 1),
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0))
        .alongWith(
            new StartEndCommand(
                () -> operator.getHID().setRumble(RumbleType.kBothRumble, 1),
                () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0)));
  }

  public Command updateAlgaeIntakeState() {
    var tempElevatorState = ElevatorConstants.ElevatorStates.GROUNDINTAKE;
    var tempEndefectorState = WristConstants.WristStates.GROUNDINTAKE;
    counter++;
    if (counter == 1) {
      tempElevatorState = ElevatorConstants.ElevatorStates.GROUNDINTAKE;
      tempEndefectorState = WristConstants.WristStates.GROUNDINTAKE;
    } else if (counter == 2) {
      tempElevatorState = ElevatorConstants.ElevatorStates.ALGAE_L2;
      tempEndefectorState = WristConstants.WristStates.REEFINTAKE;
    } else {
      counter = 0;
      tempElevatorState = ElevatorConstants.ElevatorStates.ALGAE_L3;
      tempEndefectorState = WristConstants.WristStates.REEFINTAKE;
    }
    return Commands.sequence(
        endefectorWrist.moveToState(tempEndefectorState),
        elevator.moveToState(tempElevatorState),
        endefectorRollers
            .moveToState(RollersConstants.EndefectorRollerStates.INTAKEREEF)
            .andThen(rumbleControllers()));
  }

  private static double joystickDeadbandApply(double x) {
    return MathUtil.applyDeadband(
        (Math.signum(x) * (1.01 * Math.pow(x, 2) - 0.0202 * x + 0.0101)), 0.02);
  }

  public static void logAllReefPositions() {
    for (int i = 0; i < FieldConstants.Reef.branchPositions.size(); i++) {
      Map<FieldConstants.ReefHeight, Pose3d> branch = FieldConstants.Reef.branchPositions.get(i);
      if (branch == null) {
        continue;
      }

      for (FieldConstants.ReefHeight height : FieldConstants.ReefHeight.values()) {
        Pose3d position = branch.get(height);
        if (position != null) {
          String key = "Reef/Branch " + i + "/Pose/" + height.name();
          DogLog.log(key, position);
        }
      }
    }
  }

  public void incrementAlgaeState() {
    if (counter > 3) {
      counter = 0;
    } else {
      counter++;
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommandScheduler();
  }
}
