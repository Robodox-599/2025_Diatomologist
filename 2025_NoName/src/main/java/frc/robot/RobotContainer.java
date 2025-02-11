package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbConstants.ClimbStates;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;

import frc.robot.subsystems.endefector.endefectorrollers.Rollers;
import frc.robot.subsystems.endefector.endefectorrollers.RollersConstants;
import frc.robot.subsystems.endefector.endefectorrollers.RollersIOSim;
import frc.robot.subsystems.endefector.endefectorrollers.RollersIOTalonFX;
import frc.robot.subsystems.endefector.endefectorwrist.Wrist;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants;
import frc.robot.subsystems.endefector.endefectorwrist.WristIOSim;
import frc.robot.subsystems.endefector.endefectorwrist.WristIOTalonFX;

import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.subsystemvisualizer.subsystemvisualizer;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import frc.robot.Constants.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// // import frc.robot.subsystems.vision.Vision;
// // import frc.robot.subsystems.vision.VisionIOReal;
// // import frc.robot.subsystems.vision.VisionIOSim;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.constants.RealConstants;
import java.util.Map;

public class RobotContainer {

  private final CommandXboxController driver =
      new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);
  private final CommandXboxController operator =
      new CommandXboxController(Constants.ControllerConstants.kOperatorControllerPort);

  // visual stuff
  private LEDs LEDs;
  private subsystemvisualizer subsystemVisualizer;
  // private IntakeRollers algaeRollers;
  // private IntakeWrist algaeWrist;
  private Elevator elevator;
  private Rollers endefectorRollers;
  private Wrist endefectorWrist;
  private Climb climb;
  private final Drive drive;
  // private final Vision vision;
  private int counter = 0;

  /* Path follower */
  private final AutoRoutines autoRoutines;
  private final AutoFactory autoFactory;
  public final AutoChooser autoChooser = new AutoChooser();

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        elevator = new Elevator(new ElevatorIOTalonFX());
        endefectorRollers = new Rollers(new RollersIOTalonFX());
        endefectorWrist = new Wrist(new WristIOTalonFX());
        climb = new Climb(new ClimbIOTalonFX());
        drive = new Drive(new GyroIOPigeon2(), Drive.createTalonFXModules());
        // vision =
        //     new Vision(drive::addVisionMeasurement, new
        //     VisionIOReal(RealConstants.camConstants));
        autoFactory =
            new AutoFactory(
                drive::getPose, // A function that returns the current robot pose
                drive::setPose, // The drive subsystem
                drive::followChoreoPath, // The controller for the drive subsystem
                true,
                drive);
        autoRoutines = new AutoRoutines(autoFactory);
        break;
      case SIM:
        elevator = new Elevator(new ElevatorIOSim());
        endefectorRollers = new Rollers(new RollersIOSim());
        endefectorWrist = new Wrist(new WristIOSim());
        climb = new Climb(new ClimbIOSim());
        drive = new Drive(new GyroIO() {}, Drive.createSimModules());
        autoFactory =
            new AutoFactory(
                drive::getPose, // A function that returns the current robot pose
                drive::setPose, // The drive subsystem
                drive::followChoreoPath, // The controller for the drive subsystem
                true,
                drive);
        autoRoutines = new AutoRoutines(autoFactory);
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOSim(RealConstants.camConstants, drive::getPose));
        break;
         default:
        drive = new Drive(new GyroIOPigeon2(), Drive.createTalonFXModules());
        // vision =
        //     new Vision(drive::addVisionMeasurement, new
        // VisionIOReal(RealConstants.camConstants));
        autoFactory =
            new AutoFactory(
                drive::getPose, // A function that returns the current robot pose
                drive::setPose, // The drive subsystem
                drive::followChoreoPath, // The controller for the drive subsystem
                true,
                drive);
        autoRoutines = new AutoRoutines(autoFactory);
        break;
    }

    // Add options to the chooser
    // Put the auto chooser on the dashboard
    SmartDashboard.putData(autoChooser);
    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    // Logging starting here
    autoChooser.addRoutine("rightAutoRoutine", autoRoutines::rightAutoRoutine);
    autoChooser.addRoutine("taxiAutoRoutine", autoRoutines::taxiAutoRoutine);

    // Put the auto chooser on the dashboard
    SmartDashboard.putData("AutoChooser", autoChooser);
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    // log all reef positions, useful for debugging.
    logAllReefPositions();
    subsystemVisualizer =
        new subsystemvisualizer(elevator, climb, endefectorWrist, endefectorRollers);
    DogLog.setOptions(
        new DogLogOptions().withCaptureDs(true).withCaptureNt(true).withNtPublish(true));
    configureBindings();
  }

  private void configureBindings() {
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
        .whileTrue(endefectorRollers.moveToState(RollersConstants.EndefectorRollerStates.INTAKE));//     // Default command, normal field-relative drive

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
                    joystickDeadbandApply(v.getRightX())
                        * RealConstants.MAX_ANGULAR_SPEED)));
    operator.y().onTrue(drive.zeroGyroCommand());
    v.x().onTrue(drive.zeroPosition());
    drive.zeroPosition().runsWhenDisabled();
    // // Lock to 0° when A button is held
    // operator
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -operator.getLeftY(),
    //             () -> -operator.getLeftX(),
    //             () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // operator.x().onTrue(drive.stopWithXCmd());

    // // Reset gyro to 0° when B button is pressed
    // operator
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));
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
    //  if (endefectorRollers) TODO: add get beambreak value here to make sure not running the score
    // command unless there is a coral in the intake.
    return Commands.sequence(
        endefectorWrist.moveToState(WristConstants.WristStates.SCORING),
        endefectorRollers
            .moveToState(RollersConstants.EndefectorRollerStates.SCORE)
            .andThen(rumbleControllers()));
  }

  public Command stationIntake() {
    switch (elevator.getIO().getState()) {
      case STOW:
        return Commands.sequence(
            endefectorWrist.moveToState(WristConstants.WristStates.STATIONINTAKE),
            endefectorRollers
                .moveToState(RollersConstants.EndefectorRollerStates.INTAKE)
                .andThen(rumbleControllers()));
      default:
        return Commands.sequence(elevator.moveToState(ElevatorStates.STOW), stationIntake());
    }
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
        continue; // Skip if no data for this branch
      }

      for (FieldConstants.ReefHeight height : FieldConstants.ReefHeight.values()) {
        Pose3d position = branch.get(height);
        if (position != null) {
          String key = "Reef/Branch " + i + "/Pose/" + height.name();
          DogLog.log(key, position);
        }
      }
      DogLog.setOptions(
      new DogLogOptions().withCaptureDs(true).withCaptureNt(true).withNtPublish(true));
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