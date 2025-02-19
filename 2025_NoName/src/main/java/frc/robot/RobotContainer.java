package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.*;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants.ClimbStates;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.commands.AutoAlignToField;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.constants.RealConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.endefector.endefectorrollers.Rollers;
import frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.EndefectorRollerStates;
import frc.robot.subsystems.endefector.endefectorrollers.RollersIOSim;
import frc.robot.subsystems.endefector.endefectorrollers.RollersIOTalonFX;
import frc.robot.subsystems.endefector.endefectorwrist.Wrist;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants.WristStates;
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
  private final Drive drive;
  private Elevator elevator;
  private Wrist wrist;
  private Rollers rollers;
  private Climb climb;
  private LEDs LEDs;
  private Vision vision;
  private SafetyChecker safetyChecker;

  private ElevatorStates operatorAlgaePick = ElevatorStates.GROUNDINTAKE;
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
        climb = new Climb(new ClimbIOTalonFX());
        drive = new Drive(new GyroIOPigeon2(), Drive.createTalonFXModules());
        LEDs = new LEDs(new LEDsIOReal());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOReal(RealConstants.camConstants, drive::getPose));
        autoFactory =
            new AutoFactory(drive::getPose, drive::resetPose, drive::followChoreoPath, true, drive);
        autoRoutines = new AutoRoutines(autoFactory);
        break;

      default:
        DriverStation.silenceJoystickConnectionWarning(true);
        elevator = new Elevator(new ElevatorIOSim(), safetyChecker);
        rollers = new Rollers(new RollersIOSim());
        wrist = new Wrist(new WristIOSim(), safetyChecker);
        climb = new Climb(new ClimbIOSim());
        drive = new Drive(new GyroIO() {}, Drive.createSimModules());
        LEDs = new LEDs(new LEDsIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOSim(RealConstants.camConstants, drive::getPose));
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

    // Logging setup
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DogLog.setOptions(
        new DogLogOptions().withCaptureDs(true).withCaptureNt(true).withNtPublish(true));

    configureBindings();
  }

  private void configureBindings() {
    // RobotController.getSerialNumber();

    //                               DRIVER BINDS
    drive.setDefaultCommand(
        drive.runVelocityTeleopFieldRelative(
            () ->
                new ChassisSpeeds(
                    joystickDeadbandApply(driver.getLeftY())
                        * RealConstants.MAX_LINEAR_SPEED
                        * 0.85,
                    joystickDeadbandApply(driver.getLeftX())
                        * RealConstants.MAX_LINEAR_SPEED
                        * 0.85,
                    -joystickDeadbandApply(driver.getRightX()) * RealConstants.MAX_ANGULAR_SPEED),
            driver.rightTrigger(),
            () -> operator.povUp().getAsBoolean(),
            () -> operator.povDown().getAsBoolean()));
    // ZERO GYRO
    driver.y().onTrue(drive.zeroGyroCommand());
    drive.zeroGyroCommand().runsWhenDisabled();
    // STATION INTAKE COMMAND
    driver.rightTrigger().onTrue(stationIntake());
    // ALGAE INTAKE COMMAND
    driver.leftTrigger().onTrue(algaeIntake(operatorAlgaePick));
    // AUTO ALIGN
    driver.povLeft().whileTrue(AutoAlignToField.alignToNearestLeftReef(drive, rollers, LEDs));
    driver.povRight().whileTrue(AutoAlignToField.alignToNearestRightReef(drive, rollers, LEDs));
    // CLIMB
    driver.povUp().whileTrue(climb()).onFalse(stowAll());

    //                               OPERATOR BINDS
    // SCORE L4
    operator.y().onTrue(scoring(ElevatorStates.L4));
    // SCORE L3
    operator.b().onTrue(scoring(ElevatorStates.L3));
    // SCORE L2
    operator.a().onTrue(scoring(ElevatorStates.L2));
    // SCORE L1
    operator.x().onTrue(scoring(ElevatorStates.L1));
    // STATION INTAKE
    operator.rightBumper().onTrue(stationIntake());
    // ALGAE L3 INTAKE
    operator.povUp().onTrue(algaeL3Intake());
    // ALGAE L2 INTAKE
    operator.povDown().onTrue(algaeL2Intake());
    // ALGAE GROUND INTAKE
    operator.leftBumper().onTrue(algaeGroundIntake());
    // STOW ALL
    operator.start().onTrue(stowAll());
  }

  public Command stowAll() {
    return Commands.sequence(
        Commands.parallel(
            elevator.moveToState(ElevatorStates.STOW),
            wrist.moveToState(WristStates.STOW),
            rollers.moveToState(EndefectorRollerStates.STOP),
            LEDs.runNoState()),
        rumbleControllers());
  }

  public Command stationIntake() {
    return Commands.sequence(
        Commands.parallel(
            elevator.moveToState(ElevatorStates.INTAKE),
            wrist.moveToState(WristStates.STATIONINTAKE),
            rollers.moveToState(EndefectorRollerStates.INTAKE),
            LEDs.runStationIntake()),
        rumbleControllers());
  }

  public Command algaeL2Intake() {
    return Commands.runOnce(
        () -> {
          operatorAlgaePick = ElevatorStates.ALGAE_L2;
        });
  }

  public Command algaeGroundIntake() {
    return Commands.runOnce(
        () -> {
          operatorAlgaePick = ElevatorStates.GROUNDINTAKE;
        });
  }

  public Command algaeL3Intake() {
    return Commands.runOnce(
        () -> {
          operatorAlgaePick = ElevatorStates.ALGAE_L3;
        });
  }

  public Command climbStow() {
    return climb.moveToState(ClimbStates.STOW);
  }

  public Command climbFull() {
    return climb.moveToState(ClimbStates.CLIMB);
  }

  // saftey code in subsystems, not in commands.

  public Command algaeIntake(ElevatorStates state) {
    Command algaeIntakeCommand;
    if (ElevatorStates.ALGAE_L3 == state) {
      algaeIntakeCommand =
          Commands.sequence(
              Commands.parallel(
                  elevator.moveToState(ElevatorStates.ALGAE_L3),
                  wrist.moveToState(WristStates.REEFINTAKE),
                  rollers.moveToState(EndefectorRollerStates.INTAKE),
                  LEDs.runAlgaeIntake()),
              rumbleControllers());
    } else if (ElevatorStates.ALGAE_L2 == state) {
      algaeIntakeCommand =
          Commands.sequence(
              Commands.parallel(
                  elevator.moveToState(ElevatorStates.ALGAE_L2),
                  wrist.moveToState(WristStates.REEFINTAKE),
                  rollers.moveToState(EndefectorRollerStates.INTAKE),
                  LEDs.runAlgaeIntake()),
              rumbleControllers());
    } else if (ElevatorStates.GROUNDINTAKE == state) {
      algaeIntakeCommand =
          Commands.sequence(
              Commands.parallel(
                  elevator.moveToState(ElevatorStates.GROUNDINTAKE),
                  wrist.moveToState(WristStates.GROUNDINTAKE),
                  rollers.moveToState(EndefectorRollerStates.INTAKE),
                  LEDs.runAlgaeIntake()),
              rumbleControllers());
    } else {
      algaeIntakeCommand = Commands.none();
    }

    return algaeIntakeCommand;
  }

  public Command scoring(ElevatorStates state) {
    return Commands.sequence(
        Commands.parallel(elevator.moveToState(state), wrist.moveToState(WristStates.SCORING)),
        LEDs.runReadyToScore(),
        rollers.moveToState(EndefectorRollerStates.SCORE),
        rumbleControllers());
  }

  public Command climb() {
    return Commands.sequence(
        Commands.parallel(
            elevator.moveToState(ElevatorStates.INTAKE),
            wrist.moveToState(WristStates.CLIMB),
            rollers.moveToState(EndefectorRollerStates.STOP)),
        climb.moveToState(ClimbStates.CLIMBREADY));
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

  private static double joystickDeadbandApply(double x) {
    return MathUtil.applyDeadband(
        (Math.signum(x) * (1.01 * Math.pow(x, 2) - 0.0202 * x + 0.0101)), 0.02);
  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommandScheduler();
  }
}
