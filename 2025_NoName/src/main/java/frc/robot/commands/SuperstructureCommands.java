package frc.robot.commands;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.constants.RealConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.endefector.endefectorrollers.Rollers;
import frc.robot.subsystems.endefector.endefectorwrist.Wrist;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants.WristStates;
import frc.robot.subsystems.leds.LEDs;

public class SuperstructureCommands {
  private final Drive drive;
  private final Elevator elevator;
  private final Wrist wrist;
  private final Rollers rollers;
  // private Cli climb;
  private final LEDs LEDs;
  private ElevatorStates operatorAlgaePick = ElevatorStates.GROUNDINTAKE;
  private final CommandXboxController operator;
  private final CommandXboxController driver;

  public SuperstructureCommands(
      Drive drive,
      Elevator elevator,
      Wrist wrist,
      Rollers rollers,
      // Climb climb,
      LEDs LEDs,
      CommandXboxController driver,
      CommandXboxController operator) {
    this.drive = drive;
    this.elevator = elevator;
    this.wrist = wrist;
    this.rollers = rollers;
    this.LEDs = LEDs;
    this.operator = operator;
    this.driver = driver;
  }

  public Command stowAll() {
    return Commands.sequence(
        Commands.parallel(
            elevator.moveToState(ElevatorStates.STOW),
            wrist.moveToState(WristStates.STOW),
            rollers.stop(),
            LEDs.runNoState()),
        rumbleControllers());
  }

  // public Command scoring(ElevatorStates state) {
  //   return Commands.sequence(
  //       wrist.moveToState(WristStates.PREPARE),
  //       elevator.moveToState(state),
  //       wrist.moveToState(WristStates.SCORING),
  //       LEDs.runReadyToScore().withTimeout(0.1),
  //       rollers.runRollerScore(),
  //       rumbleControllers());
  // }

  public Command moveToL1() {
    return Commands.either(
        Commands.sequence(
            wrist.moveToState(WristStates.PREPARE),
            elevator.moveToState(ElevatorStates.L1),
            LEDs.runReadyToScore().withTimeout(0.1),
            rumbleControllers()),
        Commands.none(),
        () -> rollers.isCoralDetected());
  }

  public Command moveToL2() {
    return Commands.either(
        Commands.sequence(
            wrist.moveToState(WristStates.PREPARE),
            elevator.moveToState(ElevatorStates.L2),
            LEDs.runReadyToScore().withTimeout(0.1),
            rumbleControllers()),
        Commands.none(),
        () -> rollers.isCoralDetected());
  }

  public Command moveToL3() {
    return Commands.either(
        Commands.sequence(
            wrist.moveToState(WristStates.PREPARE),
            elevator.moveToState(ElevatorStates.L3),
            LEDs.runReadyToScore().withTimeout(0.1),
            rumbleControllers()),
        Commands.none(),
        () -> rollers.isCoralDetected());
  }

  public Command moveToL4() {
    return Commands.either(
        Commands.sequence(
            wrist.moveToState(WristStates.PREPARE),
            elevator.moveToState(ElevatorStates.L4),
            LEDs.runReadyToScore().withTimeout(0.1),
            rumbleControllers()),
        Commands.none(),
        () -> rollers.isCoralDetected());
  }

  public Command stationIntake() {
    return Commands.either(
        Commands.sequence(
            wrist.moveToState(WristStates.PREPARE),
            elevator.moveToState(ElevatorStates.INTAKE),
            wrist.moveToState(WristStates.STATIONINTAKE),
            LEDs.runStationIntake().withTimeout(0.1),
            rollers.runRollersIntake(),
            rumbleControllers(),
            LEDs.runIntaked(),
            prepareToScore(),
            LEDs.runReadyToScore()),
        Commands.none(),
        () -> !rollers.isCoralDetected());
  }

  // public Command stationIntake() {
  //   return Commands.sequence(
  //       wrist.moveToState(WristStates.PREPARE),
  //       elevator.moveToState(ElevatorStates.INTAKE),
  //       wrist.moveToState(WristStates.STATIONINTAKE),
  //       LEDs.runStationIntake().withTimeout(0.1),
  //       rollers.runRollersIntake(),
  //       rumbleControllers(),
  //       LEDs.runIntaked(),
  //       prepareToScore(),
  //       LEDs.runReadyToScore());
  // }

  public Command ejectCoralIntake() {
    return rollers.runRollersReverse();
  }

  public Command prepareToScore() {
    return Commands.sequence(
        wrist.moveToState(WristStates.PREPARE), elevator.moveToState(ElevatorStates.PREP));
  }

  public boolean isReadyToScore() {
    DogLog.log("Superstructure/isCoralDetected", rollers.isCoralDetected());
    DogLog.log(
        "Superstructure/elevatorIsAtTargetPosition",
        elevator.isAtTargetPosition(elevator.getState()));
    DogLog.log(
        "Superstructure/wristIsAtTargetPosition", wrist.isAtTargetPosition(wrist.getState()));
    DogLog.log(
        "Superstructure/isReadyToScore",
        rollers.isCoralDetected()
            && elevator.isAtTargetPosition(elevator.getState())
            && wrist.isAtTargetPosition(wrist.getState()));
    return rollers.isCoralDetected()
        && elevator.isAtTargetPosition(elevator.getState())
        && wrist.isAtTargetPosition(wrist.getState());
  }

  public Command scoreCoral() {
    return Commands.either( // check if L4 or not
        Commands.sequence(
            wrist.moveToState(WristStates.SCORING),
            LEDs.runScoring(),
            rollers.runRollerScore(),
            LEDs.runScored(),
            stationIntake()),
        Commands.sequence(
            LEDs.runScoring(), rollers.runRollerScore(), LEDs.runScored(), stationIntake()),
        () -> elevator.getState() == ElevatorConstants.ElevatorStates.L4);
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

  // public Command climbStow() {
  //   return climb.moveToState(ClimbStates.STOW);
  // }

  // public Command climbFull() {
  //   return climb.moveToState(ClimbStates.CLIMB);
  // }

  // saftey code in subsystems, not in commands.

  public Command algaeIntake(ElevatorStates state) {
    Command algaeIntakeCommand;
    if (ElevatorStates.ALGAE_L3 == state) {
      algaeIntakeCommand =
          Commands.sequence(
              Commands.parallel(
                  elevator.moveToState(ElevatorStates.ALGAE_L3),
                  wrist.moveToState(WristStates.REEFINTAKE),
                  rollers.runAlgaeIntake(),
                  LEDs.runAlgaeIntake()),
              rumbleControllers());
    } else if (ElevatorStates.ALGAE_L2 == state) {
      algaeIntakeCommand =
          Commands.sequence(
              Commands.parallel(
                  elevator.moveToState(ElevatorStates.ALGAE_L2),
                  wrist.moveToState(WristStates.REEFINTAKE),
                  rollers.runAlgaeIntake(),
                  LEDs.runAlgaeIntake()),
              rumbleControllers());
    } else if (ElevatorStates.GROUNDINTAKE == state) {
      algaeIntakeCommand =
          Commands.sequence(
              Commands.parallel(
                  elevator.moveToState(ElevatorStates.GROUNDINTAKE),
                  wrist.moveToState(WristStates.GROUNDINTAKE),
                  rollers.runAlgaeIntake(),
                  LEDs.runAlgaeIntake()),
              rumbleControllers());
    } else {
      algaeIntakeCommand = Commands.none();
    }

    return algaeIntakeCommand;
  }

  // public Command climb() {
  //   return Commands.sequence(
  //       Commands.parallel(
  //           elevator.moveToState(ElevatorStates.INTAKE),
  //           wrist.moveToState(WristStates.CLIMB),
  //           rollers.moveToState(EndefectorRollerStates.STOP)),
  //       climb.moveToState(ClimbStates.CLIMBREADY));
  // }

  public Command rumbleControllers() {
    return new StartEndCommand(
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 1),
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0))
        .alongWith(
            new StartEndCommand(
                () -> operator.getHID().setRumble(RumbleType.kBothRumble, 1),
                () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0)))
        .withTimeout(0.2);
  }

  public void configureBindings() {
    //                               DRIVER BINDS
    drive.setDefaultCommand(
        drive.runVelocityTeleopFieldRelative(
            () ->
                new ChassisSpeeds(
                    -joystickDeadbandApply(driver.getLeftY())
                        * RealConstants.MAX_LINEAR_SPEED
                        * 0.85,
                    -joystickDeadbandApply(driver.getLeftX())
                        * RealConstants.MAX_LINEAR_SPEED
                        * 0.85,
                    -joystickDeadbandApply(driver.getRightX()) * RealConstants.MAX_ANGULAR_SPEED)));
    // // // ZERO GYRO
    driver.y().onTrue(drive.zeroGyroCommand());
    drive.zeroGyroCommand().runsWhenDisabled();
    driver.rightTrigger().whileTrue(stationIntake());
    driver
        .povLeft()
        .whileTrue(
            Commands.sequence(
                Commands.parallel(prepareToScore(), AutoAlignToField.alignToNearestLeftReef(drive)),
                rumbleControllers()));
    driver
        .povRight()
        .whileTrue(
            Commands.sequence(
                Commands.parallel(
                    prepareToScore(), AutoAlignToField.alignToNearestRightReef(drive)),
                rumbleControllers()));
    // OPERATOR BINDS
    // SCORE L4
    operator.x().onTrue(moveToL1());
    // SCORE L3
    operator.a().onTrue(moveToL2());
    // SCORE L2
    operator.b().onTrue(moveToL3());
    // SCORE L3
    operator.y().onTrue(moveToL4());
    // STATION INTAKE
    operator.rightBumper().onTrue(stationIntake());
    // SCORE
    operator.rightTrigger().onTrue(scoreCoral());
    // EJECT CORAL INTAKE
    operator.leftBumper().whileTrue(ejectCoralIntake());
    operator.leftBumper().onFalse(rollers.stop());
    // PREPARE
    operator.leftTrigger().onTrue(prepareToScore());
  }

  private static double joystickDeadbandApply(double x) {
    return MathUtil.applyDeadband(
        (Math.signum(x) * (1.01 * Math.pow(x, 2) - 0.0202 * x + 0.0101)), 0.02);
  }
}
