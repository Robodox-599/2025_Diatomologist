package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
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

  public Command scoring(ElevatorStates state) {
    return Commands.sequence(
        wrist.moveToState(WristStates.PREPARE),
        elevator.moveToState(state),
        wrist.moveToState(WristStates.SCORING),
        LEDs.runReadyToScore().withTimeout(0.1),
        rollers.runRollerScore(),
        rumbleControllers());
  }

  public Command stationIntake() {
    return Commands.sequence(
        wrist.moveToState(WristStates.PREPARE),
        elevator.moveToState(ElevatorStates.INTAKE),
        wrist.moveToState(WristStates.STATIONINTAKE),
        LEDs.runStationIntake().withTimeout(0.1),
        rollers.runRollersIntake(),
        rumbleControllers(),
        prepareToScore(),
        LEDs.runReadyToScore());
  }

  public Command ejectCoralIntake() {
    return rollers.runRollersReverse();
  }

  public Command prepareToScore() {
    return Commands.sequence(
        wrist.moveToState(WristStates.PREPARE), elevator.moveToState(ElevatorStates.PREP));
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
    // drive.setDefaultCommand(
    //     drive.runVelocityTeleopFieldRelative(
    //         () ->
    //             new ChassisSpeeds(
    //                 -joystickDeadbandApply(driver.getLeftY())
    //                     * RealConstants.MAX_LINEAR_SPEED
    //                     * 0.85,
    //                 -joystickDeadbandApply(driver.getLeftX())
    //                     * RealConstants.MAX_LINEAR_SPEED
    //                     * 0.85,
    //                 -joystickDeadbandApply(driver.getRightX()) *
    // RealConstants.MAX_ANGULAR_SPEED),
    //         driver.rightTrigger(),
    //         () -> operator.povUp().getAsBoolean(),
    //         () -> operator.povDown().getAsBoolean()));
    // drive.setDefaultCommand(
    //     drive.runVoltageTeleopFieldRelative(
    //         () ->
    //             new ChassisSpeeds(
    //                 joystickDeadbandApply(driver.getLeftY())
    //                     * RealConstants.MAX_LINEAR_SPEED
    //                     * 0.85,
    //                 joystickDeadbandApply(driver.getLeftX())
    //                     * RealConstants.MAX_LINEAR_SPEED
    //                     * 0.85,
    //                 -joystickDeadbandApply(driver.getRightX()) *
    // RealConstants.MAX_ANGULAR_SPEED)));
    // // // ZERO GYRO
    // driver.y().onTrue(drive.zeroGyroCommand());
    // drive.zeroGyroCommand().runsWhenDisabled();
    // // STATION INTAKE COMMAND
    // // driver.rightTrigger().onTrue(stationIntake());
    // // ALGAE INTAKE COMMAND
    // // driver.leftTrigger().onTrue(algaeIntake(operatorAlgaePick));
    // // AUTO ALIGN
    // driver.a().onTrue(wrist.moveToState(WristStates.STATIONINTAKE));
    // driver
    //     .povLeft()
    //     .whileTrue(
    //         Commands.sequence(
    //             Commands.parallel(
    //                 elevator.moveToState(ElevatorStates.PREP),
    //                 wrist.moveToState(WristStates.SCORING)),
    //             AutoAlignToField.alignToNearestLeftReef(drive)));
    // driver
    //     .povRight()
    //     .whileTrue(
    //         Commands.sequence(
    //             Commands.parallel(
    //                 elevator.moveToState(ElevatorStates.PREP),
    //                 wrist.moveToState(WristStates.SCORING)),
    //             AutoAlignToField.alignToNearestRightReef(drive)));
    // driver.x().onTrue(wrist.moveToState(WristStates.STOW));
    // driver.a().onTrue(Commands.sequence(wrist.moveToState(WristStates.SCORING)));

    // CLIMB
    // driver.povUp().whileTrue(climb()).onFalse(stowAll());

    // OPERATOR BINDS
    // SCORE L4
    driver.y().whileTrue(scoring(ElevatorStates.L4));
    // SCORE L3
    driver.b().whileTrue(scoring(ElevatorStates.L3));
    // SCORE L2
    driver.a().whileTrue(scoring(ElevatorStates.L2));
    // SCORE L1
    driver.x().whileTrue(scoring(ElevatorStates.L1));
    // STATION INTAKE
    driver.rightTrigger().whileTrue(stationIntake());
    // EJECT CORAL INTAKE
    driver.leftTrigger().whileTrue(ejectCoralIntake());

    // ALGAE L3 INTAKE
    // operator.povUp().onTrue(algaeL3Intake());
    // ALGAE L2 INTAKE
    // operator.povDown().onTrue(algaeL2Intake());
    // ALGAE GROUND INTAKE
    // operator.leftBumper().onTrue(algaeGroundIntake());
    // STOW ALL
    // operator.start().onTrue(stowAll());

    // SUBSYSTEM VISUALIZER TEST COMMANDS:

    // driver.povLeft().whileTrue(rollers.moveToState(RollersConstants.EndefectorRollerStates.STOP));

    // driver.povRight().whileTrue(rollers.moveToState(RollersConstants.EndefectorRollerStates.SCORE));

    // driver
    //     .rightTrigger()
    //     .whileTrue(elevator.moveToState(ElevatorConstants.ElevatorStates.GROUNDINTAKE));

    // driver.leftTrigger().whileTrue(elevator.moveToState(ElevatorConstants.ElevatorStates.ALGAE_L2));

    // driver.leftBumper().whileTrue(elevator.moveToState(ElevatorConstants.ElevatorStates.ALGAE_L3));

    // driver.rightBumper().whileTrue(elevator.moveToState(ElevatorConstants.ElevatorStates.INTAKE));

    // driver
    //     .povDown()
    //     .whileTrue(rollers.moveToState(RollersConstants.EndefectorRollerStates.REEFINTAKE));

    // driver.povUp().whileTrue(rollers.moveToState(RollersConstants.EndefectorRollerStates.INTAKE));

    // driver.a().whileTrue(rollers.moveToState(RollersConstants.EndefectorRollerStates.ALGAEINTAKE));

    // driver.rightTrigger().whileTrue(wrist.moveToState(WristConstants.WristStates.REEFINTAKE));

    // driver.leftTrigger().whileTrue(wrist.moveToState(WristConstants.WristStates.CLIMB));

    // driver
    //     .rightBumper()
    //     .whileTrue(elevator.moveToState(ElevatorConstants.ElevatorStates.GROUNDINTAKE));

    // driver.leftBumper().whileTrue(elevator.moveToState(ElevatorConstants.ElevatorStates.L3));
  }

  private static double joystickDeadbandApply(double x) {
    return MathUtil.applyDeadband(
        (Math.signum(x) * (1.01 * Math.pow(x, 2) - 0.0202 * x + 0.0101)), 0.02);
  }
}
