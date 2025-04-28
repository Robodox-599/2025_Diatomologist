package frc.robot.commands;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.constants.RealConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.endefector.endefectorrollers.Rollers;
import frc.robot.subsystems.endefector.endefectorwrist.Wrist;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants.WristStates;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDsConstants.LEDStates;

public class SuperstructureCommands {
  private final Drive drive;
  private final Elevator elevator;
  private final Wrist wrist;
  private final Rollers rollers;
  // private Cli climb;
  private final LEDs LEDs;
  private ElevatorStates operatorAlgaePick = ElevatorStates.ALGAEGROUNDINTAKE;
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
            LEDs.setState(LEDStates.IDLE)),
        rumbleControllers().withTimeout(0.5));
  }

  public Command autoAlignToLeft() {
    return Commands.sequence(
        LEDs.setState(LEDStates.AUTOALIGN).withTimeout(0.1), // rainbow
        AutoAlignToField.alignToNearestLeftBranch(drive),
        rumbleControllers().withTimeout(0.25));
  }

  public Command autoAlignToRight() {
    return Commands.sequence(
        LEDs.setState(LEDStates.AUTOALIGN).withTimeout(0.1), // rainbow
        AutoAlignToField.alignToNearestRightBranch(drive),
        rumbleControllers().withTimeout(0.25));
  }

  public Command autoAlignToReefFace() {
    return Commands.sequence(
        LEDs.setState(LEDStates.AUTOALIGN).withTimeout(0.1), // rainbow
        AutoAlignToField.alignToNearestReefFace(drive),
        rumbleControllers().withTimeout(0.25));
  }

  public Command coralStationIntake() {
    return Commands.either(
        Commands.sequence(
            wrist.moveToState(WristStates.PREPARE),
            elevator.moveToState(ElevatorStates.CORALSTATIONINTAKE),
            wrist.moveToState(WristStates.CORALSTATIONINTAKE),
            LEDs.setState(LEDStates.CORALSTATIONINTAKE).withTimeout(0.1), // white
            rollers.runCoralStationIntake(),
            rumbleControllers().withTimeout(0.25),
            LEDs.setState(LEDStates.INTAKED).withTimeout(0.1), // green
            prepareToScore()),
        Commands.none(),
        () -> !rollers.isCoralDetected());
  }

  public Command autoIntakeFromStart() {
    return Commands.sequence(
        wrist.moveToState(WristStates.CORALSTATIONINTAKE),
        LEDs.setState(LEDStates.CORALSTATIONINTAKE).withTimeout(0.1), // white
        rollers.runCoralStationIntake(),
        LEDs.setState(LEDStates.INTAKED).withTimeout(0.1) // green
        // prepareToScore()
        );
  }

  public Command ejectGamePiece() {
    return Commands.sequence(
        Commands.parallel(
            rollers.ejectGamePiece(), LEDs.setState(LEDStates.SCORING).withTimeout(0.1)), // red
        LEDs.setState(LEDStates.SCORED));
  }

  public Command algaeIntake(ElevatorStates state) {
    Command algaeIntakeCommand = Commands.none();
    if (!rollers.isAlgaeDetected()) { // if there is no algae detected
      if (ElevatorStates.ALGAEL3 == state) {
        algaeIntakeCommand =
            Commands.sequence(
                wrist.moveToState(WristStates.PREPARE),
                LEDs.setState(LEDStates.PREPARED).withTimeout(0.1), // blue
                elevator.moveToState(ElevatorStates.ALGAEL3),
                wrist.moveToState(WristStates.ALGAEREEFINTAKE),
                LEDs.setState(LEDStates.ALGAEINTAKE).withTimeout(0.1), // cyan
                rollers.runAlgaeIntake(),
                LEDs.setState(LEDStates.INTAKED).withTimeout(0.1), // green
                rumbleControllers().withTimeout(0.25),
                prepareToProcessor()); // blue
      } else if (ElevatorStates.ALGAEL2 == state) {
        algaeIntakeCommand =
            Commands.sequence(
                wrist.moveToState(WristStates.PREPARE),
                LEDs.setState(LEDStates.PREPARED).withTimeout(0.1), // blue
                elevator.moveToState(ElevatorStates.ALGAEL2),
                wrist.moveToState(WristStates.ALGAEREEFINTAKE),
                LEDs.setState(LEDStates.ALGAEINTAKE).withTimeout(0.1), // cyan
                rollers.runAlgaeIntake(),
                LEDs.setState(LEDStates.INTAKED).withTimeout(0.1), // green
                rumbleControllers().withTimeout(0.25),
                prepareToProcessor()); // blue
      } else if (ElevatorStates.ALGAEGROUNDINTAKE == state) {
        algaeIntakeCommand =
            Commands.sequence(
                Commands.parallel(
                    elevator.moveToState(ElevatorStates.ALGAEGROUNDINTAKE),
                    wrist.moveToState(WristStates.ALGAEGROUNDINTAKE)),
                LEDs.setState(LEDStates.ALGAEINTAKE).withTimeout(0.1), // cyan
                rollers.runAlgaeIntake(),
                LEDs.setState(LEDStates.INTAKED).withTimeout(0.1), // green
                rumbleControllers().withTimeout(0.25),
                prepareToProcessor()); // blue
      }
    }
    return algaeIntakeCommand;
  }

  public Command moveToL1() {
    return Commands.sequence(
        wrist.moveToState(WristStates.PREPARE),
        elevator.moveToState(ElevatorStates.CORALL1),
        // LEDs.runReadyToScore().withTimeout(0.1), // purple
        rumbleControllers().withTimeout(0.1));
  }

  public Command moveToL2() {
    return Commands.sequence(
        wrist.moveToState(WristStates.PREPARE),
        elevator.moveToState(ElevatorStates.CORALL2),
        // LEDs.runReadyToScore().withTimeout(0.1), // purple
        rumbleControllers().withTimeout(0.1));
  }

  public Command moveToL3() {
    return Commands.sequence(
        wrist.moveToState(WristStates.PREPARE),
        elevator.moveToState(ElevatorStates.CORALL3),
        // LEDs.runReadyToScore().withTimeout(0.1), // purple
        rumbleControllers().withTimeout(0.1));
  }

  public Command moveToL4() {
    return Commands.sequence(
        wrist.moveToState(WristStates.PREPARE),
        elevator.moveToState(ElevatorStates.CORALL4),
        // LEDs.runReadyToScore().withTimeout(0.1), // purple
        rumbleControllers().withTimeout(0.1));
  }

  public Command extendToNet() {
    return Commands.sequence(
        wrist.moveToState(WristStates.PREPARE),
        elevator.moveToState(ElevatorStates.BARGENET),
        rumbleControllers().withTimeout(0.1));
  }

  public Command prepareToProcessor() {
    return Commands.sequence(
        Commands.parallel(
            wrist.moveToState(WristStates.PREPARE), elevator.moveToState(ElevatorStates.PROCESSOR)),
        LEDs.setState(LEDStates.PREPARED).withTimeout(0.1)); // blue
  }

  public Command prepareToScore() {
    return Commands.sequence(
        Commands.parallel(
            wrist.moveToState(WristStates.PREPARE), elevator.moveToState(ElevatorStates.PREP)),
        LEDs.setState(LEDStates.PREPARED).withTimeout(0.1)); // blue
  }

  public boolean isReadyToScoreCoral() {
    boolean readyToScore =
        rollers.isCoralDetected()
            && elevator.isAtTargetPosition(elevator.getState())
            && wrist.isAtTargetPosition(wrist.getState());
    DogLog.log("Superstructure/isReadyToScore", readyToScore);
    return readyToScore;
  }

  public boolean isReadyToScoreAlgae() {
    boolean readyToScore =
        rollers.isAlgaeDetected()
            && elevator.isAtTargetPosition(elevator.getState())
            && wrist.isAtTargetPosition(wrist.getState());
    DogLog.log("Superstructure/isReadyToScore", readyToScore);
    return readyToScore;
  }

  public Command scoreCoralWithoutChecking() {
    return Commands.sequence(
        rollers.runScoreCoral(), new WaitCommand(0.25), rumbleControllers().withTimeout(0.1));
  }

  public Command scoreGamePiece() {
    return Commands.either( // check if coral or algae
        Commands.either( // if algae, check if ready to score algae
            Commands.sequence(
                LEDs.setState(LEDStates.SCORING).withTimeout(0.1), // red
                rollers.runScoreAlgae(),
                LEDs.setState(LEDStates.SCORED).withTimeout(0.1), // yellow
                rumbleControllers().withTimeout(0.1),
                new WaitCommand(0.5),
                coralStationIntake()),
            Commands.none(),
            () -> isReadyToScoreAlgae()),
        Commands.either( // if coral, check if ready to score algae
            Commands.sequence(
                LEDs.setState(LEDStates.SCORING).withTimeout(0.1), // red
                rollers.runScoreCoral(),
                new WaitCommand(0.25),
                LEDs.setState(LEDStates.SCORED).withTimeout(0.1), // yellow
                rumbleControllers().withTimeout(0.1),
                coralStationIntake()),
            Commands.none(),
            () -> isReadyToScoreCoral()),
        () -> rollers.isAlgaeDetected());
  }

  public Command scoreGamePieceWithoutIntaking() {
    return Commands.either( // check if coral or algae
        Commands.either( // if algae, check if ready to score algae
            Commands.sequence(
                LEDs.setState(LEDStates.SCORING).withTimeout(0.1), // red
                rollers.runScoreAlgae(),
                LEDs.setState(LEDStates.SCORED).withTimeout(0.1), // yellow
                rumbleControllers().withTimeout(0.1),
                new WaitCommand(0.5)),
            Commands.none(),
            () -> isReadyToScoreAlgae()),
        Commands.either( // if coral, check if ready to score algae
            Commands.sequence(
                LEDs.setState(LEDStates.SCORING).withTimeout(0.1), // red
                rollers.runScoreCoral(),
                new WaitCommand(0.25),
                LEDs.setState(LEDStates.SCORED).withTimeout(0.1), // yellow
                rumbleControllers().withTimeout(0.1)),
            Commands.none(),
            () -> isReadyToScoreCoral()),
        () -> rollers.isAlgaeDetected());
  }

  //   public Command scoreAlgae() {
  //     return Commands.either( // check if ready to score algae
  //         Commands.sequence(
  //             LEDs.runScoring().withTimeout(0.1), // red
  //             rollers.runScoreAlgae(),
  //             LEDs.runScored().withTimeout(0.1)), // yellow
  //         Commands.none(),
  //         () -> isReadyToScoreAlgae());
  //   }

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

    // ZERO GYRO
    driver.y().onTrue(drive.zeroGyroCommand());
    drive.zeroGyroCommand().runsWhenDisabled();
    // AUTO ALIGN
    driver.povLeft().whileTrue(autoAlignToLeft());
    driver.povRight().whileTrue(autoAlignToRight());
    // driver.leftBumper().onTrue(autoAlignToReefFace());
    // INTAKE ALGAE FLOOR
    driver.leftTrigger().onTrue(algaeIntake(ElevatorStates.ALGAEGROUNDINTAKE));
    // SCORE GAME PIECE
    driver.rightTrigger().onTrue(scoreGamePiece());

    //                                OPERATOR BINDS
    // // MOVE TO L1
    operator.x().onTrue(moveToL1());
    // // MOVE TO L2
    operator.a().onTrue(moveToL2());
    // // MOVE TO L3
    operator.b().onTrue(moveToL3());
    // // MOVE TO L4
    operator.y().onTrue(moveToL4());
    // // MOVE TO NET
    operator.povLeft().onTrue(extendToNet());
    operator.povRight().onTrue(extendToNet());
    // CORAL STATION INTAKE
    operator.rightBumper().onTrue(coralStationIntake());
    // PREPARE
    operator.leftBumper().onTrue(prepareToScore());
    // // EJECT CORAL
    operator.leftTrigger().whileTrue(ejectGamePiece());
    operator.leftTrigger().onFalse(rollers.stop());
    // // INTAKE ALGAE L2
    operator.povDown().onTrue(algaeIntake(ElevatorStates.ALGAEL2));
    // // INTAKE ALGAE L3
    operator.povUp().onTrue(algaeIntake(ElevatorStates.ALGAEL3));
    // // OVERRIDE TO HIGHEST LEVEL
    // operator
    //     .start()
    //     .onTrue(
    //         Commands.sequence(
    //             LEDs.runOverride().withTimeout(0.1),
    //             wrist.moveToState(WristStates.PREPARE),
    //             elevator.moveToState(ElevatorStates.CORALL4)));
  }

  private static double joystickDeadbandApply(double x) {
    return MathUtil.applyDeadband(
        (Math.signum(x) * (1.01 * Math.pow(x, 2) - 0.0202 * x + 0.0101)), 0.02);
  }

  //   public Command moveToCoralStationIntake() {
  //     return Commands.sequence(
  //         wrist.moveToState(WristStates.PREPARE),
  //         elevator.moveToState(ElevatorStates.CORALSTATIONINTAKE),
  //         wrist.moveToState(WristStates.CORALSTATIONINTAKE));
  //   }

  //   public Command scoreCoral() {
  //     return Commands.either( // check if elevator and wrist are at setpoint
  //         Commands.either( // check if L4 or not
  //             Commands.sequence(
  //                 wrist.moveToState(WristStates.SCORING),
  //                 LEDs.runScoring().withTimeout(0.1), // red
  //                 rollers.runRollerScore(),
  //                 LEDs.runScored().withTimeout(0.1), // yellow
  //                 new WaitCommand(0.5),
  //                 stationIntake()),
  //             Commands.sequence(
  //                 LEDs.runScoring().withTimeout(0.1), // red
  //                 rollers.runRollerScore(),
  //                 LEDs.runScored().withTimeout(0.1), // yellow
  //                 stationIntake()),
  //             () -> elevator.getState() == ElevatorConstants.ElevatorStates.L4),
  //         Commands.none(),
  //         () -> isReadyToScore());
  //   }

  // public Command algaeL2Intake() {
  //   return Commands.runOnce(
  //       () -> {
  //         operatorAlgaePick = ElevatorStates.ALGAE_L2;
  //       });
  // }

  // public Command algaeGroundIntake() {
  //   return Commands.runOnce(
  //       () -> {
  //         operatorAlgaePick = ElevatorStates.GROUNDINTAKE;
  //       });
  // }

  // public Command algaeL3Intake() {
  //   return Commands.runOnce(
  //       () -> {
  //         operatorAlgaePick = ElevatorStates.ALGAE_L3;
  //       });

  // public Command climbStow() {
  //   return climb.moveToState(ClimbStates.STOW);
  // }

  // public Command climbFull() {
  //   return climb.moveToState(ClimbStates.CLIMB);
  // }

  // public Command climb() {
  //   return Commands.sequence(
  //       Commands.parallel(
  //           elevator.moveToState(ElevatorStates.INTAKE),
  //           wrist.moveToState(WristStates.CLIMB),
  //           rollers.moveToState(EndefectorRollerStates.STOP)),
  //       climb.moveToState(ClimbStates.CLIMBREADY));
  // }
}
