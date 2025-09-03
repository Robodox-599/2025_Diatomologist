package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import java.util.Set;

public class Bindings extends SubsystemBase {

  /* DRIVER */
  public enum AutomationLevel {
    AUTO_ALIGN,
    AUTO_ACTION,
  }

  /* OPERATOR */
  public enum BranchAutoAlignSide {
    LEFT,
    RIGHT,
  }

  public enum TroughAutoAlignSide {
    LEFT,
    MIDDLE,
    RIGHT,
  }

  public enum CoralScoreLevel {
    POSITION_CORAL_L1,
    POSITION_CORAL_L2,
    POSITION_CORAL_L3,
    POSITION_CORAL_L4,
  }

  public enum AlgaeLevel {
    POSITION_ALGAE_BARGE,
    INTAKING_ALGAE_L3,
    INTAKING_ALGAE_L2,
    POSITION_ALGAE_PROCESSOR,
  }

  public enum GamePieceState {
    CORAL,
    ALGAE,
  }

  private final Superstructure superstructure;
  private BranchAutoAlignSide branchAutoAlignSide = BranchAutoAlignSide.LEFT;
  private TroughAutoAlignSide troughAutoAlignSide = TroughAutoAlignSide.LEFT;
  private CoralScoreLevel coralScoreLevel = CoralScoreLevel.POSITION_CORAL_L4;
  private AlgaeLevel algaeLevel = AlgaeLevel.POSITION_ALGAE_PROCESSOR;
  private GamePieceState gamePieceState = GamePieceState.CORAL;
  private AutomationLevel automationLevel = AutomationLevel.AUTO_ALIGN;

  public Bindings(
      CommandXboxController driver, CommandXboxController operator, Superstructure superstructure) {
    this.superstructure = superstructure;
    //                               DRIVER BINDS
    // ZERO GYRO
    driver.y().onTrue(superstructure.zeroGyroCommand());

    // // SET WANTED STATE TO A LOGIC STATE
    driver
        .rightBumper()
        .onTrue(setLogicStateCommand().alongWith(rumbleControllers(driver, operator)));
    // // SET WANTED STATE TO INTAKING ALGAE LOLLIPOP
    driver
        .leftBumper()
        .whileTrue(
            superstructure
                .setWantedSuperStateCommand(WantedSuperState.INTAKING_ALGAE_LOLLIPOP)
                .alongWith(rumbleDriver(driver)));
    driver
        .leftTrigger()
        .onFalse(
            superstructure.setWantedSuperStateCommand(WantedSuperState.POSITION_ALGAE_PROCESSOR));
    // // SET WANTED STATE TO INTAKING ALGAE GROUND
    driver
        .leftTrigger()
        .whileTrue(
            superstructure
                .setWantedSuperStateCommand(WantedSuperState.INTAKING_ALGAE_GROUND)
                .alongWith(rumbleControllers(driver, operator)));
    driver
        .leftTrigger()
        .onFalse(
            superstructure.setWantedSuperStateCommand(WantedSuperState.POSITION_ALGAE_PROCESSOR));
    // // SET WANTED STATE TO SCORING GAME PIECE
    driver
        .rightTrigger()
        .onTrue(setGamePieceScoreStateCommand().alongWith(rumbleControllers(driver, operator)));
    // SET WANTED STATE TO AUTO SCORE CORAL (OR AUTO ALIGN ONLY IF AUTOMATION LEVEL IS MANUAL)
    driver
        .povRight()
        .whileTrue(setAutoAlignCoralStateCommand().alongWith(rumbleControllers(driver, operator)));
    // // SET WANTED STATE TO AUTO INTAKE ALGAE FROM THE REEF (OR AUTO ALIGN ONLY IF AUTOMATION
    // LEVEL IS MANUAL)
    driver
        .povLeft()
        .whileTrue(setAutoAlignAlgaeStateCommand().alongWith(rumbleControllers(driver, operator)));
    // SET TELEOP DRIVE STATE WHEN AUTO ALIGN IS RELEASED
    driver.povLeft().onFalse(superstructure.setTeleopDriveStateCommand());
    driver.povRight().onFalse(superstructure.setTeleopDriveStateCommand());
    // SET AUTOMATION LEVEL TO AUTO SCORE (AUTO ALIGN, RAISE, AND SCORE)
    driver
        .povUp()
        .onTrue(
            setAutomationLevelCommand(AutomationLevel.AUTO_ACTION)
                .alongWith(rumbleControllers(driver, operator)));
    // SET AUTOMATION LEVEL TO MANUAL (ONLY AUTO ALIGN)
    driver
        .povDown()
        .onTrue(
            setAutomationLevelCommand(AutomationLevel.AUTO_ALIGN)
                .alongWith(rumbleControllers(driver, operator)));
    // // SET WANTED STATE TO PREPARE CLIMB
    driver
        .x()
        .and(driver.a())
        .onTrue(
            superstructure
                .setWantedSuperStateCommand(WantedSuperState.POSITION_CLIMB_PREPARED)
                .alongWith(rumbleControllers(driver, operator)));

    //                                OPERATOR BINDS
    // // QUEUE CORAL L1 OR QUEUE ALGAE L2
    operator
        .x()
        .onTrue(
            Commands.either(
                    setCoralScoreLevelCommand(CoralScoreLevel.POSITION_CORAL_L1),
                    setAlgaeLevelCommand(AlgaeLevel.INTAKING_ALGAE_L2),
                    this::isGamePieceStateCoral)
                .alongWith(rumbleOperator(operator)));
    // // SET WANTED STATE TO L1 OR SET WANTED STATE TO ALGAE L2
    // operator
    //     .x()
    //     .and(operator.leftTrigger())
    //     .onTrue(
    //         Commands.either(
    //
    // superstructure.setWantedSuperStateCommand(WantedSuperState.POSITION_CORAL_L1),
    //
    // superstructure.setWantedSuperStateCommand(WantedSuperState.INTAKING_ALGAE_L2),
    //                 this::isGamePieceStateCoral)
    //             .alongWith(rumbleOperator(operator)));
    // // QUEUE CORAL L2 OR QUEUE ALGAE PROCESSOR
    operator
        .a()
        .onTrue(
            Commands.either(
                    setCoralScoreLevelCommand(CoralScoreLevel.POSITION_CORAL_L2),
                    setAlgaeLevelCommand(AlgaeLevel.POSITION_ALGAE_PROCESSOR),
                    this::isGamePieceStateCoral)
                .alongWith(rumbleOperator(operator)));
    // // SET WANTED STATE TO L2 OR SET WANTED STATE TO ALGAE PROCESSOR
    // operator
    //     .a()
    //     .and(operator.leftTrigger())
    //     .onTrue(
    //         Commands.either(
    //
    // superstructure.setWantedSuperStateCommand(WantedSuperState.POSITION_CORAL_L2),
    //                 superstructure.setWantedSuperStateCommand(
    //                     WantedSuperState.POSITION_ALGAE_PROCESSOR),
    //                 this::isGamePieceStateCoral)
    //             .alongWith(rumbleOperator(operator)));
    // // QUEUE CORAL L3 OR QUEUE ALGAE L3
    operator
        .b()
        .onTrue(
            Commands.either(
                    setCoralScoreLevelCommand(CoralScoreLevel.POSITION_CORAL_L3),
                    setAlgaeLevelCommand(AlgaeLevel.INTAKING_ALGAE_L3),
                    this::isGamePieceStateCoral)
                .alongWith(rumbleOperator(operator)));
    // // SET WANTED STATE TO L3 OR SET WANTED STATE TO ALGAE L3
    // operator
    //     .b()
    //     .and(operator.leftTrigger())
    //     .onTrue(
    //         Commands.either(
    //
    // superstructure.setWantedSuperStateCommand(WantedSuperState.POSITION_CORAL_L3),
    //
    // superstructure.setWantedSuperStateCommand(WantedSuperState.INTAKING_ALGAE_L3),
    //                 this::isGamePieceStateCoral)
    //             .alongWith(rumbleOperator(operator)));
    // // QUEUE CORAL L4 OR QUEUE ALGAE BARGE
    operator
        .y()
        .onTrue(
            Commands.either(
                    setCoralScoreLevelCommand(CoralScoreLevel.POSITION_CORAL_L4),
                    setAlgaeLevelCommand(AlgaeLevel.POSITION_ALGAE_BARGE),
                    this::isGamePieceStateCoral)
                .alongWith(rumbleOperator(operator)));
    // // SET WANTED STATE TO L4 OR SET WANTED STATE TO ALGAE BARGE
    // operator
    //     .y()
    //     .and(operator.leftTrigger())
    //     .onTrue(
    //         Commands.either(
    //
    // superstructure.setWantedSuperStateCommand(WantedSuperState.POSITION_CORAL_L4),
    //                 superstructure.setWantedSuperStateCommand(
    //                     WantedSuperState.POSITION_ALGAE_BARGE),
    //                 this::isGamePieceStateCoral)
    //             .alongWith(rumbleOperator(operator)));
    // // SET WANTED STATE TO CORAL STATION INTAKE
    operator
        .rightBumper()
        .onTrue(
            superstructure
                .setWantedSuperStateCommand(WantedSuperState.INTAKING_CORAL_STATION)
                .alongWith(rumbleOperator(operator)));
    // // SET WANTED STATE TO PREPARE
    operator
        .leftBumper()
        .onTrue(
            superstructure
                .setWantedSuperStateCommand(WantedSuperState.POSITION_PREPARED)
                .alongWith(rumbleOperator(operator)));
    // // SET AUTO ALIGN TO LEFT
    operator
        .povLeft()
        .onTrue(
            setAutoAlignSideCommand(TroughAutoAlignSide.LEFT).alongWith(rumbleOperator(operator)));
    // // SET AUTO ALIGN TO RIGHT
    operator
        .povRight()
        .onTrue(
            setAutoAlignSideCommand(TroughAutoAlignSide.RIGHT).alongWith(rumbleOperator(operator)));
    // // SET AUTO ALIGN TO MIDDLE
    operator
        .povUp()
        .onTrue(
            setAutoAlignSideCommand(TroughAutoAlignSide.MIDDLE)
                .alongWith(rumbleOperator(operator)));
    // // SET GAME PIECE STATE TO CORAL
    operator
        .rightTrigger()
        .onTrue(setGamePieceStateCommand(GamePieceState.CORAL).alongWith(rumbleOperator(operator)));
    // // SET GAME PIECE STATE TO ALGAE
    operator
        .leftTrigger()
        .onTrue(setGamePieceStateCommand(GamePieceState.ALGAE).alongWith(rumbleOperator(operator)));
  }

  @Override
  public void periodic() {
    logBindings();
  }

  public Command rumbleControllers(CommandXboxController driver, CommandXboxController operator) {
    return new StartEndCommand(
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 1),
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0))
        .alongWith(
            new StartEndCommand(
                () -> operator.getHID().setRumble(RumbleType.kBothRumble, 1),
                () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0)))
        .withTimeout(0.25);
  }

  public Command rumbleOperator(CommandXboxController operator) {
    return new StartEndCommand(
            () -> operator.getHID().setRumble(RumbleType.kBothRumble, 1),
            () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0))
        .withTimeout(0.1);
  }

  public Command rumbleDriver(CommandXboxController driver) {
    return new StartEndCommand(
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 1),
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0))
        .withTimeout(0.2);
  }

  public void logBindings() {
    DogLog.log("Bindings/BranchAutoAlignSide", branchAutoAlignSide);
    DogLog.log("Bindings/TroughAutoAlignSide", troughAutoAlignSide);
    DogLog.log("Bindings/CoralScoreLevel", coralScoreLevel);
    DogLog.log("Bindings/AlgaeLevel", algaeLevel);
    DogLog.log("Bindings/GamePieceState", gamePieceState);
    DogLog.log("Bindings/AutomationLevel", automationLevel);
  }

  public Command setAutoAlignSideCommand(TroughAutoAlignSide side) {
    return Commands.runOnce(() -> setAutoAlignSide(side));
  }

  public void setAutoAlignSide(TroughAutoAlignSide side) {
    switch (side) {
      default:
      case LEFT:
        branchAutoAlignSide = BranchAutoAlignSide.LEFT;
        troughAutoAlignSide = TroughAutoAlignSide.LEFT;
        break;
      case MIDDLE:
        troughAutoAlignSide = TroughAutoAlignSide.MIDDLE;
        break;
      case RIGHT:
        branchAutoAlignSide = BranchAutoAlignSide.RIGHT;
        troughAutoAlignSide = TroughAutoAlignSide.RIGHT;
        break;
    }
  }

  public Command setCoralScoreLevelCommand(CoralScoreLevel level) {
    return Commands.runOnce(() -> this.coralScoreLevel = level);
  }

  public Command setAlgaeLevelCommand(AlgaeLevel level) {
    return Commands.runOnce(() -> this.algaeLevel = level);
  }

  public Command setGamePieceStateCommand(GamePieceState state) {
    return Commands.runOnce(() -> this.gamePieceState = state);
  }

  public Command setAutomationLevelCommand(AutomationLevel level) {
    return Commands.runOnce(() -> this.automationLevel = level);
  }

  public boolean isGamePieceStateCoral() {
    return gamePieceState == GamePieceState.CORAL;
  }

  public Command setAutoAlignCoralStateCommand() {
    return Commands.defer(
        () -> superstructure.setWantedSuperStateCommand(returnAutoAlignCoralState()),
        Set.of(superstructure));
  }

  public WantedSuperState returnAutoAlignCoralState() {
    if (superstructure.hasCoral()) {
      if (automationLevel == AutomationLevel.AUTO_ACTION) {
        switch (coralScoreLevel) {
          case POSITION_CORAL_L1:
            switch (troughAutoAlignSide) {
              default:
              case LEFT:
                return WantedSuperState.AUTO_SCORE_L1_LEFT;
              case MIDDLE:
                return WantedSuperState.AUTO_SCORE_L1_MIDDLE;
              case RIGHT:
                return WantedSuperState.AUTO_SCORE_L1_RIGHT;
            }
          case POSITION_CORAL_L2:
            return (branchAutoAlignSide == BranchAutoAlignSide.LEFT)
                ? WantedSuperState.AUTO_SCORE_L2_LEFT
                : WantedSuperState.AUTO_SCORE_L2_RIGHT;
          case POSITION_CORAL_L3:
            return (branchAutoAlignSide == BranchAutoAlignSide.LEFT)
                ? WantedSuperState.AUTO_SCORE_L3_LEFT
                : WantedSuperState.AUTO_SCORE_L3_RIGHT;
          case POSITION_CORAL_L4:
            return (branchAutoAlignSide == BranchAutoAlignSide.LEFT)
                ? WantedSuperState.AUTO_SCORE_L4_LEFT
                : WantedSuperState.AUTO_SCORE_L4_RIGHT;
        }
      }
    }
    if (coralScoreLevel == CoralScoreLevel.POSITION_CORAL_L1) {
      switch (troughAutoAlignSide) {
        default:
        case LEFT:
          return WantedSuperState.AUTO_ALIGN_LEFT_TROUGH;
        case MIDDLE:
          return WantedSuperState.AUTO_ALIGN_MIDDLE_TROUGH;
        case RIGHT:
          return WantedSuperState.AUTO_ALIGN_RIGHT_TROUGH;
      }
    } else {
      return (branchAutoAlignSide == BranchAutoAlignSide.LEFT)
          ? WantedSuperState.AUTO_ALIGN_LEFT_BRANCH
          : WantedSuperState.AUTO_ALIGN_RIGHT_BRANCH;
    }
  }

  public Command setAutoAlignAlgaeStateCommand() {
    return Commands.defer(
        () -> superstructure.setWantedSuperStateCommand(returnAutoAlignAlgaeState()),
        Set.of(superstructure));
  }

  public WantedSuperState returnAutoAlignAlgaeState() {
    if (automationLevel == AutomationLevel.AUTO_ACTION) {
      return WantedSuperState.AUTO_INTAKE_ALGAE;
    } else {
      return WantedSuperState.AUTO_ALIGN_MIDDLE_ALGAE;
    }
  }

  public Command setLogicStateCommand() {
    // return this.runOnce(() -> setWantedSuperState(returnLogicState()));
    return Commands.defer(
        () -> superstructure.setWantedSuperStateCommand(returnLogicState()),
        Set.of(superstructure));
  }

  public WantedSuperState returnLogicState() {
    WantedSuperState wantedSuperState = superstructure.getWantedSuperState();
    if (superstructure.hasAlgae()) {
      if (wantedSuperState == WantedSuperState.POSITION_ALGAE_PROCESSOR) {
        return WantedSuperState.POSITION_ALGAE_BARGE;
      } else if (wantedSuperState == WantedSuperState.POSITION_ALGAE_BARGE) {
        return WantedSuperState.POSITION_ALGAE_PROCESSOR;
      }
    }
    switch (gamePieceState) {
      default:
      case CORAL:
        switch (coralScoreLevel) {
          default:
          case POSITION_CORAL_L1:
            return WantedSuperState.POSITION_CORAL_L1;
          case POSITION_CORAL_L2:
            return WantedSuperState.POSITION_CORAL_L2;
          case POSITION_CORAL_L3:
            return WantedSuperState.POSITION_CORAL_L3;
          case POSITION_CORAL_L4:
            return WantedSuperState.POSITION_CORAL_L4;
        }
      case ALGAE:
        switch (algaeLevel) {
          default:
          case POSITION_ALGAE_BARGE:
            return WantedSuperState.POSITION_ALGAE_BARGE;
          case INTAKING_ALGAE_L3:
            return WantedSuperState.INTAKING_ALGAE_L3;
          case INTAKING_ALGAE_L2:
            return WantedSuperState.INTAKING_ALGAE_L2;
          case POSITION_ALGAE_PROCESSOR:
            return WantedSuperState.POSITION_ALGAE_PROCESSOR;
        }
    }
  }

  public Command setGamePieceScoreStateCommand() {
    return Commands.defer(
        () -> superstructure.setWantedSuperStateCommand(returnGamePieceScoreState()),
        Set.of(superstructure));
  }

  public WantedSuperState returnGamePieceScoreState() {
    if (superstructure.hasAlgae()) {
      return WantedSuperState.SCORING_ALGAE;
    } else {
      return WantedSuperState.SCORING_CORAL;
    }
  }
}
