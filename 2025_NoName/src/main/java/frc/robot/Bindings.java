package frc.robot;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;

public class Bindings {

    /* DRIVER */
    public enum AutomationLevel {
        MANUAL,
        AUTO_ACTION,
    }

    /* OPERATOR */
    public enum AutoAlignSide {
        LEFT,
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
    private AutoAlignSide autoAlignSide = AutoAlignSide.LEFT;
    private CoralScoreLevel coralScoreLevel = CoralScoreLevel.POSITION_CORAL_L4;
    private AlgaeLevel algaeLevel = AlgaeLevel.POSITION_ALGAE_PROCESSOR;
    private GamePieceState gamePieceState = GamePieceState.CORAL;
    private AutomationLevel automationLevel = AutomationLevel.AUTO_ACTION;

    public Bindings(CommandXboxController driver, CommandXboxController operator, Superstructure superstructure) {
        this.superstructure = superstructure;
        //                               DRIVER BINDS
        // ZERO GYRO
        driver.start().onTrue(superstructure.zeroGyroCommand());
        // // SET WANTED STATE TO A LOGIC STATE
        driver.rightBumper().onTrue(setLogicStateCommand());
        // // SET WANTED STATE TO INTAKING ALGAE GROUND
        driver
                .leftBumper()
                .onTrue(
                        superstructure.setWantedSuperStateCommand(
                                WantedSuperState.INTAKING_ALGAE_GROUND))
                .onFalse(
                        superstructure.setWantedSuperStateCommand(
                                WantedSuperState.POSITION_ALGAE_PROCESSOR));
        // // SET WANTED STATE TO SCORING CORAL
        driver
                .rightTrigger()
                .onTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.SCORING_CORAL));
        // // SET WANTED STATE TO SCORING ALGAE
        driver
                .leftTrigger()
                .onTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.SCORING_ALGAE));
        // SET WANTED STATE TO AUTO SCORE CORAL (OR AUTO ALIGN ONLY IF AUTOMATION LEVEL IS MANUAL)
        driver.povRight().whileTrue(setAutoCoralScoreStateCommand());
        // // SET WANTED STATE TO AUTO INTAKE ALGAE FROM THE REEF (OR AUTO ALIGN ONLY IF AUTOMATION
        // LEVEL IS MANUAL)
        driver.povLeft().onTrue(setAutoAlgaeIntakeStateCommand());
        // SET TELEOP DRIVE STATE WHEN AUTO ALIGN IS RELEASED
        driver.povLeft().onFalse(superstructure.setTeleopDriveStateCommand());
        driver.povRight().onFalse(superstructure.setTeleopDriveStateCommand());
        // SET AUTOMATION LEVEL TO AUTO SCORE (AUTO ALIGN, RAISE, AND SCORE)
        driver
                .povUp()
                .onTrue(setAutomationLevelCommand(AutomationLevel.AUTO_ACTION));
        // SET AUTOMATION LEVEL TO MANUAL (ONLY AUTO ALIGN)
        driver
                .povDown()
                .onTrue(setAutomationLevelCommand(AutomationLevel.MANUAL));

        //                                OPERATOR BINDS
        // // QUEUE CORAL L1 OR QUEUE ALGAE L2
        operator
                .x()
                .onTrue(
                        Commands.either(
                                setCoralScoreLevelCommand(CoralScoreLevel.POSITION_CORAL_L1),
                                setAlgaeLevelCommand(AlgaeLevel.INTAKING_ALGAE_L2),
                                this::isGamePieceStateCoral));
        // // SET WANTED STATE TO L1 OR SET WANTED STATE TO ALGAE L2
        operator
                .x()
                .and(operator.leftTrigger())
                .onTrue(
                        Commands.either(
                                superstructure.setWantedSuperStateCommand(
                                        WantedSuperState.POSITION_CORAL_L1),
                                superstructure.setWantedSuperStateCommand(
                                        WantedSuperState.INTAKING_ALGAE_L2),
                                this::isGamePieceStateCoral));
        // // QUEUE CORAL L2 OR QUEUE ALGAE PROCESSOR
        operator
                .a()
                .onTrue(
                        Commands.either(
                                setCoralScoreLevelCommand(CoralScoreLevel.POSITION_CORAL_L2),
                                setAlgaeLevelCommand(AlgaeLevel.POSITION_ALGAE_PROCESSOR),
                                this::isGamePieceStateCoral));
        // // SET WANTED STATE TO L2 OR SET WANTED STATE TO ALGAE PROCESSOR
        operator
                .a()
                .and(operator.leftTrigger())
                .onTrue(
                        Commands.either(
                                superstructure.setWantedSuperStateCommand(
                                        WantedSuperState.POSITION_CORAL_L2),
                                superstructure.setWantedSuperStateCommand(
                                        WantedSuperState.POSITION_ALGAE_PROCESSOR),
                                this::isGamePieceStateCoral));
        // // QUEUE CORAL L3 OR QUEUE ALGAE L3
        operator
                .b()
                .onTrue(
                        Commands.either(
                                setCoralScoreLevelCommand(CoralScoreLevel.POSITION_CORAL_L3),
                                setAlgaeLevelCommand(AlgaeLevel.INTAKING_ALGAE_L3),
                                this::isGamePieceStateCoral));
        // // SET WANTED STATE TO L3 OR SET WANTED STATE TO ALGAE L3
        operator
                .b()
                .and(operator.leftTrigger())
                .onTrue(
                        Commands.either(
                                superstructure.setWantedSuperStateCommand(
                                        WantedSuperState.POSITION_CORAL_L3),
                                superstructure.setWantedSuperStateCommand(
                                        WantedSuperState.INTAKING_ALGAE_L3),
                                this::isGamePieceStateCoral));
        // // QUEUE CORAL L4 OR QUEUE ALGAE BARGE
        operator
                .y()
                .onTrue(
                        Commands.either(
                                setCoralScoreLevelCommand(CoralScoreLevel.POSITION_CORAL_L4),
                                setAlgaeLevelCommand(AlgaeLevel.POSITION_ALGAE_BARGE),
                                this::isGamePieceStateCoral));
        // // SET WANTED STATE TO L4 OR SET WANTED STATE TO ALGAE BARGE
        operator
                .y()
                .and(operator.leftTrigger())
                .onTrue(
                        Commands.either(
                                superstructure.setWantedSuperStateCommand(
                                        WantedSuperState.POSITION_CORAL_L4),
                                superstructure.setWantedSuperStateCommand(
                                        WantedSuperState.POSITION_ALGAE_BARGE),
                                this::isGamePieceStateCoral));
        // // SET WANTED STATE TO CORAL STATION INTAKE
        operator
                .rightBumper()
                .onTrue(
                        superstructure.setWantedSuperStateCommand(
                                WantedSuperState.INTAKING_CORAL_STATION));
        // // SET WANTED STATE TO PREPARE
        operator
                .leftBumper()
                .onTrue(
                        superstructure.setWantedSuperStateCommand(WantedSuperState.POSITION_PREPARED));
        // // SET WANTED STATE TO PREPARE CLIMB
        operator
                .rightTrigger()
                .and(operator.leftTrigger())
                .onTrue(
                        superstructure.setWantedSuperStateCommand(
                                WantedSuperState.POSITION_CLIMB_PREPARED));
        // // SET AUTO ALIGN TO LEFT
        operator.povLeft().onTrue(setAutoAlignSideCommand(AutoAlignSide.LEFT));
        // // SET AUTO ALIGN TO RIGHT
        operator.povRight().onTrue(setAutoAlignSideCommand(AutoAlignSide.RIGHT));
        // // SET GAME PIECE STATE TO CORAL
        operator.povUp().onTrue(setGamePieceStateCommand(GamePieceState.CORAL));
        // // SET GAME PIECE STATE TO ALGAE
        operator
                .povDown()
                .onTrue(setGamePieceStateCommand(GamePieceState.ALGAE));
    }

    public Command setAutoAlignSideCommand(AutoAlignSide side) {
        return Commands.runOnce(() -> this.autoAlignSide = side);
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

    public Command setAutoCoralScoreStateCommand() {
        return Commands.defer(
            () -> superstructure.setWantedSuperStateCommand(returnAutoCoralScoreState()),
            Set.of(superstructure)
        );
    }

    public WantedSuperState returnAutoCoralScoreState() {
        if (superstructure.hasCoral()) {
            switch (automationLevel) {
                case AUTO_ACTION:
                    switch (coralScoreLevel) {
                        case POSITION_CORAL_L1:
                            switch (autoAlignSide) {
                                default:
                                case LEFT:
                                    return WantedSuperState.AUTO_SCORE_L1_LEFT;
                                case RIGHT:
                                    return WantedSuperState.AUTO_SCORE_L1_RIGHT;
                            }
                        case POSITION_CORAL_L2:
                            switch (autoAlignSide) {
                                default:
                                case LEFT:
                                    return WantedSuperState.AUTO_SCORE_L2_LEFT;
                                case RIGHT:
                                    return WantedSuperState.AUTO_SCORE_L2_RIGHT;
                            }
                        case POSITION_CORAL_L3:
                            switch (autoAlignSide) {
                                default:
                                case LEFT:
                                    return WantedSuperState.AUTO_SCORE_L3_LEFT;
                                case RIGHT:
                                    return WantedSuperState.AUTO_SCORE_L3_RIGHT;
                            }
                        case POSITION_CORAL_L4:
                            switch (autoAlignSide) {
                                default:
                                case LEFT:
                                    return WantedSuperState.AUTO_SCORE_L4_LEFT;
                                case RIGHT:
                                    return WantedSuperState.AUTO_SCORE_L4_RIGHT;
                            }
                        default:
                            break;
                    }
                default:
                    break;
            }
        }
        switch (autoAlignSide) {
            default:
            case LEFT:
                return WantedSuperState.AUTO_ALIGN_LEFT_BRANCH;
            case RIGHT:
                return WantedSuperState.AUTO_ALIGN_RIGHT_BRANCH;
        }
    }

    public Command setAutoAlgaeIntakeStateCommand() {
        return Commands.defer(
            () -> superstructure.setWantedSuperStateCommand(returnAutoAlgaeIntakeState()),
            Set.of(superstructure)
        );
    }

    public WantedSuperState returnAutoAlgaeIntakeState() {
        switch (automationLevel) {
            case AUTO_ACTION:
                return WantedSuperState.AUTO_INTAKE_ALGAE;
            default:
                return WantedSuperState.AUTO_ALIGN_MIDDLE_ALGAE;
        }
    }

    public Command setLogicStateCommand() {
        // return this.runOnce(() -> setWantedSuperState(returnLogicState()));
        return Commands.defer(
            () -> superstructure.setWantedSuperStateCommand(returnLogicState()),
            Set.of(superstructure)
        );
    }

    public WantedSuperState returnLogicState() {
        // if (rollers.isAlgaeDetected()) {
        //     switch (wantedSuperState) {
        //         case POSITION_ALGAE_PROCESSOR:
        //             return WantedSuperState.POSITION_ALGAE_BARGE;
        //         case POSITION_ALGAE_BARGE:
        //             return WantedSuperState.POSITION_ALGAE_PROCESSOR;
        //         default:
        //             break;
        //     }
        // }
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

}
