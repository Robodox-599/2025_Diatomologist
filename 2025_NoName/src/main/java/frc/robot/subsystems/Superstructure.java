package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.SafetyChecker;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endefector.endefectorrollers.Rollers;
import frc.robot.subsystems.endefector.endefectorwrist.Wrist;
import frc.robot.subsystems.leds.LEDs;

public class Superstructure extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;
  private final Elevator elevator;
  private final Wrist wrist;
  private final Rollers rollers;
  private final LEDs leds;
  // private final Climb climb;
  // private final Vision vision;
  private final SafetyChecker safetyChecker;
  private final CommandXboxController driver;
  private final CommandXboxController operator;

  private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
  private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
  private WantedSuperState nextSuperState = WantedSuperState.STOPPED;

  public enum CurrentSuperState {
    INTAKING_CORAL_STATION,
    ENSURING_CORAL,
    INTAKING_ALGAE_GROUND,
    INTAKING_ALGAE_L2,
    INTAKING_ALGAE_L3,
    POSITION_PREPARED,
    POSITION_CORAL_L1,
    POSITION_CORAL_L2,
    POSITION_CORAL_L3,
    POSITION_CORAL_L4,
    POSITION_ALGAE_PROCESSOR,
    POSITION_ALGAE_BARGE,
    POSITION_CLIMB_PREPARED,
    SCORING_CORAL,
    SCORING_ALGAE,
    CLIMBING,
    STOPPED,
  }

  public enum WantedSuperState {
    INTAKING_CORAL_STATION,
    ENSURING_CORAL,
    INTAKING_ALGAE_GROUND,
    INTAKING_ALGAE_L2,
    INTAKING_ALGAE_L3,
    POSITION_PREPARED,
    POSITION_CORAL_L1,
    POSITION_CORAL_L2,
    POSITION_CORAL_L3,
    POSITION_CORAL_L4,
    POSITION_ALGAE_PROCESSOR,
    POSITION_ALGAE_BARGE,
    POSITION_CLIMB_PREPARED,
    SCORING_CORAL,
    SCORING_ALGAE,
    CLIMBING,
    STOPPED,
    NO_STATE,
  }

  public Superstructure(
      CommandSwerveDrivetrain drivetrain,
      Elevator elevator,
      Wrist wrist,
      Rollers rollers,
      LEDs LEDs,
      // Climb climb,
      // Vision vision,
      SafetyChecker safetyChecker,
      CommandXboxController driver,
      CommandXboxController operator) {
    this.drivetrain = drivetrain;
    this.elevator = elevator;
    this.wrist = wrist;
    this.rollers = rollers;
    this.leds = LEDs;
    // this.climb = climb;
    // this.vision = vision;
    this.safetyChecker = safetyChecker;
    this.driver = driver;
    this.operator = operator;
  }

  @Override
  public void periodic() {
    drivetrain.updateInputs();
    elevator.updateInputs();
    rollers.updateInputs();
    wrist.updateInputs();
    leds.updateInputs();
    // climb.updateInputs();
    // vision.updateInputs();
    currentSuperState = handleStateTransitions();
    applyStates();

    DogLog.log("Superstructure/NextSuperState", nextSuperState);
    DogLog.log("Superstructure/WantedSuperState", wantedSuperState);
    DogLog.log("Superstructure/CurrentSuperState", currentSuperState);
  }

  private CurrentSuperState handleStateTransitions() {
    switch (wantedSuperState) {
      case INTAKING_CORAL_STATION:
        if (rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.ENSURING_CORAL;
          wantedSuperState = WantedSuperState.ENSURING_CORAL;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
        }
        break;
      case ENSURING_CORAL:
        if (rollers.isCoralEnsured()) {
          currentSuperState = CurrentSuperState.POSITION_PREPARED;
          wantedSuperState = WantedSuperState.POSITION_PREPARED;
        } else if (!rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
        } else {
          currentSuperState = CurrentSuperState.ENSURING_CORAL;
        }
        break;
      case INTAKING_ALGAE_GROUND:
        if (rollers.isAlgaeDetected()) {
          currentSuperState = CurrentSuperState.POSITION_ALGAE_PROCESSOR;
          wantedSuperState = WantedSuperState.POSITION_ALGAE_PROCESSOR;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_ALGAE_GROUND;
        }
        break;
      case INTAKING_ALGAE_L2:
        if (rollers.isAlgaeDetected()) {
          currentSuperState = CurrentSuperState.POSITION_ALGAE_PROCESSOR;
          wantedSuperState = WantedSuperState.POSITION_ALGAE_PROCESSOR;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_ALGAE_L2;
        }
        break;
      case INTAKING_ALGAE_L3:
        if (rollers.isAlgaeDetected()) {
          currentSuperState = CurrentSuperState.POSITION_ALGAE_PROCESSOR;
          wantedSuperState = WantedSuperState.POSITION_ALGAE_PROCESSOR;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_ALGAE_L3;
        }
        break;
      case POSITION_PREPARED:
        currentSuperState = CurrentSuperState.POSITION_PREPARED;
        break;
      case POSITION_CORAL_L1:
        currentSuperState = CurrentSuperState.POSITION_CORAL_L1;
        break;
      case POSITION_CORAL_L2:
        currentSuperState = CurrentSuperState.POSITION_CORAL_L2;
        break;
      case POSITION_CORAL_L3:
        currentSuperState = CurrentSuperState.POSITION_CORAL_L3;
        break;
      case POSITION_CORAL_L4:
        currentSuperState = CurrentSuperState.POSITION_CORAL_L4;
        break;
      case POSITION_ALGAE_PROCESSOR:
        currentSuperState = CurrentSuperState.POSITION_ALGAE_PROCESSOR;
        break;
      case POSITION_ALGAE_BARGE:
        currentSuperState = CurrentSuperState.POSITION_ALGAE_BARGE;
        break;
        // case POSITION_CLIMB_PREPARED:
        //   if (climb.isCageDetected()) {
        //     currentSuperState = CurrentSuperState.CLIMBING;
        //     wantedSuperState = WantedSuperState.CLIMBING;
        //   } else {
        //     currentSuperState = CurrentSuperState.POSITION_CLIMB_PREPARED;
        //   }
        //   break;
      case SCORING_CORAL:
        if (!rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
        } else {
          currentSuperState = CurrentSuperState.SCORING_CORAL;
        }
        break;
      case SCORING_ALGAE:
        if (!rollers.isAlgaeDetected()) {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
          wantedSuperState = WantedSuperState.INTAKING_CORAL_STATION;
        } else {
          currentSuperState = CurrentSuperState.SCORING_ALGAE;
        }
        break;
        // case CLIMBING:
        //   if (!climb.isCageDetected()) {
        //     currentSuperState = CurrentSuperState.POSITION_CLIMB_PREPARED;
        //     wantedSuperState = WantedSuperState.POSITION_CLIMB_PREPARED;
        //   } else {
        //     currentSuperState = CurrentSuperState.CLIMBING;
        //   }
      case STOPPED:
        currentSuperState = CurrentSuperState.STOPPED;
        break;
      default:
        currentSuperState = CurrentSuperState.STOPPED;
        break;
    }
    return currentSuperState;
  }

  private void applyStates() {
    switch (currentSuperState) {
      case INTAKING_CORAL_STATION:
        intakeCoralStation();
        break;
      case ENSURING_CORAL:
        ensureCoral();
        break;
      case INTAKING_ALGAE_GROUND:
        intakeAlgaeGround();
        break;
      case INTAKING_ALGAE_L2:
        intakeAlgaeL2();
        break;
      case INTAKING_ALGAE_L3:
        intakeAlgaeL3();
        break;
      case POSITION_PREPARED:
        prepare();
        break;
      case POSITION_CORAL_L1:
        positionToCoralL1();
        break;
      case POSITION_CORAL_L2:
        positionToCoralL2();
        break;
      case POSITION_CORAL_L3:
        positionToCoralL3();
        break;
      case POSITION_CORAL_L4:
        positionToCoralL4();
        break;
      case POSITION_ALGAE_PROCESSOR:
        positionToAlgaeProcessor();
        break;
      case POSITION_ALGAE_BARGE:
        positionToAlgaeBarge();
        break;
      case POSITION_CLIMB_PREPARED:
        positionToClimbPrepared();
        break;
      case SCORING_CORAL:
        scoreCoral();
        break;
      case SCORING_ALGAE:
        scoreAlgae();
        break;
      case CLIMBING:
        climbing();
        break;
      case STOPPED:
        stop();
        break;
      default:
        stop();
        break;
    }
  }

  private void intakeCoralStation() {
    elevator.setWantedState(Elevator.WantedState.INTAKING_CORAL_STATION);
    rollers.setWantedState(Rollers.WantedState.INTAKING_CORAL_STATION);
    wrist.setWantedState(Wrist.WantedState.INTAKING_CORAL_STATION);
    leds.setCurrentState(LEDs.CurrentState.INTAKING_CORAL_STATION);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void ensureCoral() {
    elevator.setWantedState(Elevator.WantedState.INTAKING_CORAL_STATION);
    rollers.setWantedState(Rollers.WantedState.ENSURING_CORAL);
    wrist.setWantedState(Wrist.WantedState.INTAKING_CORAL_STATION);
    leds.setCurrentState(LEDs.CurrentState.ENSURING_CORAL);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void intakeAlgaeGround() {
    elevator.setWantedState(Elevator.WantedState.INTAKING_ALGAE_GROUND);
    rollers.setWantedState(Rollers.WantedState.INTAKING_ALGAE);
    wrist.setWantedState(Wrist.WantedState.INTAKING_ALGAE_GROUND);
    leds.setCurrentState(LEDs.CurrentState.INTAKING_ALGAE_GROUND);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void intakeAlgaeL2() {
    elevator.setWantedState(Elevator.WantedState.INTAKING_ALGAE_L2);
    rollers.setWantedState(Rollers.WantedState.INTAKING_ALGAE);
    wrist.setWantedState(Wrist.WantedState.INTAKING_ALGAE_REEF);
    leds.setCurrentState(LEDs.CurrentState.INTAKING_ALGAE_L2);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void intakeAlgaeL3() {
    elevator.setWantedState(Elevator.WantedState.INTAKING_ALGAE_L3);
    rollers.setWantedState(Rollers.WantedState.INTAKING_ALGAE);
    wrist.setWantedState(Wrist.WantedState.INTAKING_ALGAE_REEF);
    leds.setCurrentState(LEDs.CurrentState.INTAKING_ALGAE_L3);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void prepare() {
    elevator.setWantedState(Elevator.WantedState.POSITION_PREPARED);
    rollers.setWantedState(Rollers.WantedState.STOPPED);
    wrist.setWantedState(Wrist.WantedState.POSITION_PREPARED);
    leds.setCurrentState(LEDs.CurrentState.POSITION_PREPARED);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToCoralL1() {
    elevator.setWantedState(Elevator.WantedState.POSITION_CORAL_L1);
    rollers.setWantedState(Rollers.WantedState.STOPPED);
    wrist.setWantedState(Wrist.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.POSITION_CORAL_L1);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToCoralL2() {
    elevator.setWantedState(Elevator.WantedState.POSITION_CORAL_L2);
    rollers.setWantedState(Rollers.WantedState.STOPPED);
    wrist.setWantedState(Wrist.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.POSITION_CORAL_L2);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToCoralL3() {
    elevator.setWantedState(Elevator.WantedState.POSITION_CORAL_L3);
    rollers.setWantedState(Rollers.WantedState.STOPPED);
    wrist.setWantedState(Wrist.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.POSITION_CORAL_L3);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToCoralL4() {
    elevator.setWantedState(Elevator.WantedState.POSITION_CORAL_L4);
    rollers.setWantedState(Rollers.WantedState.STOPPED);
    wrist.setWantedState(Wrist.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.POSITION_CORAL_L4);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToAlgaeProcessor() {
    elevator.setWantedState(Elevator.WantedState.POSITION_ALGAE_PROCESSOR);
    // rollers.setWantedState(Rollers.WantedState.HOLD_ALGAE);
    wrist.setWantedState(Wrist.WantedState.SCORING_ALGAE);
    leds.setCurrentState(LEDs.CurrentState.POSITION_ALGAE_PROCESSOR);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToAlgaeBarge() {
    elevator.setWantedState(Elevator.WantedState.POSITION_ALGAE_BARGE);
    // rollers.setWantedState(Rollers.WantedState.HOLD_ALGAE);
    wrist.setWantedState(Wrist.WantedState.SCORING_ALGAE);
    leds.setCurrentState(LEDs.CurrentState.POSITION_ALGAE_BARGE);
    // climb.setWantedState(Climb.WantedState.STOWED);
  }

  private void positionToClimbPrepared() {
    elevator.setWantedState(Elevator.WantedState.INTAKING_CORAL_STATION);
    rollers.setWantedState(Rollers.WantedState.STOPPED);
    wrist.setWantedState(Wrist.WantedState.INTAKING_CORAL_STATION);
    // climb.setWantedState(Climb.WantedState.CLIMB_PREPARED);
    leds.setCurrentState(LEDs.CurrentState.POSITION_CLIMB_PREPARED);
  }

  private void scoreCoral() {
    rollers.setWantedState(Rollers.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.SCORING_CORAL);
  }

  private void scoreAlgae() {
    rollers.setWantedState(Rollers.WantedState.SCORING_ALGAE);
    leds.setCurrentState(LEDs.CurrentState.SCORING_ALGAE);
  }

  private void climbing() {
    // climb.setWantedState(Climb.WantedState.CLIMBING);
    leds.setCurrentState(LEDs.CurrentState.CLIMBING);
  }

  private void stop() {
    elevator.setWantedState(Elevator.WantedState.STOPPED);
    rollers.setWantedState(Rollers.WantedState.STOPPED);
    wrist.setWantedState(Wrist.WantedState.STOPPED);
    leds.setCurrentState(LEDs.CurrentState.NO_STATE);
    // climb.setWantedState(Climb.WantedState.STOPPED);
  }

  public Command setNextSuperStateCommand(WantedSuperState nextState) {
    return Commands.parallel(
        this.runOnce(() -> setNextSuperState(nextState)), rumbleControllers().withTimeout(0.2));
  }

  private void setNextSuperState(WantedSuperState state) {
    nextSuperState = state;
  }

  public Command updateWantedSuperStateCommand() {
    return Commands.either(
        Commands.sequence(
            rumbleControllers().withTimeout(0.1),
            new WaitCommand(0.2),
            rumbleControllers().withTimeout(0.1)),
        Commands.parallel(
            this.runOnce(() -> updateWantedSuperState()), rumbleControllers().withTimeout(0.3)),
        (() -> nextSuperState == WantedSuperState.NO_STATE));
  }

  private void updateWantedSuperState() {
    wantedSuperState = nextSuperState;
    nextSuperState = WantedSuperState.NO_STATE;
  }

  public Command setWantedSuperStateCommand(WantedSuperState wantedState) {
    return this.runOnce(() -> setWantedSuperState(wantedState));
  }

  private void setWantedSuperState(WantedSuperState state) {
    wantedSuperState = state;
    nextSuperState = WantedSuperState.NO_STATE;
  }

  private boolean isAtWantedState() {
    return (wantedSuperState.name().equals(currentSuperState.name()))
        && safetyChecker.isAtSetpoints();
  }

  public Command zeroGyroCommand() {
    return this.runOnce(() -> drivetrain.zeroGyro());
  }

  public Command rumbleDriverController() {
    return new StartEndCommand(
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 1),
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0))
        .withTimeout(0.5);
  }

  public Command rumbleOperatorController() {
    return new StartEndCommand(
            () -> operator.getHID().setRumble(RumbleType.kBothRumble, 1),
            () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0))
        .withTimeout(0.5);
  }

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
}
