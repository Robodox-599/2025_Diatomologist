package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private final SafetyChecker safetyChecker;

  private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
  private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
  private WantedSuperState nextSuperState = WantedSuperState.STOPPED;

  public enum CurrentSuperState {
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE_GROUND,
    INTAKING_ALGAE_L2,
    INTAKING_ALGAE_L3,
    PREPARED,
    SCORING_CORAL_L1,
    SCORING_CORAL_L2,
    SCORING_CORAL_L3,
    SCORING_CORAL_L4,
    MOVING_TO_ALGAE_PROCESSOR,
    MOVING_TO_ALGAE_BARGE,
    SCORING_ALGAE,
    STOPPED,
  }

  public enum WantedSuperState {
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE_GROUND,
    INTAKING_ALGAE_L2,
    INTAKING_ALGAE_L3,
    PREPARED,
    SCORING_CORAL_L1,
    SCORING_CORAL_L2,
    SCORING_CORAL_L3,
    SCORING_CORAL_L4,
    MOVING_TO_ALGAE_PROCESSOR,
    MOVING_TO_ALGAE_BARGE,
    SCORING_ALGAE,
    STOPPED,
  }

  public Superstructure(
      CommandSwerveDrivetrain drivetrain,
      Elevator elevator,
      Wrist wrist,
      Rollers rollers,
      // Climb climb,
      LEDs LEDs,
      SafetyChecker safetyChecker) {
    this.drivetrain = drivetrain;
    this.elevator = elevator;
    this.wrist = wrist;
    this.rollers = rollers;
    this.leds = LEDs;
    this.safetyChecker = safetyChecker;
  }

  @Override
  public void periodic() {
    elevator.updateInputs();
    rollers.updateInputs();
    wrist.updateInputs();
    leds.updateInputs();
    currentSuperState = handleStateTransitions();
    applyStates();

    DogLog.log("Superstructure/NextSuperState", nextSuperState);
    DogLog.log("Superstructure/WantedSuperState", wantedSuperState);
    DogLog.log("Superstructure/CurrentSuperState", currentSuperState);
  }

  private void setNextSuperState(WantedSuperState state) {
    nextSuperState = state;
  }

  private void updateWantedSuperState() {
    wantedSuperState = nextSuperState;
  }

  private CurrentSuperState handleStateTransitions() {
    switch (wantedSuperState) {
      case INTAKING_CORAL_STATION:
        if (rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.PREPARED;
          wantedSuperState = WantedSuperState.PREPARED;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_CORAL_STATION;
        }
        break;
      case INTAKING_ALGAE_GROUND:
        if (rollers.isAlgaeDetected()) {
          currentSuperState = CurrentSuperState.MOVING_TO_ALGAE_PROCESSOR;
          wantedSuperState = WantedSuperState.MOVING_TO_ALGAE_PROCESSOR;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_ALGAE_GROUND;
        }
        break;
      case INTAKING_ALGAE_L2:
        if (rollers.isAlgaeDetected()) {
          currentSuperState = CurrentSuperState.MOVING_TO_ALGAE_PROCESSOR;
          wantedSuperState = WantedSuperState.MOVING_TO_ALGAE_PROCESSOR;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_ALGAE_L2;
        }
        break;
      case INTAKING_ALGAE_L3:
        if (rollers.isAlgaeDetected()) {
          currentSuperState = CurrentSuperState.MOVING_TO_ALGAE_PROCESSOR;
          wantedSuperState = WantedSuperState.MOVING_TO_ALGAE_PROCESSOR;
        } else {
          currentSuperState = CurrentSuperState.INTAKING_ALGAE_L3;
        }
        break;
      case PREPARED:
        currentSuperState = CurrentSuperState.PREPARED;
        break;
      case SCORING_CORAL_L1:
        if (!rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.PREPARED;
          wantedSuperState = WantedSuperState.PREPARED;
        } else {
          currentSuperState = CurrentSuperState.SCORING_CORAL_L1;
        }
        break;
      case SCORING_CORAL_L2:
        if (!rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.PREPARED;
          wantedSuperState = WantedSuperState.PREPARED;
        } else {
          currentSuperState = CurrentSuperState.SCORING_CORAL_L2;
        }
        break;
      case SCORING_CORAL_L3:
        if (!rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.PREPARED;
          wantedSuperState = WantedSuperState.PREPARED;
        } else {
          currentSuperState = CurrentSuperState.SCORING_CORAL_L3;
        }
        break;
      case SCORING_CORAL_L4:
        if (!rollers.isCoralDetected()) {
          currentSuperState = CurrentSuperState.PREPARED;
          wantedSuperState = WantedSuperState.PREPARED;
        } else {
          currentSuperState = CurrentSuperState.SCORING_CORAL_L4;
        }
        break;
      case MOVING_TO_ALGAE_PROCESSOR:
        if (!rollers.isAlgaeDetected()) {
          currentSuperState = CurrentSuperState.PREPARED;
          wantedSuperState = WantedSuperState.PREPARED;
        } else {
          currentSuperState = CurrentSuperState.MOVING_TO_ALGAE_PROCESSOR;
        }
        break;
      case MOVING_TO_ALGAE_BARGE:
        if (!rollers.isAlgaeDetected()) {
          currentSuperState = CurrentSuperState.PREPARED;
          wantedSuperState = WantedSuperState.PREPARED;
        } else {
          currentSuperState = CurrentSuperState.MOVING_TO_ALGAE_BARGE;
        }
        break;
      case SCORING_ALGAE:
        if (!rollers.isAlgaeDetected()) {
          currentSuperState = CurrentSuperState.PREPARED;
          wantedSuperState = WantedSuperState.PREPARED;
        } else {
          currentSuperState = CurrentSuperState.SCORING_ALGAE;
        }
        break;
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
      case INTAKING_ALGAE_GROUND:
        intakeAlgaeGround();
        break;
      case INTAKING_ALGAE_L2:
        intakeAlgaeL2();
        break;
      case INTAKING_ALGAE_L3:
        intakeAlgaeL3();
        break;
      case PREPARED:
        prepare();
        break;
      case SCORING_CORAL_L1:
        scoreCoralL1();
        break;
      case SCORING_CORAL_L2:
        scoreCoralL2();
        break;
      case SCORING_CORAL_L3:
        scoreCoralL3();
        break;
      case SCORING_CORAL_L4:
        scoreCoralL4();
        break;
      case MOVING_TO_ALGAE_PROCESSOR:
        moveToAlgaeProcessor();
        break;
      case MOVING_TO_ALGAE_BARGE:
        moveToAlgaeBarge();
        break;
      case SCORING_ALGAE:
        scoreAlgae();
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
  }

  private void intakeAlgaeGround() {
    elevator.setWantedState(Elevator.WantedState.INTAKING_ALGAE_GROUND);
    rollers.setWantedState(Rollers.WantedState.INTAKING_ALGAE);
    wrist.setWantedState(Wrist.WantedState.INTAKING_ALGAE_GROUND);
    leds.setCurrentState(LEDs.CurrentState.INTAKING_ALGAE_GROUND);
  }

  private void intakeAlgaeL2() {
    elevator.setWantedState(Elevator.WantedState.INTAKING_ALGAE_L2);
    rollers.setWantedState(Rollers.WantedState.INTAKING_ALGAE);
    wrist.setWantedState(Wrist.WantedState.INTAKING_ALGAE_REEF);
    leds.setCurrentState(LEDs.CurrentState.INTAKING_ALGAE_L2);
  }

  private void intakeAlgaeL3() {
    elevator.setWantedState(Elevator.WantedState.INTAKING_ALGAE_L3);
    rollers.setWantedState(Rollers.WantedState.INTAKING_ALGAE);
    wrist.setWantedState(Wrist.WantedState.INTAKING_ALGAE_REEF);
    leds.setCurrentState(LEDs.CurrentState.INTAKING_ALGAE_L3);
  }

  private void prepare() {
    elevator.setWantedState(Elevator.WantedState.PREPARED);
    rollers.setWantedState(Rollers.WantedState.STOPPED);
    wrist.setWantedState(Wrist.WantedState.PREPARED);
    leds.setCurrentState(LEDs.CurrentState.PREPARED);
  }

  private void scoreCoralL1() {
    elevator.setWantedState(Elevator.WantedState.SCORING_CORAL_L1);
    rollers.setWantedState(Rollers.WantedState.SCORING_CORAL);
    wrist.setWantedState(Wrist.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.SCORING_CORAL_L1);
  }

  private void scoreCoralL2() {
    elevator.setWantedState(Elevator.WantedState.SCORING_CORAL_L2);
    rollers.setWantedState(Rollers.WantedState.SCORING_CORAL);
    wrist.setWantedState(Wrist.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.SCORING_CORAL_L2);
  }

  private void scoreCoralL3() {
    elevator.setWantedState(Elevator.WantedState.SCORING_CORAL_L3);
    rollers.setWantedState(Rollers.WantedState.SCORING_CORAL);
    wrist.setWantedState(Wrist.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.SCORING_CORAL_L3);
  }

  private void scoreCoralL4() {
    elevator.setWantedState(Elevator.WantedState.SCORING_CORAL_L4);
    rollers.setWantedState(Rollers.WantedState.SCORING_CORAL);
    wrist.setWantedState(Wrist.WantedState.SCORING_CORAL);
    leds.setCurrentState(LEDs.CurrentState.SCORING_CORAL_L4);
  }

  private void moveToAlgaeProcessor() {
    elevator.setWantedState(Elevator.WantedState.SCORING_ALGAE_PROCESSOR);
    wrist.setWantedState(Wrist.WantedState.SCORING_ALGAE);
    leds.setCurrentState(LEDs.CurrentState.SCORING_ALGAE_PROCESSOR);
  }

  private void moveToAlgaeBarge() {
    elevator.setWantedState(Elevator.WantedState.SCORING_ALGAE_BARGE);
    wrist.setWantedState(Wrist.WantedState.SCORING_ALGAE);
    leds.setCurrentState(LEDs.CurrentState.SCORING_ALGAE_BARGE);
  }

  private void scoreAlgae() {
    rollers.setWantedState(Rollers.WantedState.SCORING_ALGAE);
  }

  private void stop() {
    elevator.setWantedState(Elevator.WantedState.STOPPED);
    rollers.setWantedState(Rollers.WantedState.STOPPED);
    wrist.setWantedState(Wrist.WantedState.STOPPED);
    leds.setCurrentState(LEDs.CurrentState.NO_STATE);
  }

  public Command setNextSuperStateCommand(WantedSuperState nextState) {
    return this.runOnce(() -> setNextSuperState(nextState));
  }

  public Command updateWantedSuperStateCommand() {
    return this.runOnce(() -> updateWantedSuperState());
  }

  public Command zeroGyroCommand() {
    return this.runOnce(() -> drivetrain.zeroGyro());
  }
}
