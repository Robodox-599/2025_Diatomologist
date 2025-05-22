package frc.robot.subsystems.elevator;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SafetyChecker;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final SafetyChecker safetyChecker;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;

  public enum WantedState {
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE_GROUND,
    INTAKING_ALGAE_L2,
    INTAKING_ALGAE_L3,
    PREPARED,
    SCORING_CORAL_L1,
    SCORING_CORAL_L2,
    SCORING_CORAL_L3,
    SCORING_CORAL_L4,
    SCORING_ALGAE_PROCESSOR,
    SCORING_ALGAE_BARGE,
    STOPPED,
  }

  public enum CurrentState {
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE_GROUND,
    INTAKING_ALGAE_L2,
    INTAKING_ALGAE_L3,
    PREPARED,
    SCORING_CORAL_L1,
    SCORING_CORAL_L2,
    SCORING_CORAL_L3,
    SCORING_CORAL_L4,
    SCORING_ALGAE_PROCESSOR,
    SCORING_ALGAE_BARGE,
    STOPPED,
  }

  public Elevator(ElevatorIO io, SafetyChecker safetyChecker) {
    this.io = io;
    this.safetyChecker = safetyChecker;
  }

  @Override
  public void periodic() {
    io.updateInputs();
    safetyChecker.setCurrentElevatorInches(io.positionInches);
    safetyChecker.updateIsAtSetpointElevator(isAtSetpoint());
    currentState = handleStateTransitions();
    applyStates();
    DogLog.log("Elevator/CurrentState", currentState);
    DogLog.log("Elevator/WantedState", wantedState);
  }

  private CurrentState handleStateTransitions() {
    if (safetyChecker.isSafeElevator()) {
      switch (wantedState) {
        case INTAKING_CORAL_STATION:
          currentState = CurrentState.INTAKING_CORAL_STATION;
          break;
        case INTAKING_ALGAE_GROUND:
          currentState = CurrentState.INTAKING_ALGAE_GROUND;
          break;
        case INTAKING_ALGAE_L2:
          currentState = CurrentState.INTAKING_ALGAE_L2;
          break;
        case INTAKING_ALGAE_L3:
          currentState = CurrentState.INTAKING_ALGAE_L3;
          break;
        case PREPARED:
          currentState = CurrentState.PREPARED;
          break;
        case SCORING_CORAL_L1:
          currentState = CurrentState.SCORING_CORAL_L1;
          break;
        case SCORING_CORAL_L2:
          currentState = CurrentState.SCORING_CORAL_L2;
          break;
        case SCORING_CORAL_L3:
          currentState = CurrentState.SCORING_CORAL_L3;
          break;
        case SCORING_CORAL_L4:
          currentState = CurrentState.SCORING_CORAL_L4;
          break;
        case SCORING_ALGAE_PROCESSOR:
          currentState = CurrentState.SCORING_ALGAE_PROCESSOR;
          break;
        case SCORING_ALGAE_BARGE:
          currentState = CurrentState.SCORING_ALGAE_BARGE;
          break;
        case STOPPED:
          currentState = CurrentState.STOPPED;
          break;
        default:
          currentState = CurrentState.STOPPED;
          break;
      }
    } else {
      currentState = CurrentState.STOPPED;
    }
    return currentState;
  }

  private void applyStates() {
    switch (currentState) {
      case INTAKING_CORAL_STATION:
        setHeight(ElevatorStates.INTAKING_CORAL_STATION);
        break;
      case INTAKING_ALGAE_GROUND:
        setHeight(ElevatorStates.INTAKING_ALGAE_GROUND);
        break;
      case INTAKING_ALGAE_L2:
        setHeight(ElevatorStates.INTAKING_ALGAE_L2);
        break;
      case INTAKING_ALGAE_L3:
        setHeight(ElevatorStates.INTAKING_ALGAE_L3);
        break;
      case PREPARED:
        setHeight(ElevatorStates.PREPARED);
        break;
      case SCORING_CORAL_L1:
        setHeight(ElevatorStates.SCORING_CORAL_L1);
        break;
      case SCORING_CORAL_L2:
        setHeight(ElevatorStates.SCORING_CORAL_L2);
        break;
      case SCORING_CORAL_L3:
        setHeight(ElevatorStates.SCORING_CORAL_L3);
        break;
      case SCORING_CORAL_L4:
        setHeight(ElevatorStates.SCORING_CORAL_L4);
        break;
      case SCORING_ALGAE_PROCESSOR:
        setHeight(ElevatorStates.SCORING_ALGAE_PROCESSOR);
        break;
      case SCORING_ALGAE_BARGE:
        setHeight(ElevatorStates.SCORING_ALGAE_BARGE);
        break;
      case STOPPED:
        stop();
        break;
      default:
        stop();
        break;
    }
  }

  /* Moves the elevator to one of the states */
  public void setHeight(ElevatorStates state) {
    io.setHeight(state);
  }

  public void stop() {
    io.stop();
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public boolean isAtSetpoint() {
    return io.atSetpoint;
  }

  public ElevatorIO getIO() {
    return io;
  }
}
