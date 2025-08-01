package frc.robot.subsystems.elevator;

import dev.doglog.DogLog;
import frc.robot.SafetyChecker;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;

public class Elevator {
  private final ElevatorIO io;
  private final SafetyChecker safetyChecker;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;
  private CurrentState previousState = CurrentState.STOPPED;

  public enum WantedState {
    INTAKING_CORAL_STATION,
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
    STOPPED,
  }

  public enum CurrentState {
    INTAKING_CORAL_STATION,
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
    STOPPED,
  }

  public Elevator(ElevatorIO io, SafetyChecker safetyChecker) {
    this.io = io;
    this.safetyChecker = safetyChecker;
  }

  public void updateInputs() {
    io.updateInputs();
    safetyChecker.setCurrentElevatorInches(io.positionInches);
    safetyChecker.updateIsAtSetpointElevator(isAtSetpoint());
    currentState = handleStateTransitions();
    applyStates();
    DogLog.log("Elevator/CurrentState", currentState);
    DogLog.log("Elevator/WantedState", wantedState);
  }

  private CurrentState handleStateTransitions() {
    previousState = currentState;
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
        case POSITION_PREPARED:
          currentState = CurrentState.POSITION_PREPARED;
          break;
        case POSITION_CORAL_L1:
          currentState = CurrentState.POSITION_CORAL_L1;
          break;
        case POSITION_CORAL_L2:
          currentState = CurrentState.POSITION_CORAL_L2;
          break;
        case POSITION_CORAL_L3:
          currentState = CurrentState.POSITION_CORAL_L3;
          break;
        case POSITION_CORAL_L4:
          currentState = CurrentState.POSITION_CORAL_L4;
          break;
        case POSITION_ALGAE_PROCESSOR:
          currentState = CurrentState.POSITION_ALGAE_PROCESSOR;
          break;
        case POSITION_ALGAE_BARGE:
          currentState = CurrentState.POSITION_ALGAE_BARGE;
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
    if (currentState != previousState) {
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
        case POSITION_PREPARED:
          setHeight(ElevatorStates.POSITION_PREPARED);
          break;
        case POSITION_CORAL_L1:
          setHeight(ElevatorStates.POSITION_CORAL_L1);
          break;
        case POSITION_CORAL_L2:
          setHeight(ElevatorStates.POSITION_CORAL_L2);
          break;
        case POSITION_CORAL_L3:
          setHeight(ElevatorStates.POSITION_CORAL_L3);
          break;
        case POSITION_CORAL_L4:
          setHeight(ElevatorStates.POSITION_CORAL_L4);
          break;
        case POSITION_ALGAE_PROCESSOR:
          setHeight(ElevatorStates.POSITION_ALGAE_PROCESSOR);
          break;
        case POSITION_ALGAE_BARGE:
          setHeight(ElevatorStates.POSITION_ALGAE_BARGE);
          break;
        case STOPPED:
          stop();
          break;
        default:
          stop();
          break;
      }
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
