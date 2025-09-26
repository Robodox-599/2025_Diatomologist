package frc.robot.subsystems.elevator;

import dev.doglog.DogLog;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants.WristStates;
import frc.robot.util.SubsystemChecker;
import frc.robot.util.Tracer;

public class Elevator {
  private final ElevatorIO io;
  private final SubsystemChecker subsystemChecker;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;
  private CurrentState previousState = CurrentState.STOPPED;

  public enum WantedState {
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE_GROUND,
    INTAKING_ALGAE_LOLLIPOP,
    POSITION_ALGAE_L2,
    POSITION_ALGAE_L3,
    POSITION_PREPARED,
    POSITION_PREPARED_AUTO,
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
    INTAKING_ALGAE_LOLLIPOP,
    POSITION_ALGAE_L2,
    POSITION_ALGAE_L3,
    POSITION_PREPARED,
    POSITION_PREPARED_AUTO,
    POSITION_CORAL_L1,
    POSITION_CORAL_L2,
    POSITION_CORAL_L3,
    POSITION_CORAL_L4,
    POSITION_ALGAE_PROCESSOR,
    POSITION_ALGAE_BARGE,
    STOPPED,
  }

  public Elevator(ElevatorIO io, SubsystemChecker subsystemChecker) {
    this.io = io;
    this.subsystemChecker = subsystemChecker;
  }

  public void updateInputs() {
    Tracer.traceFunc("UpdateIO", io::updateInputs);
    subsystemChecker.setCurrentElevatorInches(io.positionInches);
    subsystemChecker.updateIsAtSetpointElevator(isAtSetpoint());
    Tracer.traceFunc("HandleStateTransitions", this::handleStateTransitions);
    Tracer.traceFunc("ApplyStates", this::applyStates);
    DogLog.log("Elevator/CurrentState", currentState);
    DogLog.log("Elevator/WantedState", wantedState);
  }

  private void handleStateTransitions() {
    previousState = currentState;
    if (subsystemChecker.isSafeElevator()) {
      switch (wantedState) {
        case INTAKING_CORAL_STATION:
          currentState = CurrentState.INTAKING_CORAL_STATION;
          break;
        case INTAKING_ALGAE_GROUND:
          currentState = CurrentState.INTAKING_ALGAE_GROUND;
          break;
        case INTAKING_ALGAE_LOLLIPOP:
          currentState = CurrentState.INTAKING_ALGAE_LOLLIPOP;
          break;
        case POSITION_ALGAE_L2:
          currentState = CurrentState.POSITION_ALGAE_L2;
          break;
        case POSITION_ALGAE_L3:
          currentState = CurrentState.POSITION_ALGAE_L3;
          break;
        case POSITION_PREPARED:
          currentState = CurrentState.POSITION_PREPARED;
          break;
        case POSITION_PREPARED_AUTO:
          if (subsystemChecker.isAtPositionWrist(WristStates.POSITION_PREPARED)) {
            currentState = CurrentState.POSITION_PREPARED_AUTO;
          } else {
            currentState = CurrentState.STOPPED;
          }
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
        case INTAKING_ALGAE_LOLLIPOP:
          setHeight(ElevatorStates.INTAKING_ALGAE_LOLLIPOP);
          break;
        case POSITION_ALGAE_L2:
          setHeight(ElevatorStates.POSITION_ALGAE_L2);
          break;
        case POSITION_ALGAE_L3:
          setHeight(ElevatorStates.POSITION_ALGAE_L3);
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

  public double getPositionInches() {
    return io.positionInches;
  }

  public void stop() {
    io.stop();
  }

  public void zeroEncoder() {
    io.zeroEncoder();
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public void getWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public void getCurrentState(CurrentState currentState) {
    this.currentState = currentState;
  }

  public boolean isAtSetpoint() {
    return io.atSetpoint;
  }

  public boolean isAtHeight(double height) {
    return Math.abs(io.positionInches - height) < ElevatorConstants.positionToleranceInches;
  }

  public ElevatorIO getIO() {
    return io;
  }
}
