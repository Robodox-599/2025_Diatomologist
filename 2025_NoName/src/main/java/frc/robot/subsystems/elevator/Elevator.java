package frc.robot.subsystems.elevator;

import dev.doglog.DogLog;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants.WristStates;
import frc.robot.util.SubsystemChecker;
import frc.robot.util.SubsystemUtil;
import frc.robot.util.Tracer;

public class Elevator {
  private final ElevatorIO io;
  private final SubsystemChecker subsystemChecker;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;
  private CurrentState previousState = CurrentState.STOPPED;

  public enum WantedState {
    POSITION_CORAL_STATION,
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
    POSITION_CORAL_L4_AUTO,
    POSITION_ALGAE_PROCESSOR,
    POSITION_ALGAE_BARGE,
    STOPPED,
  }

  public enum CurrentState {
    POSITION_CORAL_STATION,
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
    POSITION_CORAL_L4_AUTO,
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
    Tracer.traceFunc("HandleStateTransitions", this::handleStateTransitions);
    Tracer.traceFunc("ApplyStates", this::applyStates);
    Tracer.traceFunc("CalculateSoftLimits", this::calculateSoftLimits);
    DogLog.log("Elevator/CurrentState", currentState);
    DogLog.log("Elevator/WantedState", wantedState);
  }

  private void handleStateTransitions() {
    previousState = currentState;
    if (currentState == CurrentState.POSITION_CORAL_L4
        && wantedState != WantedState.POSITION_CORAL_L4) {
      if (!subsystemChecker.isAtWristPosition(WristStates.POSITION_PREPARED)) {
        currentState = CurrentState.POSITION_CORAL_L4;
        return;
      }
    }
    switch (wantedState) {
      case POSITION_CORAL_STATION:
        currentState = CurrentState.POSITION_CORAL_STATION;
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
        if (subsystemChecker.isAtWristPosition(WristStates.POSITION_PREPARED)) {
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
      case POSITION_CORAL_L4_AUTO:
        currentState = CurrentState.POSITION_CORAL_L4_AUTO;
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
  }

  private void applyStates() {
    switch (currentState) {
      case POSITION_CORAL_STATION:
        setHeight(ElevatorStates.POSITION_CORAL_STATION);
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
      case POSITION_PREPARED_AUTO:
        setHeight(ElevatorStates.POSITION_CORAL_L4_AUTO);
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

  public void calculateSoftLimits() {
    io.elevatorSoftLowerLimit = subsystemChecker.calculateElevatorSoftLowerLimit();
    io.elevatorSoftUpperLimit = subsystemChecker.calculateElevatorSoftUpperLimit();
  }

  /* Moves the elevator to one of the states */
  public void setHeight(ElevatorStates state) {
    io.setHeight(state);
  }

  public double getHeightInches() {
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

  public boolean isAtSetpoint(ElevatorStates state) {
    return isAtHeight(SubsystemUtil.elevatorStateToHeightInches(state));
  }

  public boolean isAtHeight(double height) {
    return Math.abs(io.positionInches - height) < ElevatorConstants.positionToleranceInches;
  }

  public ElevatorIO getIO() {
    return io;
  }
}
