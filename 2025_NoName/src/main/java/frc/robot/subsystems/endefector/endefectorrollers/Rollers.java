package frc.robot.subsystems.endefector.endefectorrollers;

import dev.doglog.DogLog;
import frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.EndefectorRollerStates;
import frc.robot.util.SubsystemChecker;
import frc.robot.util.Tracer;

public class Rollers {
  private final RollersIO io;
  private final SubsystemChecker subsystemChecker;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;
  private CurrentState previousState = CurrentState.STOPPED;

  public Rollers(RollersIO io, SubsystemChecker subsystemChecker) {
    this.io = io;
    this.subsystemChecker = subsystemChecker;
  }

  public enum WantedState {
    INTAKING_CORAL_STATION,
    ENSURING_CORAL_FORWARDS,
    ENSURING_CORAL_BACKWARDS,
    INTAKING_ALGAE,
    HOLD_CORAL,
    HOLD_ALGAE,
    SCORING_CORAL_TROUGH,
    SCORING_CORAL_BRANCH,
    SCORING_ALGAE,
    STOPPED,
  }

  public enum CurrentState {
    INTAKING_CORAL_STATION,
    ENSURING_CORAL_FORWARDS,
    ENSURING_CORAL_BACKWARDS,
    INTAKING_ALGAE,
    HOLD_CORAL,
    HOLD_ALGAE,
    SCORING_CORAL_TROUGH,
    SCORING_CORAL_BRANCH,
    SCORING_ALGAE,
    STOPPED,
  }

  public void updateInputs() {
    Tracer.traceFunc("UpdateIO", io::updateInputs);
    Tracer.traceFunc("HandleStateTransitions", this::handleStateTransitions);
    Tracer.traceFunc("ApplyStates", this::applyStates);
    DogLog.log("Rollers/CurrentState", currentState);
    DogLog.log("Rollers/WantedState", wantedState);
  }

  private void handleStateTransitions() {
    previousState = currentState;
    switch (wantedState) {
      case INTAKING_CORAL_STATION:
        if (isCoralInRamp() && isCoralIntakedInEndefector()) {
          wantedState = WantedState.ENSURING_CORAL_FORWARDS;
          currentState = CurrentState.ENSURING_CORAL_FORWARDS;
        } else {
          currentState = CurrentState.INTAKING_CORAL_STATION;
        }
        break;
      case ENSURING_CORAL_FORWARDS:
        if (!isCoralInRamp() && isCoralIntakedInEndefector()) {
          wantedState = WantedState.ENSURING_CORAL_BACKWARDS;
          currentState = CurrentState.ENSURING_CORAL_BACKWARDS;
        } else if (isCoralInRamp() && isCoralIntakedInEndefector()) {
          currentState = CurrentState.ENSURING_CORAL_FORWARDS;
        } else {
          wantedState = WantedState.INTAKING_CORAL_STATION;
          currentState = CurrentState.INTAKING_CORAL_STATION;
        }
        break;
      case ENSURING_CORAL_BACKWARDS:
        if (isCoralInRamp() && isCoralIntakedInEndefector()) {
          wantedState = WantedState.HOLD_CORAL;
          currentState = CurrentState.HOLD_CORAL;
        } else if (!isCoralInRamp() && isCoralIntakedInEndefector()) {
          currentState = CurrentState.ENSURING_CORAL_BACKWARDS;
        } else {
          wantedState = WantedState.INTAKING_CORAL_STATION;
          currentState = CurrentState.INTAKING_CORAL_STATION;
        }
      case INTAKING_ALGAE:
        currentState = CurrentState.INTAKING_ALGAE;
        break;
      case HOLD_CORAL:
        if (!isCoralIntakedInEndefector()) {
          currentState = CurrentState.STOPPED;
        } else {
          currentState = CurrentState.HOLD_CORAL;
        }
        break;
      case HOLD_ALGAE:
        if (isAlgaeIntaked()) {
          currentState = CurrentState.HOLD_ALGAE;
        } else {
          currentState = CurrentState.STOPPED;
        }
        break;
      case SCORING_CORAL_TROUGH:
        currentState = CurrentState.SCORING_CORAL_TROUGH;
        break;
      case SCORING_CORAL_BRANCH:
        currentState = CurrentState.SCORING_CORAL_BRANCH;
        break;
      case SCORING_ALGAE:
        currentState = CurrentState.SCORING_ALGAE;
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
    if (currentState != previousState) {
      switch (currentState) {
        case INTAKING_CORAL_STATION:
          setVelocity(EndefectorRollerStates.INTAKING_CORAL_STATION);
          break;
        case ENSURING_CORAL_FORWARDS:
          setVelocity(EndefectorRollerStates.ENSURING_CORAL_FORWARDS);
          break;
        case ENSURING_CORAL_BACKWARDS:
          setVelocity(EndefectorRollerStates.ENSURING_CORAL_BACKWARDS);
          break;
        case INTAKING_ALGAE:
          setVelocity(EndefectorRollerStates.INTAKING_ALGAE);
          break;
        case HOLD_CORAL:
          stop();
          break;
        case HOLD_ALGAE:
          grabOrHoldAlgae();
          break;
        case SCORING_CORAL_TROUGH:
          setVelocity(EndefectorRollerStates.SCORING_CORAL_TROUGH);
          break;
        case SCORING_CORAL_BRANCH:
          setVelocity(EndefectorRollerStates.SCORING_CORAL_BRANCH);
          break;
        case SCORING_ALGAE:
          setVelocity(EndefectorRollerStates.SCORING_ALGAE);
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

  public void setVelocity(EndefectorRollerStates state) {
    io.setVelocity(state);
  }

  public double getVelocity() {
    return io.getVelocity();
  }

  public void grabOrHoldAlgae() {
    io.grabOrHoldAlgae();
  }

  public void stop() {
    io.stop();
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

  public boolean isCoralInRamp() {
    return io.isCoralInRamp;
  }

  public boolean isCoralIntakedInEndefector() {
    return io.isCoralIntakedInEndefector;
  }

  public boolean isCoralEnsured() {
    return (currentState == CurrentState.HOLD_CORAL);
  }

  public boolean isCoralTroughScored() {
    return io.isCoralTroughScored;
  }

  public boolean isCoralBranchScored() {
    return io.isCoralBranchScored;
  }

  public boolean isAlgaeIntaked() {
    return io.isAlgaeIntaked;
  }

  public boolean isAlgaeScored() {
    return io.isAlgaeScored;
  }

  public void setCoralStateSim(boolean state) {
    io.setCoralStateSim(state);
  }
}
