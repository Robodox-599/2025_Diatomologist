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
    ENSURING_CORAL,
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
    ENSURING_CORAL,
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
        currentState = CurrentState.INTAKING_CORAL_STATION;
        break;
      case ENSURING_CORAL:
        currentState = CurrentState.ENSURING_CORAL;
        break;
      case INTAKING_ALGAE:
        currentState = CurrentState.INTAKING_ALGAE;
        break;
      case HOLD_CORAL:
        currentState = CurrentState.HOLD_CORAL;
        break;
      case HOLD_ALGAE:
        if (isAlgaeDetected()) {
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
        case ENSURING_CORAL:
          setVelocity(EndefectorRollerStates.ENSURING_CORAL);
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

  public boolean isCoralDetected() {
    return io.isCoralDetected;
  }

  public boolean isCoralEnsured() {
    return io.isCoralEnsured;
  }

  public boolean isAlgaeDetected() {
    return io.isAlgaeDetected;
  }
}
