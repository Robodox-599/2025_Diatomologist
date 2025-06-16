package frc.robot.subsystems.endefector.endefectorrollers;

import dev.doglog.DogLog;
import frc.robot.SafetyChecker;
import frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.EndefectorRollerStates;

public class Rollers {
  private final RollersIO io;
  private final SafetyChecker safetyChecker;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;

  public Rollers(RollersIO io, SafetyChecker safetyChecker) {
    this.io = io;
    this.safetyChecker = safetyChecker;
  }

  public enum WantedState {
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE,
    HOLD_CORAL,
    HOLD_ALGAE,
    SCORING_CORAL,
    SCORING_ALGAE,
    STOPPED,
  }

  public enum CurrentState {
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE,
    HOLD_CORAL,
    HOLD_ALGAE,
    SCORING_CORAL,
    SCORING_ALGAE,
    STOPPED,
  }

  public void updateInputs() {
    io.updateInputs();
    currentState = handleStateTransitions();
    applyStates();
    DogLog.log("Rollers/CurrentState", currentState);
    DogLog.log("Rollers/WantedState", wantedState);
  }

  private CurrentState handleStateTransitions() {
    switch (wantedState) {
      case INTAKING_CORAL_STATION:
        if (isCoralDetected()) {
          currentState = CurrentState.HOLD_CORAL;
          wantedState = WantedState.HOLD_CORAL;
        } else {
          currentState = CurrentState.INTAKING_CORAL_STATION;
        }
        break;
      case INTAKING_ALGAE:
        if (isAlgaeDetected()) {
          currentState = CurrentState.HOLD_ALGAE;
          wantedState = WantedState.HOLD_ALGAE;
        } else {
          currentState = CurrentState.INTAKING_ALGAE;
        }
        break;
      case HOLD_CORAL:
        currentState = CurrentState.HOLD_CORAL;
        break;
      case HOLD_ALGAE:
        currentState = CurrentState.HOLD_ALGAE;
        break;
      case SCORING_CORAL:
        if (safetyChecker.isAtSetpoints()) {
          if (!isCoralDetected()) {
            currentState = CurrentState.STOPPED;
            wantedState = WantedState.STOPPED;
          }
          currentState = CurrentState.SCORING_CORAL;
        }
        break;
      case SCORING_ALGAE:
        if (safetyChecker.isAtSetpoints()) {
          if (!isAlgaeDetected()) {
            currentState = CurrentState.STOPPED;
            wantedState = WantedState.STOPPED;
          }
          currentState = CurrentState.SCORING_ALGAE;
        }
        break;
      case STOPPED:
        currentState = CurrentState.STOPPED;
        break;
      default:
        currentState = CurrentState.STOPPED;
        break;
    }
    return currentState;
  }

  private void applyStates() {
    switch (currentState) {
      case INTAKING_CORAL_STATION:
        setVelocity(EndefectorRollerStates.INTAKING_CORAL_STATION);
        break;
      case INTAKING_ALGAE:
        setVelocity(EndefectorRollerStates.INTAKING_ALGAE);
        break;
      case HOLD_CORAL:
        stop();
        break;
      case HOLD_ALGAE:
        holdAlgae();
        break;
      case SCORING_CORAL:
        setVelocity(EndefectorRollerStates.SCORING_CORAL);
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

  public void setVelocity(EndefectorRollerStates state) {
    io.setVelocity(state);
  }

  public void holdAlgae() {
    io.holdAlgae();
  }

  public void stop() {
    io.stop();
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public boolean isCoralDetected() {
    return io.isCoralDetected;
  }

  public boolean isAlgaeDetected() {
    return io.isAlgaeDetected;
  }
}
