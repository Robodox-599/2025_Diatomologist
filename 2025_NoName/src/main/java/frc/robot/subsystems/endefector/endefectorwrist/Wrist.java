package frc.robot.subsystems.endefector.endefectorwrist;

import dev.doglog.DogLog;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants.WristStates;
import frc.robot.util.SafetyChecker;
import frc.robot.util.Tracer;

public class Wrist {
  private final WristIO io;
  private final SafetyChecker safetyChecker;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;
  private CurrentState previousState = CurrentState.STOPPED;

  public Wrist(WristIO io, SafetyChecker safetyChecker) {
    this.io = io;
    this.safetyChecker = safetyChecker;
  }

  public enum WantedState {
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE_GROUND,
    INTAKING_ALGAE_REEF,
    POSITION_PREPARED,
    POSITION_TROUGH,
    HOLDING_ALGAE,
    SCORING_CORAL,
    SCORING_ALGAE,
    STOPPED,
  }

  public enum CurrentState {
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE_GROUND,
    INTAKING_ALGAE_REEF,
    POSITION_PREPARED,
    POSITION_TROUGH,
    HOLDING_ALGAE,
    SCORING_CORAL,
    SCORING_ALGAE,
    STOPPED,
  }

  public void updateInputs() {
    Tracer.traceFunc("UpdateIO", io::updateInputs);
    safetyChecker.setCurrentWristDegrees(io.currentPositionDegrees);
    safetyChecker.updateIsAtSetpointWrist(isAtSetpoint());
    Tracer.traceFunc("HandleStateTransitions", this::handleStateTransitions);
    Tracer.traceFunc("ApplyStates", this::applyStates);
    DogLog.log("Wrist/CurrentState", currentState);
    DogLog.log("Wrist/WantedState", wantedState);
  }

  private void handleStateTransitions() {
    previousState = currentState;
    switch (wantedState) {
      case INTAKING_CORAL_STATION:
        if (safetyChecker.isSafeWrist()) {
          currentState = CurrentState.INTAKING_CORAL_STATION;
        }
        break;
      case INTAKING_ALGAE_GROUND:
        currentState = CurrentState.INTAKING_ALGAE_GROUND;
        break;
      case INTAKING_ALGAE_REEF:
        if (safetyChecker.isAtSetpointElevator()) {
          currentState = CurrentState.INTAKING_ALGAE_REEF;
        } else {
          currentState = CurrentState.POSITION_PREPARED;
        }
        break;
      case POSITION_PREPARED:
        currentState = CurrentState.POSITION_PREPARED;
        break;
      case POSITION_TROUGH:
        currentState = CurrentState.POSITION_TROUGH;
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
        case INTAKING_CORAL_STATION:
          setAngle(WristStates.INTAKING_CORAL_STATION);
          break;
        case INTAKING_ALGAE_GROUND:
          setAngle(WristStates.INTAKING_ALGAE_GROUND);
          break;
        case INTAKING_ALGAE_REEF:
          setAngle(WristStates.INTAKING_ALGAE_REEF);
          break;
        case POSITION_PREPARED:
          setAngle(WristStates.POSITION_PREPARED);
          break;
        case POSITION_TROUGH:
          setAngle(WristStates.POSITION_TROUGH);
          break;
        case STOPPED:
          stop();
          break;
        default:
          stop();
          break;
      
    }
  }

  public void setAngle(WristStates state) {
    io.setAngle(state);
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public boolean isAtSetpoint() {
    return io.atSetpoint;
  }

  public void stop() {
    io.stop();
  }

  public WristIO getIO() {
    return io;
  }
}
