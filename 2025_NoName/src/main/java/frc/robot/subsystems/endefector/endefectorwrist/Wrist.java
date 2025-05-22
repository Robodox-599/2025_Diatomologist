package frc.robot.subsystems.endefector.endefectorwrist;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SafetyChecker;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants.WristStates;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final SafetyChecker safetyChecker;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;

  public Wrist(WristIO io, SafetyChecker safetyChecker) {
    this.io = io;
    this.safetyChecker = safetyChecker;
  }

  public enum WantedState {
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE_GROUND,
    INTAKING_ALGAE_REEF,
    PREPARED,
    SCORING_CORAL,
    SCORING_ALGAE,
    STOPPED,
  }

  public enum CurrentState {
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE_GROUND,
    INTAKING_ALGAE_REEF,
    PREPARED,
    SCORING_CORAL,
    SCORING_ALGAE,
    STOPPED,
  }

  @Override
  public void periodic() {
    io.updateInputs();
    safetyChecker.setCurrentWristDegrees(io.currentPositionDegrees);
    safetyChecker.updateIsAtSetpointWrist(isAtSetpoint());
    currentState = handleStateTransitions();
    applyStates();
    DogLog.log("Wrist/CurrentState", currentState);
    DogLog.log("Wrist/WantedState", wantedState);
  }

  private CurrentState handleStateTransitions() {
    if (safetyChecker.isSafeWrist()) {
      switch (wantedState) {
        case INTAKING_CORAL_STATION:
          currentState = CurrentState.INTAKING_CORAL_STATION;
          break;
        case INTAKING_ALGAE_GROUND:
          currentState = CurrentState.INTAKING_ALGAE_GROUND;
          break;
        case INTAKING_ALGAE_REEF:
          currentState = CurrentState.INTAKING_ALGAE_REEF;
          break;
        case PREPARED:
          currentState = CurrentState.PREPARED;
          break;
        case SCORING_CORAL:
          currentState = CurrentState.SCORING_CORAL;
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
    } else {
      currentState = CurrentState.STOPPED;
    }
    return currentState;
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
      case PREPARED:
        setAngle(WristStates.PREPARE);
        break;
      case SCORING_CORAL:
        setAngle(WristStates.SCORING_CORAL);
        break;
      case SCORING_ALGAE:
        setAngle(WristStates.SCORING_ALGAE);
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
