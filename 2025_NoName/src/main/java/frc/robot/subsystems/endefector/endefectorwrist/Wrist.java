package frc.robot.subsystems.endefector.endefectorwrist;

import dev.doglog.DogLog;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants.WristStates;
import frc.robot.util.SubsystemChecker;
import frc.robot.util.SubsystemUtil;
import frc.robot.util.Tracer;

public class Wrist {
  private final WristIO io;
  private final SubsystemChecker subsystemChecker;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;
  private CurrentState previousState = CurrentState.STOPPED;

  public Wrist(WristIO io, SubsystemChecker subsystemChecker) {
    this.io = io;
    this.subsystemChecker = subsystemChecker;
  }

  public enum WantedState {
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE_LOLLIPOP,
    INTAKING_ALGAE_GROUND,
    INTAKING_ALGAE_REEF,
    POSITION_PREPARED,
    POSITION_TROUGH,
    HOLDING_ALGAE,
    SCORING_CORAL,
    SCORING_ALGAE_BARGE,
    STOPPED,
  }

  public enum CurrentState {
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE_LOLLIPOP,
    INTAKING_ALGAE_GROUND,
    INTAKING_ALGAE_REEF,
    POSITION_PREPARED,
    POSITION_TROUGH,
    HOLDING_ALGAE,
    SCORING_CORAL,
    SCORING_ALGAE_BARGE,
    STOPPED,
  }

  public void updateInputs() {
    Tracer.traceFunc("UpdateIO", io::updateInputs);
    subsystemChecker.setCurrentWristDegrees(io.currentPositionDegrees);
    subsystemChecker.updateIsAtSetpointWrist(isAtSetpoint());
    Tracer.traceFunc("HandleStateTransitions", this::handleStateTransitions);
    Tracer.traceFunc("ApplyStates", this::applyStates);
    DogLog.log("Wrist/CurrentState", currentState);
    DogLog.log("Wrist/WantedState", wantedState);
    SubsystemUtil.elevatorStateToHeightTicks(ElevatorStates.INTAKING_ALGAE_LOLLIPOP);
  }

  private void handleStateTransitions() {
    previousState = currentState;
    switch (wantedState) {
      case INTAKING_CORAL_STATION:
        if (subsystemChecker.isSafeWrist()) {
          currentState = CurrentState.INTAKING_CORAL_STATION;
        } else {
          currentState = CurrentState.POSITION_PREPARED;
        }
        break;
      case INTAKING_ALGAE_GROUND:
        currentState = CurrentState.INTAKING_ALGAE_GROUND;
        break;
      case INTAKING_ALGAE_LOLLIPOP:
        if (subsystemChecker.isAtSetpointElevator()) {
          currentState = CurrentState.INTAKING_ALGAE_LOLLIPOP;
        } else {
          currentState = CurrentState.POSITION_PREPARED;
        }
        break;
      case INTAKING_ALGAE_REEF:
        if (subsystemChecker.isAtSetpointElevator()) {
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
      case SCORING_ALGAE_BARGE:
        if (subsystemChecker.isAtSetpointElevator()) {
          currentState = CurrentState.SCORING_ALGAE_BARGE;
        } else {
          currentState = CurrentState.POSITION_PREPARED;
        }
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
          setAngle(WristStates.INTAKING_CORAL_STATION);
          break;
        case INTAKING_ALGAE_GROUND:
          setAngle(WristStates.INTAKING_ALGAE_GROUND);
          break;
        case INTAKING_ALGAE_LOLLIPOP:
          setAngle(WristStates.INTAKING_ALGAE_LOLLIPOP);
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
        case SCORING_ALGAE_BARGE:
          setAngle(WristStates.SCORING_ALGAE_BARGE);
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

  public void setAngle(WristStates state) {
    io.setAngle(state);
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

  public boolean isAtAngle(double angle) {
    return Math.abs(io.currentPositionDegrees - angle) < WristConstants.wristPositionTolerance;
  }

  public void stop() {
    io.stop();
  }

  public WristIO getIO() {
    return io;
  }
}
