package frc.robot.subsystems.endefector.endefectorwrist;

import dev.doglog.DogLog;
import frc.robot.subsystems.elevator.ElevatorConstants;
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
    INTAKING_ALGAE_REEF_L2,
    INTAKING_ALGAE_REEF_L3,
    POSITION_PREPARED,
    POSITION_TROUGH,
    POSITION_BRANCH_L2,
    POSITION_BRANCH_L3,
    POSITION_BRANCH_L4,
    HOLDING_ALGAE,
    SCORING_ALGAE_BARGE,
    STOPPED,
  }

  public enum CurrentState {
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE_LOLLIPOP,
    INTAKING_ALGAE_GROUND,
    INTAKING_ALGAE_REEF_L2,
    INTAKING_ALGAE_REEF_L3,
    POSITION_PREPARED,
    POSITION_TROUGH,
    POSITION_BRANCH_L2,
    POSITION_BRANCH_L3,
    POSITION_BRANCH_L4,
    HOLDING_ALGAE,
    SCORING_ALGAE_BARGE,
    STOPPED,
  }

  public void updateInputs() {
    Tracer.traceFunc("UpdateIO", io::updateInputs);
    Tracer.traceFunc("HandleStateTransitions", this::handleStateTransitions);
    Tracer.traceFunc("ApplyStates", this::applyStates);
    DogLog.log("Wrist/CurrentState", currentState);
    DogLog.log("Wrist/WantedState", wantedState);
    SubsystemUtil.elevatorStateToHeightTicks(ElevatorStates.INTAKING_ALGAE_LOLLIPOP);

    io.isCoralInEndefector = subsystemChecker.isCoralInEndefector();
  }

  private void handleStateTransitions() {
    previousState = currentState;
    switch (wantedState) {
      case INTAKING_CORAL_STATION:
        if (subsystemChecker.isAtElevatorHeight(
            ElevatorConstants.ElevatorStates.INTAKING_CORAL_STATION)) {
          currentState = CurrentState.INTAKING_CORAL_STATION;
        } else {
          currentState = CurrentState.POSITION_PREPARED;
        }
        break;
      case INTAKING_ALGAE_GROUND:
        currentState = CurrentState.INTAKING_ALGAE_GROUND;
        break;
      case INTAKING_ALGAE_LOLLIPOP:
        if (subsystemChecker.isAtElevatorHeight(
            ElevatorConstants.ElevatorStates.INTAKING_ALGAE_LOLLIPOP)) {
          currentState = CurrentState.INTAKING_ALGAE_LOLLIPOP;
        } else {
          currentState = CurrentState.POSITION_PREPARED;
        }
        break;
      case INTAKING_ALGAE_REEF_L2:
        if (subsystemChecker.isAtElevatorHeight(
            ElevatorConstants.ElevatorStates.POSITION_ALGAE_L2)) {
          currentState = CurrentState.INTAKING_ALGAE_REEF_L2;
        } else {
          currentState = CurrentState.POSITION_PREPARED;
        }
        break;
      case INTAKING_ALGAE_REEF_L3:
        if (subsystemChecker.isAtElevatorHeight(
            ElevatorConstants.ElevatorStates.POSITION_ALGAE_L3)) {
          currentState = CurrentState.INTAKING_ALGAE_REEF_L3;
        } else {
          currentState = CurrentState.POSITION_PREPARED;
        }
        break;
      case POSITION_PREPARED:
        currentState = CurrentState.POSITION_PREPARED;
        break;
      case POSITION_TROUGH:
        if (subsystemChecker.isSafeDistanceFromReef(true)) {
          currentState = CurrentState.POSITION_TROUGH;
        } else {
          currentState = CurrentState.POSITION_PREPARED;
        }
        // currentState = CurrentState.POSITION_TROUGH;
        break;
      case POSITION_BRANCH_L2:
        if (subsystemChecker.isSafeDistanceFromReef(false)) {
          currentState = CurrentState.POSITION_BRANCH_L2;
        } else {
          currentState = CurrentState.POSITION_PREPARED;
        }
        // currentState = CurrentState.POSITION_BRANCH_L2;
        break;
      case POSITION_BRANCH_L3:
        if (subsystemChecker.isSafeDistanceFromReef(false)) {
          currentState = CurrentState.POSITION_BRANCH_L3;
        } else {
          currentState = CurrentState.POSITION_PREPARED;
        }
        // currentState = CurrentState.POSITION_BRANCH_L3;
        break;
      case POSITION_BRANCH_L4:
        if (subsystemChecker.isAtElevatorHeight(
            ElevatorConstants.ElevatorStates.POSITION_CORAL_L4)) {
          currentState = CurrentState.POSITION_BRANCH_L4;
        } else {
          currentState = CurrentState.POSITION_PREPARED;
        }
        break;
      case SCORING_ALGAE_BARGE:
        if (subsystemChecker.isAtElevatorHeight(
            ElevatorConstants.ElevatorStates.POSITION_ALGAE_BARGE)) {
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
      case INTAKING_ALGAE_REEF_L2:
        setAngle(WristStates.INTAKING_ALGAE_REEF);
        break;
      case INTAKING_ALGAE_REEF_L3:
        setAngle(WristStates.INTAKING_ALGAE_REEF);
        break;
      case POSITION_PREPARED:
        setAngle(WristStates.POSITION_PREPARED);
        break;
      case POSITION_TROUGH:
        setAngle(WristStates.POSITION_TROUGH);
        break;
      case POSITION_BRANCH_L2:
        setAngle(WristStates.POSITION_BRANCH_L2);
        break;
      case POSITION_BRANCH_L3:
        setAngle(WristStates.POSITION_BRANCH_L3);
        break;
      case POSITION_BRANCH_L4:
        setAngle(WristStates.POSITION_BRANCH_L4);
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

  public void setAngle(WristStates state) {
    io.setAngle(state);
  }

  public double getPosition() {
    return io.currentPosition;
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

  public boolean isAtSetpoint(WristStates state) {
    return isAtAngle(WristConstants.setpoints[state.getIndex()]);
  }

  public boolean isAtAngle(double angle) {
    return Math.abs(io.currentPosition - angle) < WristConstants.wristPositionTolerance;
  }

  public void stop() {
    io.stop();
  }

  public WristIO getIO() {
    return io;
  }
}
