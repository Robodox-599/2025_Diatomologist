package frc.robot.subsystems.endefector.endefectorrollers;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.EndefectorRollerStates;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants.WristStates;
import frc.robot.util.SubsystemChecker;
import frc.robot.util.Tracer;

public class Rollers {
  private final RollersIO io;
  private final SubsystemChecker subsystemChecker;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;
  private RampRollersWantedState rampRollersWantedState =
      RampRollersWantedState.INTAKING_CORAL_STATION_PASSIVE;
  private RampRollersCurrentState rampRollersCurrentState =
      RampRollersCurrentState.INTAKING_CORAL_STATION_PASSIVE;
  private CurrentState previousState = CurrentState.STOPPED;

  public Rollers(RollersIO io, SubsystemChecker subsystemChecker) {
    this.io = io;
    this.subsystemChecker = subsystemChecker;
  }

  public enum WantedState {
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE,
    HOLD_CORAL,
    HOLD_ALGAE,
    SCORING_CORAL_TROUGH,
    SCORING_CORAL_L2_L3,
    SCORING_CORAL_L4,
    SCORING_ALGAE,
    STOPPED,
  }

  public enum CurrentState {
    INTAKING_CORAL_STATION,
    INTAKING_ALGAE,
    HOLD_CORAL,
    HOLD_ALGAE,
    SCORING_CORAL_TROUGH,
    SCORING_CORAL_L2_L3,
    SCORING_CORAL_L4,
    SCORING_ALGAE,
    STOPPED,
  }

  public enum RampRollersWantedState {
    INTAKING_CORAL_STATION_PASSIVE,
    INTAKING_CORAL_STATION_ACTIVE,
  }

  public enum RampRollersCurrentState {
    INTAKING_CORAL_STATION_PASSIVE,
    HOLD_CORAL,
    INTAKING_CORAL_STATION_ACTIVE,
    STOPPED,
  }

  public void updateInputs() {
    Tracer.traceFunc("UpdateIO", io::updateInputs);
    Tracer.traceFunc("HandleStateTransitions", this::handleStateTransitions);
    Tracer.traceFunc("ApplyStates", this::applyStates);
    handleRampRollersStateTransitions();
    applyRampRollersStates();
    DogLog.log("Rollers/CurrentState", currentState);
    DogLog.log("Rollers/WantedState", wantedState);
    DogLog.log("Rollers/RampRollersWantedState", rampRollersWantedState);
    DogLog.log("Rollers/RampRollersCurrentState", rampRollersCurrentState);
    DogLog.log("Rollers/IsCoralEnsured", isCoralEnsured());

    if (DriverStation.isDisabled()
        && (io.endefectorRollersPosition > 50 || io.rampRollersPosition > 50)) {
      io.resetRollersPosition();
      io.setEndefectorHoldCoralPosition();
      io.setRampHoldCoralPosition();
    }
  }

  private void handleStateTransitions() {
    if (currentState == CurrentState.INTAKING_CORAL_STATION
        && wantedState != WantedState.INTAKING_CORAL_STATION) {
      rampRollersWantedState = RampRollersWantedState.INTAKING_CORAL_STATION_PASSIVE;
    }
    previousState = currentState;
    switch (wantedState) {
      case INTAKING_CORAL_STATION:
        if (isCoralIntakedInEndefector() && !isCoralInTransition()) {
          currentState = CurrentState.HOLD_CORAL;
        } else {
          currentState = CurrentState.INTAKING_CORAL_STATION;
        }
        break;
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
        if (isAlgaeScored()) {
          currentState = CurrentState.STOPPED;
        } else {
          currentState = CurrentState.HOLD_ALGAE;
        }
        break;
      case SCORING_CORAL_TROUGH:
        currentState = CurrentState.SCORING_CORAL_TROUGH;
        break;
      case SCORING_CORAL_L2_L3:
        currentState = CurrentState.SCORING_CORAL_L2_L3;
        break;
      case SCORING_CORAL_L4:
        currentState = CurrentState.SCORING_CORAL_L4;
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
    switch (currentState) {
      case INTAKING_CORAL_STATION:
        setEndefectorVelocity(EndefectorRollerStates.INTAKING_CORAL_STATION);
        rampRollersWantedState = RampRollersWantedState.INTAKING_CORAL_STATION_ACTIVE;
        break;
      case INTAKING_ALGAE:
        setEndefectorVelocity(EndefectorRollerStates.INTAKING_ALGAE);
        break;
      case HOLD_CORAL:
        io.endefectorHoldCoral();
        break;
      case HOLD_ALGAE:
        holdAlgae();
        break;
      case SCORING_CORAL_TROUGH:
        setEndefectorVelocity(EndefectorRollerStates.SCORING_CORAL_TROUGH);
        break;
      case SCORING_CORAL_L2_L3:
        setEndefectorVelocity(EndefectorRollerStates.SCORING_CORAL_L2_L3);
        break;
      case SCORING_CORAL_L4:
        setEndefectorVelocity(EndefectorRollerStates.SCORING_CORAL_L4);
        break;
      case SCORING_ALGAE:
        setEndefectorVelocity(EndefectorRollerStates.SCORING_ALGAE);
        break;
      case STOPPED:
        stop();
        break;
      default:
        stop();
        break;
    }
  }

  private void handleRampRollersStateTransitions() {
    switch (rampRollersWantedState) {
      default:
      case INTAKING_CORAL_STATION_PASSIVE:
        if (isCoralInTransition()) {
          rampRollersCurrentState = RampRollersCurrentState.HOLD_CORAL;
        } else if (isCoralIntakedInEndefector()) {
          rampRollersCurrentState = RampRollersCurrentState.STOPPED;
        }
        else {
          rampRollersCurrentState = RampRollersCurrentState.INTAKING_CORAL_STATION_PASSIVE;
        }
        break;
      case INTAKING_CORAL_STATION_ACTIVE:
        if (!isCoralInTransition() && isCoralIntakedInEndefector()) {
          rampRollersWantedState = RampRollersWantedState.INTAKING_CORAL_STATION_PASSIVE;
          rampRollersCurrentState = RampRollersCurrentState.INTAKING_CORAL_STATION_PASSIVE;
        } else if (subsystemChecker.isAtWristPosition(WristStates.POSITION_CORAL_STATION)
            && subsystemChecker.isAtElevatorHeight(ElevatorStates.POSITION_CORAL_STATION)) {
          rampRollersCurrentState = RampRollersCurrentState.INTAKING_CORAL_STATION_ACTIVE;
        } else {
          rampRollersCurrentState = RampRollersCurrentState.STOPPED;
        }
        break;
    }
  }

  private void applyRampRollersStates() {
    switch (rampRollersCurrentState) {
      default:
      case INTAKING_CORAL_STATION_PASSIVE:
        setRampVelocity(RollersConstants.rampRollersVelocitySetpoint * 0.5);
        break;
      case HOLD_CORAL:
        io.rampHoldCoral();
        break;
      case INTAKING_CORAL_STATION_ACTIVE:
        setRampVelocity(RollersConstants.rampRollersVelocitySetpoint);
        break;
      case STOPPED:
        io.setRampVelocity(0);
        break;
    }
  }

  public void setEndefectorVelocity(EndefectorRollerStates state) {
    io.setEndefectorVelocity(state);
  }

  public void setRampVelocity(double speed) {
    io.setRampVelocity(speed);
  }

  public void holdAlgae() {
    io.holdAlgae();
  }

  public void resetRollersPosition() {
    io.resetRollersPosition();
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

  public boolean isCoralInTransition() {
    return io.isCoralInTransition;
  }

  public boolean isCoralEnsured() {
    return (currentState == CurrentState.HOLD_CORAL)
        || (currentState == CurrentState.SCORING_CORAL_TROUGH && !isCoralTroughScored())
        || (currentState == CurrentState.SCORING_CORAL_L2_L3 && !isCoralBranchScored())
        || (currentState == CurrentState.SCORING_CORAL_L4 && !isCoralBranchScored());
  }

  public boolean isCoralTroughScored() {
    return io.isCoralTroughScored;
  }

  public boolean isCoralBranchScored() {
    return io.isCoralBranchScored;
  }

  public boolean isGroundAlgaeIntaked() {
    return io.isGroundAlgaeIntaked;
  }

  public boolean isReefAlgaeIntaked() {
    return io.isReefAlgaeIntaked;
  }

  public boolean isAlgaeScored() {
    return io.isAlgaeScored;
  }

  public void setCoralStateSim(boolean state) {
    io.setCoralStateSim(state);
    wantedState = WantedState.HOLD_CORAL;
  }
}
