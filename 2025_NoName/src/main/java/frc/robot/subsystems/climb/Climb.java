package frc.robot.subsystems.climb;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimbConstants.ClimbStates;
import frc.robot.util.SubsystemUtil;

public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private WantedState wantedState = WantedState.STOWED;
  private CurrentState currentState = CurrentState.STOWED;

  public Climb(ClimbIO io) {
    this.io = io;
  }

  public enum WantedState {
    PREPARE_CLIMB,
    CLIMBING,
    STOWED,
    STOPPED;
  }

  public enum CurrentState {
    RELEASE_RAMP,
    DEPLOY_CLIMB,
    INTAKE_CAGE,
    READY_TO_CLIMB,
    CLIMBING,
    STOWED,
    STOPPED;
  }

  public void updateInputs() {
    io.updateInputs();
    handleStateTransitions();
    applyStates();
    DogLog.log("Climb/CurrentState", currentState);
    DogLog.log("Climb/WantedState", wantedState);
  }

  private CurrentState handleStateTransitions() {
    switch (wantedState) {
      case PREPARE_CLIMB:
        if (!io.isRampReleased) {
          currentState = CurrentState.RELEASE_RAMP;
        } else if (io.isRampReleased && !isClimbDeployed()) {
          currentState = CurrentState.DEPLOY_CLIMB;
        } else if (io.isRampReleased && isClimbDeployed() && !io.isCageDetected) {
          currentState = CurrentState.INTAKE_CAGE;
        } else if (isClimbReady()) {
          currentState = CurrentState.READY_TO_CLIMB;
        } else {
          currentState = CurrentState.STOPPED;
        }
        break;
      case CLIMBING:
        if (isClimbed()) {
          currentState = CurrentState.STOPPED;
        } else {
          currentState = CurrentState.CLIMBING;
        }
        break;
      case STOWED:
        currentState = CurrentState.STOWED;
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
      case RELEASE_RAMP:
        setClimbVoltage(ClimbStates.STOPPED);
        setRollersVelocity(0);
        releaseRamp();
        break;
      case DEPLOY_CLIMB:
        setClimbVoltage(ClimbStates.DEPLOYING_CLIMB);
        setRollersVelocity(0);
        break;
      case INTAKE_CAGE:
        setClimbVoltage(ClimbStates.STOPPED);
        setRollersVelocity(1);
        break;
      case READY_TO_CLIMB:
        setClimbVoltage(ClimbStates.STOPPED);
        holdCage();
        break;
      case CLIMBING:
        setClimbVoltage(ClimbStates.CLIMBING);
        holdCage();
        break;
      case STOWED:
        setClimbVoltage(ClimbStates.STOPPED);
        setRollersVelocity(0);
        break;
      case STOPPED:
        stop();
        break;
    }
  }

  // public void releaseFlapServos() {
  //   io.releaseFlapServos();
  // }

  // public void releaseRampServos() {
  //   io.releaseRampServos();
  // }

  public void releaseRamp() {
    io.releaseRamp();
  }

  public boolean isClimbReady() {
    return isClimbDeployed() && io.isRampReleased && io.isCageDetected;
  }

  public boolean isClimbDeployed() {
    return io.climbPosition >= getClimbPosition(ClimbStates.DEPLOYING_CLIMB);
  }

  public boolean isClimbed() {
    return
    // io.isCageDetected &&
    io.isRampReleased && io.climbPosition <= getClimbPosition(ClimbStates.CLIMBING);
  }

  public void setClimbVoltage(ClimbStates state) {
    io.setClimbVoltage(SubsystemUtil.climbStateToVoltage(state));
  }

  public double getClimbPosition(ClimbStates state) {
    return SubsystemUtil.climbStateToPosition(state);
  }

  public void setRollersVelocity(double velocity) {
    io.setRollersVelocity(velocity);
  }

  public void holdCage() {
    io.stallRollers();
  }

  public void stop() {
    io.stop();
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public boolean isCageDetected() {
    return io.isCageDetected;
  }

  public ClimbIO getIO() {
    return io;
  }
}
