package frc.robot.subsystems.climb;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    CLIMB_PREPARED,
    CLIMBING,
    STOWED,
    STOPPED;
  }

  public enum CurrentState {
    RELEASE_FLAP,
    RELEASE_RAMP,
    DEPLOY_CLIMB,
    CLIMB_PREPARED,
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
      case CLIMB_PREPARED:
        if (!io.isFlapsReleased) {
          currentState = CurrentState.RELEASE_FLAP;
        } else if (!io.isClimbDeployed && io.isFlapsReleased) {
          currentState = CurrentState.DEPLOY_CLIMB;
        } else if (!io.isRampReleased && io.isClimbDeployed && io.isFlapsReleased) {
          currentState = CurrentState.RELEASE_RAMP;
        } else if (isClimbPrepared()) {
          wantedState = WantedState.CLIMB_PREPARED;
          currentState = CurrentState.CLIMB_PREPARED;
        }
        break;
      case CLIMBING:
        currentState = CurrentState.CLIMBING;
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
      case RELEASE_FLAP:
        releaseFlapServos();
        break;
      case RELEASE_RAMP:
        releaseRampServos();
        break;
      case DEPLOY_CLIMB:
        setClimbVoltage(ClimbStates.DEPLOYING_CLIMB);
        setRollersVelocity(0);
        break;
      case CLIMB_PREPARED:
        setClimbVoltage(ClimbStates.STOPPED);
        setRollersVelocity(6);
        releaseRampServo();
        break;
      case CLIMBING:
        setClimbVoltage(ClimbStates.CLIMBING);
        setRollersVelocity(0);
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

  public void releaseFlapServos() {
    io.releaseFlapServos();
  }

  public void releaseRampServos() {
    io.releaseRampServos();
  }

  public boolean isClimbPrepared() {
    return io.isClimbDeployed && io.isRampReleased && io.isFlapsReleased;
  }

  public void setClimbVoltage(ClimbStates state) {
    io.setClimbVoltage(SubsystemUtil.climbStateToVoltage(state));
  }

  public void setRollersVelocity(double velocity) {
    io.setRollersVelocity(velocity);
  }

  public void releaseRampServo() {
    io.releaseRampServos();
  }

  public void zeroEncoder() {
    io.zeroEncoder();
  }

  public boolean isAtTargetPosition() {
    return io.atSetpoint;
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.setClimbVoltage(0);
        });
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
