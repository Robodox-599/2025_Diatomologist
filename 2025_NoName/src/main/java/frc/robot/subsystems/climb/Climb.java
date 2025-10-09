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
    PREPARE_CLIMB,
    CLIMBING_UP,
    CLIMBING_DOWN,
    STOWED,
    STOPPED;
  }

  public enum CurrentState {
    RELEASE_FLAP,
    RELEASE_RAMP,
    DEPLOY_CLIMB,
    INTAKE_CAGE,
    READY_TO_CLIMB,
    CLIMBING_UP,
    CLIMBING_DOWN,
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
        if (!io.isFlapsReleased) {
          currentState = CurrentState.RELEASE_FLAP;
        } else if (!io.isClimbDeployed && io.isFlapsReleased) {
          currentState = CurrentState.DEPLOY_CLIMB;
        } else if (!io.isRampReleased && io.isClimbDeployed && io.isFlapsReleased) {
          currentState = CurrentState.RELEASE_RAMP;
        } else if (io.isRampReleased
            && io.isClimbDeployed
            && io.isFlapsReleased
            && !io.isCageDetected) {
          currentState = CurrentState.INTAKE_CAGE;
        } else if (isClimbReady()) {
          currentState = CurrentState.READY_TO_CLIMB;
        } else {
          currentState = CurrentState.STOPPED;
        }
        break;
      case CLIMBING_UP:
        currentState = CurrentState.CLIMBING_UP;
        break;
      case CLIMBING_DOWN:
        currentState = CurrentState.CLIMBING_DOWN;
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
        setClimbVoltage(ClimbStates.STOPPED);
        setRollersVelocity(0);
        releaseFlapServos();
        break;
      case RELEASE_RAMP:
        setClimbVoltage(ClimbStates.STOPPED);
        setRollersVelocity(0);
        releaseRampServos();
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
      case CLIMBING_UP:
        setClimbVoltage(ClimbStates.CLIMBING_UP);
        holdCage();
        break;
      case CLIMBING_DOWN:
        setClimbVoltage(ClimbStates.CLIMBING_DOWN);
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

  public void releaseFlapServos() {
    io.releaseFlapServos();
  }

  public void releaseRampServos() {
    io.releaseRampServos();
  }

  public boolean isClimbReady() {
    return io.isClimbDeployed && io.isRampReleased && io.isFlapsReleased && io.isCageDetected;
  }

  public void setClimbVoltage(ClimbStates state) {
    io.setClimbVoltage(SubsystemUtil.climbStateToVoltage(state));
  }

  public void setRollersVelocity(double velocity) {
    io.setRollersVelocity(velocity);
  }

  public void holdCage() {
    io.stallRollers();
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
