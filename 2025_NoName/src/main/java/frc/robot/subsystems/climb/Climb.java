package frc.robot.subsystems.climb;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimbConstants.ClimbStates;

public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;

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
        currentState = CurrentState.CLIMB_PREPARED;
        break;
      case CLIMBING:
        currentState = CurrentState.CLIMB_PREPARED;
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
      case CLIMB_PREPARED:
        setClimbPosition(ClimbStates.CLIMB_PREPARED);
        setRollersVelocity(0.5);
        break;
      case CLIMBING:
        setClimbPosition(ClimbStates.CLIMBING);
        io.stallRollers();
        break;
      case STOWED:
        setClimbPosition(ClimbStates.STOWED);
        setRollersVelocity(0);
        break;
      case STOPPED:
        stop();
        break;
    }
  }

  public void setClimbPosition(ClimbStates state) {
    io.setClimbPosition(state);
  }

  public void setRollersVelocity(double velocity) {
    io.setRollersVelocity(velocity);
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
