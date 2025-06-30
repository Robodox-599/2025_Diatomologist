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
    CLIMB_READY,
    CLIMB,
    STOPPED;
  }

  public enum CurrentState {
    CLIMB_READY,
    CLIMB,
    STOPPED;
  }

  public void updateInputs() {
    io.updateInputs();
    currentState = handleStateTransitions();
    applyStates();
    DogLog.log("Climb/CurrentState", currentState);
    DogLog.log("Climb/WantedState", wantedState);
  }

  private CurrentState handleStateTransitions() {
    switch (wantedState) {
      case CLIMB_READY:
        if (isCageDetected()) {
          currentState = CurrentState.CLIMB;
          wantedState = WantedState.CLIMB;
        } else {
          currentState = CurrentState.CLIMB_READY;
        }
        break;
      case CLIMB:
        if (isCageDetected()) {
          currentState = CurrentState.CLIMB;
          wantedState = WantedState.CLIMB;
        } else {
          currentState = CurrentState.CLIMB_READY;
        }
        break;
      case STOPPED:
      default:
        currentState = CurrentState.STOPPED;
        break;
    }
    return currentState;
  }

  private void applyStates() {
    switch (currentState) {
      case CLIMB_READY:
        setClimb(ClimbStates.CLIMB_READY);
        setRollers(0.5);
        break;
      case CLIMB:
        setClimb(ClimbStates.CLIMB);
        io.stallRollers();
        break;
      case STOPPED:
        stop();
        break;
    }
  }

  public void setClimb(ClimbStates state) {
    io.setClimb(state);
  }

  public void setRollers(double velocity) {
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
