package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SafetyChecker;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.util.SubsystemUtil;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final SafetyChecker safetyChecker;
  private ElevatorStates internalState;

  public Elevator(ElevatorIO io, SafetyChecker safetyChecker) {
    this.io = io;
    this.safetyChecker = safetyChecker;
    this.internalState = ElevatorStates.STOW;
  }

  @Override
  public void periodic() {
    io.updateInputs();
    safetyChecker.setCurrentElevatorInches(io.positionInches);
    if (safetyChecker.isSafeElevator(SubsystemUtil.elevatorStateToHeightInches(internalState))) {
      io.setState(internalState);
    }
  }

  public boolean isAtTargetPosition(ElevatorConstants.ElevatorStates state) {
    return (Math.abs(io.positionInches - SubsystemUtil.elevatorStateToHeightInches(state))
        < ElevatorConstants.positionToleranceInches);
  }

  /* Moves the elevator to one of the states */
  public Command moveToState(ElevatorConstants.ElevatorStates state) {
    return this.run(
            () -> {
              this.internalState = state;
            })
        .until(() -> isAtTargetPosition(state));
  }

  public ElevatorConstants.ElevatorStates getState() {
    return io.getState();
  }

  public Command move(double volt) {
    return this.runOnce(
        () -> {
          io.setVoltage(volt);
        });
  }

  public ElevatorIO getIO() {
    return io;
  }
}
