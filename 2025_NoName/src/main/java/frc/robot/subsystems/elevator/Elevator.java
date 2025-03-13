package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SafetyChecker;
import frc.robot.util.SubsystemUtil;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final SafetyChecker safetyChecker;

  public Elevator(ElevatorIO io, SafetyChecker safetyChecker) {
    this.io = io;
    this.safetyChecker = safetyChecker;
  }

  @Override
  public void periodic() {
    io.updateInputs();
    safetyChecker.setCurrentElevatorInches(io.getPositionInches());
  }

  public boolean isAtTargetPosition(ElevatorConstants.ElevatorStates state) {
    return (Math.abs(io.getPositionInches() - SubsystemUtil.elevatorStateToHeight(state))
        < ElevatorConstants.positionToleranceInches);
  }

  /* Moves the elevator to one of the states */
  public Command moveToState(ElevatorConstants.ElevatorStates state) {

    // return Commands.repeatingSequence(
    //         this.runOnce(
    //                 () -> {
    //                   io.setState(state);
    //                 })
    //             .until(this::isAtTargetPosition)
    //             .onlyIf(
    //                 () ->
    // safetyChecker.isSafeElevator(SubsystemUtil.elevatorStateToHeight(state))))
    //     .until(this::isAtTargetPosition);

    return this.run(
            () -> {
              io.setState(state);
            })
        .until(() -> isAtTargetPosition(state));
    // .onlyIf(
    //     () -> safetyChecker.isSafeElevator(SubsystemUtil.elevatorStateToHeight(state))));
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

  /*Homes elevator with limit switch, could be rewritten to home with current but hopefully nah*/

  public Command homeElevator() {
    return Commands.sequence(
            move(2),
            new WaitUntilCommand(() -> io.limitSwitchValue),
            move(0),
            new InstantCommand(() -> io.zeroEncoder()))
        .onlyIf(() -> !io.limitSwitchValue);
  }

  public ElevatorIO getIO() {
    return io;
  }
}
