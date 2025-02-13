package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SafetyChecker;
import frc.robot.util.ElevatorUtil;

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

  public boolean isAtTargetPosition() {
    return io.atSetpoint;
  }

  /* Moves the elevator to one of the states */
  public Command moveToState(ElevatorConstants.ElevatorStates state) {
    return Commands.sequence(
      Commands.either(
        Commands.run(
            () -> {
              io.setState(state);
            }),
        Commands.run(
              () -> {
                io.setState(state);
              }) 
              getCurrentPositionDegrees).andThen(
                Commands.waitUntil(this::isAtTargetPosition)));
  }

  public Command move(double volt) {
    return Commands.run(
        () -> {
          io.setVoltage(volt);
        });
  }

  /*Homes elevator with limit switch, could be rewritten to home with current but hopefully nah*/

  public Command homeElevator() {
    return Commands.sequence(
            new InstantCommand(() -> io.setVoltage(2)),
            new WaitUntilCommand(() -> io.limitSwitchValue),
            new InstantCommand(() -> io.setVoltage(0)),
            new InstantCommand(() -> io.zeroEncoder()))
        .onlyIf(() -> !io.limitSwitchValue);
  }

  public ElevatorIO getIO() {
    return io;
  }
}
