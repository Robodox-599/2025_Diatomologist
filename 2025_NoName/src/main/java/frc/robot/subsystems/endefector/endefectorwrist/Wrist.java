package frc.robot.subsystems.endefector.endefectorwrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SafetyChecker;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final SafetyChecker safetyChecker;

  public Wrist(WristIO io, SafetyChecker safetyChecker) {
    this.io = io;
    this.safetyChecker = safetyChecker;
  }

  @Override
  public void periodic() {
    io.updateInputs();
    safetyChecker.setCurrentWristDegrees(io.currentPositionDegrees);
  }

  public Command moveToState(WristConstants.WristStates state) {
    // return Commands.run(
    //         () -> {
    //           io.setState(state);
    //         })
    //     .andThen(Commands.waitUntil(this::isAtTargetPosition))
    //     .onlyIf(() -> safetyChecker.isSafeWrist(EndefectorUtil.stateToSetpoint(state)));
    safetyChecker.setCurrentWristDegrees(io.currentPositionDegrees);
    return Commands.repeatingSequence(
            this.runOnce(
                    () -> {
                      io.setState(state);
                    })
                .onlyIf(() -> safetyChecker.isSafeWrist(getCurrentPosition())))
        .until(this::isAtTargetPosition);
  }

  public boolean isAtTargetPosition() {
    return io.atSetpoint;
  }

  public double getCurrentPosition() {
    return io.currentPositionDegrees;
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.setVoltage(0);
        });
  }

  public WristIO getIO() {
    return io;
  }
}
