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
  }

  public Command goToPose(double pose) {
    return Commands.run(
        () -> {
          io.goToPose(pose);
        });
  }

  public Command moveToState(WristConstants.WristStates state) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              io.setState(state);
            }),
        Commands.waitUntil(this::isAtTargetPosition));
  }

  public boolean isAtTargetPosition() {
    return io.atSetpoint;
  }

  public double getPose() {
    return io.getPose();
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
