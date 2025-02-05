package frc.robot.subsystems.endefector.endefectorrollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
  private final RollersIO io;

  public Rollers(RollersIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs();
  }

  public Command applyVoltage(double voltage) {
    return Commands.run(
        () -> {
          io.setVoltage(voltage);
        });
  }

  public Command setReverse(double voltage) {
    return Commands.run(
        () -> {
          io.setVoltage(voltage * -1);
        });
  }

  public Command moveToState(RollersConstants.EndefectorRollerStates state) {
    return Commands.runOnce(
            () -> {
              io.setState(state);
            })
        .withTimeout(0.6);
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.setVoltage(0);
        });
  }

  public Command setVelocity(double velocity) {
    return Commands.run(
        () -> {
          io.setVelocity(velocity);
        });
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  public RollersIO getIO() {
    return io;
  }
}
