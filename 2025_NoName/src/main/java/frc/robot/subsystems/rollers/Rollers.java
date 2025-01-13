package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// testing live share :O

public class Rollers extends SubsystemBase {
  private final RollersIO io;

  public Rollers(RollersIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs();
  }

  public Command runVelocity(double velocity) {
    return Commands.run(
        () -> {
          io.runVelocity(velocity);
        });
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.stop();
        });
  }
}
