package frc.robot.subsystems.endefector.rollers;

import static frc.robot.subsystems.endefector.rollers.RollersConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
  private final RollersIO io;

  public Rollers(RollersIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs();
  }

  public Command runVoltage(double runVoltage) {
    return this.runOnce(
        () -> {
          io.setVoltage(runVoltage);
        });
  }
}
