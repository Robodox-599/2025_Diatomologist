package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private final LEDsIO io;

  public LEDs(
      LEDsIO
          io) { // TODO: post integration, add other subsystems here so we can switch from running a
    // command to using the periodic to grab subsystem states and update LEDs that way.
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs();
    disableAction();
  }

  public Command runLEDStationIntake() {
    return runOnce(() -> io.enableStationIntake());
  }

  public Command runLEDAlgaeIntake() {
    return runOnce(() -> io.enableAlgaeIntake());
  }

  public Command runLEDNoState() {
    return runOnce(() -> io.enableNoState());
  }

  public Command runLEDScored() {
    return runOnce(() -> io.enableScored());
  }

  public Command runLEDScoring() {
    return runOnce(() -> io.enableScoring());
  }

  public Command runLEDIntaked() {
    return runOnce(() -> io.enableIntaked());
  }

  public Command runLEDAutoAlign() {
    return runOnce(() -> io.enableAutoAlign());
  }

  public Command runLEDPrepared() {
    return runOnce(() -> io.enablePrepared());
  }

  public Command runLEDReadyToScore() {
    return runOnce(() -> io.enableReadyToScore());
  }

  public Command runLEDOverride() {
    return runOnce(() -> io.enableOverride());
  }

  private void disableAction() {
    if (DriverStation.isDisabled()) {
      io.enableNoState();
    }
  }
}
