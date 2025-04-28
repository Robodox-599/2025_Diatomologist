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

  public Command setState(LEDsConstants.LEDStates state) {
    return runOnce(() -> io.setState(state));
  }

  private void disableAction() {
    if (DriverStation.isDisabled()) {
      io.setState(LEDsConstants.LEDStates.IDLE);
    }
  }
}
