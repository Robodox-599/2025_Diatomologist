package frc.robot.subsystems.leds;

import dev.doglog.DogLog;
import frc.robot.subsystems.leds.LEDsConstants.LEDStates;

public class LEDsIOSim extends LEDsIO {
  private LEDStates state = LEDStates.IDLE;

  public LEDsIOSim() {}

  @Override
  public void updateInputs() {
    super.connected = true;
    super.currentState = state;

    DogLog.log("LEDs/Connected", super.connected);
    DogLog.log("LEDs/CurrentState", super.currentState);
  }

  @Override
  public void setState(LEDsConstants.LEDStates state) {
    super.currentState = state;
  }

  @Override
  public LEDsConstants.LEDStates getState() {
    return super.currentState;
  }
}
