package frc.robot.subsystems.leds;

import frc.robot.subsystems.leds.LEDsConstants.LEDStates;

public abstract class LEDsIO {
  protected boolean connected = false;
  protected LEDStates currentState = LEDStates.IDLE;

  public void updateInputs() {}

  public void setState(LEDsConstants.LEDStates state) {}

  public LEDsConstants.LEDStates getState() {
    return currentState;
  }
}
