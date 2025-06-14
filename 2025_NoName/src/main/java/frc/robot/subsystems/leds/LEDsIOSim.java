package frc.robot.subsystems.leds;

import dev.doglog.DogLog;

public class LEDsIOSim extends LEDsIO {

  public LEDsIOSim() {}

  public void updateInputs() {
    super.connected = true;

    DogLog.log("LEDs/Connected", super.connected);
  }
}
