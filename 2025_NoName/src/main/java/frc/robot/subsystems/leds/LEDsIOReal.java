package frc.robot.subsystems.leds;

import dev.doglog.DogLog;

public class LEDsIOReal extends LEDsIO {
  // public final CANdle candleReal;

  public LEDsIOReal() {
    // candleReal = new CANdle(LEDsConstants.canID, LEDsConstants.CANbus);
    // CANdleConfiguration configAll = new CANdleConfiguration();
    // configAll.statusLedOffWhenActive = false;
    // configAll.disableWhenLOS = true;
    // configAll.stripType = LEDStripType.GRB;
    // configAll.brightnessScalar = 0.1;
    // configAll.vBatOutputMode = VBatOutputMode.Modulated;
    // candleReal.configAllSettings(configAll);
  }

  @Override
  public void updateInputs() {
    super.connected = true;
    DogLog.log("LEDs/Connected", super.connected);
  }
}
