package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import dev.doglog.DogLog;
import frc.robot.subsystems.leds.LEDsConstants.LEDStates;
import frc.robot.util.SubsystemUtil;

public class LEDsIOReal extends LEDsIO {
  public final CANdle candleReal;
  private LEDStates state = LEDStates.IDLE;

  public LEDsIOReal() {
    candleReal = new CANdle(LEDsConstants.canID, LEDsConstants.CANbus);
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = false;
    configAll.disableWhenLOS = true;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    candleReal.configAllSettings(configAll);
  }

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
    switch (state) {
      case AUTOALIGN:
        candleReal.animate(new RainbowAnimation(1, 1, 64), 1);
        break;
      default:
        double[] colors = SubsystemUtil.LEDsStateToColor(state);
        candleReal.animate(
            new StrobeAnimation(
                (int) colors[0],
                (int) colors[1],
                (int) colors[2],
                (int) colors[3],
                colors[4],
                LEDsConstants.LEDS_PER_ANIMATION,
                0),
            1);
        break;
    }
  }

  @Override
  public LEDsConstants.LEDStates getState() {
    return super.currentState;
  }
}
