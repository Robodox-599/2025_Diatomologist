package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import dev.doglog.DogLog;
import frc.robot.subsystems.leds.LEDsConstants.LEDAnim;

public class LEDsIOReal extends LEDsIO {
  public final CANdle candle;
  private int channel = 0;
  private LEDAnim state = LEDAnim.NoState;

  public LEDsIOReal() {
    candle = new CANdle(LEDsConstants.canID, LEDsConstants.CANbus);
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = false;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(configAll, 100);
  }

  @Override
  public void updateInputs() {
    super.connected = true;
    super.anim = state;

    DogLog.log("LEDs/Connected", super.connected);
    DogLog.log("LEDs/Anim", super.anim);
  }

  @Override
  public void updateAnim(LEDAnim anim) {
    state = anim;
    switch (anim) {
      default:
      case NoState:
        channel = 0;
        candle.setLEDs(255, 0, 0);
        break;
      case NoGamePiece:
        channel = 1;
        candle.animate(
            new ColorFlowAnimation(
                128,
                20,
                70,
                0,
                0.7,
                LEDsConstants.LEDS_PER_ANIMATION,
                Direction.Forward,
                channel * LEDsConstants.LEDS_PER_ANIMATION + 8),
            channel);
        break;
      case YesGamePiece:
        channel = 2;
        candle.animate(
            new RainbowAnimation(
                1,
                0.7,
                LEDsConstants.LEDS_PER_ANIMATION,
                false,
                channel * LEDsConstants.LEDS_PER_ANIMATION + 8),
            channel);
        break;
      case CanScore:
        channel = 3;
        candle.animate(
            new StrobeAnimation(
                240,
                10,
                180,
                0,
                0.01,
                LEDsConstants.LEDS_PER_ANIMATION,
                channel * LEDsConstants.LEDS_PER_ANIMATION + 8),
            channel);
        break;
      case Intaking:
        channel = 4;
        candle.animate(
            new StrobeAnimation(
                240,
                10,
                180,
                0,
                0.01,
                LEDsConstants.LEDS_PER_ANIMATION,
                channel * LEDsConstants.LEDS_PER_ANIMATION + 8),
            channel);
        break;
      case Climb:
        channel = 5;
        candle.animate(
            new StrobeAnimation(
                240,
                10,
                180,
                0,
                0.01,
                LEDsConstants.LEDS_PER_ANIMATION,
                channel * LEDsConstants.LEDS_PER_ANIMATION + 8),
            channel);
        break;
    }
  }
}
