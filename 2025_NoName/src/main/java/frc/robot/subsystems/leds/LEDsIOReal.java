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
  public final CANdle candleReal;
  private LEDAnim state = LEDAnim.NoState;

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
    super.anim = state;

    DogLog.log("LEDs/Connected", super.connected);
    DogLog.log("LEDs/Anim", super.anim);
  }

  @Override
  public void enableStationIntake() {
    state = LEDAnim.StationIntake;
    // white
    candleReal.animate(
        new StrobeAnimation(255, 255, 255, 100, 0.30, LEDsConstants.LEDS_PER_ANIMATION, 0), 1);
  }

  @Override
  public void enableAlgaeIntake() {
    // green ?????
    state = LEDAnim.AlgaeIntake;
    candleReal.animate(
        new StrobeAnimation(0, 255, 0, 100, 0.30, LEDsConstants.LEDS_PER_ANIMATION, 0), 1);
  }

  @Override
  public void enableIntaked() {
    state = LEDAnim.AlgaeIntake;
    // red
    candleReal.animate(
        new StrobeAnimation(255,0, 0, 100, 0.30, LEDsConstants.LEDS_PER_ANIMATION, 0), 1);
  }

  @Override
  public void enableNoState() {
    state = LEDAnim.NoState;
    // no color
    candleReal.setLEDs(0, 0, 0);
  }

  @Override
  public void enableScored() {
    // dark green ?????
    state = LEDAnim.Scored;
    candleReal.animate(
        new StrobeAnimation(255, 91, 0, 0, 0.30, LEDsConstants.LEDS_PER_ANIMATION, 0), 1);
  }

  @Override
  public void enableScoring() {
    // neon teal
    state = LEDAnim.Scored;
    candleReal.animate(
        new StrobeAnimation(7, 242, 241, 100, 0.30, LEDsConstants.LEDS_PER_ANIMATION, 0), 1);
  }

  @Override
  public void enableClimb() {
    // blue
    state = LEDAnim.Climb;
    candleReal.animate(
        new ColorFlowAnimation(
            0, 0, 255, 0, 0.70, LEDsConstants.LEDS_PER_ANIMATION, Direction.Forward, 0),
        1);
  }

  @Override
  public void enableAutoAlign() {
    //rainbow
    state = LEDAnim.AutoAlign;
    candleReal.animate(new RainbowAnimation(1, 1, 64), 1);
  }

  @Override
  public void enableReadyToScore() {
    state = LEDAnim.ReadyToScore;
    // hot pink
    candleReal.animate(
        new StrobeAnimation(255, 0, 127, 0, 0.55, LEDsConstants.LEDS_PER_ANIMATION, 0), 1);
  }
}
