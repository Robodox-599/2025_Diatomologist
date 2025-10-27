package frc.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import dev.doglog.DogLog;

public class LEDsIOReal extends LEDsIO {
  public final CANdle candleReal;

  private static final RGBWColor kIntakingCoralStation = new RGBWColor(255, 60, 0, 255); // orange
  private static final RGBWColor kEnsuringCoral = new RGBWColor(255, 255, 255, 255); // white
  private static final RGBWColor kIntakingAlgae = new RGBWColor(0, 0, 255, 255); // blue
  private static final RGBWColor kPrepared = new RGBWColor(0, 255, 0, 255); // green
  private static final RGBWColor kPositionCoral = new RGBWColor(255, 0, 255, 255); // purple
  private static final RGBWColor kPositionAlgae = new RGBWColor(0, 255, 255, 255); // cyan
  private static final RGBWColor KPositionClimbPrepared = new RGBWColor(200, 255, 0, 255); // lime
  private static final RGBWColor kScoringGamePiece = new RGBWColor(255, 15, 122, 255); // pink
  private static final RGBWColor kClimbing = new RGBWColor(255, 255, 0, 255); // yellow
  private static final RGBWColor kStopped = new RGBWColor(255, 0, 0, 255); // red

  public LEDsIOReal() {
    candleReal = new CANdle(LEDsConstants.canID, LEDsConstants.CANbus);
    CANdleConfiguration cfg = new CANdleConfiguration();

    cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Enabled;

    cfg.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;
    cfg.LED.StripType = StripTypeValue.RGB;
    cfg.LED.BrightnessScalar = 0.5;

    candleReal.getConfigurator().apply(cfg);
  }

  @Override
  public void updateInputs() {
    super.connected = true;
    DogLog.log("LEDs/Connected", super.connected);
  }

  @Override
  public void LEDsPositionCoralStation() {
    candleReal.setControl(
        new StrobeAnimation(0, LEDsConstants.MAX_LEDS).withColor(kEnsuringCoral).withFrameRate(6));
  }

  @Override
  public void LEDsIntakingCoralStation() {
    candleReal.setControl(
        new StrobeAnimation(0, LEDsConstants.MAX_LEDS)
            .withColor(kIntakingCoralStation)
            .withFrameRate(6));
  }

  @Override
  public void LEDsIntakingAlgae() {
    candleReal.setControl(
        new StrobeAnimation(0, LEDsConstants.MAX_LEDS).withColor(kIntakingAlgae).withFrameRate(6));
  }

  @Override
  public void LEDsPositionPrepared() {
    candleReal.setControl(
        new StrobeAnimation(0, LEDsConstants.MAX_LEDS).withColor(kPrepared).withFrameRate(6));
  }

  @Override
  public void LEDsPositionCoral() {
    candleReal.setControl(
        new StrobeAnimation(0, LEDsConstants.MAX_LEDS).withColor(kPositionCoral).withFrameRate(6));
  }

  @Override
  public void LEDsPositionAlgae() {
    candleReal.setControl(
        new StrobeAnimation(0, LEDsConstants.MAX_LEDS).withColor(kPositionAlgae).withFrameRate(6));
  }

  @Override
  public void LEDsPositionClimbPrepared() {
    candleReal.setControl(
        new StrobeAnimation(0, LEDsConstants.MAX_LEDS)
            .withColor(KPositionClimbPrepared)
            .withFrameRate(6));
  }

  @Override
  public void LEDsScoringGamePiece() {
    candleReal.setControl(
        new StrobeAnimation(0, LEDsConstants.MAX_LEDS)
            .withColor(kScoringGamePiece)
            .withFrameRate(6));
  }

  @Override
  public void LEDsClimbing() {
    candleReal.setControl(
        new StrobeAnimation(0, LEDsConstants.MAX_LEDS).withColor(kClimbing).withFrameRate(6));
  }

  @Override
  public void LEDsStopped() {
    candleReal.setControl(
        new StrobeAnimation(0, LEDsConstants.MAX_LEDS).withColor(kStopped).withFrameRate(6));
  }
}
