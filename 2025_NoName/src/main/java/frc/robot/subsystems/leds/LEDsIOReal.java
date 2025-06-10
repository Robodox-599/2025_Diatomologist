package frc.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import com.ctre.phoenix6.signals.RGBWColor;

import dev.doglog.DogLog;

public class LEDsIOReal extends LEDsIO {
  public final CANdle candleReal;

  private static final RGBWColor kIntakingCoralStation = new RGBWColor(255, 165, 0, 127); // light orange
  private static final RGBWColor kIntakingAlgae = new RGBWColor(0, 0, 255, 64); // blue
  private static final RGBWColor kPrepared = new RGBWColor(0, 255, 0, 64); // green
  private static final RGBWColor kPositionCoral = new RGBWColor(255, 0, 255, 127); // light purple
  private static final RGBWColor kPositionAlgae = new RGBWColor(0, 255, 255, 127); // cyan
  private static final RGBWColor kScoringGamePiece = new RGBWColor(0, 255, 255, 127); // light red
  private static final RGBWColor kNoState = new RGBWColor(255, 0, 0, 0); // dark red


  public LEDsIOReal() {
    candleReal = new CANdle(LEDsConstants.canID, LEDsConstants.CANbus);
    CANdleConfiguration cfg = new CANdleConfiguration();

    cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Enabled;
    cfg.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Modulated;

    cfg.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;
    cfg.LED.StripType = StripTypeValue.GRB;
    cfg.LED.BrightnessScalar = 0.5;

    candleReal.getConfigurator().apply(cfg);
  }

  @Override
  public void updateInputs() {
    super.connected = true;
    DogLog.log("LEDs/Connected", super.connected);
  }

  @Override
  public void LEDsIntakingCoralStation() {
    candleReal.setControl(new ColorFlowAnimation(0, LEDsConstants.MAX_LEDS)
        .withColor(kIntakingCoralStation));
  }

  @Override
  public void LEDsIntakingAlgaeGround() {
    candleReal.setControl(new ColorFlowAnimation(0, (LEDsConstants.MAX_LEDS / 3))
        .withColor(kIntakingAlgae));
  }

  @Override
  public void LEDsIntakingAlgaeL2() {
    candleReal.setControl(new ColorFlowAnimation(0, ((LEDsConstants.MAX_LEDS / 3) * 2))
        .withColor(kIntakingAlgae));
  }

  @Override
  public void LEDsIntakingAlgaeL3() {
    candleReal.setControl(new ColorFlowAnimation(0, (LEDsConstants.MAX_LEDS))
        .withColor(kIntakingAlgae));
  }

  @Override
  public void LEDsPositionPrepared() {
    candleReal.setControl(new SolidColor(0, LEDsConstants.MAX_LEDS).withColor(kPrepared));
  }

  @Override
  public void LEDsPositionCoralL1() {
    candleReal.setControl(new SolidColor(0, (LEDsConstants.MAX_LEDS / 4))
        .withColor(kPositionCoral));
  }

  @Override
  public void LEDsPositionCoralL2() {
    candleReal.setControl(new SolidColor(0, ((LEDsConstants.MAX_LEDS / 4) * 2))
        .withColor(kPositionCoral));
  }

  @Override
  public void LEDsPositionCoralL3() {
    candleReal.setControl(new SolidColor(0, ((LEDsConstants.MAX_LEDS / 4) * 3))
        .withColor(kPositionCoral));
  }

  @Override
  public void LEDsPositionCoralL4() {
    candleReal.setControl(new SolidColor(0, LEDsConstants.MAX_LEDS)
        .withColor(kPositionCoral));
  }

  @Override
  public void LEDsPositionAlgaeProcessor() {
    candleReal.setControl(new SolidColor(0, (LEDsConstants.MAX_LEDS / 2))
        .withColor(kPositionAlgae));
  }

  @Override
  public void LEDsPositionAlgaeBarge() {
    candleReal.setControl(new SolidColor(0, LEDsConstants.MAX_LEDS)
        .withColor(kPositionAlgae));
  }

  @Override
  public void LEDsScoringGamePiece() {
    candleReal.setControl(new StrobeAnimation(0, LEDsConstants.MAX_LEDS).withColor(kScoringGamePiece));
  }

  @Override
  public void LEDsNoState() {
    candleReal.setControl(new SolidColor(0, LEDsConstants.MAX_LEDS).withColor(kNoState));
  }
}
