package frc.robot.subsystems.leds;

import dev.doglog.DogLog;
import frc.robot.subsystems.leds.LEDsConstants.LEDAnim;

public class LEDsIOSim extends LEDsIO {
  private LEDAnim state = LEDAnim.NoState;

  public LEDsIOSim() {}

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
  }

  @Override
  public void enableAlgaeIntake() {
    state = LEDAnim.AlgaeIntake;
  }

  @Override
  public void enableNoState() {
    state = LEDAnim.NoState;
  }

  @Override
  public void enableScored() {
    state = LEDAnim.Scored;
  }

  @Override
  public void enableClimb() {
    state = LEDAnim.Climb;
  }

  @Override
  public void enableAutoAlign() {
    state = LEDAnim.AutoAlign;
  }

  @Override
  public void enableReadyToScore() {
    state = LEDAnim.ReadyToScore;
  }
}
