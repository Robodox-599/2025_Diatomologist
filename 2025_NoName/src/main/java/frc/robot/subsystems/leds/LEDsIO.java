package frc.robot.subsystems.leds;

import frc.robot.subsystems.leds.LEDsConstants.LEDAnim;

public abstract class LEDsIO {
  protected boolean connected = false;
  protected LEDAnim anim = LEDAnim.NoState;

  public void updateInputs() {}

  public void enableStationIntake() {}

  public void enableAlgaeIntake() {}

  public void enableClimb() {}

  public void enableNoState() {}

  public void enableReadyToScore() {}

  public void enableScored() {}

  public void enableAutoAlign() {}
}
