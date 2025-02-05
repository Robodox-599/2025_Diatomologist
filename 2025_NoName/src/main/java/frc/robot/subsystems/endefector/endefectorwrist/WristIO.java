package frc.robot.subsystems.endefector.endefectorwrist;

import frc.robot.subsystems.endefector.endefectorwrist.WristConstants.WristStates;
import frc.robot.util.EndefectorUtil;

public abstract class WristIO {
  protected double tempCelsius = 0.0;
  protected double currentAmps = 0.0;
  protected double appliedVolts = 0.0;
  protected double velocity = 0.0;
  protected double targetPosition = 0.0;
  protected double currentPosition = 0.0;
  protected double position = 0.0;
  protected boolean atSetpoint = false;
  protected WristConstants.WristStates state = WristConstants.WristStates.STOW;

  public void updateInputs() {}

  public void setVoltage(double voltage) {}

  public void goToPose(double position) {}

  public double getPose() {
    return 0.0;
  }

  public void stop() {}

  public void setBrake(boolean brake) {}

  public void setState(WristStates state) {}

  public WristConstants.WristStates getCurrentState() {
    return state;
  }

  public double GetCurrentVolts() {
    return appliedVolts;
  }

  public double getCurrentPosition() {
    return EndefectorUtil.convertToTicks(currentPosition);
  }
}
