package frc.robot.subsystems.endefector.endefectorwrist;

import frc.robot.subsystems.endefector.endefectorwrist.WristConstants.WristStates;

public abstract class WristIO {
  protected double tempCelsius = 0.0;
  protected double currentAmps = 0.0;
  protected double appliedVolts = 0.0;
  protected double velocity = 0.0;
  // protected double acceleration = 0.0;
  protected double targetPosition = 0.0;
  protected double currentPosition = 0.0;
  protected boolean atSetpoint = false;

  protected boolean isCoralInEndefector = false;

  public void updateInputs() {}

  public void setVoltage(double voltage) {}

  public void goToPose(double position) {}

  public void stop() {}

  public void setBrake(boolean brake) {}

  public void setAngle(WristStates state) {}

  public double getCurrentVolts() {
    return appliedVolts;
  }
}
