package frc.robot.subsystems.endefector.endefectorrollers;

public abstract class RollersIO {
  protected double tempCelsius = 0.0;
  protected double currentAmps = 0.0;
  protected double appliedVolts = 0.0;
  protected double velocity = 0.0;
  protected double desiredVelocity = 0.0;
  protected boolean atSetpoint = false;
  protected RollersConstants.EndefectorRollerStates currentState =
      RollersConstants.EndefectorRollerStates.STOP;

  public void updateInputs() {}

  public void setVoltage(double voltage) {}

  public void stop() {}

  public void setVelocity(double velocity) {}

  public void setState(RollersConstants.EndefectorRollerStates state) {}

  public void setBrake(boolean brake) {}

  public double GetCurrentVolts() {
    return appliedVolts;
  }

  public RollersConstants.EndefectorRollerStates getCurrentState() {
    return currentState;
  }

  public boolean rangeDeviceDetected(){
    return false;
  }
}
