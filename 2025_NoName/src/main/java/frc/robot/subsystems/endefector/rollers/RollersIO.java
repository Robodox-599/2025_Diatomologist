package frc.robot.subsystems.endefector.rollers;

public abstract class RollersIO {
  protected double tempCelsius = 0.0;
  protected double currentAmps = 0.0;
  protected double appliedVolts = 0.0;
  protected double velocity = 0.0;
  protected double desiredVelocity = 0.0;

  public void updateInputs() {}
  
  public void setVoltage(double voltage) {}
  
  public void stop(){}
  
  public void setVelocity(double velocity){}

  public void setBrake(boolean brake){}

  public boolean rangeDeviceDetected(){
    return false;
  }
}