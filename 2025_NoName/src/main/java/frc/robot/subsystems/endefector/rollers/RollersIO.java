package frc.robot.subsystems.endefector.rollers;

public abstract class RollersIO {
  public void updateInputs() {}
  
  public void setVoltage(double voltage) {}
  
  public void stop(){}
  
  public void setVelocity(double velocity){}

  public void setBrake(boolean brake){}
}