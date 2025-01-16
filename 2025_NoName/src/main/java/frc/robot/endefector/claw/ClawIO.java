package frc.robot.endefector.claw;

public abstract class ClawIO {
    public static class ClawIOInputs {
      // protected double velocityRadsPerSec = 0.0;
     
      // protected double appliedVoltage = 0.0;
     
      // protected double currentAmps = 0.0;
     
      // protected double tempCelcius = 0.0;
    }
    
    public void updateInputs() {}
    
    public void setVoltage(double voltage) {}
    
    public void stop(){}
    
    public void goToPose(double pose){}

    public double getPose() {
      return 0.0;
    }
  
    public void setBrake(boolean brake){}
  }
  