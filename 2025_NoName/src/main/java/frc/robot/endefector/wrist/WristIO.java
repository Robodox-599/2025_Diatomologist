package frc.robot.subsystems.endefector.wrist;

public abstract class WristIO {
   public static class WristIOInputs {
  //  protected double targetPosition = 0.0;
  //  protected double currentPosition = 0.0;
    }

  public void updateInputs(WristIOInputs inputs){}
  
  public void setVoltage(double voltage){}

  public void goToPose(){}

  public double getPose() {
    return 0.0;
  }

  public void stop() {}

  public void setBrake(boolean brake) {}
}


