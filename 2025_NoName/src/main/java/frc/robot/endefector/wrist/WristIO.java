package frc.robot.endefector.wrist;

public abstract class WristIO {
  //  public static class WristIOInputs {
  // //  protected double targetPosition = 0.0;
  // //  protected double currentPosition = 0.0;
  //   }

  public void updateInputs(){}
  
  public void setVoltage(double voltage){}

  public void goToPose(double position){}

  public double getPose() {
    return 0.0;
  }

  public void stop() {}

  public void setBrake(boolean brake) {}
}


