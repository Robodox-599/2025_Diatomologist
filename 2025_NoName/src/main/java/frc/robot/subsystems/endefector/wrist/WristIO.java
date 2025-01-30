package frc.robot.subsystems.endefector.wrist;
import static frc.robot.subsystems.endefector.wrist.WristConstants.*;

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

  public void setState(WristStates state) {}
}