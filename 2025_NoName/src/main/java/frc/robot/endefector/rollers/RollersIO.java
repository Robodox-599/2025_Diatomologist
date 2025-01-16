package frc.robot.endefector.rollers;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.TalonFX;
// // import dev.doglog.DogLog;

public abstract class RollersIO {
  public static class RollersIOInputs {
    // protected double velocityRadsPerSec = 0.0;
   
    // protected double appliedVoltage = 0.0;
   
    // protected double currentAmps = 0.0;
   
    // protected double tempCelcius = 0.0;
  }
  
  public void updateInputs() {}
  
  public void setVoltage(double voltage) {}
  
  public void stop(){}
  
  public void setVelocity(double velocity){}

  public void setBrake(boolean brake){}
}
