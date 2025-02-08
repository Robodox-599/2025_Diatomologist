package frc.robot.subsystems.endefector.coraldistance;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralDistance extends SubsystemBase {
  private final CoralDistanceIO io;
  private CANrange CANrange;
  
  public CoralDistance(CoralDistanceIO io) {
    this.io = io;
  }

  public void periodic() {
    if (deviceDetected()) {
    }
  }

  public boolean deviceDetected() {
      String deviceDetected = CANrange.getIsDetected().toString();
      boolean isDeviceDetected = false;

      if(deviceDetected == "true"){
          isDeviceDetected = true;
      } else {
          isDeviceDetected = false;
      }

      return isDeviceDetected;
     }

   public double getDistance(){
        return CANrange.getDistance().getValueAsDouble();
    }
    

    public CoralDistanceIO getIO() {
    return io;
    }
}
