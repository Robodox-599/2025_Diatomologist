package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class Simlog extends DogLog {
  public static void log(String key, DCMotorSim motor) {
    log(key + "/StatorCurrentAmps", motor.getCurrentDrawAmps());
    log(key + "/PositionRotations", motor.getAngularPositionRotations());
    log(key + "/VelocityRotsPerSec", motor.getAngularVelocityRPM() / 60);
    log(key + "/AppliedVoltage", motor.getInputVoltage());
    log(key + "/TempCelcius", 60);    
  }
}