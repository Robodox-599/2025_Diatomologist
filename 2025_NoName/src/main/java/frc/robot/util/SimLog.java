package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimLog extends DogLog {
  public static void log(String key, DCMotorSim motor) {

    //Endefector
    log(key + "/Velocity", motor.getAngularVelocityRPM() / 60);
    log(key + "/Voltage", motor.getInputVoltage());
    log(key + "/Amps", motor.getCurrentDrawAmps());
    log(key + "/Temp", 60);

    log(key + "/StatorCurrentAmps", motor.getCurrentDrawAmps());
    log(key + "/PositionRotations", motor.getAngularPositionRotations());
    log(key + "/VelocityRotsPerSec", motor.getAngularVelocityRPM() / 60);
    log(key + "/AppliedVoltage", motor.getInputVoltage());

    log(key + "/TempCelcius", 60);    
  }
}
