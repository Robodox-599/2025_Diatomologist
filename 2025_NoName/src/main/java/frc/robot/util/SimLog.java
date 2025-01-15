package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimLog extends DogLog {
  public static void log(String key, DCMotorSim motor) {
    log(key + "/velocityRadsPerSec", motor.getAngularVelocityRPM() / 60);
    log(key + "/appliedVoltage", motor.getInputVoltage());
    log(key + "/currentAmps", motor.getCurrentDrawAmps());
    log(key + "/tempCelcius", 60);
    log(key + "/setpointPosition", motor.getAngularPositionRotations());
  }
}