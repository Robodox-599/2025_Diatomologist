package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimLog extends DogLog {
  public static void log(String key, DCMotorSim motor) {
    log(key + "RollersVelocity", motor.getAngularVelocityRPM() / 60);
    log(key + "RollersVoltage", motor.getInputVoltage());
    log(key + "RollersAmps", motor.getCurrentDrawAmps());
    log(key + "RollersTemp", 60);
    
    log(key + "WristVelocity", motor.getAngularVelocityRPM() / 60);
    log(key + "WristVoltage", motor.getInputVoltage());
    log(key + "WristAmps", motor.getCurrentDrawAmps());
    log(key + "WristTemp", 60);
    log(key + "WristPosition", motor.getAngularPositionRotations());

    log(key + "ClawVelocity", motor.getAngularVelocityRPM() / 60);
    log(key + "ClawVoltage", motor.getInputVoltage());
    log(key + "ClawAmps", motor.getCurrentDrawAmps());
    log(key + "ClawTemp", 60);
    log(key + "ClawPosition", motor.getAngularPositionRotations());
  }
}