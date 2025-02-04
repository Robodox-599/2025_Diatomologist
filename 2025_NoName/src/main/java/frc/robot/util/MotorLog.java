package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;

public class MotorLog extends DogLog {
  public static void log(String key, TalonFX motor) {
    log(key + "/Velocity", motor.getVelocity().getValueAsDouble());
    log(key + "/Voltage", motor.getSupplyVoltage().getValueAsDouble());
    log(key + "/CurrentAmps", motor.getStatorCurrent().getValueAsDouble());
    log(key + "/Temp", motor.getDeviceTemp().getValueAsDouble());
  }
}
