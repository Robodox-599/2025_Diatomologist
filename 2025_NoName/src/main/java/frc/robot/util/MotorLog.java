package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;

public class MotorLog extends DogLog {
  public static void log(String key, TalonFX motor) {
    log(key + "/StatorCurrentAmps", motor.getStatorCurrent().getValueAsDouble());
    log(key + "/PositionRotations", motor.getPosition().getValueAsDouble());
    log(key + "/VelocityRotsPerSec", motor.getVelocity().getValueAsDouble());
    log(key + "/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    log(key + "/TempCelcius", motor.getDeviceTemp().getValueAsDouble());
  }
}
