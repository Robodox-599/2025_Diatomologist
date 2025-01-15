package frc.robot.subsystems.algaegroundintake.utils;

import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import frc.robot.subsystems.algaegroundintake.rollers.Rollers;
import frc.robot.subsystems.algaegroundintake.rollers.RollersConstants;

public class MotorLog extends DogLog {
    public static void log(String key, TalonFX motor) {
    log(key + "/StatorCurrentAmps", motor.getStatorCurrent().getValueAsDouble());
    log(key + "/PositionRotations", motor.getPosition().getValueAsDouble());
    log(key + "/VelocityRotsPerSec", motor.getVelocity().getValueAsDouble());
    log(key + "/AppliedVoltage", motor.getSupplyVoltage().getValueAsDouble());
    log(key+ "/TempCelcius", motor.getDeviceTemp().getValueAsDouble());
    log(key + "/CurrentAmps", motor.getSupplyCurrent().getValueAsDouble());
    }

}
  

