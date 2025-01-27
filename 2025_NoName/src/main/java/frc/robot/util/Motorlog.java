package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class Motorlog extends DogLog {
  public static void log(String key, TalonFX motor) {
    log(key + "/StatorCurrentAmps", motor.getStatorCurrent().getValueAsDouble());
    log(key + "/PositionRotations", motor.getPosition().getValueAsDouble());
    log(key + "/VelocityRotsPerSec", motor.getVelocity().getValueAsDouble());
    log(key + "/AppliedVoltage", motor.getSupplyVoltage().getValueAsDouble());
    log(key+ "/TempCelcius", motor.getDeviceTemp().getValueAsDouble());
    log(key + "/PositionInches", motor.getPosition().getValueAsDouble() * ElevatorConstants.inchesPerRev);
    log(key + "/VelocityInchesPerSec", motor.getVelocity().getValueAsDouble() * ElevatorConstants.inchesPerRev);
    log(key + "/CurrentAmps", motor.getSupplyCurrent().getValueAsDouble());
  }
}