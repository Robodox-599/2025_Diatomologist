package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;

import frc.robot.subsystems.climb.ClimbConstants;

public class MotorLog extends DogLog{
    public static void log(String key, TalonFX motor){
        //Endefector
        log(key + "/Velocity", motor.getVelocity().getValueAsDouble());
        log(key + "/Voltage", motor.getSupplyVoltage().getValueAsDouble());
        log(key + "/CurrentAmps", motor.getStatorCurrent().getValueAsDouble());
        log(key + "/Temp", motor.getDeviceTemp().getValueAsDouble());
        //Climb
        log(key + "/StatorCurrentAmps", motor.getStatorCurrent().getValueAsDouble());
        log(key + "/PositionRotations", motor.getPosition().getValueAsDouble());
        log(key + "/VelocityRotsPerSec", motor.getVelocity().getValueAsDouble());
        log(key + "/AppliedVoltage", motor.getSupplyVoltage().getValueAsDouble());
        log(key+ "/TempCelcius", motor.getDeviceTemp().getValueAsDouble());
        log(key + "/PositionInches", motor.getPosition().getValueAsDouble() * ClimbConstants.inchesPerRev);
        log(key + "/VelocityInchesPerSec", motor.getVelocity().getValueAsDouble() * ClimbConstants.inchesPerRev);
        log(key + "/CurrentAmps", motor.getSupplyCurrent().getValueAsDouble());
    }
}

