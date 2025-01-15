package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;

public class MotorLog extends DogLog{
    public static void log(String key, TalonFX motor){

        //Rollers
        log(key + "/velocityRadsPerSec", motor.getVelocity().getValueAsDouble());
        
        log(key + "/appliedVoltage", motor.getSupplyVoltage().getValueAsDouble());
        
        log(key + "/currentAmps", motor.getStatorCurrent().getValueAsDouble());

        log(key + "/tempCelcius", motor.getDeviceTemp().getValueAsDouble());

        //Wrist velocityRadsPerSec
        log(key + "Wrist", motor.getVelocity().getValueAsDouble());

        //Wrist appliedVoltage
        log(key + "Wrist", motor.getSupplyVoltage().getValueAsDouble());

        //Wrist currentAmps
        log(key + "Wrist", motor.getStatorCurrent().getValueAsDouble());

        //Wrist tempCelcius
        log(key + "Rollers", motor.getDeviceTemp().getValueAsDouble());

        //Wrist setpointAngle
        log(key + "Wrist", motor.getPosition().getValueAsDouble());
    }
}
