package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;

public class MotorLog extends DogLog{
    public static void log(String key, TalonFX motor){

        //Rollers
        log(key + "Rollers", motor.getVelocity().getValueAsDouble());
        
        log(key + "Rollers", motor.getSupplyVoltage().getValueAsDouble());
        
        log(key + "Rollers", motor.getStatorCurrent().getValueAsDouble());

        log(key + "Rollers", motor.getDeviceTemp().getValueAsDouble());


        //Wrist velocityRadsPerSec
        log(key + "Wrist", motor.getVelocity().getValueAsDouble());

        //Wrist appliedVoltage
        log(key + "Wrist", motor.getSupplyVoltage().getValueAsDouble());

        //Wrist currentAmps
        log(key + "Wrist", motor.getStatorCurrent().getValueAsDouble());

        //Wrist tempCelcius
        log(key + "Wrist", motor.getDeviceTemp().getValueAsDouble());

        //Wrist setpointAngle
        log(key + "Wrist", motor.getPosition().getValueAsDouble());


        log(key + "Claw", motor.getVelocity().getValueAsDouble());

        //Claw appliedVoltage
        log(key + "Claw", motor.getSupplyVoltage().getValueAsDouble());

        //Claw currentAmps
        log(key + "Claw", motor.getStatorCurrent().getValueAsDouble());

        //Claw tempCelcius
        log(key + "Claw", motor.getDeviceTemp().getValueAsDouble());

        //Claw setpointAngle
        log(key + "Claw", motor.getPosition().getValueAsDouble());
    }
}
