package frc.robot.endefector.rollers;
import frc.robot.util.MotorLog;
import frc.robot.util.PhoenixUtil;

import static frc.robot.endefector.rollers.RollersConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;

public class RollersIOTalonFX extends RollersIO{
    
    private final TalonFX rollersMotor;
    TalonFXConfiguration rollersConfig;
    
    private double desiredVelocity;
 
    {
        rollersMotor = new TalonFX(1, rollersMotorCANBus);
        rollersConfig = new TalonFXConfiguration();

        rollersConfig.Slot0.kP = realP;
        rollersConfig.Slot0.kI = realI;
        rollersConfig.Slot0.kD = realD;
        rollersConfig.Slot0.kS = realS;
        rollersConfig.Slot0.kV = realV;
        
        rollersConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
        rollersConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
        rollersConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
        rollersConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

        PhoenixUtil.tryUntilOk(5, ()-> rollersMotor.getConfigurator().apply(rollersConfig));
        rollersMotor.optimizeBusUtilization();
        PhoenixUtil.tryUntilOk(5, ()-> rollersMotor.getConfigurator().apply(rollersConfig));
    }

    @Override
    public void updateInputs(){
        MotorLog.log("Rollers", rollersMotor);
        DogLog.log("Rollers/VelocitySetpoint", desiredVelocity);
    }

    @Override
    public void setVoltage(double voltage) {
        rollersMotor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        rollersMotor.setVoltage(0);
    }

    @Override
    public void setVelocity(double velocity) {
        desiredVelocity = velocity;
        rollersMotor.set(velocity);
    }

    @Override
    public void setBrake(boolean brake) {
    rollersMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}

