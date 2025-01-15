package frc.robot.subsystems.endefector.rollers;
import frc.robot.util.MotorLog;
import static frc.robot.subsystems.endefector.rollers.RollersConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;

public class RollersIOTalonFX extends RollersIO{
    
    private final TalonFX rollersMotor;
    TalonFXConfiguration rollersConfig;
    private double desiredVelocity;
 
    {
        rollersMotor = new TalonFX(rollersMotorID, rollersMotorCANBus);
        rollersConfig = new TalonFXConfiguration();
        
        rollersConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
        rollersConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
        rollersConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
        rollersConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

        rollersMotor.optimizeBusUtilization();
        rollersMotor.getConfigurator().apply(rollersConfig);
    }
    
    @Override
    public void updateInputs(){
    super.velocityRadsPerSec = rollersMotor.getVelocity().getValueAsDouble();
    super.appliedVoltage = rollersMotor.getSupplyVoltage().getValueAsDouble();
    super.currentAmps = rollersMotor.getStatorCurrent().getValueAsDouble();
    super.tempCelcius = rollersMotor.getDeviceTemp().getValueAsDouble();
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

