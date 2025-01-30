package frc.robot.subsystems.algaegroundintake.rollers;

import static frc.robot.subsystems.algaegroundintake.rollers.RollersConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import frc.robot.util.MotorLog;
import frc.robot.util.PhoenixUtil;

public class RollersIOTalonFX extends RollersIO {
    private TalonFX rollersMotor;
    TalonFXConfiguration rollersConfig;


    private final VelocityVoltage rollersVelocityVoltage = new VelocityVoltage(0).withSlot(1);

    public RollersIOTalonFX() {
        rollersMotor = new TalonFX(rollersMotorID, rollersMotorCANBus);
        rollersConfig = new TalonFXConfiguration();

        rollersConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
        rollersConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
        rollersConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
        rollersConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;


        rollersMotor.optimizeBusUtilization();
        PhoenixUtil.tryUntilOk(5, ()-> rollersMotor.getConfigurator().apply(rollersConfig));
    }


    @Override
    public void updateInputs() {
      super.appliedVolts = rollersMotor.getMotorVoltage().getValueAsDouble();
      super.currentAmps = rollersMotor.getSupplyCurrent().getValueAsDouble();
      super.velocity = rollersMotor.getVelocity().getValueAsDouble();
      super.tempCelsius = rollersMotor.getDeviceTemp().getValueAsDouble();
      super.desiredVelocity = desiredVelocity;
      
      MotorLog.log("Rollers", rollersMotor);
      DogLog.log("Rollers/VelocitySetpoint", desiredVelocity);
    }
    @Override
    public void setVoltage(double voltage){
      rollersMotor.setVoltage(voltage);
    }
    public void setBrake(boolean brake) {
        if (brake) {
            rollersMotor.setNeutralMode(NeutralModeValue.Brake);
         } else {
            rollersMotor.setNeutralMode(NeutralModeValue.Coast);
        }
  }

  @Override
  public void setVelocity(double velocity) {
    rollersMotor.setControl(rollersVelocityVoltage.withVelocity(Units.radiansPerSecondToRotationsPerMinute(velocity)/60));
  }

  @Override
  public void stop() {
    rollersMotor.setVoltage(0);
  }
}   
