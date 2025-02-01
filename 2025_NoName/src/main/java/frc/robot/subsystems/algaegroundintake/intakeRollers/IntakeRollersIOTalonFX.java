package frc.robot.subsystems.algaegroundintake.intakeRollers;

import static frc.robot.subsystems.algaegroundintake.intakeRollers.IntakeRollersConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import frc.robot.util.MotorLog;

public class IntakeRollersIOTalonFX extends IntakeRollersIO {
        
  private final TalonFX rollersMotor;
  TalonFXConfiguration rollersConfig;
  
  private double desiredVelocity;

  {
      rollersMotor = new TalonFX(rollersMotorID, rollersMotorCANBus);
      rollersConfig = new TalonFXConfiguration();

      rollersConfig.Slot0.kP = IntakeRollersConstants.realP;
      rollersConfig.Slot0.kI = IntakeRollersConstants.realI;
      rollersConfig.Slot0.kD = IntakeRollersConstants.realD;
      rollersConfig.Slot0.kS = IntakeRollersConstants.realS;
      rollersConfig.Slot0.kV = IntakeRollersConstants.realV;
      
      rollersConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
      rollersConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
      rollersConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
      rollersConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

      rollersMotor.optimizeBusUtilization();
      rollersMotor.getConfigurator().apply(rollersConfig);
  }

  @Override
  public void updateInputs(){
      super.appliedVolts = rollersMotor.getMotorVoltage().getValueAsDouble();
      super.currentAmps = rollersMotor.getSupplyCurrent().getValueAsDouble();
      super.velocity = rollersMotor.getVelocity().getValueAsDouble();
      super.tempCelsius = rollersMotor.getDeviceTemp().getValueAsDouble();
      super.desiredVelocity = desiredVelocity;

      MotorLog.log("intakeRollers", rollersMotor);
      DogLog.log("intakeRollers/VelocitySetpoint", desiredVelocity);
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
