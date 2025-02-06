package frc.robot.subsystems.endefector.endefectorrollers;

import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import frc.robot.util.MotorLog;
import com.ctre.phoenix6.hardware.CANrange;

public class RollersIOTalonFX extends RollersIO {

  private final TalonFX rollersMotor;
  TalonFXConfiguration rollersConfig;
  private  CANrange CANrange;

  private double desiredVelocity;

  {
    rollersMotor = new TalonFX(rollersMotorID, rollersMotorCANBus);
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

    rollersMotor.optimizeBusUtilization();
    rollersMotor.getConfigurator().apply(rollersConfig);
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

  @Override
  public void setState(RollersConstants.EndefectorRollerStates state) {
    super.currentState = state;
    switch (state) {
      case STOP:
        setVelocity(0);
        break;
      case SCORE:
        setVelocity(rollersScoreVelocity);
        break;
      case INTAKE:
        setVelocity(-rollersScoreVelocity);
        break;
      default:
        setVelocity(0);
        break;
    }
  }

  @Override
  public boolean rangeDeviceDetected() {
  double rangeSignal = 0.0;
  rangeSignal = CANrange.getDistance().getValueAsDouble();

  if (rangeSignal >= rangeTolerance) {
      return true;
  } else {
      return false;
  }
 }
}
