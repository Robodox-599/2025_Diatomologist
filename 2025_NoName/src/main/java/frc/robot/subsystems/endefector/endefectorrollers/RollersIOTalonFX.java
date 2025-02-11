package frc.robot.subsystems.endefector.endefectorrollers;

import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.*;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.MotorLog;

public class RollersIOTalonFX extends RollersIO {

  private final TalonFX rollersMotor;
  TalonFXConfiguration rollersConfig;
  private CANrange CANrange;
  private Timer CANrangeTimer;

  private double desiredVelocity;

  {
    rollersMotor = new TalonFX(rollersMotorID, rollersMotorCANBus);
    rollersConfig = new TalonFXConfiguration();
    CANrange CANrange = new CANrange(CANrangeId, CANrangeCANbus);
    Timer CANrangeTimer = new Timer();
    CANrangeConfiguration configs = new CANrangeConfiguration();

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
    CANrange.getConfigurator().apply(configs);
  }

  @Override
  public void updateInputs() {
    super.appliedVolts = rollersMotor.getMotorVoltage().getValueAsDouble();
    super.currentAmps = rollersMotor.getSupplyCurrent().getValueAsDouble();
    super.velocity = rollersMotor.getVelocity().getValueAsDouble();
    super.tempCelsius = rollersMotor.getDeviceTemp().getValueAsDouble();
    super.canrangeDistance = CANrange.getDistance().getValueAsDouble() - noCoralDistance;
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
  public void startTimer() {
    CANrangeTimer.start();
  }

  @Override
  public double getTimer() {
    double time = CANrangeTimer.getTimestamp();
    return time;
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
  public boolean deviceDetected() {
    double rangeDistance = CANrange.getDistance().getValueAsDouble() * 39.3701;
    boolean isDeviceDetected = false;
    if (rangeDistance <= RollersConstants.detectionDistance) {
      isDeviceDetected = true;
      CANrangeTimer.reset();
    }
    return isDeviceDetected;
  }
}
