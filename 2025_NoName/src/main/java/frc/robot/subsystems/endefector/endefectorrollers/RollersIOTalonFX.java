package frc.robot.subsystems.endefector.endefectorrollers;

import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class RollersIOTalonFX extends RollersIO {

  private final TalonFX rollersMotor;
  TalonFXConfiguration rollersConfig;
  private TorqueCurrentFOC torqueCurrent;
  private CANrange CANrange;
  Debouncer CANrangeDebouncer = new Debouncer(0.1);
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<Distance> distance;

  private double desiredVelocity;

  {
    rollersMotor = new TalonFX(rollersMotorID, rollersMotorCANBus);
    torqueCurrent = new TorqueCurrentFOC(65);
    rollersConfig = new TalonFXConfiguration();
    this.CANrange = new CANrange(CANrangeId, CANrangeCANbus);
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

    PhoenixUtil.tryUntilOk(10, () -> rollersMotor.getConfigurator().apply(rollersConfig, 1));
    rollersMotor.optimizeBusUtilization();
    CANrange.getConfigurator().apply(configs);
    position = rollersMotor.getPosition();
    velocity = rollersMotor.getVelocity();
    appliedVolts = rollersMotor.getMotorVoltage();
    current = rollersMotor.getStatorCurrent();
    temperature = rollersMotor.getDeviceTemp();
    distance = CANrange.getDistance();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, distance, temperature, position, current, appliedVolts);
  }

  @Override
  public void updateInputs() {
    super.appliedVolts = appliedVolts.getValueAsDouble();
    super.currentAmps = current.getValueAsDouble();
    super.velocity = velocity.getValueAsDouble();
    super.tempCelsius = temperature.getValueAsDouble();
    super.canrangeDistance = distance.getValueAsDouble() - noCoralDistance;
    super.desiredVelocity = desiredVelocity;
    if (super.currentAmps >= 10) {
      super.isAlgaeDetected = true;
    } else {
      super.isAlgaeDetected = false;
    }

    DogLog.log("Rollers/StatorCurrentAmps", super.currentAmps);
    DogLog.log("Rollers/Velocity", super.velocity);
    DogLog.log("Rollers/AppliedVoltage", super.appliedVolts);
    DogLog.log("Rollers/TempCelcius", super.tempCelsius);
    DogLog.log("Rollers/VelocitySetpoint", desiredVelocity);
    DogLog.log("Rollers/State", super.currentState);
    DogLog.log("Rollers/AlgaeDetected", super.isAlgaeDetected);
    DogLog.log("Rollers/CANRangeDistance", super.canrangeDistance);
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
  public void setSpeed(double speed) {
    rollersMotor.set(speed);
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
        setSpeed(0);
        break;
      case SCORE:
        setSpeed(rollersScoreSpeed);
        break;
      case INTAKE:
        setSpeed(-rollersScoreSpeed);
        break;
      case ALGAEINTAKE:
        rollersMotor.setControl(torqueCurrent);
      default:
        setSpeed(0);
        break;
    }
  }

  @Override
  public boolean isDetected() {
    double rangeDistance = Units.metersToInches(CANrange.getDistance().getValueAsDouble());
    return CANrangeDebouncer.calculate(rangeDistance <= RollersConstants.detectionDistance);
  }

  @Override
  public double getCoralDistance() {
    return CANrange.getDistance().getValueAsDouble();
  }
}
