package frc.robot.subsystems.endefector.endefectorrollers;

import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.PhoenixUtil;

public class RollersIOTalonFX extends RollersIO {

  private final TalonFX rollersMotor;
  TalonFXConfiguration rollersConfig;
  private TorqueCurrentFOC torqueCurrent;
  Debouncer CANrangeDebouncer = new Debouncer(0.03);
  private Timer beamBreakTimer = new Timer();
  private Timer PETimer = new Timer();
  private DigitalInput m_BeamBreak2;
  private DigitalInput PESensor;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final StatusSignal<Temperature> temperature;

  private double desiredVelocity;

  public RollersIOTalonFX() {
    rollersMotor = new TalonFX(rollersMotorID, rollersMotorCANBus);
    m_BeamBreak2 = new DigitalInput(RollersConstants.beakBreakPort);
    PESensor = new DigitalInput(RollersConstants.PESensorPort);
    torqueCurrent = new TorqueCurrentFOC(65);
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

    PhoenixUtil.tryUntilOk(10, () -> rollersMotor.getConfigurator().apply(rollersConfig, 1));
    rollersMotor.optimizeBusUtilization();
    position = rollersMotor.getPosition();
    velocity = rollersMotor.getVelocity();
    appliedVolts = rollersMotor.getMotorVoltage();
    current = rollersMotor.getStatorCurrent();
    temperature = rollersMotor.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, temperature, position, current, appliedVolts);
  }

  @Override
  public void updateInputs() {
    super.appliedVolts = appliedVolts.getValueAsDouble();
    super.currentAmps = current.getValueAsDouble();
    super.velocity = velocity.getValueAsDouble();
    super.tempCelsius = temperature.getValueAsDouble();
    super.desiredVelocity = desiredVelocity;
    if (super.currentAmps >= 10) {
      super.isAlgaeDetected = true;
    } else {
      super.isAlgaeDetected = false;
    }
    if (m_BeamBreak2.get()) {
      beamBreakTimer.restart();
    }
    if (PESensor.get()) {
      PETimer.restart();
    }
    DogLog.log("Rollers/StatorCurrentAmps", super.currentAmps);
    DogLog.log("Rollers/Velocity", super.velocity);
    DogLog.log("Rollers/AppliedVoltage", super.appliedVolts);
    DogLog.log("Rollers/TempCelcius", super.tempCelsius);
    DogLog.log("Rollers/VelocitySetpoint", desiredVelocity);
    DogLog.log("Rollers/State", super.currentState);
    DogLog.log("Rollers/AlgaeDetected", super.isAlgaeDetected);
    DogLog.log("Rollers/CoralDetected", this.isCoralDetected());
    DogLog.log("Rollers/AlgaeDetected", this.isAlgaeDetected());
    DogLog.log("Rollers/CANRangeDistance", super.canrangeDistance);
  }

  @Override
  public void stop() {
    setVelocity(0);
  }

  @Override
  public void setVelocity(double velocity) {
    desiredVelocity = velocity;
    rollersMotor.set(velocity);
  }

  @Override
  public void holdCoral() {
    rollersMotor.setControl(new PositionVoltage(rollersMotor.getPosition().getValueAsDouble()));
  }

  @Override
  public void holdCoralAfterIntake() {
    rollersMotor.setControl(
        new PositionVoltage(
            rollersMotor.getPosition().getValueAsDouble() + RollersConstants.distanceToMove));
  }

  @Override
  public void holdAlgae() {
    rollersMotor.setControl(new DutyCycleOut(rollersDutyCycleAlgaeHoldVoltage));
  }

  @Override
  public void setState(RollersConstants.EndefectorRollerStates state) {
    super.currentState = state;
    switch (state) {
      case SCORECORAL:
        setVelocity(rollersCoralScoreSpeed);
        break;
      case SCOREALGAE:
        setVelocity(rollersAlgaeScoreSpeed);
        break;
      case CORALSTATIONINTAKE:
        setVelocity(rollersCoralStationIntakeSpeed);
        break;
      case ALGAEINTAKE:
        setVelocity(rollersAlgaeIntakeSpeed);
        break;
      case HOLDCORAL:
        holdCoral();
        break;
      case HOLDCORALAFTERSTATIONINTAKE:
        holdCoralAfterIntake();
        break;
      case HOLDALGAE:
        holdAlgae();
        break;
      case STOP:
        setVelocity(0);
        break;
      case EJECT:
        setVelocity(rollersEjectSpeed);
        break;
      default:
        setVelocity(0);
        break;
    }
  }

  @Override
  public boolean isCoralDetected() {
    DogLog.log("Rollers/isDetected", (beamBreakTimer.get() >= 0.1));
    return (beamBreakTimer.get() >= 0.1);
  }

  @Override
  public boolean isAlgaeDetected() {
    return (PETimer.get() >= 0.1);
  }

  // @Override
  // public void setVoltage(double voltage) {
  //   rollersMotor.setVoltage(voltage);
  // }

  // @Override
  // public void setSpeed(double speed) {
  //   rollersMotor.set(speed);
  // }

  // @Override
  // public void setBrake(boolean brake) {
  //   rollersMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  // }

}
