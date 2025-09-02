package frc.robot.subsystems.endefector.endefectorrollers;

import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.EndefectorRollerStates;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SubsystemUtil;

public class RollersIOTalonFX extends RollersIO {
  private final TalonFX rollersMotor;
  TalonFXConfiguration rollersConfig;
  Debouncer algaeStallDebouncer = new Debouncer(algaeDebounce);
  Debouncer coralBeamBreakDebouncer = new Debouncer(beamBreakDebounce);
  Debouncer ensureCoralBeamBreakDebouncer = new Debouncer(ensureCoralDebounce);
  private DigitalInput m_BeamBreak2;

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temperature;

  private double desiredVelocity;

  public RollersIOTalonFX() {
    rollersMotor = new TalonFX(rollersMotorID, rollersMotorCANBus);
    m_BeamBreak2 = new DigitalInput(RollersConstants.beakBreakPort);

    rollersConfig = new TalonFXConfiguration();

    rollersMotor.setNeutralMode(NeutralModeValue.Brake);

    rollersConfig.Slot0.kP = realP;
    rollersConfig.Slot0.kI = realI;
    rollersConfig.Slot0.kD = realD;
    rollersConfig.Slot0.kS = realS;
    rollersConfig.Slot0.kV = realV;

    rollersConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
    rollersConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
    rollersConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
    rollersConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

    coralBeamBreakDebouncer.setDebounceType(DebounceType.kBoth);
    ensureCoralBeamBreakDebouncer.setDebounceType(DebounceType.kFalling);

    PhoenixUtil.tryUntilOk(10, () -> rollersMotor.getConfigurator().apply(rollersConfig, 1));
    rollersMotor.optimizeBusUtilization();
    velocity = rollersMotor.getVelocity();
    appliedVolts = rollersMotor.getMotorVoltage();
    statorCurrent = rollersMotor.getStatorCurrent();
    temperature = rollersMotor.getDeviceTemp();
    supplyCurrent = rollersMotor.getSupplyCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, temperature, supplyCurrent, statorCurrent, appliedVolts);
  }

  @Override
  public void updateInputs() {
    BaseStatusSignal.refreshAll(velocity, temperature, statorCurrent, supplyCurrent, appliedVolts);
    super.appliedVolts = appliedVolts.getValueAsDouble();
    super.statorCurrentAmps = statorCurrent.getValueAsDouble();
    super.supplyCurrentAmps = supplyCurrent.getValueAsDouble();

    super.velocity = velocity.getValueAsDouble();
    super.tempCelsius = temperature.getValueAsDouble();
    super.desiredVelocity = desiredVelocity;
    super.isAlgaeDetected = algaeStallDebouncer.calculate(super.statorCurrentAmps >= 20);
    super.isCoralDetected = coralBeamBreakDebouncer.calculate(!m_BeamBreak2.get());
    super.isCoralEnsured = ensureCoralBeamBreakDebouncer.calculate(!m_BeamBreak2.get());

    DogLog.log("Rollers/StatorCurrentAmps", super.statorCurrentAmps);
    DogLog.log("Rollers/SupplyCurrentAmps", super.supplyCurrentAmps);

    DogLog.log("Rollers/Velocity", super.velocity);
    DogLog.log("Rollers/AppliedVoltage", super.appliedVolts);
    DogLog.log("Rollers/TempCelcius", super.tempCelsius);

    DogLog.log("Rollers/AlgaeDetected", super.isAlgaeDetected);
    DogLog.log("Rollers/CoralDetected", super.isCoralDetected);
    DogLog.log("Rollers/CoralEnsured", super.isCoralEnsured);
    DogLog.log("Rollers/BeamBreak", m_BeamBreak2.get());
  }

  @Override
  public void stop() {
    setVelocity(EndefectorRollerStates.STOPPED);
  }

  @Override
  public void setVelocity(EndefectorRollerStates state) {
    double velocity = SubsystemUtil.rollersStateToVelocity(state);
    rollersMotor.set(velocity);
  }

  @Override
  public void grabOrHoldAlgae() {
    rollersMotor.setControl(new DutyCycleOut(rollersDutyCycleOutHoldAlgae));
  }
}
